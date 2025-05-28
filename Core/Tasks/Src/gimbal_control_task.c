/*
 * gimbal_control_task.c
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_control.h"
#include "motor_config.h"
#include "can_msg_processor.h"
#include "gimbal_control_task.h"
#include "bsp_lk_motor.h"
#include "INS_task.h"
#include "bsp_damiao.h"
#include "dm4310_drv.h"

extern uint8_t control_mode;

extern uint8_t aimbot_mode;

extern EventGroupHandle_t gimbal_event_group;
extern float g_chassis_yaw;
extern motor_data_t g_can_motors[24];
extern motor_data_t g_pitch_motor;
extern gimbal_control_t gimbal_ctrl_data;
extern orientation_data_t imu_heading;
extern QueueHandle_t telem_motor_queue;
extern chassis_control_t chassis_ctrl_data;
extern int32_t chassis_rpm;
//static float rel_pitch_angle;

extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

static float prev_pit;
static float prev_yaw;

extern int g_spinspin_mode;
#ifdef YAW_FEEDFORWARD
static pid_data_t g_yaw_ff_pid = {
			.kp = YAW_FF_SPD_KP,
			.ki = YAW_FF_SPD_KI,
			.kd = YAW_FF_SPD_KD,
			.int_max = YAW_FF_INT_MAX,
			.max_out = YAW_FF_MAX_OUTPUT
};
#endif
//static float g_chassis_rot;
float curr_rot;

/**
 *
 * FreeRTOS task for gimbal controls
 * Has HIGH2 priority
 *
 */
void gimbal_control_task(void *argument) {
	TickType_t start_time;
	while (1) {
#if PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_SPD || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_ANG || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_MULTI_ANG
		lk_read_motor_sang(&g_pitch_motor);
#endif
		xEventGroupWaitBits(gimbal_event_group, 0b11, pdTRUE, pdFALSE, portMAX_DELAY);
		start_time = xTaskGetTickCount();

		if (gimbal_ctrl_data.enabled) {
			if (gimbal_ctrl_data.imu_mode) {
				gimbal_control(&g_pitch_motor,
						g_can_motors + YAW_MOTOR_ID - 1);
			} else {
				gimbal_angle_control(&g_pitch_motor,
						g_can_motors + YAW_MOTOR_ID - 1);
			}
		} else {

#if YAW_MOTOR_TYPE != TYPE_DM4310_MIT
			g_can_motors[YAW_MOTOR_ID - 1].output = 0;
#else
			dm_yaw_motor.angle_pid.output = 0;
#endif

#if PITCH_MOTOR_TYPE != TYPE_DM4310_MIT
			g_pitch_motor.output = 0;
#else
			dm_pitch_motor.angle_pid.output = 0;
#endif
		}
		prev_yaw = imu_heading.yaw;;

		status_led(2, off_led);
		xEventGroupClearBits(gimbal_event_group, 0b11);
		vTaskDelayUntil(&start_time, GIMBAL_DELAY);
	}
	//should not run here
}

/**
 * This function controls the gimbals based on IMU reading
 * @param 	pitch_motor		Pointer to pitch motor struct
 * 			yaw_motor		Pointer to yaw motor struct
 * @note both pitch and yaw are currently on CAN2 with ID5 and 6.
 * Need to check if having ID4 (i.e. 0x208) + having the launcher motors (ID 1-3, 0x201 to 0x203)
 * still provides a fast enough response
 */
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {
	//todo: add in roll compensation
	if (prev_yaw == imu_heading.yaw || prev_pit == imu_heading.pit) {
		return;}

	pitch_control(pitch_motor);
	yaw_control(yaw_motor);
}

void pitch_control(motor_data_t *pitch_motor) {
#ifndef PITCH_ARM  // for robots that do not have a 4 bar linkage on the pitch motor to the pitch assembly
	calculate_direct_pitch(pitch_motor);
#else
	 calculate_linkage_pitch(pitch_motor);
#endif
}

uint8_t limit_pitch(float *rel_pitch_angle, motor_data_t *pitch_motor) {
	uint8_t pit_lim = 0;
	if (*rel_pitch_angle > pitch_motor->angle_data.phy_max_ang) {
		*rel_pitch_angle = pitch_motor->angle_data.phy_max_ang;
		pit_lim = 1;
	}
	if (*rel_pitch_angle < pitch_motor->angle_data.phy_min_ang) {
		*rel_pitch_angle = pitch_motor->angle_data.phy_min_ang;
		pit_lim = 1;
	}
	return pit_lim;
}

void calculate_direct_pitch(motor_data_t *pitch_motor) {
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
	float target_pitch = gimbal_ctrl_data.pitch;

    if (target_pitch > dm_pitch_motor.angle_data.phy_max_ang) {
        target_pitch = dm_pitch_motor.angle_data.phy_max_ang;
    } else if(target_pitch < dm_pitch_motor.angle_data.phy_min_ang) {
        target_pitch = dm_pitch_motor.angle_data.phy_min_ang;
    }

	yaw_pid(target_pitch, imu_heading.pit, &dm_pitch_motor.angle_pid);
	if (gimbal_ctrl_data.enabled == 1){
		dm_pitch_motor.ctrl.tor_set = dm_pitch_motor.angle_pid.output + PITCH_CONST;
//		dm_pitch_motor.ctrl.tor_set = 0.4842f*imu_heading.pit - 2.3124f - gimbal_pid_pitch.output;
//		dm_pitch_motor.ctrl.tor_set += 1.1;
	} else {
		dm_pitch_motor.ctrl.tor_set = 0.0f;
	}

	dm_pitch_motor.para.heartbeat = 0;
	dm4310_ctrl_send(PITCH_MOTOR_CAN_PTR, &dm_pitch_motor);

//	if ((motor[Motor1].para.heartbeat == 0 || motor[Motor1].para.state != 9) && motor[Motor1].para.disconnect_time > 100) {
//		motor[Motor1].para.disconnect_time = 0;
//		motor[Motor1].para.online = 0;
//	} else if ((motor[Motor1].para.heartbeat == 0 || motor[Motor1].para.state != 9)) {
//		motor[Motor1].para.disconnect_time++;
//	} else {
//		motor[Motor1].para.disconnect_time = 0;
//		motor[Motor1].para.online = 1;
//	}
//	if (motor[Motor1].para.online == 1) {
//		joint_motor_online = 1;
//	} else {
//		joint_motor_online = 0;
//		HAL_CAN_Stop(&hcan1);
//		osDelay(10);  // Wait for motor power stabilization
//		HAL_CAN_Start(&hcan1);
//		osDelay(10);
//		dm4310_enable(&hcan1, &motor[Motor1]);
//		vTaskDelay(1);
//	}

#else

	uint8_t pit_lim = 0;
	float rel_pitch_angle = pitch_motor->angle_data.adj_ang
				+ gimbal_ctrl_data.pitch - imu_heading.pit;

	pit_lim = limit_pitch(&rel_pitch_angle, pitch_motor);

	if (pit_lim == 1) {
		gimbal_ctrl_data.pitch = rel_pitch_angle + imu_heading.pit
				- (pitch_motor->angle_data.adj_ang);
	}

	yangle_pid(gimbal_ctrl_data.pitch,imu_heading.pit, pitch_motor,
			imu_heading.pit, &prev_pit,1);

	int32_t temp_pit_output = pitch_motor->rpm_pid.output + PITCH_CONST;
	temp_pit_output = (temp_pit_output < -20000) ? -20000 :
						(temp_pit_output > 20000) ? 20000 : temp_pit_output;

	pitch_motor->output = temp_pit_output;
#endif

}

// Pitch calculation for robots with 4 arm linkage
void calculate_linkage_pitch(motor_data_t *pitch_motor) {
	uint8_t pit_lim = 0;

	float rel_pitch_angle = gimbal_ctrl_data.pitch; // insert function where input desired gimbal pitch angle and output corresponding motor angle

	pit_lim = limit_pitch(&rel_pitch_angle, pitch_motor);

	if (pit_lim == 1) {
		gimbal_ctrl_data.pitch = rel_pitch_angle;
	}

#if PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_SPD || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_ANG || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_MULTI_ANG

	//lazy max
	//covnert radians back to degrees
	int32_t pitch_ang = rel_pitch_angle * 57320;
	g_pitch_motor.output = pitch_ang;
	if (HAL_CAN_GetTxMailboxesFreeLevel(g_pitch_motor.can) == 0){
//		HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
		vTaskDelay(1);
	}
	lk_motor_multturn_ang(&g_pitch_motor);

#else
	angle_pid(rel_pitch_angle, pitch_motor->angle_data.adj_ang, pitch_motor);
#endif
}


void yaw_control(motor_data_t *yaw_motor) {
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT

	float turn_ang = imu_heading.yaw - prev_yaw;

	while (turn_ang > PI) {
		turn_ang -= 2 * PI;
	}
	while (turn_ang < -PI) {
		turn_ang += 2 * PI;
	}

	prev_yaw = imu_heading.yaw;
	gimbal_ctrl_data.delta_yaw -= turn_ang;

//	if (gimbal_ctrl_data.delta_yaw > 1.5 * PI) {
//		gimbal_ctrl_data.delta_yaw = 1.5 * PI;
//	}
//	if (gimbal_ctrl_data.delta_yaw < -1.5 * PI) {
//		gimbal_ctrl_data.delta_yaw = -1.5 * PI;
//	}

	 yaw_pid(0, -gimbal_ctrl_data.delta_yaw, &dm_yaw_motor.angle_pid);

//	// TODO: use another logic for error checking (link to beeping sounds)
//	if (dm_pitch_motor.para.state != 9 && dm_pitch_motor.para.disconnect_time > 100) {
//		dm_pitch_motor.para.disconnect_time = 0;
//		dm_pitch_motor.para.online = 0;
//	} else {
//		dm_pitch_motor.para.disconnect_time = 0;
//		dm_pitch_motor.para.online = 1;
//	}
//
//	if (dm_pitch_motor.para.online == 1) {
//		joint_motor_online = 1;
//	} else {
//		joint_motor_online = 0;
//	}
//
//	if (dm_yaw_motor.para.state != 9 && dm_yaw_motor.para.disconnect_time > 100) {
//		dm_yaw_motor.para.disconnect_time = 0;
//		dm_yaw_motor.para.online = 0;
//	} else {
//		dm_yaw_motor.para.disconnect_time = 0;
//		dm_yaw_motor.para.online = 1;
//	}
//
//	if (dm_yaw_motor.para.online == 1) {
//		joint_motor_online = 1;
//	} else {
//		joint_motor_online = 0;
//	}

	 dm_yaw_motor.ctrl.vel_set = dm_yaw_motor.angle_pid.output +
			chassis_ctrl_data.yaw * (YAW_SPINSPIN_CONSTANT/CHASSIS_SPINSPIN_MAX);
	 dm_yaw_motor.ctrl.tor_set = FEEDFORWARD_CONST * dm_yaw_motor.para.vel;

	dm4310_ctrl_send(YAW_MOTOR_CAN_PTR, &dm_yaw_motor);

#else
	uint8_t yaw_lim = 0;
	float rel_yaw_angle = yaw_motor->angle_data.adj_ang + gimbal_ctrl_data.yaw
			- imu_heading.yaw;

	//if yaw has overflowed (i.e. goes to the next round) move it back into pi to -pi range
	if (rel_yaw_angle > PI) {
		rel_yaw_angle -= 2 * PI;
	}
	if (rel_yaw_angle < -PI) {
		rel_yaw_angle += 2 * PI;
	}
	//check limits
	if (rel_yaw_angle > yaw_motor->angle_data.phy_max_ang) {
		rel_yaw_angle = yaw_motor->angle_data.phy_max_ang;
		yaw_lim = 1;
	}
	if (rel_yaw_angle < yaw_motor->angle_data.phy_min_ang) {
		rel_yaw_angle = yaw_motor->angle_data.phy_min_ang;
		yaw_lim = 1;
	}
	if (yaw_lim == 1) {
		gimbal_ctrl_data.yaw = rel_yaw_angle + imu_heading.yaw
				- yaw_motor->angle_data.adj_ang;
	}
	float turn_ang = imu_heading.yaw - prev_yaw;
	if (turn_ang > PI){
		turn_ang -= 2 * PI;
	} else if (turn_ang < -PI){
		turn_ang += 2 * PI;

	}
	xSemaphoreTake(gimbal_ctrl_data.yaw_semaphore,portMAX_DELAY);
	gimbal_ctrl_data.delta_yaw -= turn_ang;
	yangle_pid(gimbal_ctrl_data.delta_yaw, 0, yaw_motor,
			imu_heading.yaw, &prev_yaw,0);
	xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);

//	yangle_pid(gimbal_ctrl_data.yaw, imu_heading.yaw, yaw_motor,
//			imu_heading.yaw, &prev_yaw);
//		oangle_pid(gimbal_ctrl_data.yaw, imu_heading.yaw, yaw_motor, g_chassis_rot);

	int32_t temp_output = yaw_motor->rpm_pid.output  + (chassis_ctrl_data.yaw * YAW_SPINSPIN_CONSTANT/CHASSIS_SPINSPIN_MAX);
	temp_output = (temp_output > 20000) ? 20000 : (temp_output < -20000) ? -20000 : temp_output;
	yaw_motor->output = temp_output;

#ifdef YAW_FEEDFORWARD
	speed_pid(yaw_motor->raw_data.rpm + yaw_motor->angle_pid.output, g_chassis_rot, &g_yaw_ff_pid);
	yaw_motor->output += g_yaw_ff_pid.output;
	yaw_motor->output = (yaw_motor->output < - 20000) ? -20000 : (yaw_motor->output > 20000) ? 20000:yaw_motor->output;
#endif
#endif
}

void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {

	if (gimbal_ctrl_data.pitch > pitch_motor->angle_data.max_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.pitch < pitch_motor->angle_data.min_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.min_ang;
	}

	if (gimbal_ctrl_data.yaw > yaw_motor->angle_data.max_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.yaw < yaw_motor->angle_data.min_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.min_ang;
	}
	angle_pid(gimbal_ctrl_data.pitch, pitch_motor->angle_data.adj_ang,
			pitch_motor);
	angle_pid(gimbal_ctrl_data.yaw, yaw_motor->angle_data.adj_ang, yaw_motor);

	pitch_motor->output = pitch_motor->rpm_pid.output;
	yaw_motor->output = yaw_motor->rpm_pid.output;
}
