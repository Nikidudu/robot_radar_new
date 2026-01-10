/*
 * motor_config.c
 *
 *  Created on: 5 Jan 2022
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "gimbal_control_task.h"
#include "launcher_control_task.h"
#include "motor_control_task.h"
#include "can_msg_processor.h"
#include "bsp_lk_motor.h"
#include "chassis_can_message_task.h"

extern TaskHandle_t master_task_handle;
extern TaskHandle_t gimbal_control_task_handle;
extern TaskHandle_t chassis_can_message_task_handle;
extern TaskHandle_t launcher_control_task_handle;
extern TaskHandle_t motor_calib_task_handle;
extern TaskHandle_t telemetry_task_handle;
extern TaskHandle_t motor_control_task_handle;

extern EventGroupHandle_t gimbal_event_group;
extern EventGroupHandle_t chassis_event_group;
extern EventGroupHandle_t launcher_event_group;

extern gimbal_control_t gimbal_ctrl_data;

extern QueueHandle_t g_buzzing_task_msg;

dm_motor_t dm_pitch_motor;
dm_motor_t dm_yaw_motor;

void motor_calib_task(void *argument) {
	can_start(&hcan1, 0x00000000, 0x00000000);
	can_start(&hcan2, 0x00000000, 0x00000000);
	vTaskDelay(1000);
	//config_motors();

	//check motors
	//start motor control tasks after initialisation of motors
	//todo shift function to master task.c probably

	//xTaskCreate(motor_control_task, "motor_control_task", 512, (void*) 3,
	//		(UBaseType_t) 8, &motor_control_task_handle);

	if (chassis_event_group == NULL) {
		//error handler
	} else {
		xTaskCreate(chassis_can_message_task, "chassis_task",
		configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
				&chassis_can_message_task_handle);
	}


	if (launcher_event_group == NULL) {
		//error handler
	} else {
		xTaskCreate(launcher_control_task, "launcher_task",
		configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
				&launcher_control_task_handle);
	}

	if (gimbal_event_group == NULL) {
		//error handler implement next time!
	} else {
		xTaskCreate(gimbal_control_task, "gimbal_task",
		configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 7,
				&gimbal_control_task_handle);
	}
	while (1) {
		vTaskDelay(1000);
	}
}

uint8_t lk_set_pid(motor_data_t *motor, uint32_t timeout){
	uint32_t timeout_time = get_microseconds() + timeout;
	uint32_t curr_time = get_microseconds();
	while (curr_time < timeout_time){
		curr_time = get_microseconds();
		if (motor->last_time[0] != 0 && (curr_time - motor->last_time[0]) < 10000){
			lk_write_pid(motor->can, motor);
			return 1;
		} else {
			lk_read_pid(motor->can, motor);
			vTaskDelay(1);
		}
	}
	//motor timed out, cannot write pid
	return 0;
}

void set_motor_config(motor_data_t *motor) {
	//general config:
	switch (motor->motor_type) {
	case TYPE_M3508_ANGLE:
	case TYPE_M3508_STEPS:
	case TYPE_M3508:
		motor->angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
		motor->angle_pid.physical_max = M3508_MAX_RPM;
		motor->rpm_pid.physical_max = M3508_MAX_OUTPUT;
		motor->angle_data.min_ticks = -4096 * M3508_GEARBOX_RATIO;
		motor->angle_data.max_ticks = 4096 * M3508_GEARBOX_RATIO;
		motor->angle_data.tick_range = motor->angle_data.max_ticks
				- motor->angle_data.min_ticks;
		motor->angle_data.min_ang = -PI;
		motor->angle_data.max_ang = PI;
		motor->angle_data.max_raw_ticks = 4096;
		motor->angle_data.min_raw_ticks = -4096;
		motor->angle_data.raw_ticks_range = motor->angle_data.max_raw_ticks - motor->angle_data.min_raw_ticks;
		motor->angle_data.ang_range = motor->angle_data.max_ang - motor->angle_data.min_ang;
		break;

	case TYPE_M3508_NGEARBOX:
		motor->angle_data.gearbox_ratio = 1;
		motor->angle_pid.physical_max = M3508_MAX_RPM;
		motor->rpm_pid.physical_max = M3508_MAX_OUTPUT;
		motor->angle_data.min_ticks = -4096;
		motor->angle_data.max_ticks = 4096;
		motor->angle_data.tick_range = motor->angle_data.max_ticks
				- motor->angle_data.min_ticks;

		motor->angle_data.max_raw_ticks = 4096;
		motor->angle_data.min_raw_ticks = -4096;
		motor->angle_data.raw_ticks_range = motor->angle_data.max_raw_ticks - motor->angle_data.min_raw_ticks;
		motor->angle_data.min_ang = -PI;
		motor->angle_data.max_ang = PI;
		motor->angle_data.ang_range = motor->angle_data.max_ang
				- motor->angle_data.min_ang;
		break;

	case TYPE_GM6020:
		motor->angle_data.gearbox_ratio = 1;
		motor->angle_pid.physical_max = GM6020_MAX_RPM;
		motor->rpm_pid.physical_max = GM6020_MAX_OUTPUT;
		motor->angle_data.min_ticks = -4096;
		motor->angle_data.max_ticks = 4096;
		motor->angle_data.tick_range = motor->angle_data.max_ticks
				- motor->angle_data.min_ticks;

		motor->angle_data.max_raw_ticks = 4096;
		motor->angle_data.min_raw_ticks = -4096;
		motor->angle_data.raw_ticks_range = motor->angle_data.max_raw_ticks - motor->angle_data.min_raw_ticks;
		motor->angle_data.max_ang = PI;
		motor->angle_data.min_ang = -PI;
		motor->angle_data.ang_range = motor->angle_data.max_ang
				- motor->angle_data.min_ang;
		break;

	case TYPE_GM6020_720:
		motor->angle_data.gearbox_ratio = 1;
		motor->angle_pid.physical_max = GM6020_MAX_RPM;
		motor->rpm_pid.physical_max = GM6020_MAX_OUTPUT;
		motor->angle_data.min_ticks = -8192;	//-4096*2
		motor->angle_data.max_ticks = 8192;	//4096*2
		motor->angle_data.tick_range = motor->angle_data.max_ticks
				- motor->angle_data.min_ticks;

		motor->angle_data.max_raw_ticks = 4096;
		motor->angle_data.min_raw_ticks = -4096;
		motor->angle_data.raw_ticks_range = motor->angle_data.max_raw_ticks - motor->angle_data.min_raw_ticks;
		motor->angle_data.min_ang = -2 * PI;
		motor->angle_data.max_ang = 2 * PI;
		motor->angle_data.ang_range = motor->angle_data.max_ang
				- motor->angle_data.min_ang;
		break;

	case TYPE_M2006:
	case TYPE_M2006_STEPS:
	case TYPE_M2006_ANGLE:
		motor->angle_data.gearbox_ratio = M2006_GEARBOX_RATIO;
		motor->angle_pid.physical_max = M2006_MAX_RPM;
		motor->rpm_pid.physical_max = M2006_MAX_OUTPUT;
		motor->angle_data.min_ticks = -4096 * M2006_GEARBOX_RATIO;
		motor->angle_data.max_ticks = 4096 * M2006_GEARBOX_RATIO;
		motor->angle_data.tick_range = motor->angle_data.max_ticks
				- motor->angle_data.min_ticks;

		motor->angle_data.max_raw_ticks = 4096;
		motor->angle_data.min_raw_ticks = -4096;
		motor->angle_data.raw_ticks_range = motor->angle_data.max_raw_ticks - motor->angle_data.min_raw_ticks;
		motor->angle_data.min_ang = -PI;
		motor->angle_data.max_ang = PI;
		motor->angle_data.ang_range = motor->angle_data.max_ang
				- motor->angle_data.min_ang;
		break;

	case TYPE_LK_MG5010E_SPD:
	case TYPE_LK_MG5010E_ANG:
	case TYPE_LK_MG5010E_MULTI_ANG:
		motor->angle_data.gearbox_ratio = 10;
		motor->angle_pid.physical_max = 100;
		motor->rpm_pid.physical_max = LK_MG5010E_MAX_RPM;
		motor->angle_data.min_ticks = -180000;
		motor->angle_data.max_ticks = 180000;
		motor->angle_data.tick_range = motor->angle_data.max_ticks - motor->angle_data.min_ticks;
		motor->angle_data.max_raw_ticks = 180000;
		motor->angle_data.min_raw_ticks = -180000;
		motor->angle_data.raw_ticks_range = motor->angle_data.max_raw_ticks - motor->angle_data.min_raw_ticks;
		motor->angle_data.max_ang = PI;
		motor->angle_data.min_ang = -PI;
		motor->angle_data.ang_range = motor->angle_data.max_ang
				- motor->angle_data.min_ang;
//		map_lk_motor(motor->id, motor);
		lk_set_pid(motor, 500000);
		break;

	case TYPE_DM4310_DJI_MODE:
		motor->angle_data.gearbox_ratio = 1;
		motor->angle_pid.physical_max = DM_TO_DJI_MAX_RPM;
		motor->rpm_pid.physical_max = DM_TO_DJI_MAX_OUTPUT;
		motor->angle_data.min_ticks = -4096;
		motor->angle_data.max_ticks = 4096;
		motor->angle_data.tick_range = motor->angle_data.max_ticks
				- motor->angle_data.min_ticks;

		motor->angle_data.max_raw_ticks = 4096;
		motor->angle_data.min_raw_ticks = -4096;
		motor->angle_data.raw_ticks_range = motor->angle_data.max_raw_ticks - motor->angle_data.min_raw_ticks;
		motor->angle_data.max_ang = PI;
		motor->angle_data.min_ang = -PI;
		motor->angle_data.ang_range = motor->angle_data.max_ang
				- motor->angle_data.min_ang;
		break;

	default:
		break;
	// todo: add switch case for TYPE_DM4310_DJI_MODE
	}
	motor->angle_data.init = 0;
}

// Only checks if pitch and yaw are damiao motors (for now)
void dm_set_motor_config() {
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
	dm_set_pitch_motor();
#endif
#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
	dm_set_yaw_motor();
#endif
}

void dm_set_pitch_motor() {
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
	memset(&dm_pitch_motor, 0, sizeof(dm_pitch_motor));
	dm_pitch_motor.id = PITCH_MOTOR_ID;
	dm_pitch_motor.ctrl.mode = 0; // 0 - MIT, 1 - Position, 2 - Speed
	dm4310_enable(PITCH_MOTOR_CAN, &dm_pitch_motor);

	dm_pitch_motor.angle_pid.kp = DM_PITCH_KP;
	dm_pitch_motor.angle_pid.ki = DM_PITCH_KI;
	dm_pitch_motor.angle_pid.kd = DM_PITCH_KD;
	dm_pitch_motor.angle_pid.int_max = DM_PITCH_INT_MAX;
	dm_pitch_motor.angle_pid.max_out = DM_PITCH_MAX_OUT;

	dm_pitch_motor.cmd.kp_set = DM_PITCH_MIT_KP;
	dm_pitch_motor.cmd.kd_set = DM_PITCH_MIT_KD;
	dm_pitch_motor.cmd.pos_set = DM_PITCH_MIT_POS;
	dm_pitch_motor.cmd.vel_set = DM_PITCH_MIT_VEL;
	dm_pitch_motor.cmd.tor_set = DM_PITCH_MIT_TOR;

	dm_pitch_motor.angle_data.center_ang = PITCH_CENTER;
	dm_pitch_motor.angle_data.phy_max_ang = PITCH_MAX_ANG;
	dm_pitch_motor.angle_data.phy_min_ang = PITCH_MIN_ANG;
#endif
}

void dm_set_yaw_motor() {
#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
  	memset(&dm_yaw_motor, 0, sizeof(dm_yaw_motor));
  	dm_yaw_motor.id = YAW_MOTOR_ID;
  	dm_yaw_motor.ctrl.mode = 0; // 0 - MIT, 1 - Position, 2 - Speed
  	dm4310_enable(YAW_MOTOR_CAN_PTR, &dm_yaw_motor);

//    PID_Init(&gimbal_pid_yaw, DM_YAW_MIT_KP, DM_YAW_MIT_KI, DM_YAW_MIT_KD,
//    		DM_YAW_MIT_INT_MAX, DM_YAW_MIT_MAX_OUT);

    dm_yaw_motor.angle_pid.kp = DM_YAW_KP;
    dm_yaw_motor.angle_pid.ki = DM_YAW_KI;
    dm_yaw_motor.angle_pid.kd = DM_YAW_KD;
    dm_yaw_motor.angle_pid.int_max = DM_YAW_INT_MAX;
    dm_yaw_motor.angle_pid.max_out = DM_YAW_MAX_OUT;

    dm_yaw_motor.cmd.kp_set = DM_YAW_MIT_KP;
    dm_yaw_motor.cmd.kd_set = DM_YAW_MIT_KD;
    dm_yaw_motor.cmd.pos_set = DM_YAW_MIT_POS;
    dm_yaw_motor.cmd.vel_set = DM_YAW_MIT_VEL;
    dm_yaw_motor.cmd.tor_set = DM_YAW_MIT_TOR;

    dm_yaw_motor.angle_data.center_ang = YAW_CENTER;
    dm_yaw_motor.angle_data.phy_max_ang = YAW_MAX_ANG;
    dm_yaw_motor.angle_data.phy_min_ang = YAW_MIN_ANG;
#endif
}

// fills 8-byte data packet with motor output in slot corresponding to motor id
void CAN_set_motor_output(CAN_TxHeaderTypeDef *CAN_tx_message, uint8_t *data, uint8_t motor_id, uint8_t motor_type, int16_t output) {
	CAN_tx_message->IDE = CAN_ID_STD;
	CAN_tx_message->RTR = CAN_RTR_DATA;
	CAN_tx_message->DLC = 0x08;

	if (motor_id > 4) {
		switch (motor_type) {
		case TYPE_M2006:
		case TYPE_M2006_STEPS:
		case TYPE_M2006_ANGLE:
			CAN_tx_message->StdId = CAN_2006_5_TO_8_ID;
			break;
		case TYPE_M3508:
		case TYPE_M3508_NGEARBOX:
		case TYPE_M3508_STEPS:
		case TYPE_M3508_ANGLE:
			CAN_tx_message->StdId = CAN_3508_5_TO_8_ID;
			break;
		case TYPE_GM6020:
		case TYPE_GM6020_720:
			CAN_tx_message->StdId = CAN_6020_5_TO_8_ID;
			break;
		case TYPE_DM4310_DJI_MODE:
			CAN_tx_message->StdId = CAN_DM_5_TO_8_ID;
			break;
		}
	} else {
		switch (motor_type) {
		case TYPE_M2006:
		case TYPE_M2006_STEPS:
		case TYPE_M2006_ANGLE:
			CAN_tx_message->StdId = CAN_2006_1_TO_4_ID;
			break;
		case TYPE_M3508:
		case TYPE_M3508_NGEARBOX:
		case TYPE_M3508_STEPS:
		case TYPE_M3508_ANGLE:
			CAN_tx_message->StdId = CAN_3508_1_TO_4_ID;
			break;
		case TYPE_GM6020:
		case TYPE_GM6020_720:
			CAN_tx_message->StdId = CAN_6020_1_TO_4_ID;
			break;
		case TYPE_DM4310_DJI_MODE:
			CAN_tx_message->StdId = CAN_DM_1_TO_4_ID;
			break;
		}
	}

	if (motor_id > 4) motor_id -= 4;

    uint8_t idx = (motor_id - 1) * 2;
    if (idx < 8) {
        data[idx]     = (output >> 8) & 0xFF;
        data[idx + 1] = output & 0xFF;
    }
}

void config_motors() {
//	for (uint8_t i = 0; i < 24; i++) {
//		//reset all the values to 0
//		g_can_motors[i].motor_type = 0;
//		g_can_motors[i].rpm_pid.output = 0;
//		g_can_motors[i].rpm_pid.integral = 0;
//		g_can_motors[i].angle_pid.output = 0;
//		g_can_motors[i].angle_pid.integral = 0;
//		g_can_motors[i].angle_data.ticks = 0;
//	}
//
//	//initialise motor data
//	uint8_t motor_id;
//
//#if defined(PITCH_MOTOR_ID) && PITCH_MOTOR_TYPE != TYPE_DM4310_MIT
//	g_pitch_motor.motor_type = PITCH_MOTOR_TYPE;
//	g_pitch_motor.id = PITCH_MOTOR_ID;
//	g_pitch_motor.angle_data.center_ang = PITCH_CENTER;
//	g_pitch_motor.angle_data.wheel_circ = 0;
//	g_pitch_motor.angle_pid.kp = PITCH_ANGLE_KP;
//	g_pitch_motor.angle_pid.ki = PITCH_ANGLE_KI;
//	g_pitch_motor.angle_pid.kd = PITCH_ANGLE_KD;
//	g_pitch_motor.angle_pid.int_max = PITCH_ANGLE_INT_MAX;
//	g_pitch_motor.angle_pid.max_out = PITCH_MAX_RPM;
//	g_pitch_motor.rpm_pid.kp = PITCHRPM_KP;
//	g_pitch_motor.rpm_pid.ki = PITCHRPM_KI;
//	g_pitch_motor.rpm_pid.kd = PITCHRPM_KD;
//	g_pitch_motor.rpm_pid.int_max = PITCHRPM_INT_MAX;
//	g_pitch_motor.rpm_pid.max_out = PITCH_MAX_CURRENT;
//	g_pitch_motor.angle_data.phy_max_ang = PITCH_MAX_ANG;
//	g_pitch_motor.angle_data.phy_min_ang = PITCH_MIN_ANG;
//	g_pitch_motor.can = PITCH_MOTOR_CAN_PTR;
//	set_motor_config(&g_pitch_motor);
//#else
//	dm_set_motor_config();
//#endif
//
//#if defined(YAW_MOTOR_ID) && (YAW_MOTOR_TYPE != TYPE_DM4310_MIT)
//	motor_id = YAW_MOTOR_ID - 1;
//	g_can_motors[motor_id].id = YAW_MOTOR_ID;
//	g_can_motors[motor_id].can = YAW_MOTOR_CAN_PTR;
//	g_can_motors[motor_id].angle_data.center_ang = YAW_CENTER;
//	g_can_motors[motor_id].angle_data.phy_max_ang = YAW_MAX_ANG;
//	g_can_motors[motor_id].angle_data.phy_min_ang = YAW_MIN_ANG; //angle before it overflows
//	g_can_motors[motor_id].angle_data.wheel_circ = 0;
//	g_can_motors[motor_id].angle_pid.kp = YAW_ANGLE_KP;
//	g_can_motors[motor_id].angle_pid.ki = YAW_ANGLE_KI;
//	g_can_motors[motor_id].angle_pid.kd = YAW_ANGLE_KD;
//	g_can_motors[motor_id].angle_pid.int_max = YAW_ANGLE_INT_MAX;
//	g_can_motors[motor_id].angle_pid.max_out = YAW_MAX_RPM;
//	g_can_motors[motor_id].rpm_pid.kp = YAWRPM_KP;
//	g_can_motors[motor_id].rpm_pid.ki = YAWRPM_KI;
//	g_can_motors[motor_id].rpm_pid.kd = YAWRPM_KD;
//	g_can_motors[motor_id].rpm_pid.int_max = YAWRPM_INT_MAX;
//	g_can_motors[motor_id].rpm_pid.max_out = YAW_MAX_CURRENT;
//	//need to change below for dm
//
//#ifndef YAW_M3508
//	g_can_motors[motor_id].motor_type = TYPE_GM6020;
//#else
//	g_can_motors[motor_id].motor_type = TYPE_M3508_ANGLE;
//#endif
//
//	set_motor_config(&g_can_motors[motor_id]);
//
//#ifdef YAW_BELT
//	g_can_motors[motor_id].angle_data.gearbox_ratio = g_can_motors[motor_id].angle_data.gearbox_ratio * YAW_BELT_GEAR_RATIO;
//	g_can_motors[motor_id].angle_data.min_ticks = -4096 * g_can_motors[motor_id].angle_data.gearbox_ratio;
//	g_can_motors[motor_id].angle_data.max_ticks = 4096 * g_can_motors[motor_id].angle_data.gearbox_ratio;
//	g_can_motors[motor_id].angle_data.tick_range = g_can_motors[motor_id].angle_data.max_ticks
//			- g_can_motors[motor_id].angle_data.min_ticks;
//#endif
//#else
//	dm_set_motor_config();
//#endif
}
