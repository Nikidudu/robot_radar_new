/*
 * gimbal_control_task.c
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

/* Private includes ----------------------------------------------------------*/
#include "board_lib.h"
#include "gimbal_control_task.h"
#include "INS_task.h"
#include "motor_control.h"
#include "imu_processing_task.h"
#include "control_input_task.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static float prev_pit;
static float prev_yaw;

motor_data_t yaw_motor;
motor_data_t pitch_motor;

#ifdef YAW_FEEDFORWARD
static pid_data_t g_yaw_ff_pid = {
			.kp = YAW_FF_SPD_KP,
			.ki = YAW_FF_SPD_KI,
			.kd = YAW_FF_SPD_KD,
			.int_max = YAW_FF_INT_MAX,
			.max_out = YAW_FF_MAX_OUTPUT
};
#endif

/* From other tasks (extern) */
// input variables
extern uint8_t control_mode;
extern uint8_t g_safety_toggle;

extern uint8_t gimbal_upper_bound;
extern uint8_t gimbal_lower_bound;

// dm motors (todo: to be removed)
extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

/* Private function prototypes -----------------------------------------------*/
void yaw_init();
void pitch_init();
void send_current_to_yaw_motor();
void send_current_to_pitch_motor();
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor);
void pitch_control(motor_data_t *pitch_motor);
void calculate_lead_screw_pitch(motor_data_t *pitch_motor);
void calculate_direct_pitch(motor_data_t *pitch_motor);
void calculate_linkage_pitch(motor_data_t *pitch_motor);
uint8_t limit_pitch(float *rel_pitch_angle, motor_data_t *pitch_motor);
void yaw_control(motor_data_t *yaw_motor);
void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor);

/* Private user code ---------------------------------------------------------*/

void gimbal_control_task(void *argument) {
	TickType_t start_time;
	pitch_init();
	yaw_init();

	while (1) {
#if PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_SPD || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_ANG || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_MULTI_ANG
		lk_read_motor_sang(&pitch_motor);
#endif
		start_time = xTaskGetTickCount();

		if (gimbal_ctrl_data.enabled) {
			if (gimbal_ctrl_data.imu_mode) {
				gimbal_control(&pitch_motor, &yaw_motor);
			} else {
				gimbal_angle_control(&pitch_motor, &yaw_motor);
			}
		} else {
			// set gimbal motor outputs to 0
#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
			dm_yaw_motor.angle_pid.output = 0;
            dm4310_clear_para(&dm_yaw_motor);
#else
			yaw_motor.output = 0;
#endif
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
			dm_pitch_motor.angle_pid.output = 0;
			dm4310_clear_para(&dm_pitch_motor);
#else
            pitch_motor.output = 0;
#endif
		}

		prev_yaw = imu_heading.yaw;
		status_led(2, off_led);

		send_current_to_pitch_motor();
		send_current_to_yaw_motor();

		vTaskDelayUntil(&start_time, GIMBAL_DELAY);
	}
	//should not run here
	osThreadTerminate(NULL);
}

void yaw_init() {
#if defined(YAW_MOTOR_ID) && (YAW_MOTOR_TYPE != TYPE_DM4310_MIT)
	yaw_motor.motor_type = YAW_MOTOR_TYPE;
	yaw_motor.can = YAW_MOTOR_CAN;

	yaw_motor.angle_data.center_ang = YAW_CENTER;
	yaw_motor.angle_data.phy_max_ang = YAW_MAX_ANG;
	yaw_motor.angle_data.phy_min_ang = YAW_MIN_ANG; //angle before it overflows
	yaw_motor.angle_data.wheel_circ = 0;

	yaw_motor.angle_pid.kp = YAW_ANGLE_KP;
	yaw_motor.angle_pid.ki = YAW_ANGLE_KI;
	yaw_motor.angle_pid.kd = YAW_ANGLE_KD;
	yaw_motor.angle_pid.int_max = YAW_ANGLE_INT_MAX;
	yaw_motor.angle_pid.max_out = YAW_MAX_RPM;

	yaw_motor.rpm_pid.kp = YAWRPM_KP;
	yaw_motor.rpm_pid.ki = YAWRPM_KI;
	yaw_motor.rpm_pid.kd = YAWRPM_KD;
	yaw_motor.rpm_pid.int_max = YAWRPM_INT_MAX;
	yaw_motor.rpm_pid.max_out = YAW_MAX_CURRENT;

	set_motor_config(&yaw_motor);

#ifdef YAW_BELT
	yaw_motor.angle_data.gearbox_ratio = yaw_motor.angle_data.gearbox_ratio * YAW_BELT_GEAR_RATIO;
	yaw_motor.angle_data.min_ticks = -4096 * yaw_motor.angle_data.gearbox_ratio;
	yaw_motor.angle_data.max_ticks = 4096 * yaw_motor.angle_data.gearbox_ratio;
	yaw_motor.angle_data.tick_range = yaw_motor.angle_data.max_ticks
			- yaw_motor.angle_data.min_ticks;
#endif

#elif YAW_MOTOR_TYPE == TYPE_DM4310_MIT
	dm_set_yaw_motor();
#endif
}

void pitch_init() {
#if defined(PITCH_MOTOR_ID) && (PITCH_MOTOR_TYPE != TYPE_DM4310_MIT)
	pitch_motor.motor_type = PITCH_MOTOR_TYPE;
	pitch_motor.can = PITCH_MOTOR_CAN;

	pitch_motor.angle_data.center_ang = PITCH_CENTER;
	pitch_motor.angle_data.wheel_circ = 0;
	pitch_motor.angle_data.phy_max_ang = PITCH_MAX_ANG;
	pitch_motor.angle_data.phy_min_ang = PITCH_MIN_ANG;

	pitch_motor.angle_pid.kp = PITCH_ANGLE_KP;
	pitch_motor.angle_pid.ki = PITCH_ANGLE_KI;
	pitch_motor.angle_pid.kd = PITCH_ANGLE_KD;
	pitch_motor.angle_pid.int_max = PITCH_ANGLE_INT_MAX;
	pitch_motor.angle_pid.max_out = PITCH_MAX_RPM;

	pitch_motor.rpm_pid.kp = PITCHRPM_KP;
	pitch_motor.rpm_pid.ki = PITCHRPM_KI;
	pitch_motor.rpm_pid.kd = PITCHRPM_KD;
	pitch_motor.rpm_pid.int_max = PITCHRPM_INT_MAX;
	pitch_motor.rpm_pid.max_out = PITCH_MAX_CURRENT;

	set_motor_config(&pitch_motor);

#elif PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
	dm_set_pitch_motor();
#endif

}

void send_current_to_yaw_motor() {
#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
		dm4310_ctrl_send(YAW_MOTOR_CAN, &dm_yaw_motor);

#else
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box[3];

	// Clear entire packet first
	memset(CAN_send_data, 0, 8);

	// fill data packet with yaw data
	if (!(g_safety_toggle || g_remote_cmd.sw == SW_SHUTDOWN)) {
	    CAN_set_motor_output(&CAN_tx_message, CAN_send_data, YAW_MOTOR_ID, yaw_motor.motor_type, yaw_motor.output);
	}

	HAL_CAN_AddTxMessage(YAW_MOTOR_CAN, &CAN_tx_message, CAN_send_data,
			send_mail_box);
#endif
}

void send_current_to_pitch_motor() {
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
	dm4310_ctrl_send(PITCH_MOTOR_CAN, &dm_pitch_motor);

#elif PITCH_MOTOR_TYPE == TYPE_DM4310_DJI_MODE
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box[3];

	// Clear entire packet first
	memset(CAN_send_data, 0, 8);

	// fill data packet with pitch data
	if (!(g_safety_toggle || g_remote_cmd.sw == SW_SHUTDOWN)) {
	    CAN_set_motor_output(&CAN_tx_message, CAN_send_data, PITCH_MOTOR_ID, yaw_motor.motor_type, pitch_motor.output);
	}

	HAL_CAN_AddTxMessage(PITCH_MOTOR_CAN, &CAN_tx_message, CAN_send_data,
			send_mail_box);
#endif
}

/**
 * This function controls the gimbals based on IMU reading
 * @param 	pitch_motor		Pointer to pitch motor struct
 * 			yaw_motor		Pointer to yaw motor struct
 */
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {
	//todo: add in roll compensation
	if (prev_yaw == imu_heading.yaw || prev_pit == imu_heading.pit) {
		return;
	}
	pitch_control(pitch_motor);
	yaw_control(yaw_motor);
}

void pitch_control(motor_data_t *pitch_motor) {

#if defined(PITCH_ARM)
    // 4-bar linkage mechanism (Tali)
    calculate_linkage_pitch(pitch_motor);

#elif defined(LEAD_SCREW)
    // Lead screw mechanism (new hero)
    calculate_lead_screw_pitch(pitch_motor);

#else
	// Direct drive (most robots pitch)
	calculate_direct_pitch(pitch_motor);
#endif
}

void calculate_lead_screw_pitch(motor_data_t *pitch_motor) {
	angle_pid(gimbal_ctrl_data.pitch, imu_heading.pit, pitch_motor, 0);

	pitch_motor->output = pitch_motor->rpm_pid.output;

	// upper and lower bound microswitch
	if (gimbal_upper_bound == 1 && pitch_motor->output > 0) {
		pitch_motor->output = 0;
	}
	if (gimbal_lower_bound == 1 && pitch_motor->output < 20) {
		pitch_motor->output = 0;
	}

	// code for servo for zoom-in lens
	// todo: add a keyboard button to toggle between positions
	double pulseWidth = (180.0 / 270.0) * 2000 + 500;
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulseWidth / 10);

	// code for self-adjusting VTM
	double angle = imu_heading.pit * 180.0 / PI;
	double pulseWidth2 = (angle + 90.0 / 270.0) * 2000 + 500;
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pulseWidth2 / 10);
}

void calculate_direct_pitch(motor_data_t *pitch_motor) {
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
	dm4310_set(&dm_pitch_motor);
//	float target_pitch = gimbal_ctrl_data.pitch;
	uint8_t pit_lim = 0;
//    if (target_pitch > dm_pitch_motor.angle_data.phy_max_ang) {
//        target_pitch = dm_pitch_motor.angle_data.phy_max_ang;
//    } else if(target_pitch < dm_pitch_motor.angle_data.phy_min_ang) {
//        target_pitch = dm_pitch_motor.angle_data.phy_min_ang;
//    }

#ifdef SENTRY
	// this is calculation for sentry pitch
	xSemaphoreTake(gimbal_ctrl_data.pitch_semaphore,portMAX_DELAY);
	float rel_pitch_angle = dm_pitch_motor.angle_data.adj_ang
				+ gimbal_ctrl_data.pitch - INS.Pitch;

	if (rel_pitch_angle > dm_pitch_motor.angle_data.phy_max_ang) {
		rel_pitch_angle = dm_pitch_motor.angle_data.phy_max_ang;
		pit_lim = 1;
	}
	if (rel_pitch_angle < dm_pitch_motor.angle_data.phy_min_ang) {
		rel_pitch_angle = dm_pitch_motor.angle_data.phy_min_ang;
		pit_lim = 1;
	}
	if (pit_lim == 1) {
		gimbal_ctrl_data.pitch = rel_pitch_angle + INS.Pitch
				- (dm_pitch_motor.angle_data.adj_ang);
	}

	yaw_pid(gimbal_ctrl_data.pitch, INS.Pitch, &dm_pitch_motor.angle_pid);
	xSemaphoreGive(gimbal_ctrl_data.pitch_semaphore);

	dm_pitch_motor.ctrl.tor_set = 0.4842f*imu_heading.pit - 2.3124f - dm_pitch_motor.angle_pid.output;
	dm_pitch_motor.ctrl.tor_set += 1.1;
#else
	xSemaphoreTake(gimbal_ctrl_data.pitch_semaphore, portMAX_DELAY);
	float rel_pitch_angle = gimbal_ctrl_data.pitch;

	if (rel_pitch_angle > dm_pitch_motor.angle_data.phy_max_ang) {
		rel_pitch_angle = dm_pitch_motor.angle_data.phy_max_ang;
		pit_lim = 1;
	}
	if (rel_pitch_angle < dm_pitch_motor.angle_data.phy_min_ang) {
		rel_pitch_angle = dm_pitch_motor.angle_data.phy_min_ang;
		pit_lim = 1;
	}
	if (pit_lim == 1) {
		gimbal_ctrl_data.pitch = rel_pitch_angle;
	}

	speed_pid(gimbal_ctrl_data.pitch, imu_heading.pit,
			&dm_pitch_motor.angle_pid);
	xSemaphoreGive(gimbal_ctrl_data.pitch_semaphore);
	dm_pitch_motor.ctrl.tor_set = dm_pitch_motor.angle_pid.output
			+ (-0.8841 * imu_heading.pit * imu_heading.pit
					- 0.8798 * imu_heading.pit + 1.3045);
#endif

#else

	uint8_t pit_lim = 0;
	xSemaphoreTake(gimbal_ctrl_data.pitch_semaphore,portMAX_DELAY);
	float rel_pitch_angle = pitch_motor->angle_data.adj_ang
				+ gimbal_ctrl_data.pitch - imu_heading.pit;

	pit_lim = limit_pitch(&rel_pitch_angle, pitch_motor);

//	if (pit_lim == 1) {
//		gimbal_ctrl_data.pitch = rel_pitch_angle + imu_heading.pit
//				- (pitch_motor->angle_data.adj_ang);
//	}

	yangle_pid(gimbal_ctrl_data.pitch,imu_heading.pit, pitch_motor,
			imu_heading.pit, &prev_pit,0);
	xSemaphoreGive(gimbal_ctrl_data.pitch_semaphore);

	int32_t temp_pit_output = pitch_motor->rpm_pid.output + PITCH_CONST;
	temp_pit_output = (temp_pit_output < -20000) ? -20000 :
						(temp_pit_output > 20000) ? 20000 : temp_pit_output;

	pitch_motor->output = temp_pit_output;
#endif

}

// Pitch calculation for robots with 4 arm linkage
void calculate_linkage_pitch(motor_data_t *pitch_motor) {
	uint8_t pit_lim = 0;
	xSemaphoreTake(gimbal_ctrl_data.pitch_semaphore, portMAX_DELAY);
	float rel_pitch_angle = gimbal_ctrl_data.pitch; // insert function where input desired gimbal pitch angle and output corresponding motor angle

	if (rel_pitch_angle > pitch_motor->angle_data.phy_max_ang) {
		rel_pitch_angle = pitch_motor->angle_data.phy_max_ang;
		pit_lim = 1;
	}
	if (rel_pitch_angle < pitch_motor->angle_data.phy_min_ang) {
		rel_pitch_angle = pitch_motor->angle_data.phy_min_ang;
		pit_lim = 1;
	}
	if (pit_lim == 1) {
		gimbal_ctrl_data.pitch = rel_pitch_angle;
	}
	xSemaphoreGive(gimbal_ctrl_data.pitch_semaphore);

#if PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_SPD || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_ANG || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_MULTI_ANG

	//lazy max
	//covnert radians back to degrees
	int32_t pitch_ang = rel_pitch_angle * 57320;
	pitch_motor.output = pitch_ang;
	if (HAL_CAN_GetTxMailboxesFreeLevel(pitch_motor.can) == 0){
//		HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
		vTaskDelay(1);
	}
	lk_motor_multturn_ang(&pitch_motor);
#else
	angle_pid(rel_pitch_angle, pitch_motor->angle_data.adj_ang, pitch_motor, 1);
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

void yaw_control(motor_data_t *yaw_motor) {
#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
	dm4310_set(&dm_yaw_motor);
	float turn_ang = imu_heading.yaw - prev_yaw;

	while (turn_ang > PI) {
		turn_ang -= 2 * PI;
	}
	while (turn_ang < -PI) {
		turn_ang += 2 * PI;
	}

	xSemaphoreTake(gimbal_ctrl_data.yaw_semaphore,portMAX_DELAY);
	gimbal_ctrl_data.delta_yaw -= turn_ang;

//	if (gimbal_ctrl_data.delta_yaw > 1.5 * PI) {
//		gimbal_ctrl_data.delta_yaw = 1.5 * PI;
//	}
//	if (gimbal_ctrl_data.delta_yaw < -1.5 * PI) {
//		gimbal_ctrl_data.delta_yaw = -1.5 * PI;
//	}

	 yaw_pid(0, -gimbal_ctrl_data.delta_yaw, &dm_yaw_motor.angle_pid);
	 xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);

	 dm_yaw_motor.ctrl.vel_set = dm_yaw_motor.angle_pid.output +
			chassis_ctrl_data.yaw * (YAW_SPINSPIN_CONSTANT/CHASSIS_SPINSPIN_MAX);
	 dm_yaw_motor.ctrl.tor_set = FEEDFORWARD_CONST * dm_yaw_motor.para.vel;
#else

	float turn_ang = imu_heading.yaw - prev_yaw;
	if (turn_ang > PI) {
		turn_ang -= 2 * PI;
	} else if (turn_ang < -PI) {
		turn_ang += 2 * PI;

	}
	xSemaphoreTake(gimbal_ctrl_data.yaw_semaphore, portMAX_DELAY);
	gimbal_ctrl_data.delta_yaw -= turn_ang;

	if (gimbal_ctrl_data.delta_yaw > PI) {
		gimbal_ctrl_data.delta_yaw = PI;
	}
	if (gimbal_ctrl_data.delta_yaw < -PI) {
		gimbal_ctrl_data.delta_yaw = -PI;
	}

	yangle_pid(gimbal_ctrl_data.delta_yaw, 0, yaw_motor, imu_heading.yaw,
			&prev_yaw, 0);
	xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);

//	yangle_pid(gimbal_ctrl_data.yaw, imu_heading.yaw, yaw_motor,
//			imu_heading.yaw, &prev_yaw);
//		oangle_pid(gimbal_ctrl_data.yaw, imu_heading.yaw, yaw_motor, g_chassis_rot);

//	float chassis_yaw_speed = g_chassis_yaw * FR_DIST * 2 * PI * chassis_rpm / 19.2;
	int32_t temp_output = yaw_motor->rpm_pid.output
			+ (chassis_ctrl_data.yaw * YAW_SPINSPIN_CONSTANT
					/ CHASSIS_SPINSPIN_MAX);
	temp_output = (temp_output > 20000) ? 20000 :
					(temp_output < -20000) ? -20000 : temp_output;
	yaw_motor->output = temp_output;

#ifdef YAW_FEEDFORWARD
//	speed_pid(yaw_motor->raw_data.rpm + yaw_motor->angle_pid.output, g_chassis_rot, &g_yaw_ff_pid);
//	yaw_motor->output += g_yaw_ff_pid.output;
//	yaw_motor->output = (yaw_motor->output < - 20000) ? -20000 : (yaw_motor->output > 20000) ? 20000:yaw_motor->output;
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
			pitch_motor, 1);
	angle_pid(gimbal_ctrl_data.yaw, yaw_motor->angle_data.adj_ang, yaw_motor,
			1);

	pitch_motor->output = pitch_motor->rpm_pid.output;
	yaw_motor->output = yaw_motor->rpm_pid.output;
}
