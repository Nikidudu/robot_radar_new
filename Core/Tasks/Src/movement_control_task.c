/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

/* Private includes ----------------------------------------------------------*/
#include "board_lib.h"
#include "movement_control_task.h"
#include "motor_control.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// For chassis kinematics calculations
float motor_yaw_mult[4];
static float lvl_max_speed;
static float lvl_max_accel;
static float lvl_max_spin;
static float spin_accel = SPIN_ACCELERATION;
float act_forward = 0.0f;
float act_horizontal = 0.0f;
float act_yaw = 0.0f;

motor_data_t chassis_wheel[4];

/* From other tasks (extern) */
// input variables
extern chassis_control_t chassis_ctrl_data;
extern uint8_t g_safety_toggle;
extern remote_cmd_t g_remote_cmd;
// yaw motor data
extern motor_data_t yaw_motor;
// referee system data
extern ref_game_robot_data_t ref_robot_data;
extern uint32_t ref_power_data_txno;
// supercap data
extern uint8_t charging_state;

/* Private function prototypes -----------------------------------------------*/
void chassis_init();
void send_current_to_motor();
void chassis_motion_control();
void level_config(float *lvl_max_speed, float *lvl_max_accel,
		float *lvl_max_spin);
float rpm_ramp(float target_value, float current_value, float *lvl_max_accel);

/* Private user code ---------------------------------------------------------*/

void movement_control_task(void *argument) {
	TickType_t start_time;
	chassis_init();

	while (1) {
//		todo: add remote/keyboard comms here, thus removing need for control_input_task.c
//		chassis_data_update();

		status_led(3, on_led);
		start_time = xTaskGetTickCount();
		if (chassis_ctrl_data.enabled) {
			chassis_motion_control(&chassis_wheel[FR_MOTOR_ID],
					&chassis_wheel[FL_MOTOR_ID], &chassis_wheel[BL_MOTOR_ID],
					&chassis_wheel[BR_MOTOR_ID]);
		} else {
			chassis_wheel[FR_MOTOR_ID].output = 0;
			chassis_wheel[FL_MOTOR_ID].output = 0;
			chassis_wheel[BL_MOTOR_ID].output = 0;
			chassis_wheel[BR_MOTOR_ID].output = 0;

		}

		status_led(3, off_led);

		send_chassis_current_to_motor();

		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}

void chassis_init() {
	// 1--0
	// 2--3

	//initialise in an array so it's possible to for-loop it later
	motor_yaw_mult[FR_MOTOR_ID] = FR_YAW_MULT;
	motor_yaw_mult[FL_MOTOR_ID] = FL_YAW_MULT;
	motor_yaw_mult[BL_MOTOR_ID] = BL_YAW_MULT;
	motor_yaw_mult[BR_MOTOR_ID] = BR_YAW_MULT;

	for (size_t i = 0; i < sizeof(chassis_wheel) / sizeof(chassis_wheel[0]);
			i++) {
		chassis_wheel[i].motor_type = TYPE_M3508;
//		chassis_wheel[i].id = CAN_3508_ALL_ID + i;
		chassis_wheel[i].can = CHASSIS_MOTOR_CAN;

		chassis_wheel[i].rpm_pid.kp = CHASSIS_KP;
		chassis_wheel[i].rpm_pid.ki = CHASSIS_KI;
		chassis_wheel[i].rpm_pid.kd = CHASSIS_KD;
		chassis_wheel[i].rpm_pid.int_max = CHASSIS_INT_MAX;
		chassis_wheel[i].rpm_pid.max_out = CHASSIS_MAX_CURRENT;
		chassis_wheel[i].rpm_pid.physical_max = M3508_MAX_OUTPUT;

		chassis_wheel[i].angle_pid.kp = 0;
		chassis_wheel[i].angle_pid.ki = 0;
		chassis_wheel[i].angle_pid.kd = 0;
		chassis_wheel[i].angle_pid.int_max = 0;
		chassis_wheel[i].angle_pid.max_out = 0;
		chassis_wheel[i].angle_pid.physical_max = M3508_MAX_RPM;

		chassis_wheel[i].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
		chassis_wheel[i].angle_data.min_ticks = -4096 * M3508_GEARBOX_RATIO;
		chassis_wheel[i].angle_data.max_ticks = 4096 * M3508_GEARBOX_RATIO;
		chassis_wheel[i].angle_data.tick_range =
				chassis_wheel[i].angle_data.max_ticks
						- chassis_wheel[i].angle_data.min_ticks;
		chassis_wheel[i].angle_data.min_ang = -PI;
		chassis_wheel[i].angle_data.max_ang = PI;
		chassis_wheel[i].angle_data.max_raw_ticks = 4096;
		chassis_wheel[i].angle_data.min_raw_ticks = -4096;
		chassis_wheel[i].angle_data.raw_ticks_range =
				chassis_wheel[i].angle_data.max_raw_ticks
						- chassis_wheel[i].angle_data.min_raw_ticks;
		chassis_wheel[i].angle_data.ang_range =
				chassis_wheel[i].angle_data.max_ang
						- chassis_wheel[i].angle_data.min_ang;
	}
}

void send_chassis_current_to_motor() {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box[3];
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;

	CAN_tx_message.StdId = 0x200; // CAN_3508_1_TO_4_ID

	if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
		CAN_send_data[0] = 0;
		CAN_send_data[1] = 0;
		CAN_send_data[2] = 0;
		CAN_send_data[3] = 0;
		CAN_send_data[4] = 0;
		CAN_send_data[5] = 0;
		CAN_send_data[6] = 0;
		CAN_send_data[7] = 0;
	} else {
		CAN_send_data[0] = (chassis_wheel[0].output >> 8) & 0xFF;
		CAN_send_data[1] = (chassis_wheel[0].output) & 0xFF;
		CAN_send_data[2] = (chassis_wheel[1].output >> 8) & 0xFF;
		CAN_send_data[3] = (chassis_wheel[1].output) & 0xFF;
		CAN_send_data[4] = (chassis_wheel[2].output >> 8) & 0xFF;
		CAN_send_data[5] = (chassis_wheel[2].output) & 0xFF;
		CAN_send_data[6] = (chassis_wheel[3].output >> 8) & 0xFF;
		CAN_send_data[7] = (chassis_wheel[3].output) & 0xFF;

	}

	HAL_CAN_AddTxMessage(CHASSIS_MOTOR_CAN, &CAN_tx_message, CAN_send_data,
			send_mail_box);
}

void chassis_motion_control(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr) {
	//get the angle between the gun and the chassis
	//so that movement is relative to gun, not chassis
	float rel_angle = yaw_motor.angle_data.adj_ang;
	float translation_rpm[4] = { 0, };
	float yaw_rpm[4] = { 0, };

	// Setting translational and rotational speed and acceleration base on robot level
	level_config(&lvl_max_speed, &lvl_max_accel, &lvl_max_spin);

	// Sets maximum wheel RPM (used to cap output)
	float max_rpm = M3508_MAX_RPM;

	//rotate angle of the movement :)
	//MA1513/MA1508E is useful!!
	float speed_limit = lvl_max_speed;
	float spin_limit = lvl_max_spin;

	// Increase speed when spinspin mode is deactivated
	if (chassis_ctrl_data.g_spinspin_mode == 0) {
		speed_limit += CHASSIS_SPEED_BOOST;
	}

	//Clamp the values between -limit to limit
	float limit_forward = fmaxf(-speed_limit,
			fminf(chassis_ctrl_data.forward, speed_limit));
	float limit_horizontal = fmaxf(-speed_limit,
			fminf(chassis_ctrl_data.horizontal, speed_limit));
	float limit_yaw = fmaxf(-spin_limit,
			fminf(chassis_ctrl_data.yaw, spin_limit));

	// Smooths speed changes over time using acceleration constraints
	act_forward = rpm_ramp(limit_forward, act_forward, &lvl_max_accel); //gear shifter multipliers
	act_horizontal = rpm_ramp(limit_horizontal, act_horizontal, &lvl_max_accel);
	act_yaw = rpm_ramp(limit_yaw, act_yaw, &spin_accel);

	// translation and rotation speed of chassis for chassis yaw angle relative to gimbal
	float rel_forward = ((-act_horizontal * sin(-rel_angle))
			+ (act_forward * cos(-rel_angle)));
	float rel_horizontal = ((-act_horizontal * cos(-rel_angle))
			+ (act_forward * -sin(-rel_angle)));
	float rel_yaw = act_yaw;

	// calculate theoretical wheel rpm for chassis translation
	translation_rpm[0] = ((rel_forward * FR_VY_MULT)
			+ (rel_horizontal * FR_VX_MULT));
	translation_rpm[1] = ((rel_forward * FL_VY_MULT)
			+ (rel_horizontal * FL_VX_MULT));
	translation_rpm[2] = ((rel_forward * BL_VY_MULT)
			+ (rel_horizontal * BL_VX_MULT));
	translation_rpm[3] = ((rel_forward * BR_VY_MULT)
			+ (rel_horizontal * BR_VX_MULT));

	yaw_rpm[FR_MOTOR_ID] = rel_yaw * motor_yaw_mult[FR_MOTOR_ID];
	yaw_rpm[FL_MOTOR_ID] = rel_yaw * motor_yaw_mult[FL_MOTOR_ID];
	yaw_rpm[BR_MOTOR_ID] = rel_yaw * motor_yaw_mult[BR_MOTOR_ID];
	yaw_rpm[BL_MOTOR_ID] = rel_yaw * motor_yaw_mult[BL_MOTOR_ID];

	float rpm_mult = 1;
	float rpm_sum = 0;
	for (uint8_t i = 0; i < 4; i++) {
		float temp_add = fabs(yaw_rpm[i] + translation_rpm[i]);
		rpm_sum = rpm_sum + temp_add; // total combined magnitude of all wheels' RPMs
		if (temp_add > rpm_mult) {	   // the maximum RPM among the four wheels
			rpm_mult = temp_add;
		}
	}

	// ensures that individual rpm will not be more than max_rpm
	for (uint8_t j = 0; j < 4; j++) {
		translation_rpm[j] = (translation_rpm[j]// sum theoretical wheel rpm for translation and yaw
		+ yaw_rpm[j]) * max_rpm / rpm_mult;	// for no spinning modulate wheel rpm by dividing by highest rpm
	}

	// todo: maybe better to change the values of PID here instead of center_yaw()?
	speed_pid(translation_rpm[0], motorfr->raw_data.rpm, &motorfr->rpm_pid);
	speed_pid(translation_rpm[1], motorfl->raw_data.rpm, &motorfl->rpm_pid);
	speed_pid(translation_rpm[2], motorbl->raw_data.rpm, &motorbl->rpm_pid);
	speed_pid(translation_rpm[3], motorbr->raw_data.rpm, &motorbr->rpm_pid);

	motorfr->output = motorfr->rpm_pid.output;
	motorfl->output = motorfl->rpm_pid.output;
	motorbl->output = motorbl->rpm_pid.output;
	motorbr->output = motorbr->rpm_pid.output;
}

void level_config(float *lvl_max_speed, float *lvl_max_accel,
		float *lvl_max_spin) {
#ifdef LVL_TUNING
	//	static uint8_t prev_robot_level = -1;

	//	// Hopefully with this, we can adjust pid values without it being overwritten all the time
	//	if (prev_robot_level == ref_robot_data.robot_level) return;
	//	prev_robot_level = ref_robot_data.robot_level;
	uint8_t curr_level = ref_robot_data.robot_level;

	if (chassis_ctrl_data.supercap_dash && chassis_ctrl_data.supercap_enabled) {
		curr_level += 10;
	}

	switch (curr_level) {
	case 1:
		*lvl_max_speed = LV1_MAX_SPEED;
		*lvl_max_accel = LV1_MAX_ACCEL;
		*lvl_max_spin = LV1_CHASSIS_YAW_MAX_RPM;
		break;

	case 2:
		*lvl_max_speed = LV2_MAX_SPEED;
		*lvl_max_accel = LV2_MAX_ACCEL;
		*lvl_max_spin = LV2_CHASSIS_YAW_MAX_RPM;
		break;

	case 3:
		*lvl_max_speed = LV3_MAX_SPEED;
		*lvl_max_accel = LV3_MAX_ACCEL;
		*lvl_max_spin = LV3_CHASSIS_YAW_MAX_RPM;
		break;

	case 4:
		*lvl_max_speed = LV4_MAX_SPEED;
		*lvl_max_accel = LV4_MAX_ACCEL;
		*lvl_max_spin = LV4_CHASSIS_YAW_MAX_RPM;
		break;

	case 5:
		*lvl_max_speed = LV5_MAX_SPEED;
		*lvl_max_accel = LV5_MAX_ACCEL;
		*lvl_max_spin = LV5_CHASSIS_YAW_MAX_RPM;
		break;

	case 6:
		*lvl_max_speed = LV6_MAX_SPEED;
		*lvl_max_accel = LV6_MAX_ACCEL;
		*lvl_max_spin = LV6_CHASSIS_YAW_MAX_RPM;
		break;

	case 7:
		*lvl_max_speed = LV7_MAX_SPEED;
		*lvl_max_accel = LV7_MAX_ACCEL;
		*lvl_max_spin = LV7_CHASSIS_YAW_MAX_RPM;
		break;

	case 8:
		*lvl_max_speed = LV8_MAX_SPEED;
		*lvl_max_accel = LV8_MAX_ACCEL;
		*lvl_max_spin = LV8_CHASSIS_YAW_MAX_RPM;
		break;

	case 9:
		*lvl_max_speed = LV9_MAX_SPEED;
		*lvl_max_accel = LV9_MAX_ACCEL;
		*lvl_max_spin = LV9_CHASSIS_YAW_MAX_RPM;
		break;

	case 10:
		*lvl_max_speed = LV10_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 11:
		*lvl_max_speed = LV11_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 12:
		*lvl_max_speed = LV12_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 13:
		*lvl_max_speed = LV13_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 14:
		*lvl_max_speed = LV14_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 15:
		*lvl_max_speed = LV15_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 16:
		*lvl_max_speed = LV16_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 17:
		*lvl_max_speed = LV17_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 18:
		*lvl_max_speed = LV18_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 19:
		*lvl_max_speed = LV19_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	case 20:
		*lvl_max_speed = LV20_MAX_SPEED;
		*lvl_max_accel = LV10_MAX_ACCEL;
		*lvl_max_spin = LV10_CHASSIS_YAW_MAX_RPM;
		break;

	default:
		*lvl_max_speed = LV1_MAX_SPEED;
		*lvl_max_accel = LV1_MAX_ACCEL;
		*lvl_max_spin = LV1_CHASSIS_YAW_MAX_RPM;
	}
#else

	*lvl_max_speed = MAX_SPEED;
	*lvl_max_accel = MAX_ACCEL;
	*lvl_max_spin  = CHASSIS_YAW_MAX_RPM;

#endif
	*lvl_max_speed = (*lvl_max_speed < 0) ? 0 : *lvl_max_speed; //Make sure is within 0 - 1 since it is a percentage
	*lvl_max_speed = (*lvl_max_speed > 1) ? 1 : *lvl_max_speed; // Cap the max speed of motor
}

float rpm_ramp(float target_value, float current_value, float *lvl_max_accel) {
	double dt = CHASSIS_DELAY / 1000.0; // Converting dt to minutes
	double accel = *lvl_max_accel; //Default Chassis_Accel_max is LV1_ACCEL_MAX

	double ramp_rate = accel * dt; //Calc ramp_rate from max_accel
	float delta = target_value - current_value;

	if (target_value == 0) {
		return 0; //Instantly stop the robot;
	} else if (fabs(delta) < ramp_rate) {
		return target_value;  // close enough, just snap to target
	} else {
		return current_value + (delta > 0 ? ramp_rate : -ramp_rate);
	}
}

