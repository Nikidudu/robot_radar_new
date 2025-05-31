/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"
#include "bsp_hall.h"

extern EventGroupHandle_t chassis_event_group;

extern chassis_control_t chassis_ctrl_data;

extern remote_cmd_t g_remote_cmd;
extern motor_data_t g_can_motors[24];
extern referee_limit_t g_referee_limiters;
extern ref_game_robot_data_t ref_robot_data;
extern uint32_t ref_power_data_txno;
extern speed_shift_t gear_speed;
float g_chassis_yaw = 0;
int32_t chassis_rpm = MAX_SPEED;
uint8_t g_gimbal_state = 0;
extern uint8_t hall_state;
extern int g_spinspin_mode;

extern uint8_t charging_state;

uint8_t zero_start = 0;
uint32_t zeroing_start_time = 0;
int16_t current_rpm;

float motor_yaw_mult[4];

extern QueueHandle_t telem_motor_queue;
extern int supercap_dash;

void movement_control_task(void *argument) {
	TickType_t start_time;
	//initialise in an array so it's possible to for-loop it later
	motor_yaw_mult[0] = FR_YAW_MULT;
	motor_yaw_mult[1] = FL_YAW_MULT;
	motor_yaw_mult[2] = BL_YAW_MULT;
	motor_yaw_mult[3] = BR_YAW_MULT;

#ifdef HALL_ZERO
#endif
	while (1) {

#ifndef CHASSIS_MCU

		EventBits_t motor_bits;
		//wait for all motors to have updated data before PID is allowed to run
		motor_bits = xEventGroupWaitBits(chassis_event_group, 0b1111, pdTRUE,
		pdTRUE,
		portMAX_DELAY);
		if (motor_bits == 0b1111) {
			status_led(3, on_led);
			start_time = xTaskGetTickCount();
			if (chassis_ctrl_data.enabled) {

#ifdef HALL_ZERO
			if (check_yaw()){ g_gimbal_state = 1; }

			if (g_gimbal_state){
				if (hall_state == HALL_ON){
				yaw_zeroing(g_can_motors + FR_MOTOR_ID - 1,
						g_can_motors + FL_MOTOR_ID - 1,
						g_can_motors + BL_MOTOR_ID - 1,
						g_can_motors + BR_MOTOR_ID - 1);
				} else {
#endif
					chassis_motion_control(g_can_motors + FR_MOTOR_ID - 1,
							g_can_motors + FL_MOTOR_ID - 1,
							g_can_motors + BL_MOTOR_ID - 1,
							g_can_motors + BR_MOTOR_ID - 1);

#ifdef HALL_ZERO
				}
			}
#endif

			} else {
				g_can_motors[FR_MOTOR_ID - 1].output = 0;
				g_can_motors[FL_MOTOR_ID - 1].output = 0;
				g_can_motors[BL_MOTOR_ID - 1].output = 0;
				g_can_motors[BR_MOTOR_ID - 1].output = 0;

			}
#else
		chassis_MCU_send_CAN();
#endif
			status_led(3, off_led);
		} else {
			//motor timed out
			g_can_motors[FR_MOTOR_ID - 1].output = 0;
			g_can_motors[FL_MOTOR_ID - 1].output = 0;
			g_can_motors[BL_MOTOR_ID - 1].output = 0;
			g_can_motors[BR_MOTOR_ID - 1].output = 0;
		}
		//clear bits if it's not already cleared
		xEventGroupClearBits(chassis_event_group, 0b1111);
		//delays task for other tasks to run
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}
void chassis_MCU_send_CAN() {

}


float filtered_rpm_fr;
float filtered_rpm_fl;
float filtered_rpm_bl;
float filtered_rpm_br;
float vforwardrpm;
float vyaw;
float vhorizontal;

void chassis_motion_control(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr) {
	//get the angle between the gun and the chassis
	//so that movement is relative to gun, not chassis
	float rel_angle = g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
	float translation_rpm[4] = { 0, };
	float yaw_rpm[4] = { 0, };
	float total_power = 0;

	int32_t curr_avg_rpm = (abs(motorfr->raw_data.rpm)
			+ abs(motorfl->raw_data.rpm) + abs(motorbr->raw_data.rpm)
			+ abs(motorbl->raw_data.rpm)) / 4;

	uint32_t lvl_max_speed;
	uint32_t lvl_max_accel;

	switch (ref_robot_data.robot_level) {
			case 1: lvl_max_speed = LV1_MAX_SPEED;
					lvl_max_accel = LV1_MAX_ACCEL; break;

			case 2: lvl_max_speed = LV2_MAX_SPEED;
					lvl_max_accel = LV2_MAX_ACCEL; break;

			case 3: lvl_max_speed = LV3_MAX_SPEED;
					lvl_max_accel = LV3_MAX_ACCEL; break;

			case 4: lvl_max_speed = LV4_MAX_SPEED;
					lvl_max_accel = LV4_MAX_ACCEL; break;

			case 5: lvl_max_speed = LV5_MAX_SPEED;
					lvl_max_accel = LV5_MAX_ACCEL; break;

			case 6: lvl_max_speed = LV6_MAX_SPEED;
					lvl_max_accel = LV6_MAX_ACCEL; break;

			case 7: lvl_max_speed = LV7_MAX_SPEED;
					lvl_max_accel = LV7_MAX_ACCEL; break;

			case 8: lvl_max_speed = LV8_MAX_SPEED;
					lvl_max_accel = LV8_MAX_ACCEL; break;

			case 9: lvl_max_speed = LV9_MAX_SPEED;
					lvl_max_accel = LV9_MAX_ACCEL; break;

			case 10: lvl_max_speed = LV10_MAX_SPEED;
					 lvl_max_accel = LV10_MAX_ACCEL; break;

			default: lvl_max_speed = LV1_MAX_SPEED + (ref_robot_data.chassis_power_limit - 60) / 10 * 1500;
				     lvl_max_accel = LV1_MAX_ACCEL;
		}

	lvl_max_speed = (lvl_max_speed < MIN_SPEED) ? MIN_SPEED : lvl_max_speed;

	lvl_max_speed = (lvl_max_speed > MAX_SPEED) ? MAX_SPEED : lvl_max_speed; // Cap the max speed of motor

	chassis_rpm = lvl_max_speed;

	//rotate angle of the movement :)
	//MA1513/MA1508E is useful!!
	float act_forward = chassis_ctrl_data.forward * gear_speed.trans_mult;  //gear shifter multipliers
	float act_horizontal = chassis_ctrl_data.horizontal * gear_speed.trans_mult;
	float act_yaw = chassis_ctrl_data.yaw * gear_speed.spin_mult;


	float rel_forward = ((-act_horizontal * sin(-rel_angle))  //translation and rotation speed of chassis for chassis yaw angle relative to gimbal
			+ (act_forward * cos(-rel_angle)));
	float rel_horizontal = ((-act_horizontal * cos(-rel_angle))
			+ (act_forward * -sin(-rel_angle)));
	float rel_yaw = act_yaw;

	translation_rpm[0] = ((rel_forward * FR_VY_MULT)   //calculate theoretical wheel rpm for chassis translation
			+ (rel_horizontal * FR_VX_MULT));
	translation_rpm[1] = ((rel_forward * FL_VY_MULT)
			+ (rel_horizontal * FL_VX_MULT));
	translation_rpm[2] = ((rel_forward * BL_VY_MULT)
			+ (rel_horizontal * BL_VX_MULT));
	translation_rpm[3] = ((rel_forward * BR_VY_MULT)
			+ (rel_horizontal * BR_VX_MULT));

	yaw_rpm[0] = rel_yaw * motor_yaw_mult[0] * CHASSIS_YAW_MAX_RPM;  //calculate theoretical wheel rpm for yaw
	yaw_rpm[1] = rel_yaw * motor_yaw_mult[1] * CHASSIS_YAW_MAX_RPM;
	yaw_rpm[2] = rel_yaw * motor_yaw_mult[2] * CHASSIS_YAW_MAX_RPM;
	yaw_rpm[3] = rel_yaw * motor_yaw_mult[3] * CHASSIS_YAW_MAX_RPM;

	float rpm_mult = 1;
	float rpm_sum = 0;
	for (uint8_t i = 0; i < 4; i++) {
		float temp_add = fabs(yaw_rpm[i] + translation_rpm[i]);
		rpm_sum = rpm_sum + temp_add;  //get sum of wheel rpm
		if (temp_add > rpm_mult){	   //get highest wheel rpm
			rpm_mult = temp_add;
		}
	}


	int rpm1 = motorfr->raw_data.rpm;
	int rpm2 = motorfl->raw_data.rpm;
	int rpm3 = motorbr->raw_data.rpm;
	int rpm4 = motorbl->raw_data.rpm;

	int maxRPM = rpm1;

	if (rpm2 > maxRPM) maxRPM = rpm2;
	if (rpm3 > maxRPM) maxRPM = rpm3;
	if (rpm4 > maxRPM) maxRPM = rpm4;


	current_rpm = maxRPM;
	int16_t target_rpm = chassis_rpm;
	double dt = 0.005;
	uint32_t accel = lvl_max_accel; //50000 //Default Chassis_Accel_max is LV1_ACCEL_MAX

	if (target_rpm > current_rpm) {
		current_rpm += accel * dt;
		if (current_rpm > target_rpm) {
			current_rpm = target_rpm;
		}
	} else if (target_rpm < current_rpm) {
		current_rpm -= accel * dt;
		if (current_rpm < target_rpm) {
			current_rpm = target_rpm;
		}
	}

	current_rpm = (current_rpm > MAX_SPEED) ? MAX_SPEED : current_rpm; //Max speed check for motor protection

	// translation rpm will not be more than chassis_rpm
	int32_t avg_trans = 0;
	for (uint8_t j = 0; j < 4; j++) {
		if (g_spinspin_mode == 1) { // if spinning
			translation_rpm[j] = (translation_rpm[j]							// sum theoretical wheel rpm for translation and yaw
									+ yaw_rpm[j]) * current_rpm / (rpm_sum/4);  // for spinning modulate wheel rpm by dividing by average rpm
			avg_trans += fabs(translation_rpm[j]);
		} else {
			translation_rpm[j] = (translation_rpm[j]							// sum theoretical wheel rpm for translation and yaw
						+ yaw_rpm[j]) * current_rpm / rpm_mult;					// for no spinning modulate wheel rpm by dividing by highest rpm
			avg_trans += fabs(translation_rpm[j]);
		}
	}

	speed_pid(translation_rpm[0], motorfr->raw_data.rpm, &motorfr->rpm_pid);
	total_power += fabs(motorfr->rpm_pid.output);
	speed_pid(translation_rpm[1], motorfl->raw_data.rpm, &motorfl->rpm_pid);
	total_power += fabs(motorfl->rpm_pid.output);
	speed_pid(translation_rpm[2], motorbl->raw_data.rpm, &motorbl->rpm_pid);
	total_power += fabs(motorbl->rpm_pid.output);
	speed_pid(translation_rpm[3], motorbr->raw_data.rpm, &motorbr->rpm_pid);
	total_power += fabs(motorbr->rpm_pid.output);

	motorfr->output = motorfr->rpm_pid.output;
	motorfl->output = motorfl->rpm_pid.output;
	motorbl->output = motorbl->rpm_pid.output;
	motorbr->output = motorbr->rpm_pid.output;
}

#ifdef HALL_ZERO
void yaw_zeroing(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr){
	if (!zero_start && (g_remote_cmd.right_switch == ge_RSW_ALL_ON)) {
		zeroing_start_time = HAL_GetTick();
		zero_start = 1;
	}
	if (zero_start && (HAL_GetTick() - zeroing_start_time > HALL_TIMEOUT)){
		hall_int();
		g_can_motors[YAW_MOTOR_ID-1].angle_data.center_ang = 0;
	}
	float yaw_rpm[4];
	yaw_rpm[0] = ZERO_SPEED * motor_yaw_mult[0];
	yaw_rpm[1] = ZERO_SPEED * motor_yaw_mult[1];
	yaw_rpm[2] = ZERO_SPEED * motor_yaw_mult[2];
	yaw_rpm[3] = ZERO_SPEED * motor_yaw_mult[3];


	speed_pid(yaw_rpm[0], motorfr->raw_data.rpm, &motorfr->rpm_pid);
	speed_pid(yaw_rpm[1], motorfl->raw_data.rpm, &motorfl->rpm_pid);
	speed_pid(yaw_rpm[2], motorbl->raw_data.rpm, &motorbl->rpm_pid);
	speed_pid(yaw_rpm[3], motorbr->raw_data.rpm, &motorbr->rpm_pid);

	motorfr->output = motorfr->rpm_pid.output;
	motorfl->output = motorfl->rpm_pid.output;
	motorbl->output = motorbl->rpm_pid.output;
	motorbr->output = motorbr->rpm_pid.output;
}
#endif

