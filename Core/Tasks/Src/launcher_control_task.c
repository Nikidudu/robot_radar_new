/*
 * launcher_control_task.c
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

/* Private includes ----------------------------------------------------------*/
#include "board_lib.h"
#include "launcher_control_task.h"
#include "motor_control.h"
#include "control_input_task.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BULLET_17_HEAT 10
#define BULLET_42_HEAT 100

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
motor_data_t flywheel_motor[4]; // 4 friction wheels max
motor_data_t feeder_motor;

enum launcher_state_e flywheel_state;
enum feeder_state_e feeder_state;

/* From other tasks (extern) */
extern uint8_t projectile_loaded;
// referee system data
extern ref_game_state_t ref_game_state;
extern ref_game_robot_data_t ref_robot_data;
extern ref_robot_power_data_t ref_power_data;

extern uint32_t ref_power_data_txno;
static uint32_t prev_power_data_no = 0;

/* Private function prototypes -----------------------------------------------*/
void launcher_init();
void send_flywheel_current_to_motor();
void send_feeder_current_to_motor();
/* Private user code ---------------------------------------------------------*/

void launcher_control_task(void *argument) {
	TickType_t launcher_ctrl_time;
	launcher_init();

	while (1) {
		status_led(4, on_led);
		launcher_ctrl_time = xTaskGetTickCount();

		if (launcher_ctrl_data.enabled) {
			// Flywheel control
#ifdef ACTIVE_GUIDANCE
			guidance_flywheel(&flywheel_motor[LFRICTION_MOTOR_ID - 1],
					&flywheel_motor[RFRICTION_MOTOR_ID - 1],
					&flywheel_motor[BFRICTION_MOTOR_ID - 1]);
#else
			flywheel_control(&flywheel_motor[LFRICTION_MOTOR_ID - 1],
					&flywheel_motor[RFRICTION_MOTOR_ID - 1]);
#endif

			// Feeder control
#ifdef ACTIVE_GUIDANCE
			guidance_feeder(&flywheel_motor[LFRICTION_MOTOR_ID - 1],
					&flywheel_motor[RFRICTION_MOTOR_ID - 1],
					&flywheel_motor[BFRICTION_MOTOR_ID - 1],
					&flywheel_motor[GFRICTION_MOTOR_ID - 1],
					&feeder_motor);
#elif defined(ANGLE_FEEDER)
			launcher_angle_control(&flywheel_motor[LFRICTION_MOTOR_ID - 1],
					&flywheel_motor[RFRICTION_MOTOR_ID - 1],
					&feeder_motor);

#else
			launcher_control(&flywheel_motor[LFRICTION_MOTOR_ID - 1],
					&flywheel_motor[RFRICTION_MOTOR_ID - 1],
					&feeder_motor);
#endif

		} else {
			flywheel_motor[LFRICTION_MOTOR_ID - 1].output = 0;
			flywheel_motor[RFRICTION_MOTOR_ID - 1].output = 0;
			feeder_motor.output = 0;
#ifdef ACTIVE_GUIDANCE
			flywheel_motor[BFRICTION_MOTOR_ID - 1].output = 0;
			flywheel_motor[GFRICTION_MOTOR_ID - 1].output = 0;
#endif
		}
		status_led(4, off_led);

		send_flywheel_current_to_motor();
		send_feeder_current_to_motor();

		vTaskDelayUntil(&launcher_ctrl_time, LAUNCHER_DELAY);
	}

}

void launcher_init() {
#ifdef ANGLE_FEEDER
	feeder_motor.motor_type = TYPE_M3508_ANGLE;
#elif FEEDER_MOTOR_TYPE == TYPE_M3508
	feeder_motor.motor_type = TYPE_M3508;
	feeder_motor.can = LAUNCHER_MOTOR_CAN;

	feeder_motor.rpm_pid.kp = FEEDER_KP;
	feeder_motor.rpm_pid.ki = FEEDER_KI;
	feeder_motor.rpm_pid.kd = FEEDER_KD;
	feeder_motor.rpm_pid.int_max = FEEDER_MAX_INT;
	feeder_motor.rpm_pid.max_out = FEEDER_MAX_CURRENT;

	feeder_motor.angle_pid.kp = FEEDER_ANGLE_KP;
	feeder_motor.angle_pid.ki = FEEDER_ANGLE_KI;
	feeder_motor.angle_pid.kd = FEEDER_ANGLE_KD;
	feeder_motor.angle_pid.int_max = FEEDER_ANGLE_INT_MAX;
	feeder_motor.angle_pid.max_out = FEEDER_MAX_RPM;

	feeder_motor.angle_data.wheel_circ = 0;
	set_motor_config(&feeder_motor);

	int number_of_flywheels = 2; // LFRICTION + RFRICTION
#else
	feeder_motor.motor_type = TYPE_M2006;
	feeder_motor.can = LAUNCHER_MOTOR_CAN;

	feeder_motor.rpm_pid.kp = FEEDER_KP;
	feeder_motor.rpm_pid.ki = FEEDER_KI;
	feeder_motor.rpm_pid.kd = FEEDER_KD;
	feeder_motor.rpm_pid.int_max = FEEDER_MAX_INT;
	feeder_motor.rpm_pid.max_out = FEEDER_MAX_CURRENT;

	feeder_motor.angle_pid.kp = FEEDER_ANGLE_KP;
	feeder_motor.angle_pid.ki = FEEDER_ANGLE_KI;
	feeder_motor.angle_pid.kd = FEEDER_ANGLE_KD;
	feeder_motor.angle_pid.int_max = FEEDER_ANGLE_INT_MAX;
	feeder_motor.angle_pid.max_out = FEEDER_MAX_RPM;

	feeder_motor.angle_data.wheel_circ = 0;
	set_motor_config(&feeder_motor);

	int number_of_flywheels = 2; // LFRICTION + RFRICTION
#endif

#ifdef ACTIVE_GUIDANCE
	microswitch_int();
	number_of_flywheels = 4; // + BFRICTION + GFRICTION
#endif

	for (size_t i = 0; i < number_of_flywheels; i++) {
			flywheel_motor[i].motor_type = TYPE_M3508_NGEARBOX;
			flywheel_motor[i].can = LAUNCHER_MOTOR_CAN;

			flywheel_motor[i].rpm_pid.kp = FRICTION_KP;
			flywheel_motor[i].rpm_pid.ki = FRICTION_KI;
			flywheel_motor[i].rpm_pid.kd = FRICTION_KD;
			flywheel_motor[i].rpm_pid.int_max = FRICTION_MAX_INT;
			flywheel_motor[i].rpm_pid.max_out = FRICTION_MAX_CURRENT;
			flywheel_motor[i].rpm_pid.physical_max = M3508_MAX_OUTPUT;

			flywheel_motor[i].angle_pid.kp = 0;
			flywheel_motor[i].angle_pid.ki = 0;
			flywheel_motor[i].angle_pid.kd = 0;
			flywheel_motor[i].angle_pid.int_max = 0;
			flywheel_motor[i].angle_pid.max_out = 0;
			flywheel_motor[i].angle_pid.physical_max = M3508_MAX_RPM;

			flywheel_motor[i].angle_data.gearbox_ratio = 1;
			flywheel_motor[i].angle_pid.physical_max = M3508_MAX_RPM;
			flywheel_motor[i].angle_data.min_ticks = -4096;
			flywheel_motor[i].angle_data.max_ticks = 4096;
			flywheel_motor[i].angle_data.tick_range =
					flywheel_motor[i].angle_data.max_ticks
							- flywheel_motor[i].angle_data.min_ticks;
			flywheel_motor[i].angle_data.min_ang = -PI;
			flywheel_motor[i].angle_data.max_ang = PI;
			flywheel_motor[i].angle_data.ang_range =
					flywheel_motor[i].angle_data.max_ang
							- flywheel_motor[i].angle_data.min_ang;
			flywheel_motor[i].angle_data.max_raw_ticks = 4096;
			flywheel_motor[i].angle_data.min_raw_ticks = -4096;
			flywheel_motor[i].angle_data.raw_ticks_range =
					flywheel_motor[i].angle_data.max_raw_ticks
							- flywheel_motor[i].angle_data.min_raw_ticks;
	}
}


void send_flywheel_current_to_motor() {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box[3];
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;

	// send to friction wheels
	CAN_tx_message.StdId = CAN_3508_1_TO_4_ID;
	if (g_safety_toggle || g_remote_cmd.sw == SW_SHUTDOWN){
		CAN_send_data[0] = 0;
		CAN_send_data[1] = 0;
		CAN_send_data[2] = 0;
		CAN_send_data[3] = 0;
		CAN_send_data[4] = 0;
		CAN_send_data[5] = 0;
		CAN_send_data[6] = 0;
		CAN_send_data[7] = 0;
	} else {
		CAN_send_data[0] = (flywheel_motor[0].output >> 8) & 0xFF;
		CAN_send_data[1] = (flywheel_motor[0].output) & 0xFF;
		CAN_send_data[2] = (flywheel_motor[1].output >> 8) & 0xFF;
		CAN_send_data[3] = (flywheel_motor[1].output) & 0xFF;
#ifdef ACTIVE_GUIDANCE
		CAN_send_data[4] = (flywheel_motor[2].output >> 8) & 0xFF;
		CAN_send_data[5] = (flywheel_motor[2].output) & 0xFF;
		CAN_send_data[6] = (flywheel_motor[3].output >> 8) & 0xFF;
		CAN_send_data[7] = (flywheel_motor[3].output) & 0xFF;
#else
		CAN_send_data[4] = 0;
		CAN_send_data[5] = 0;
		CAN_send_data[6] = 0;
		CAN_send_data[7] = 0;
#endif
	}
	HAL_CAN_AddTxMessage(LAUNCHER_MOTOR_CAN, &CAN_tx_message, CAN_send_data,
			send_mail_box);
}
void send_feeder_current_to_motor() {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box[3];

	// Clear entire packet first
	memset(CAN_send_data, 0, 8);

	// fill data packet with feeder data
	if (!(g_safety_toggle || g_remote_cmd.sw == SW_SHUTDOWN)) {
	    CAN_set_motor_output(&CAN_tx_message, CAN_send_data, FEEDER_MOTOR_ID, feeder_motor.motor_type, feeder_motor.output);
	}

	  HAL_CAN_AddTxMessage(FEEDER_MOTOR_CAN, &CAN_tx_message, CAN_send_data,
	      send_mail_box);
}

uint16_t check_overheat() {

#ifdef OVERHEAT_PROTECTION
	int32_t ammo_remaining;
	static uint32_t last_time;
	if (ref_robot_data.robot_id == 0) {
		//referee system not connected
		return 10;
	}

#ifdef BULLET_17
	uint8_t active_feeder = 2;
	//else active_feeder == 2, for both heat0 and heat 1 launchers

	//if double barrel launcher, check launcher with more heat only
	if (ref_power_data.shooter_17mm_1_barrel_heat >= ref_power_data.shooter_17mm_2_barrel_heat) {
		active_feeder = 0;
	} else {
		active_feeder = 1;
	}

	if (active_feeder == 0) {
		ammo_remaining = (((ref_robot_data.shooter_barrel_heat_limit
				- ref_power_data.shooter_17mm_1_barrel_heat - OVERHEAT_OFFSET)) / BULLET_17_HEAT);
	} else if (active_feeder == 1) {
		ammo_remaining = (((ref_robot_data.shooter_barrel_heat_limit
				- ref_power_data.shooter_17mm_2_barrel_heat - OVERHEAT_OFFSET)) / BULLET_17_HEAT);
	} else {
		ammo_remaining = 1000;
	}
	if (prev_power_data_no != ref_power_data_txno) {
		prev_power_data_no = ref_power_data_txno;
		last_time = get_microseconds();
		if (ammo_remaining < OVERHEAT_MARGIN) {
			return 0;
		} else {
			return ammo_remaining;
		}
	} else {
		//no updated heat information, guessing ammo remaining;
		uint32_t time_diff = get_microseconds() - last_time;
		if (active_feeder == 0) {
			ammo_remaining += (ref_robot_data.shooter_barrel_cooling_value
					* time_diff / TIMER_FREQ);
			ammo_remaining -= FEEDER_SPEED * time_diff
					/ (TIMER_FREQ * 60);
		} else if (active_feeder == 1) {

			ammo_remaining += (ref_robot_data.shooter_barrel_cooling_value
					* time_diff / TIMER_FREQ);
			ammo_remaining -= FEEDER_SPEED * time_diff
					/ (TIMER_FREQ * 60);
		}
		if (ammo_remaining < OVERHEAT_MARGIN) {
			return 0;
		} else {
			return ammo_remaining;
		}
		return ammo_remaining;
	}
#endif

#ifdef BULLET_42
	ammo_remaining = (ref_robot_data.shooter_barrel_heat_limit
			- ref_power_data.shooter_42mm_barrel_heat - OVERHEAT_OFFSET) / BULLET_42_HEAT;
					//ammo_remaining = 100;

	if (ammo_remaining < OVERHEAT_MARGIN) {
		return 0;
	} else {
		return ammo_remaining;
	}
#endif

	return ammo_remaining;

#else
	return 100; // no overheat protection enabled
#endif

}

void flywheel_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel) {

	int16_t friction_wheel_speed = PROJECTILE_SPEED * PROJECTILE_SPEED_RATIO;
	static uint32_t clear_time = 0;

	/**
	 * Finite state machine for flywheel
	 */
	switch (flywheel_state) {
	case WHEEL_STANDBY:
		if (launcher_ctrl_data.override == 1
				|| launcher_ctrl_data.firing != 0) {
			flywheel_state = WHEEL_FIRING;
		}
		break;

	case WHEEL_FIRING:
		if (launcher_ctrl_data.firing == 0
				&& launcher_ctrl_data.override == 0) {
			flywheel_state = WHEEL_CLEARING;
			clear_time = HAL_GetTick();
		}
		break;

	case WHEEL_CLEARING:
		if (launcher_ctrl_data.firing != 0 ) {
			flywheel_state = WHEEL_FIRING;
		} else if (HAL_GetTick() - clear_time > CLEAR_DELAY && feeder_state == FEEDER_STANDBY) {
			flywheel_state = WHEEL_STANDBY;
		}
		break;

	default:
		flywheel_state = WHEEL_STANDBY;
	}

	switch (flywheel_state) {
	case WHEEL_STANDBY:
		if (FRICTION_SB_SPIN_ON == 2 || (FRICTION_SB_SPIN_ON == 1 && ref_game_state.game_progress == 4)){
			speed_pid(friction_wheel_speed * FRICTION_SB_SPIN * FRICTION_INVERT,
					l_flywheel->raw_data.rpm, &l_flywheel->rpm_pid);
			speed_pid(-friction_wheel_speed * FRICTION_SB_SPIN * FRICTION_INVERT,
					r_flywheel->raw_data.rpm, &r_flywheel->rpm_pid);
			l_flywheel->output = l_flywheel->rpm_pid.output + FRICTION_OFFSET * FRICTION_INVERT;
			r_flywheel->output = r_flywheel->rpm_pid.output - FRICTION_OFFSET * FRICTION_INVERT;
		} else {
			l_flywheel->output = 0;
			r_flywheel->output = 0;
		}
		break;

	case WHEEL_CLEARING:
	case WHEEL_FIRING:
		speed_pid(friction_wheel_speed * FRICTION_INVERT,
				l_flywheel->raw_data.rpm, &l_flywheel->rpm_pid);
		speed_pid(-friction_wheel_speed * FRICTION_INVERT,
				r_flywheel->raw_data.rpm, &r_flywheel->rpm_pid);
		l_flywheel->output = l_flywheel->rpm_pid.output;
		r_flywheel->output = r_flywheel->rpm_pid.output;
		break;

	default:
		l_flywheel->output = 0;
		r_flywheel->output = 0;
	}

}

#ifndef ANGLE_FEEDER

void launcher_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,
		motor_data_t *feeder) {

	static uint32_t jam_start_time = 0;

	int16_t feeder_speed = launcher_ctrl_data.firing
			* FEEDER_SPEED * FEEDER_INVERT
			/ FEEDER_SPEED_RATIO;
	int16_t friction_wheel_speed = PROJECTILE_SPEED * PROJECTILE_SPEED_RATIO;

	int16_t rpm_diff = abs(l_flywheel->raw_data.rpm + r_flywheel->raw_data.rpm);
	int16_t avg_rpm = abs(l_flywheel->raw_data.rpm - r_flywheel->raw_data.rpm)
			/ 2;

	/**
	 * Finite state machine for feeder
	 */
	switch (feeder_state) {
	case FEEDER_STANDBY:
		if (launcher_ctrl_data.firing != 0) {
			feeder_state = FEEDER_SPINUP;
		}
		break;
	case FEEDER_SPINUP:
		if (abs(avg_rpm - friction_wheel_speed) < LAUNCHER_MARGIN) {
			if (rpm_diff < LAUNCHER_DIFF_MARGIN) {
				feeder_state = FEEDER_FIRING;
			}
		}
		break;

	case FEEDER_FIRING:
		//check for feeder jam first, prioritise unjamming
		if (abs(feeder->raw_data.torque)
				> (FEEDER_JAM_TORQUE) && (abs(feeder->raw_data.rpm) < FEEDER_JAM_RPM)) {
			jam_start_time = get_microseconds();
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_JAM;
			break;
		}

		if (launcher_ctrl_data.firing == 0) {
			feeder_state = FEEDER_STANDBY;
			break;
		}

		if (check_overheat() == 0) {
			feeder_state = FEEDER_OVERHEAT;
			break;
		}
		//one side will always be negative

		if (abs(avg_rpm - friction_wheel_speed) > LAUNCHER_MARGIN) {
			if (rpm_diff > LAUNCHER_DIFF_MARGIN) {
				feeder_state = FEEDER_SPINUP;
				break;
			}
		}
		//else stay firing
		break;

	case FEEDER_JAM:
		//check if either after unjam time
		if ((get_microseconds() - jam_start_time) > FEEDER_UNJAM_TIME) {
			feeder_state = FEEDER_SPINUP;
		}

//		if ((feeder->raw_data.torque * FEEDER_INVERT)
//				< -(FEEDER_JAM_TORQUE * FEEDER_INVERT)) {
//			feeder_state = FEEDER_SPINUP;
//		}
		break;

	case FEEDER_OVERHEAT:
		if (check_overheat() > OVERHEAT_EXCESS) {
			if (launcher_ctrl_data.firing != 0) {
				feeder_state = FEEDER_SPINUP;
			} else {
				feeder_state = FEEDER_STANDBY;
			}
		}
		break;

	default:
		feeder_state = FEEDER_STANDBY;
	}

	switch (feeder_state) {
	case FEEDER_STANDBY:
	case FEEDER_SPINUP:
	case FEEDER_OVERHEAT:
		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
//		feeder->output = 0;
		break;

	case FEEDER_FIRING:
		speed_pid(feeder_speed * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_JAM:
		speed_pid(
				FEEDER_UNJAM_SPD * feeder->angle_data.gearbox_ratio
						* FEEDER_INVERT, feeder->raw_data.rpm,
				&feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	default:
		feeder->output = 0;
	}

}
#else

void launcher_angle_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,
		motor_data_t *feeder) {
	static uint32_t jam_start_time = 0;
	int16_t friction_wheel_speed = PROJECTILE_SPEED * PROJECTILE_SPEED_RATIO;
	static uint32_t last_fire;
	static float target_ang = 0;

	int16_t rpm_diff = abs(l_flywheel->raw_data.rpm + r_flywheel->raw_data.rpm);
	int16_t avg_rpm = abs(l_flywheel->raw_data.rpm - r_flywheel->raw_data.rpm)
			/ 2;
	uint32_t curr_time = HAL_GetTick();
	float ang_diff;
	static uint8_t fired = 0;
	ang_diff = target_ang - feeder->angle_data.adj_ang;
	/**
	 * Finite state machine for feeder
	 */
	switch (feeder_state) {
	case FEEDER_STANDBY:
		if (launcher_ctrl_data.firing != 0) {
			if (check_overheat() == 0) {
				feeder_state = FEEDER_OVERHEAT;
				break;
			}

			if (curr_time - last_fire < ANGLE_FEEDER_DELAY) {
				last_fire = curr_time;
				break;
			}
			last_fire = curr_time;
			feeder_state = FEEDER_SPINUP;
		}
		break;
	case FEEDER_SPINUP:
		if (abs(avg_rpm - friction_wheel_speed) < LAUNCHER_MARGIN) {
			if (rpm_diff < LAUNCHER_DIFF_MARGIN) {
				if (fired == 0){
					target_ang = feeder->angle_data.adj_ang
							+ ((2 * PI / FEEDER_SPEED_RATIO) * FEEDER_INVERT);
					if (target_ang > PI) {
						target_ang -= 2 * PI;
					} else if (target_ang < -PI) {
						target_ang += 2 * PI;
					}
					fired = 1;
				}
				feeder_state = FEEDER_STEP;
			}
		}
		break;

	case FEEDER_STEP:
		//check for feeder jam first, prioritise unjamming
		if ((feeder->raw_data.torque) > (FEEDER_JAM_TORQUE * FEEDER_INVERT)) {
			jam_start_time = HAL_GetTick();
			feeder->angle_pid.integral = 0;
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_JAM;
			break;
		}
		ang_diff = target_ang - feeder->angle_data.adj_ang;
		if (ang_diff > PI) {
			ang_diff -= 2 * PI;
		} else if (ang_diff < -PI) {
			ang_diff += 2 * PI;
		}

		if (((fabs(ang_diff) < ANGLE_FEEDER_MARGIN
				&& fabs(feeder->raw_data.rpm / feeder->angle_data.gearbox_ratio)
						< ANGLE_FEEDER_SPD_MARGIN))
				|| (curr_time - last_fire) > ANGLE_FEEDER_TIMEOUT) {
			fired = 0;
			feeder->angle_pid.integral = 0;
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_STANDBY;
			last_fire = curr_time;
			break;
		}

		//don't need to check for flywheel, it'll just mess it up
//			if (abs(avg_rpm - friction_wheel_speed) > LAUNCHER_MARGIN) {
//				if (rpm_diff > LAUNCHER_DIFF_MARGIN) {
//					feeder_state = FEEDER_SPINUP;
//					break;
//				}
//			}
		//else stay firing
		break;

	case FEEDER_JAM:
		//check if either after unjam time
		if ((HAL_GetTick() - jam_start_time) > FEEDER_UNJAM_TIME) {
			feeder_state = FEEDER_STANDBY;
		}

		if ((feeder->raw_data.torque * FEEDER_INVERT)
				< -(FEEDER_JAM_TORQUE * FEEDER_INVERT)) {
			feeder_state = FEEDER_STANDBY;
		}
		break;

	case FEEDER_OVERHEAT:
		if (check_overheat() >= OVERHEAT_EXCESS) {
			feeder->angle_pid.integral = 0;
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_STANDBY;
		}

		break;

	default:
		feeder->angle_pid.integral = 0;
		feeder->rpm_pid.integral = 0;
		feeder_state = FEEDER_STANDBY;
		break;
	}


	ang_diff = target_ang - feeder->angle_data.adj_ang;
	if (ang_diff > PI) {
		ang_diff -= 2 * PI;
	} else if (ang_diff < -PI) {
		ang_diff += 2 * PI;
	}

	switch (feeder_state) {
	case FEEDER_STANDBY:
	case FEEDER_SPINUP:
	case FEEDER_OVERHEAT:

				speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
				feeder->output = feeder->rpm_pid.output;
//		speed_pid(ang_diff, 0, &feeder->angle_pid);
//		speed_pid(feeder->angle_pid.output * feeder->angle_data.gearbox_ratio,
//				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
//		feeder->output = 0;
		break;

	case FEEDER_STEP:
		speed_pid(ang_diff, 0, &feeder->angle_pid);
		speed_pid(feeder->angle_pid.output * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_JAM:
		speed_pid(FEEDER_UNJAM_SPD * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	default:
		feeder->output = 0;
	}
}
#endif

void guidance_flywheel(motor_data_t *l_flywheel, motor_data_t *r_flywheel, motor_data_t *b_flywheel) {
	int16_t friction_wheel_speed = PROJECTILE_SPEED * PROJECTILE_SPEED_RATIO;
	static uint32_t clear_time = 0;

	/**
	 * Finite state machine for flywheel
	 */
	switch (flywheel_state) {
	case WHEEL_STANDBY:
		if (launcher_ctrl_data.override == 1
				|| launcher_ctrl_data.firing != 0) {
			flywheel_state = WHEEL_FIRING;
		}
		break;

	case WHEEL_FIRING:
		if (launcher_ctrl_data.firing == 0
				&& launcher_ctrl_data.override == 0) {
			flywheel_state = WHEEL_CLEARING;
			clear_time = HAL_GetTick();
		}
		break;

	case WHEEL_CLEARING:
		if (launcher_ctrl_data.firing != 0 ) {
			flywheel_state = WHEEL_FIRING;
		} else if (HAL_GetTick() - clear_time > CLEAR_DELAY) {
			flywheel_state = WHEEL_STANDBY;
		}
		break;

	default:
		flywheel_state = WHEEL_STANDBY;
	}

	switch (flywheel_state) {
	case WHEEL_STANDBY:
		if (FRICTION_SB_SPIN_ON == 2 || (FRICTION_SB_SPIN_ON == 1 && ref_game_state.game_progress == 4)){
			speed_pid(friction_wheel_speed * FRICTION_SB_SPIN * FRICTION_INVERT,
					l_flywheel->raw_data.rpm, &l_flywheel->rpm_pid);
			speed_pid(-friction_wheel_speed * FRICTION_SB_SPIN * FRICTION_INVERT,
					r_flywheel->raw_data.rpm, &r_flywheel->rpm_pid);
			speed_pid(-friction_wheel_speed * FRICTION_SB_SPIN * FRICTION_INVERT,
					b_flywheel->raw_data.rpm, &b_flywheel->rpm_pid);
			l_flywheel->output = l_flywheel->rpm_pid.output + FRICTION_OFFSET * FRICTION_INVERT;
			r_flywheel->output = r_flywheel->rpm_pid.output - FRICTION_OFFSET * FRICTION_INVERT;
			b_flywheel->output = b_flywheel->rpm_pid.output - FRICTION_OFFSET * FRICTION_INVERT;
		} else {
			l_flywheel->output = 0;
			r_flywheel->output = 0;
			b_flywheel->output = 0;

		}
		break;

	case WHEEL_CLEARING:
	case WHEEL_FIRING:
		speed_pid(friction_wheel_speed * FRICTION_INVERT,
				l_flywheel->raw_data.rpm, &l_flywheel->rpm_pid);
		speed_pid(-friction_wheel_speed * FRICTION_INVERT,
				r_flywheel->raw_data.rpm, &r_flywheel->rpm_pid);
		speed_pid(-friction_wheel_speed * FRICTION_INVERT,
				b_flywheel->raw_data.rpm, &b_flywheel->rpm_pid);
		l_flywheel->output = l_flywheel->rpm_pid.output;
		r_flywheel->output = r_flywheel->rpm_pid.output;
		b_flywheel->output = b_flywheel->rpm_pid.output;
		break;

	default:
		l_flywheel->output = 0;
		r_flywheel->output = 0;
		b_flywheel->output = 0;
		break;
	}

}

void guidance_feeder(motor_data_t *l_flywheel, motor_data_t *r_flywheel, motor_data_t *b_flywheel,
		motor_data_t *g_flywheel, motor_data_t *feeder) {

	static uint32_t jam_start_time = 0;

	int16_t feeder_speed = FEEDER_SPEED * FEEDER_INVERT
			/ FEEDER_SPEED_RATIO;
	int16_t friction_wheel_speed = PROJECTILE_SPEED * PROJECTILE_SPEED_RATIO;

	int16_t firing_flywheel[3] = {abs(l_flywheel->raw_data.rpm), abs(r_flywheel->raw_data.rpm), abs(b_flywheel->raw_data.rpm)};

	int16_t max_rpm = firing_flywheel[0];
	int16_t min_rpm = firing_flywheel[0];
	int16_t sum_rpm = firing_flywheel[0];

	for (int i = 1; i < 3; i++) {
		max_rpm = (max_rpm < firing_flywheel[i]) ? firing_flywheel[i] : max_rpm;
		min_rpm = (min_rpm > firing_flywheel[i]) ? firing_flywheel[i] : min_rpm;
		sum_rpm += firing_flywheel[i];
	}

	int16_t rpm_diff = max_rpm - min_rpm;
	int16_t avg_rpm = sum_rpm / 3;

	/**
	 * Finite state machine for feeder & guidance flywheel
	 */
	switch (feeder_state) {
	// Spin the feeder if microswitch is not triggered
	case FEEDER_STANDBY:
		if (launcher_ctrl_data.firing) {
			feeder_state = FEEDER_SPINUP;
			break;
		}

		if (launcher_ctrl_data.firing != 0 && projectile_loaded) {
			feeder_state = FEEDER_LOADED;
			break;
		}
		break;
	// Keep spinning the feeder while microswitch is not triggered
	case FEEDER_SPINUP:
		if (!launcher_ctrl_data.firing) {
			feeder_state = FEEDER_STANDBY;
			break;
		}
		if ((feeder->raw_data.torque)
				> (FEEDER_JAM_TORQUE * FEEDER_INVERT) && (abs(feeder->raw_data.rpm) < FEEDER_JAM_RPM)) {
			jam_start_time = HAL_GetTick();
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_JAM;
			break;
		}
		break;
	case FEEDER_LOADED:
		// Only allow active guidance flywheel to fire if no overheat and firing
		if (check_overheat() == 0) {
			feeder_state = FEEDER_OVERHEAT;
			break;
		}
		if (launcher_ctrl_data.firing != 0) {
			if (abs(avg_rpm - friction_wheel_speed) < LAUNCHER_MARGIN && rpm_diff < LAUNCHER_DIFF_MARGIN)
			feeder_state = FEEDER_FIRING;
			break;
		} else {
			feeder_state = FEEDER_STANDBY;
			break;
		}
		break;
	case FEEDER_FIRING:
		if (!projectile_loaded) {
			feeder_state =  FEEDER_FIRING_2; //FEEDER_SPINUP
			break;
		}
		if (abs(avg_rpm - friction_wheel_speed) > LAUNCHER_MARGIN) {
			if (rpm_diff > LAUNCHER_DIFF_MARGIN) {
				feeder_state = FEEDER_LOADED;
				break;
			}
		}
		break;
	case FEEDER_FIRING_2:
		if (projectile_loaded) {
			feeder_state = FEEDER_FIRING_3;
			break;
		}
		if ((feeder->raw_data.torque)
				> (FEEDER_JAM_TORQUE * FEEDER_INVERT) && (abs(feeder->raw_data.rpm) < FEEDER_JAM_RPM)) {
			jam_start_time = HAL_GetTick();
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_JAM;
			break;
		}
		break;
	case FEEDER_FIRING_3:
//		if (projectile_loaded) {
//			feeder_state = FEEDER_LOADED;
//			break;
//		} else {
			feeder_state = FEEDER_STANDBY;
//		}
		break;
	case FEEDER_JAM:
		//check if either after unjam time
		if ((HAL_GetTick() - jam_start_time) > FEEDER_UNJAM_TIME) {
			feeder_state = FEEDER_SPINUP;
		}

		if ((feeder->raw_data.torque * FEEDER_INVERT)
				< -(FEEDER_JAM_TORQUE * FEEDER_INVERT)) {
			feeder_state = FEEDER_SPINUP;
		}
		break;
	case FEEDER_OVERHEAT:
		if (check_overheat() > OVERHEAT_EXCESS) {
			feeder_state = FEEDER_LOADED;
		}
		break;

	default:
		feeder_state = FEEDER_STANDBY;
	}

	// The feeder only turns during spinup and jam
	// Actual firing is left to the active flywheel
	switch (feeder_state) {
	case FEEDER_STANDBY:
	case FEEDER_OVERHEAT:
	case FEEDER_LOADED:
		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_SPINUP:
		speed_pid(feeder_speed * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_FIRING:
		// spins feeder forward until projectile not loaded
		// then move on to FEEDER_FIRING_2
		speed_pid(feeder_speed * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_FIRING_2:
		// spins feeder forward until 2nd projectile loaded
		// then move on to FEEDER_FIRING_2

		speed_pid(feeder_speed * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_FIRING_3:
		// actually fires the 1st projectile
		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid); // stops feeder
		feeder->output = feeder->rpm_pid.output;
		break;


	case FEEDER_JAM:
		speed_pid(FEEDER_UNJAM_SPD * feeder->angle_data.gearbox_ratio * FEEDER_INVERT,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	default:
		feeder->output = 0;
	}
}
