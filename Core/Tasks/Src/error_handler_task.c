/*
 * error_handler_task.c
 *
 *  Created on: Sep 17, 2025
 *      Author: gskang
 */

#include "board_lib.h"
#include "error_handler_task.h"
#include "gimbal_control_task.h"
#include "control_input_task.h"

extern QueueHandle_t g_buzzing_task_msg;

uint16_t g_motor_fault; // for debugging

/*
 * Motor Mapping (number of beeps):
 * disconnect_high, disconnect_low, overheat_high, overheat_low
 *
 * {1, 1, 1, 1}, // FR wheel
 * {1, 2, 1, 2}, // FL wheel
 * {1, 3, 1, 3}, // BL wheel
 * {1, 4, 1, 4}, // BR wheel
 *
 * {2, 1, 2, 1}, // LFRICTION
 * {2, 2, 2, 2}, // RFRICTION
 * {2, 3, 2, 3}, // FEEDER
 * {2, 4, 2, 4}, // BFRICTION
 * {2, 5, 2, 5}, // GFRICTION
 *
 * {3, 1, 3, 1}, // PITCH
 * {2, 1, 3, 2}, // YAW
 * */

void error_handler_task(void *argument) {
	//insert can tester?
	uint16_t error = check_motors();
	uint32_t delay = 0;
	vTaskDelay(50);
	if (MOTOR_ONLINE_CHECK == 1) {
		while (error != 0) {
			delay = 500;
			error = check_motors();
			// chassis motors
			for (uint8_t i = 0; i < 4; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(1, i + 1);
					delay += 600;
				}
			}
			// launcher motors (2 flywheels + 1 feeder)
			for (uint8_t i = 4; i < 7; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(2, (i - 3));
					delay += 600;
				}
			}
			// gimbal motors (pitch + yaw)
			for (uint8_t i = 7; i < 9; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(3, (i - 6));
					delay += 600;
				}
			}
#ifdef ACTIVE_GUIDANCE
			// bfriction + gfriction (launcher motors for active guidance)
			for (uint8_t i = 9; i < 11; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(2, (i - 5));
					delay += 600;
				}
			}
#endif
			// bottom dev C
			for (uint8_t i = 11; i < 12; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(4, (i - 10));
					delay += 600;
				}
			}

			vTaskDelay(delay);
		}
	}

	uint8_t temp_msg;
	if (error == 0) {
		temp_msg = ok;
	} else {
		temp_msg = not_ok;
	}
	xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
	uint32_t last_check = HAL_GetTick();

	while (1) {
		error = 0;

		error = check_motors();
		g_motor_fault = error;
		if (HAL_GetTick() - last_check > 5000) {
			delay = 1000;
			last_check = HAL_GetTick();
			if (MOTOR_ONLINE_CHECK == 1) {
				for (uint8_t i = 0; i < 4; i++) {
					if (error & (1 << (i))) {
						bz_buzzer(1, i + 1);
						delay += 600;
					}
				}
				for (uint8_t i = 4; i < 7; i++) {
					if (error & (1 << (i))) {
						bz_buzzer(2, (i - 3));
						delay += 600;
					}
				}
				for (uint8_t i = 7; i < 9; i++) {
					if (error & (1 << (i))) {
						bz_buzzer(3, (i - 6));
						delay += 600;
					}
				}
#ifdef ACTIVE_GUIDANCE
				for (uint8_t i = 9; i < 11; i++) {
					if (error & (1 << (i))) {
						bz_buzzer(2, (i - 5));
						delay+=600;
					}
				}
#endif
				for (uint8_t i = 11; i < 12; i++) {
					if (error & (1 << (i))) {
						bz_buzzer(4, (i - 10));
						delay += 600;
					}
				}

				vTaskDelay(delay);
				continue;
			} else if (MOTOR_ONLINE_CHECK == 0) {
				if (error != 0) {
					bz_buzzer(0, 2);
					vTaskDelay(5000);
					continue;
				}
			} else {
				error = 0;
			}
		}
		vTaskDelay(1000);
	}

	//future calibration code, if any
	// task takes highest priority over....everything so make sure to kill all motors first!
	//implement mutexes so this task doesn't check while the motor tasks do their thing

	//write in task here to calibrate then stop lol
}

void bz_buzzer(uint8_t high, uint8_t low) {
	uint8_t temp_msg = bz_debug_high;
	for (uint8_t i = 0; i < high; i++) {
		xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
	}
	temp_msg = bz_debug_low;
	for (int8_t i = 0; i < low; i++) {
		xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
	}
	temp_msg = bz_debug_rest;
	xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
}

/*
 * Plays high temp warning beeping(by sending it to buzzing_task)
 */
void motor_temp_bz(uint8_t hi, uint8_t low) {
	 // high-temp warning intro sequence (3 notes in quick succession)
	uint8_t temp_msg = bz_debug_hi_temp;
	xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
	// a short pause
	temp_msg = bz_debug_rest;
	xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
	// plays hi number of high-pitch beeps
	for (int8_t i = 0; i < hi; i++) {
		temp_msg = bz_temp_hi;
		xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
	}
	// plays low number of low-pitch beeps
	for (int8_t i = 0; i < low; i++) {
		temp_msg = bz_temp_low;
		xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
	}

	temp_msg = bz_debug_rest;
	xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);

}

uint16_t check_motors() {
	uint16_t error = 0;
	uint32_t curr_time = get_microseconds();

//	// chassis wheels
//	if (curr_time - chassis_wheel[FR_MOTOR_ID].last_time[0] > MOTOR_TIMEOUT_MAX) {
//		error |= 1 << (0);
//	} else if (chassis_wheel[FR_MOTOR_ID].raw_data.temp > HITEMP_WARNING) {
//		motor_temp_bz(1, 1);
//	}
//	if (curr_time - chassis_wheel[FL_MOTOR_ID].last_time[0] > MOTOR_TIMEOUT_MAX) {
//		error |= 1 << (1);
//	} else if (chassis_wheel[FL_MOTOR_ID].raw_data.temp > HITEMP_WARNING) {
//		motor_temp_bz(1, 2);
//	}
//	if (curr_time - chassis_wheel[BL_MOTOR_ID].last_time[0] > MOTOR_TIMEOUT_MAX) {
//		error |= 1 << (2);
//	} else if (chassis_wheel[BL_MOTOR_ID].raw_data.temp > HITEMP_WARNING) {
//		motor_temp_bz(1, 3);
//	}
//	if (curr_time - chassis_wheel[BR_MOTOR_ID].last_time[0] > MOTOR_TIMEOUT_MAX) {
//		error |= 1 << (3);
//	} else if (chassis_wheel[BR_MOTOR_ID].raw_data.temp > HITEMP_WARNING) {
//		motor_temp_bz(1, 4);
//	}

	// launcher flywheels and feeder
	if (curr_time
			- flywheel_motor[LFRICTION_MOTOR_ID - 1].last_time[0]> MOTOR_TIMEOUT_MAX) {
		error |= 1 << (4);
	} else if (flywheel_motor[LFRICTION_MOTOR_ID - 1].raw_data.temp > HITEMP_WARNING) {
		motor_temp_bz(2, 1);
	}
	if (curr_time
			- flywheel_motor[RFRICTION_MOTOR_ID - 1].last_time[0]> MOTOR_TIMEOUT_MAX) {
		error |= 1 << (5);
	} else if (flywheel_motor[RFRICTION_MOTOR_ID - 1].raw_data.temp > HITEMP_WARNING) {
		motor_temp_bz(2, 2);
	}
	if (curr_time
			- feeder_motor.last_time[0]> MOTOR_TIMEOUT_MAX) {
		error |= 1 << 6;
	} else if (feeder_motor.raw_data.temp > HITEMP_WARNING) {
		motor_temp_bz(2, 3);
	}

#ifdef ACTIVE_GUIDANCE
	if (curr_time
			- flywheel_motor[BFRICTION_MOTOR_ID - 1].last_time[0]> MOTOR_TIMEOUT_MAX) {
		error |= 1 << 9;
	} else if (flywheel_motor[BFRICTION_MOTOR_ID - 1].raw_data.temp > HITEMP_WARNING) {
			motor_temp_bz(2, 4);
	}
	if (curr_time
			- flywheel_motor[GFRICTION_MOTOR_ID - 1].last_time[0]> MOTOR_TIMEOUT_MAX) {
		error |= 1 << 10;
	} else if (flywheel_motor[GFRICTION_MOTOR_ID - 1].raw_data.temp > HITEMP_WARNING) {
			motor_temp_bz(2, 5);
	}
#endif

	// gimbal motors
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
	if (curr_time - dm_pitch_motor.disconnect_time > MOTOR_TIMEOUT_MAX) {
		// pitch motor not returning data (para) to dev c
		error |= 1 << 7;
		dm_set_pitch_motor();
	} else if (dm_pitch_motor.para.state != 9) {
		// pitch motor not accepting data from dev c
		dm_pitch_motor.para.disconnect_time++;
		if (dm_pitch_motor.para.state != 9 && dm_pitch_motor.para.disconnect_time > 100) {
			error |= 1 << 7;
			dm_set_pitch_motor();
		}
	} else {
		if (dm_pitch_motor.para.Tcoil > HITEMP_WARNING) {
			motor_temp_bz(3, 1);
		}
		dm_pitch_motor.para.disconnect_time = 0;
	}
#else
	if (curr_time
			- pitch_motor.last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << 7;
	} else if (pitch_motor.raw_data.temp > HITEMP_WARNING) {
		motor_temp_bz(3, 1);
	}
#endif

#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
	if (curr_time - dm_yaw_motor.disconnect_time > MOTOR_TIMEOUT_MAX) {
		error |= 1 << 8;
		dm_set_yaw_motor();
	} else if (dm_yaw_motor.para.state != 9) {
		// yaw motor not accepting data from dev c
        dm_yaw_motor.para.disconnect_time++;
		if (dm_yaw_motor.para.state != 9 && dm_yaw_motor.para.disconnect_time > 100) {
			error |= 1 << 8;
			dm_set_yaw_motor();
		}
	} else if (dm_yaw_motor.para.Tcoil > HITEMP_WARNING) {
		motor_temp_bz(3, 2);
	}
    dm_yaw_motor.para.disconnect_time = 0;
#else
	if (curr_time - yaw_motor.last_time[0]> MOTOR_TIMEOUT_MAX) {
		error |= 1 << 8;
	} else if (yaw_motor.raw_data.temp > HITEMP_WARNING) {
		motor_temp_bz(3, 2);
	}
#endif

	// bottom dev C
	if (curr_time - chassis_ctrl_data.last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << 8;
	}

	return error;
}
