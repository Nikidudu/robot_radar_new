/*
 * can_msg_processor.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "motor_config.h"
#include "can_msg_processor.h"
#include "supercap_comm_task.h"
#include "chassis_can_message_task.h"
#include "gimbal_control_task.h"

extern EventGroupHandle_t gimbal_event_group;
extern EventGroupHandle_t chassis_event_group;
extern EventGroupHandle_t launcher_event_group;

//where is this number from lmao
motor_map_t dm_motor_map[15];

extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

extern motor_data_t chassis_wheel[4];
extern motor_data_t flywheel_motor[4];
extern motor_data_t feeder_motor;

/* Function Prototypes */
void process_bot_dev_c_can_msg(uint32_t* msg_id, uint8_t* rx_buffer);

/**
 * CAN ISR function, triggered upon RX_FIFO0_MSG_PENDING or RxFifo1MsgPendingCallback
 * converts the raw can data to the motor_data struct form as well
 */
void can_ISR(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[CAN_BUFFER_SIZE];

	// check which CAN bus received it
	// required because the 2 can buses use seperate FIFOs for receive
	// CAN1: FIFO0; CAN2: FIFO1

	if (hcan->Instance == CAN1) {
		if (can1_get_msg(&RxHeader, RxData) != HAL_OK) {
			return;
		}

		switch (RxHeader.StdId) {
		// information from bottom dev C
		case DEV_C_BOT_TO_TOP_ID:
			process_bot_dev_c_can_msg(&RxHeader.StdId, (uint8_t*) RxData);
			break;

		// feeder motor
		case CAN_3508_ALL_ID + 4:
			if (FEEDER_MOTOR_CAN == &hcan1) {
				convert_raw_can_data(&feeder_motor, RxHeader.StdId,
						(uint8_t*) RxData);
			}
			break;

		// pitch motor
		case DM_PITCH_MOTOR_ID: //todo: replace with normal pitch code when switched to DJI mode
			if (PITCH_MOTOR_CAN == &hcan1) {
				dm4310_fbdata(&dm_pitch_motor, &RxData[0]);
			}
			break;
//		case PITCH_MOTOR_ID:
//			if(PITCH_MOTOR_CAN_PTR == &hcan1) {
//
//			}

		// yaw motor
		case CAN_6020_ALL_ID + YAW_MOTOR_ID:
			if (YAW_MOTOR_CAN == &hcan1) {
				convert_raw_can_data(&yaw_motor,
						RxHeader.StdId, (uint8_t*) RxData);
			}
			break;
		default:

		}
	}

	else if (hcan->Instance == CAN2) {
		if (can2_get_msg(&RxHeader, RxData) != HAL_OK) {
			return;
		}

		switch (RxHeader.StdId) {
		// launcher motors (flywheels)
		case CAN_3508_ALL_ID + LFRICTION_MOTOR_ID:
		case CAN_3508_ALL_ID + RFRICTION_MOTOR_ID:
		case CAN_3508_ALL_ID + BFRICTION_MOTOR_ID:
		case CAN_3508_ALL_ID + GFRICTION_MOTOR_ID:
			if (LAUNCHER_MOTOR_CAN == &hcan2) {
				convert_raw_can_data(
						&flywheel_motor[RxHeader.StdId - CAN_3508_ALL_ID - 1],
						RxHeader.StdId, (uint8_t*) RxData);
			}
			break;
		default:

		}
	}
}

void process_bot_dev_c_can_msg(uint32_t* msg_id, uint8_t* rx_buffer) {
    supercap.charging_state = rx_buffer[0];

    if (supercap.charging_state < SUPERCAP_DISABLE_THRESHOLD) {
    	supercap.supercap_enabled = 0;
    }

	supercap.last_time[1] = supercap.last_time[0];
	supercap.last_time[0] = get_microseconds();
}

/*
 * Converts raw CAN data over to the motor_data_t struct
 * 7 bytes of CAN data is sent from the motors:
 * High byte for motor angle data
 * Low byte for motor angle data
 * High byte for RPM
 * Low byte for RPM
 * High byte for Torque
 * Low byte for Torque
 * 1 byte for temperature
 *
 * This function combines the respective high and low bytes into 1 single 16bit integer, then stores them
 * in the struct for the motor.
 *
 * For GM6020 motors, it recenters the motor angle data and converts it to radians.
 */
void convert_raw_can_data(motor_data_t *can_motor_data, uint16_t motor_id,
		uint8_t *rx_buffer) {

	motor_data_t *curr_motor = can_motor_data;
	//convert the raw data back into the respective values
	curr_motor->id = motor_id;
	curr_motor->raw_data.angle[1] = curr_motor->raw_data.angle[0];
	curr_motor->raw_data.angle[0] = (rx_buffer[0] << 8) | rx_buffer[1];
	int16_t temp_rpm = (rx_buffer[2] << 8) | rx_buffer[3];
	curr_motor->raw_data.rpm = curr_motor->raw_data.rpm * SPEED_LPF
			+ temp_rpm * (1 - SPEED_LPF);
	curr_motor->raw_data.torque = (rx_buffer[4] << 8) | rx_buffer[5];
	curr_motor->raw_data.temp = (rx_buffer[6]);
	curr_motor->last_time[1] = curr_motor->last_time[0];
	curr_motor->last_time[0] = get_microseconds();

	float rds_passed = (float) (curr_motor->raw_data.angle[0]
			- curr_motor->raw_data.angle[1]) / 8192;
	float time_diff = (float) (curr_motor->last_time[0]
			- curr_motor->last_time[1]) / (float) (TIMER_FREQ * 60);
	curr_motor->angle_data.hires_rpm = curr_motor->angle_data.hires_rpm * 0.95
			+ (rds_passed * time_diff * 0.05);
	//process the angle data differently depending on the motor type to get radians in the
	//adj_angle value

	//motor must be initialised in motor_config.c first
	if (curr_motor->motor_type > 0) {
		switch (curr_motor->motor_type) {
		case TYPE_GM6020:
			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
					curr_motor->last_time);
			angle_offset(&curr_motor->raw_data, &curr_motor->angle_data);
			break;
		case TYPE_M2006:
		case TYPE_M3508:
			break;
		case TYPE_M2006_STEPS:
		case TYPE_M3508_STEPS:
//			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
//					curr_motor->last_time);
			break;
		case TYPE_M2006_ANGLE:
		case TYPE_M3508_ANGLE:
		case TYPE_GM6020_720:
			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
					curr_motor->last_time);
			angle_offset(&curr_motor->raw_data, &curr_motor->angle_data);
			break;
		default:
			break;

		}
	}
}

/**
 * Centers the raw motor angle to between -Pi to +Pi
 */
void angle_offset(raw_data_t *motor_data, angle_data_t *angle_data) {
	int32_t temp_ang = 0;

	//if there's a gearbox, use the ticks after the gearbox.
	//make sure center angle is properly set with respect to the zero-ing angle
	//YOUR ROBOT MUST HAVE A WAY TO ZERO THIS ANGLE AND IMPLEMENT A ZEROING FUNCTION AT STARTUP
	//IF NOT IT WON'T WORK 							-wx
	temp_ang = angle_data->ticks - angle_data->center_ang;
	if (temp_ang > angle_data->max_ticks) {
		temp_ang -= angle_data->tick_range;
	} else if (temp_ang < angle_data->min_ticks) {
		temp_ang += angle_data->tick_range;
	}
//	angle_data->ticks = temp_ang;
	angle_data->adj_ang = (float) temp_ang * angle_data->ang_range
			/ angle_data->tick_range;
}

void motor_calc_odometry(raw_data_t *motor_data, angle_data_t *angle_data,
		uint32_t feedback_times[]) {
	int16_t abs_angle_diff;
	if (angle_data->init == 0) {
		angle_data->ticks = motor_data->angle[0];
		if (angle_data->ticks > angle_data->max_ticks) {
			angle_data->ticks -= angle_data->tick_range;
		}
		if (angle_data->ticks < angle_data->min_ticks) {
			angle_data->ticks += angle_data->tick_range;
		}
		motor_data->angle[1] = motor_data->angle[0];
		angle_data->init = 1;
		return;
	}
	abs_angle_diff = motor_data->angle[0] - motor_data->angle[1];
	//generally the motor won't exceed half a turn between each feedback
	if (abs_angle_diff > angle_data->max_raw_ticks) {
		abs_angle_diff -= angle_data->raw_ticks_range;
	} else if (abs_angle_diff < angle_data->min_raw_ticks) {
		abs_angle_diff += angle_data->raw_ticks_range;
	}

	uint16_t gear_ticks = angle_data->raw_ticks_range
			* angle_data->gearbox_ratio;
	angle_data->ticks += abs_angle_diff;
	while (angle_data->ticks > angle_data->max_ticks) {
		angle_data->ticks -= angle_data->tick_range;
	}
	while (angle_data->ticks < angle_data->min_ticks) {
		angle_data->ticks += angle_data->tick_range;
	}

	angle_data->dist = angle_data->ticks * angle_data->wheel_circ / gear_ticks;
	motor_data->angle[1] = motor_data->angle[0];
}

