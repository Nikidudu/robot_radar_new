/*
 * motor_config.c
 *
 *  Created on: 5 Jan 2022
 *      Author: wx
 */

#include "board_lib.h"

void set_motor_config(motor_data_t *motor) {
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

