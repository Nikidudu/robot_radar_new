/*
 * chassis_can_message_task.c
 *
 *  Created on: Oct 25, 2025
 *      Author: zhan-hao
 */

#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "typedefs.h"
#include "master_task.h"
#include "board_lib.h"

// CAN message IDs
#define CHASSIS_DATA_1_ID 0x100
#define CHASSIS_DATA_2_ID 0x101
#define CHASSIS_HB_ID 	  0x119

// CAN transmission period
#define CAN_TX_PERIOD_MS 10

// Chassis heartbeat transmission period
#define CAN_HB_PERIOD_MS 1000

//Global Variables
extern chassis_control_t chassis_ctrl_data;
extern TaskHandle_t chassis_heartbeat_task_handle;
extern ref_game_robot_data_t ref_robot_data;

// Function Declarations
void chassis_heartbeat_task(void *argument);
void level_config(float *lvl_max_speed, float *lvl_max_accel, float *lvl_max_spin);
float rpm_ramp(float target_value, float current_value, float *lvl_max_accel);

//Global Variables (only in this file)
static float lvl_max_speed;
static float lvl_max_accel;
static float lvl_max_spin;
static float spin_accel = SPIN_ACCELERATION;

void chassis_can_message_task(void *argument) {

	xTaskCreate(chassis_heartbeat_task, "chassis_heartbeat_task",
			configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
					&chassis_heartbeat_task_handle);

    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_buffer[8];
    uint32_t tx_mailbox;
    TickType_t xLastWakeTime;

    // Configure CAN TX header
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    //Speed and acceleration control variables
    float limit_forward;
    float limit_horizontal;
    float limit_yaw;
    float act_forward;
    float act_horizontal;
    float act_yaw;

    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();

    while(1) {

    	float rel_angle = g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;

    	// Setting translational and rotational speed and acceleration base on robot level
    	level_config(&lvl_max_speed, &lvl_max_accel, &lvl_max_spin);

    	float speed_limit = lvl_max_speed;
    	float spin_limit = lvl_max_spin;

    	// Increase speed when spinspin mode is deactivated
    	if (chassis_ctrl_data.g_spinspin_mode == 0) {
    		speed_limit += CHASSIS_SPEED_BOOST;
    	}

    	// Clamp the values between -limit to limit
    	limit_forward = fmaxf(-speed_limit,
    			fminf(chassis_ctrl_data.forward, speed_limit));
    	limit_horizontal = fmaxf(-speed_limit,
    			fminf(chassis_ctrl_data.horizontal, speed_limit));
    	limit_yaw = fmaxf(-spin_limit,
    			fminf(chassis_ctrl_data.yaw, spin_limit));

    	// Smooths speed changes over time using acceleration constraints
    	act_forward = rpm_ramp(limit_forward, act_forward, &lvl_max_accel);
    	act_horizontal = rpm_ramp(limit_horizontal, act_horizontal, &lvl_max_accel);
    	act_yaw = rpm_ramp(limit_yaw, act_yaw, &spin_accel);

    	// translation and rotation speed of chassis for chassis yaw angle relative to gimbal
    	float rel_forward = ((-act_horizontal * sin(-rel_angle))
    			+ (act_forward * cos(-rel_angle)));
    	float rel_horizontal = ((-act_horizontal * cos(-rel_angle))
    			+ (act_forward * -sin(-rel_angle)));
    	float rel_yaw = act_yaw;

    	// ===== Send CHASSIS_DATA_1 ===== //
        memset(tx_buffer, 0, 8);
        memcpy(&tx_buffer[0], &rel_forward, sizeof(float));
        memcpy(&tx_buffer[4], &rel_horizontal, sizeof(float));

//        // typecase from float to scaled int16_t
//        memcpy(&tx_buffer[0], &act_forward, sizeof(int16_t));
//        memcpy(&tx_buffer[2], &act_horizontal, sizeof(int16_t));
//        memcpy(&tx_buffer[4], &act_yaw, sizeof(int16_t));
//        memcpy(&tx_buffer[6], &chassis_ctrl_data.enabled, sizeof(uint8_t));
//	    memcpy(&tx_buffer[7], &ref_robot_data.chassis_power_limit, sizeof(uint8_t));

        tx_header.StdId = CHASSIS_DATA_1_ID;

        // Wait for a free mailbox and send
        while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
            vTaskDelay(1);  // Wait 1ms if all mailboxes are full
        }

        HAL_GPIO_WritePin(RED_LED_TIM_GPIO_Port, RED_LED_TIM_Pin, 1);
        if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buffer, &tx_mailbox) != HAL_OK) {
            // Handle error if needed
            Error_Handler();
        }

        // ===== Send CHASSIS_DATA_2 ====== //
//        -> chassis_power_limit
//        <- charging_state
        memset(tx_buffer, 0, 8);
        memcpy(&tx_buffer[0], &rel_yaw, sizeof(float));
        memcpy(&tx_buffer[4], &chassis_ctrl_data.enabled, sizeof(uint8_t));
        memcpy(&tx_buffer[5], &chassis_ctrl_data.g_spinspin_mode, sizeof(uint8_t));//
	    memcpy(&tx_buffer[6], &chassis_ctrl_data.supercap_dash, sizeof(uint8_t));//
	    memcpy(&tx_buffer[7], &chassis_ctrl_data.supercap_enabled, sizeof(uint8_t));//

        tx_header.StdId = CHASSIS_DATA_2_ID;

        // Wait for a free mailbox and send
        while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
            vTaskDelay(1);
        }

        if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buffer, &tx_mailbox) != HAL_OK) {
            Error_Handler();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_TX_PERIOD_MS));
    }
}


void chassis_heartbeat_task(void *argument) {
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t tx_buffer[8];
	CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;


	tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;


	while (1) {

		memset(tx_buffer, 0, 8);

		tx_header.StdId = CHASSIS_HB_ID;

		// Wait for a free mailbox and send
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
			vTaskDelay(1);  // Wait 1ms if all mailboxes are full
		}

		if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buffer, &tx_mailbox) != HAL_OK) {
			Error_Handler();
		}


		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_HB_PERIOD_MS));
	}
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

