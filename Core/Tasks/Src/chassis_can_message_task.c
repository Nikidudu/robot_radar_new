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

//Global Variables (only in this file)
static float lvl_max_speed;
static float lvl_max_accel;
static float lvl_max_spin;

void chassis_can_message_task(void *argument) {

	xTaskCreate(chassis_heartbeat_task, "chassis_heartbeat_task",
			configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
					&chassis_heartbeat_task_handle);

	level_config(&lvl_max_speed, &lvl_max_accel, &lvl_max_spin);

    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_buffer[8];
    uint32_t tx_mailbox;
    TickType_t xLastWakeTime;

    // Configure CAN TX header
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    //Clamped control values
	float speed_limit = lvl_max_speed;
	float spin_limit = lvl_max_spin;
    float limit_forward;
    float limit_horizontal;
    float limit_yaw;

    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // ===== Send CHASSIS_DATA_1 ===== //
    	limit_forward = fmaxf(-speed_limit,
    			fminf(chassis_ctrl_data.forward, speed_limit));
    	limit_horizontal = fmaxf(-speed_limit,
    			fminf(chassis_ctrl_data.horizontal, speed_limit));
    	limit_yaw = fmaxf(-spin_limit,
    			fminf(chassis_ctrl_data.yaw, spin_limit));

        memset(tx_buffer, 0, 8);
        memcpy(&tx_buffer[0], &limit_forward, sizeof(float));
        memcpy(&tx_buffer[4], &limit_horizontal, sizeof(float));

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
        memset(tx_buffer, 0, 8);
        memcpy(&tx_buffer[0], &limit_yaw, sizeof(float));
        memcpy(&tx_buffer[4], &chassis_ctrl_data.enabled, sizeof(uint8_t));
        memcpy(&tx_buffer[5], &chassis_ctrl_data.g_spinspin_mode, sizeof(uint8_t));
	    memcpy(&tx_buffer[6], &chassis_ctrl_data.supercap_dash, sizeof(uint8_t));
	    memcpy(&tx_buffer[7], &chassis_ctrl_data.supercap_enabled, sizeof(uint8_t));

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
