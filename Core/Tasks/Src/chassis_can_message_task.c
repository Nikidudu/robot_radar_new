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

void chassis_can_message_task(void *argument) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_buffer[8];
    uint32_t tx_mailbox;
    TickType_t xLastWakeTime;

    // Configure CAN TX header
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // ===== Send CHASSIS_DATA_1 ===== //

        memset(tx_buffer, 0, 8);
        memcpy(&tx_buffer[0], &chassis_ctrl_data.forward, sizeof(float));
        memcpy(&tx_buffer[4], &chassis_ctrl_data.horizontal, sizeof(float));

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
        memcpy(&tx_buffer[0], &chassis_ctrl_data.yaw, sizeof(float));
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

	tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;


	while (1) {

		memset(tx_buffer, 0, 8);
		memcpy(&tx_buffer[0], &chassis_ctrl_data.forward, sizeof(float));
		memcpy(&tx_buffer[4], &chassis_ctrl_data.horizontal, sizeof(float));

		tx_header.StdId = CHASSIS_HB_ID;

		// Wait for a free mailbox and send
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
			vTaskDelay(1);  // Wait 1ms if all mailboxes are full
		}


		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_HB_PERIOD_MS));
	}
}
