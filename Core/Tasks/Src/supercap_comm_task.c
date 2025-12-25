/*
 * supercap_comm_task.c
 *
 *  Created on: Jul 16, 2025
 *      Author: gskan
 */

#include "board_lib.h"
#include "supercap_comm_task.h"
#include "robot_config.h"
#include "can_msg_processor.h"

#include "motor_config.h"

CAN_TxHeaderTypeDef TxHeader;

float chassis_power;
uint8_t charging_state;
uint32_t supercap_last_receive_time = 0;
int supercap_enabled = 1;

#ifdef SUPERCAP_PRESENT

void supercap_comm_task(void *argument) {
	 uint8_t enable_supercap_module = 1;
	 uint8_t reset_supercap_module = 0;
	 extern ref_game_robot_data2_t ref_robot_data;
	 extern ref_game_state_t ref_game_state;

	 CAN_TxHeaderTypeDef TxHeader;
	 ref_msg_packet txMsg;

	 txHeaderConfig(&TxHeader);

	 while (1) {
		 // Disable use of supercap if charge falls below threshold
		 // DOES NOT DISABLE THE SUPERCAP
		 if (charging_state < SUPERCAP_DISABLE_THRESHOLD) {
			 supercap_enabled = 0;
		 } else if (!supercap_enabled && charging_state > SUPERCAP_DISABLE_THRESHOLD) {
			 supercap_enabled = 1;
		 }

		if ((HAL_GetTick() - supercap_last_receive_time > SUPERCAP_TIMEOUT)) {
			reset_supercap_module = 1;
		}

		 txMsg.enable_module = enable_supercap_module;
		 txMsg.reset = 0;
		 if (reset_supercap_module)
			 reset_supercap_module = 0;
		 txMsg.pow_limit = ref_robot_data.chassis_power_limit + SUPER_CAP_OFFSET; // -n cuz supercap not very accurate
		 txMsg.energy_buffer = 100; // hmm where this 100 came from
		 uint32_t TxMailbox;  // Declare TxMailbox here

		 // Transmit data
		 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
			 // Optionally add a timeout here to prevent infinite loop
		 }

		 if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, (uint8_t *)&txMsg, &TxMailbox) != HAL_OK) {
			 uint8_t i = 1;
		 }

		 if(ref_robot_data.current_HP <= 0 || ref_game_state.game_progress == 5)
			 enable_supercap_module = 0;
		 else
			 enable_supercap_module = 1;

		 osDelay(100);

		 portYIELD();
	 }
}

void txHeaderConfig(CAN_TxHeaderTypeDef* TxHeader) {
	 TxHeader->StdId = SUPERCAP_NODE_ID;
	 TxHeader->ExtId = 0;
	 TxHeader->RTR = CAN_RTR_DATA;
	 TxHeader->IDE = CAN_ID_STD;
	 TxHeader->DLC = 5;
	 TxHeader->TransmitGlobalTime = DISABLE;
 }

 void supercapISR(uint8_t* rxdata){
//	 supercap_msg_packet *supercap_packet = (supercap_msg_packet*)rxdata;
	 supercap_msg_packet *supercap_packet;
	 memcpy(&supercap_packet, rxdata, sizeof(supercap_packet));
	 chassis_power = supercap_packet->chassis_power;
	 charging_state = supercap_packet->cap_energy * 100 / 255;
	 supercap_last_receive_time = HAL_GetTick();
 }

#endif

