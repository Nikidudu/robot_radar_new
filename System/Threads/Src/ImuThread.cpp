/*
 * dummy_thread.cpp
 *
 *  Created on: Feb 13, 2024
 *      Author: Yassine
 */

#include <ImuThread.h>
//#include <supercap_def.h>
#include <Telemetry.h>
#include "referee_msgs.h"


ImuCommThread* ImuCommInstance = nullptr;

float pitch_deg;
float roll_deg;
float imu_ax;
//extern ref_game_robot_data2_t ref_robot_data;
//extern ref_game_state_t ref_game_state;

ImuCommThread::~ImuCommThread(){
}

void ImuCommThread::init(){
	txHeaderConfig();
}

void ImuCommThread::loop()
{
	txMsg.pit = 1.2345f;
	txMsg.rol = 2.3456f;
//	txMsg.pow_limit = ref_robot_data.chassis_power_limit;
//	txMsg.energy_buffer = 100;
    uint32_t TxMailbox;  // Declare TxMailbox here

    // Transmit data
//	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
//		// Optionally add a timeout here to prevent infinite loop
//	}
//
//	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, (uint8_t *)&txMsg, &TxMailbox) != HAL_OK)
//		int i = 1;

//	if(ref_robot_data.current_HP <= 0 || ref_game_state.game_progress == 5)
//		enable_supercap_module = false;
//	else
//		enable_supercap_module = true;
	osDelay(1000);

	portYIELD();
}

void ImuCommThread::txHeaderConfig(){
    TxHeader.StdId = CURRENT_NODE_ID;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 5;
    TxHeader.TransmitGlobalTime = DISABLE;
}

//float test_dt = 0;
//uint32_t lastTick = HAL_GetTick();
void ImuISR(uint8_t* rxdata){
	uint32_t currentTick = HAL_GetTick();
	// Calculate dt in seconds (since HAL_GetTick returns milliseconds)
//	test_dt = (currentTick - lastTick) / 1000.0f;
//	lastTick = currentTick; // update for next call
	from_imu_packet *imu_packet = (struct from_imu_packet*)rxdata;
	uint8_t i = 0;
	pitch_deg = (float)(imu_packet->pit/1000.0f);
	roll_deg = (float)(imu_packet->rol/1000.0f);
	imu_ax = (float)(imu_packet->ax/100.0f);
}

