/*
 * dummy_cmd_thread.cpp
 *
 *  Created on: Apr 14, 2025
 *      Author: JL
 */

#include <dummy_cmd_thread.h>
#include <Telemetry.h>

static uint8_t test_nums[3];

#define AUTO_SPIN				false
#define AUTO_SPINSPIN_TIMEOUT 	1000 // in ms

extern int g_spinspin_mode;
dummyCmdThread* dummyCmdInstance = nullptr;

dummyCmdThread::~dummyCmdThread(){
}

void dummyCmdThread::init() {
	dummyCmdInstance = this;
	last_receive_time = 0;
}

void dummyCmdThread::loop() {
//	test_num[0] = packet->num1;
//	test_num[1] = packet->num2;
//	test_num[2] = packet->num3;

#if AUTO_SPIN
	/*
	Competition State
	------------------------------
	0: pre-competition period
	1: set-up period
	2: 15-second Referee System initialization period
	3: 5-second countdown
	4: in competition
	5: competition result calculation
	*/

	// To add competition state check later on
	if ((HAL_GetTick() - last_receive_time > AUTO_SPINSPIN_TIMEOUT)) {
		g_spinspin_mode = 1;
	}

#endif

	osDelay(1);
	portYIELD();
}

void dummyCmdThread::handle_dummy_cmd(uint8_t sender_id, dummyPacket* packet) {
	if (!(IS_RELIABLE(*packet))) {
			return;
	}

	test_nums[0] = packet->num1;
	test_nums[1] = packet->num2;
	test_nums[2] = packet->num3;

	dummyCmdInstance->last_receive_time = HAL_GetTick();
}
