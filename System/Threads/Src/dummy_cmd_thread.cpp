/*
 * dummy_cmd_thread.cpp
 *
 *  Created on: Apr 14, 2025
 *      Author: JL
 */

#include <dummy_cmd_thread.h>
#include <Telemetry.h>

static uint8_t test_nums[3];

dummyCmdThread* dummyCmdInstance = nullptr;

dummyCmdThread::~dummyCmdThread(){
}

void dummyCmdThread::init() {
	dummyCmdInstance = this;
}

void dummyCmdThread::loop() {
//	test_num[0] = packet->num1;
//	test_num[1] = packet->num2;
//	test_num[2] = packet->num3;

	osDelay(1);
	portYIELD();
}

void dummyCmdThread::handle_dummy_cmd(uint8_t sender_id, dummyPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
			return;
		}
	test_nums[0] = packet->num1;
	test_nums[1] = packet->num2;
	test_nums[2] = packet->num3;
}
