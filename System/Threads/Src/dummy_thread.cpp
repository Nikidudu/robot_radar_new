/*
 * dummy_thread.cpp
 *
 *  Created on: Feb 13, 2024
 *      Author: cw
 */

#include <dummy_thread.h>
#include <Telemetry.h>

dummyThread* dummyInstance = nullptr;
int count_dummy = 0;

dummyThread::~dummyThread(){
}

void dummyThread::init(){
	int j = 1;
}

// Declare your data with the proper data structure defined in DataStructures.h
static dummyData dummy_data;


// Declare the RoCo packet with the proper data structure defined in RoCo/Src/Protocol/Protocol24
static dummyPacket dummy_packet;
static int i = 0;

void dummyThread::loop()
{
	++i;
	dummy_data.num[0] = i;
	dummy_data.num[1] = i*2;
	dummy_data.num[2] = i*10;


	dummy_data.toArray((uint8_t*) &dummy_packet);

	MAKE_IDENTIFIABLE(dummy_packet);
	MAKE_RELIABLE(dummy_packet);
//	Telemetry::set_id(OTHER_NODE_ID);

	UART_network->send(&dummy_packet);
//	CAN1_network->send(&dummy_packet);

	osDelay(10);

	portYIELD();
}

void dummyThread::handle_dummy(uint8_t sender_id, dummyPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
//		console.printf_error("Unreliable IMU calibration packet");
		return;
	}
	count_dummy = packet->num2;


}
