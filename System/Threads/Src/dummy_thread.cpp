/*
 * dummy_thread.cpp
 *
 *  Created on: Feb 13, 2024
 *      Author: cw
 */

#include <dummy_thread.h>
#include <Telemetry.h>

dummyThread* dummyInstance = nullptr;
int count_dummy[6];
int16_t test[6];
float imu_test[6];

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
	Telemetry::set_id(OTHER_NODE_ID);

	//CAN1_network->send(&dummy_packet);

	osDelay(1000);

	portYIELD();
}

void dummyThread::handle_dummy(uint8_t sender_id, dummyPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
//		console.printf_error("Unreliable IMU calibration packet");
		return;
	}
	 count_dummy[0] = packet->num1;
	 count_dummy[1] = packet->num2;
	 count_dummy[2] = packet->num3;

	 // Unpack and handle signed values
	 test[0] = (int16_t)((packet->num1 >> 16) & 0xFFFF);// Extract MSB for first number
	 test[1] = (int16_t)(packet->num1 & 0xFFFF);      // Extract LSB for second number
	 test[2] = (int16_t)((packet->num2 >> 16) & 0xFFFF);// Extract MSB for first number
	 test[3] = (int16_t)(packet->num2 & 0xFFFF);
	 test[4] = (int16_t)((packet->num3 >> 16) & 0xFFFF);// Extract MSB for first number
	 test[5] = (int16_t)(packet->num3 & 0xFFFF);                        // Sign-extend to 16-bit
	 imu_test[0] = (float)test[0]/1000.0f; //ax
	 imu_test[1] = (float)test[1]/1000.0f; //ay
	 imu_test[2] = (float)test[2]/1000.0f; //az
	 imu_test[3] = (float)test[3]/1000.0f; //gx
	 imu_test[4] = (float)test[4]/1000.0f; //gy
	 imu_test[5] = (float)test[5]/1000.0f; //gz
}

