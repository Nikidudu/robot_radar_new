/*
 * dummy_thread.cpp
 *
 *  Created on: Feb 13, 2024
 *      Author: cw
 */

#include <dummy_thread.h>
#include <Telemetry.h>
#include <math.h>
#include "board_lib.h"
#define DEG_TO_RAD (3.142f / 180.0f)

dummyThread* dummyInstance = nullptr;
int count_dummy[6];
int16_t test[6];
float imu_test[6];
float chassis_pitch_angle_deg;
float chassis_pit_rad_and_omega[3] = {0};//[0]rad[1]omega[2]las rad
float chassis_roll_angle_deg;
float chassis_rol_rad_and_omega[3] = {0};//[0]rad[1]omega[2]las rad
uint8_t imu_online_ping[2] = {0};

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

float test_dt = 0;
uint32_t lastTick = HAL_GetTick();
void dummyThread::handle_dummy(uint8_t sender_id, dummyPacket* packet) {
    // Use HAL_GetTick to get current tick count in milliseconds
    // Optional: Output dt for debugging
    // printf("dt: %f seconds\n", dt);

    // Process only reliable packets
    if (!(IS_RELIABLE(*packet))) {
        return;
    }
    uint32_t currentTick = HAL_GetTick();
    imu_online_ping[1] = 0;
        // Calculate dt in seconds (since HAL_GetTick returns milliseconds)
    test_dt = (currentTick - lastTick) / 1000.0f;
    lastTick = currentTick; // update for next call
    chassis_pitch_angle_deg = (float)(packet->num1 / 1000000.0f);
    chassis_roll_angle_deg = (float)(packet->num2 /1000000.0f);
    //count_dummy[2] = packet->num3;
    chassis_pit_rad_and_omega[0] = chassis_pitch_angle_deg * DEG_TO_RAD;
    chassis_pit_rad_and_omega[1] = (chassis_pit_rad_and_omega[0] - chassis_pit_rad_and_omega[2])/test_dt;
    chassis_pit_rad_and_omega[2] = chassis_pit_rad_and_omega[0];
    chassis_rol_rad_and_omega[0] = chassis_roll_angle_deg * DEG_TO_RAD;
    chassis_rol_rad_and_omega[1] = (chassis_rol_rad_and_omega[0] - chassis_rol_rad_and_omega[2])/test_dt;
    chassis_rol_rad_and_omega[2] = chassis_rol_rad_and_omega[0];
    // Unpack and handle signed values
//    test[0] = (int16_t)((packet->num1 >> 16) & 0xFFFF);
//    test[1] = (int16_t)(packet->num1 & 0xFFFF);
//    test[2] = (int16_t)((packet->num2 >> 16) & 0xFFFF);
//    test[3] = (int16_t)(packet->num2 & 0xFFFF);
    test[4] = (int16_t)((packet->num3 >> 16) & 0xFFFF);
    test[5] = (int16_t)(packet->num3 & 0xFFFF);

//    imu_test[0] = (float)test[0] / 1000.0f; // ax
//    imu_test[1] = (float)test[1] / 1000.0f; // ay
//    imu_test[2] = (float)test[2] / 1000.0f; // az
//    imu_test[3] = (float)test[3] / 1000.0f; // gx
    imu_test[4] = (float)test[4] / 1000.0f; // ax
    imu_test[5] = (float)test[5] / 1000.0f; // ay
}


