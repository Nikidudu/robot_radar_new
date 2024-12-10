/*
 * dummy_thread.cpp
 *
 *  Created on: Feb 13, 2024
 *      Author: Yassine
 */

#include <SuperCapCommThread.h>
//#include <supercap_def.h>
#include <Telemetry.h>
#include "referee_msgs.h"


SuperCapCommThread* SuperCapCommInstance = nullptr;

float chassis_power;
float cap_voltage;
uint8_t charging_state;
extern ref_game_robot_data2_t ref_robot_data;
SuperCapCommThread::~SuperCapCommThread(){
}

void SuperCapCommThread::init(){
	;;
}

// Declare your data with the proper data structure defined in DataStructures.h
static MaxChassisPowerData chassis_power_data;


// Declare the RoCo packet with the proper data structure defined in RoCo/Src/Protocol/Protocol24
static MaxChassisPowerPacket chassis_power_packet;
static int i = 0;

void SuperCapCommThread::loop()
{
	chassis_power_data.maxChassisPower = ref_robot_data.chassis_power_limit;

	chassis_power_data.toArray((uint8_t*) &chassis_power_packet);

	MAKE_IDENTIFIABLE(chassis_power_packet);
	MAKE_RELIABLE(chassis_power_packet);
	Telemetry::set_id(OTHER_NODE_ID);

//	CAN1_network->send(&chassis_power_packet);

	osDelay(100);

	portYIELD();
}

void SuperCapCommThread::handle_supercap(uint8_t sender_id, SuperCapDataPacket* packet){
	if(!(IS_RELIABLE(*packet))) {
//		console.printf_error("Unreliable IMU calibration packet");
		return;
	}
	cap_voltage = packet->V_cap;
	chassis_power = packet->P_chassis;
	charging_state = packet->charge_state;
}

