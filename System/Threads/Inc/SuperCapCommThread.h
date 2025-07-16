///*
// * dummy_thread.h
// *
// *  Created on: Feb 13, 2024
// *      Author: Yassine
// */
//
//#ifndef THREADS_INC_SUPERCAP_COMM_THREAD_H_
//#define THREADS_INC_SUPERCAP_COMM_THREAD_H_
//
//#include <stm32f4xx_hal.h>
//#include <main.h>
//#include <Thread.h>
//#include "DataStructures.h"
//
//#include "Telemetry.h"
//
//struct ref_msg_packet {
//    uint8_t enable_module;	//enable once and leave it (regulation on or off)
//    uint8_t reset;			//reset in case got error eg cap voltage too low, UVLO active
//    uint8_t pow_limit;		//set power regulation point, ie set to current level power
//    uint16_t energy_buffer;	//send over refsys "virtual energy buffer" to abuse
//} __attribute__((packed));
//
//
//struct supercap_msg_packet {
//	float chassis_power;	//originally meant for feedback,  but not really relevant now, use it however you want eg if exceed too long and sc is dead kill motors for a while??
//	uint8_t error;			//any error state
//	uint8_t cap_energy;		//normalized energy left in supercap (impt one)
//} __attribute__((packed));
//
//class SuperCapCommThread : public Thread {
//public:
//
//	SuperCapCommThread(): Thread("SuperCapComm"), V_cap(0), P_chassis(0), charge_state(0) {};
//	~SuperCapCommThread();
//
//	void init();
//	void loop();
//
//	static void handle_supercap(uint8_t sender_id, SuperCapDataPacket* packet);
//
//private:
//
//	float V_cap;
//	float P_chassis;
//	uint8_t charge_state;
//
//	void txHeaderConfig();
//
//	CAN_TxHeaderTypeDef TxHeader;
//	CAN_RxHeaderTypeDef RxHeader;
//
//	ref_msg_packet txMsg;
//	supercap_msg_packet rxMsg;
//};
//
//void supercapISR(uint8_t* rxdata);
//
//extern SuperCapCommThread* SuperCapCommInstance;
//
//#endif /* THREADS_INC_DUMMY_THREAD_H_ */
