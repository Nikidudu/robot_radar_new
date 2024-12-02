/*
 * dummy_thread.h
 *
 *  Created on: Feb 13, 2024
 *      Author: Yassine
 */

#ifndef THREADS_INC_SUPERCAP_COMM_THREAD_H_
#define THREADS_INC_SUPERCAP_COMM_THREAD_H_

#include <stm32f4xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

class SuperCapCommThread : public Thread {
public:

	SuperCapCommThread(): Thread("SuperCapComm"), V_cap(0), P_chassis(0), charge_state(0) {};
	~SuperCapCommThread();

	void init();
	void loop();

	static void handle_supercap(uint8_t sender_id, SuperCapDataPacket* packet);

private:

	float V_cap;
	float P_chassis;
	uint8_t charge_state;

};


extern SuperCapCommThread* SuperCapCommInstance;

#endif /* THREADS_INC_DUMMY_THREAD_H_ */
