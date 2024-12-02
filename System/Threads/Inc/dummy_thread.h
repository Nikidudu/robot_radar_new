/*
 * dummy_thread.h
 *
 *  Created on: Feb 13, 2024
 *      Author: cw
 */

#ifndef THREADS_INC_DUMMY_THREAD_H_
#define THREADS_INC_DUMMY_THREAD_H_

#include <stm32f4xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

class dummyThread : public Thread {
public:

	dummyThread(): Thread("dummy"), var1(0), var2(0), var3(0) {};
	~dummyThread();

	void init();
	void loop();

	static void handle_dummy(uint8_t sender_id, dummyPacket* packet);


private:

	int var1;
	int var2;
	int var3;



};


extern dummyThread* dummyInstance;

#endif /* THREADS_INC_DUMMY_THREAD_H_ */
