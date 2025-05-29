/*
 * dummy_cmd_thread.h
 *
 *  Created on: Apr 14, 2025
 *      Author: JL
 */

#ifndef THREADS_INC_DUMMY_CMD_THREAD_H_
#define THREADS_INC_DUMMY_CMD_THREAD_H_

#include <stm32f4xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

class dummyCmdThread : public Thread {
public:

	dummyCmdThread(): Thread("dummyCmd"){};
	~dummyCmdThread();

	void init();
	void loop();

	static void handle_dummy_cmd(uint8_t sender_id, dummyPacket* packet);

};



#endif /* THREADS_INC_DUMMY_CMD_THREAD_H_ */
