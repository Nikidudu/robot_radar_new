/*
 * status_thread.h
 *
 *  Created on: Mar 19, 2025
 *      Author: cw
 */

#ifndef THREADS_INC_STATUS_THREAD_H_
#define THREADS_INC_STATUS_THREAD_H_

#include <stm32f4xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

class statusThread: public Thread {
public:

	statusThread(): Thread("Status") {};
	~statusThread();

	void init();
	void loop();

private:

	uint32_t last_ref_game_state_txno = 0;
	uint32_t last_ref_robot_data_txno = 0;
	uint32_t last_ref_robot_hp_txno = 0;

};

extern statusThread* statusInstance;

#endif /* THREADS_INC_STATUS_THREAD_H_ */
