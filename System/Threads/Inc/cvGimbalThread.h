/*
 * cvGimbalThread.h
 *
 *  Created on: Dec 15, 2024
 *      Author: cw
 */

#ifndef THREADS_INC_CVGIMBALTHREAD_H_
#define THREADS_INC_CVGIMBALTHREAD_H_

#include <stm32f4xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

class cvGimbalThread : public Thread {
public:

	cvGimbalThread(): Thread("cvGimbal"), pitch(0), yaw(0) {};
	~cvGimbalThread();

	void init();
	void loop();

	static void handle_cv_gimbal(uint8_t sender_id, cvGimbalCommandPacket* packet);


private:

	int pitch;
	int yaw;

};


extern cvGimbalThread* cvGimbalInstance;

#endif /* THREADS_INC_CVGIMBALTHREAD_H_ */
