/*
 * ImuThread.h
 *
 *  Created on: Mar 20, 2025
 *      Author: YI MING
 */

#ifndef THREADS_INC_IMUTHREAD_H_
#define THREADS_INC_IMUTHREAD_H_

#include <stm32f4xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

struct to_imu_packet {
	int8_t test;

} __attribute__((packed));


struct from_imu_packet {
	int16_t pit;
	int16_t rol;
	int8_t ax;
} __attribute__((packed));


class ImuCommThread : public Thread {
public:

	ImuCommThread(): Thread("SuperCapComm"), pit(0), rol(0), ax(0) {};
	~ImuCommThread();

	void init();
	void loop();

	static void handle_imu(uint8_t sender_id, IMUDataPacket* packet);

private:

	float pit;
	float rol;
	float ax;

	void txHeaderConfig();

	CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;

	from_imu_packet txMsg;
	to_imu_packet rxMsg;

};

void ImuISR(uint8_t* rxdata);

extern ImuCommThread* ImuCommInstance;

#endif /* THREADS_INC_IMUTHREAD_H_ */
