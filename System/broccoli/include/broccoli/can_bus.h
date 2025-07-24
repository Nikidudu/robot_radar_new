/*
 *  NUS Calibur Robotics
 *  can_bus.h
 *
 *  Created on: 20 Dec 2023
 *      Author: Yassine
 *      Edited by : JL
 */

#ifndef CANBUS_H_
#define CANBUS_H_

#include "build.h"

#ifdef BUILD_WITH_CAN_BUS

#include "io_bus.h"

#define CAN_FRAME_SIZE 64


class CANBus : public IOBus {
public:
	CANBus(IODriver* driver); // Constructor is inherited

private:
	uint8_t can_frame[CAN_FRAME_SIZE];
};


#endif /* BUILD_WITH_CAN_BUS */

#endif /* CANBUS_H_ */
