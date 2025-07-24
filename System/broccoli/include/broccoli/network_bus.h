/*
 *  NUS Calibur Robotics
 *  network_bus.h
 *
 *  Created on: 6 Dec 2024
 *      Author: Yassine
 *      Edited by : JL
 */

#ifndef NETWORKBUS_H_
#define NETWORKBUS_H_

#include "build.h"


#ifdef BUILD_WITH_NETWORK_BUS


#include "io_bus.h"

#define NETWORK_FRAME_SIZE 256


class NetworkBus : public IOBus {
public:
	NetworkBus(IODriver* driver); // Constructor is inherited

private:
	uint8_t network_frame[NETWORK_FRAME_SIZE];
};


#endif /* BUILD_WITH_NETWORK_BUS */

#endif /* NETWORKBUS_H_ */
