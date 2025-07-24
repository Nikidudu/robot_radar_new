/*
 *  NUS Calibur Robotics
 *  io_driver.h
 *
 *  Created on: 6 Dec 2024
 *      Author: Yassine
 *      Edited by : JL
 */

#ifndef IODRIVER_H_
#define IODRIVER_H_

#include <cstdint>
#include <functional>


class IODriver {
public:
	virtual ~IODriver() {}
	virtual void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) = 0;
	virtual void transmit(uint8_t* buffer, uint32_t length) = 0;
//	virtual void transmit(uint8_t* buffer, uint32_t length, uint8_t id) = 0;
};


#endif /* IODRIVER_H_ */
