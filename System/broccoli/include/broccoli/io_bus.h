/*
 *  NUS Calibur Robotics
 *  io_bus.h
 *
 *  Created on: 6 Dec 2024
 *      Author: Yassine
 *      Edited by : JL
 */

#ifndef IOBUS_H_
#define IOBUS_H_

#include "message_bus.h"
#include "io_driver.h"

class IOBus : public MessageBus {
public:
	IOBus(IODriver* driver, uint8_t* buffer, uint32_t length);
	IODriver* get_driver();
protected:
	void transmit();

private:
	IODriver* driver;
	uint8_t* packet_buffer;
	uint32_t buffer_length;
	uint32_t buffer_index;

	void receive(uint8_t sender_id, uint8_t* buffer, uint32_t length);
	uint32_t append(uint8_t* buffer, uint32_t length);
};

#endif /* IOBUS_H_ */