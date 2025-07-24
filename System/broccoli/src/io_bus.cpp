/*
 *  NUS Calibur Robotics
 *  io_bus.cpp
 *
 *  Created on: 6 Dec 2024
 *      Author: Yassine
 *      Edited by : JL
 */

#include "io_bus.h"

#include <cstring>
#include <algorithm>

IOBus::IOBus(IODriver* driver, uint8_t* buffer, uint32_t length) {
	this->driver = driver;
	this->packet_buffer = buffer;
	this->buffer_length = length;
	this->buffer_index = 0;

	using namespace std::placeholders;
	driver->receive(std::bind(&IOBus::receive, this, _1, _2, _3));
}

void IOBus::receive(uint8_t sender_id, uint8_t* buffer, uint32_t length) {
	while(length > buffer_length) {
		MessageBus::receive(sender_id, buffer, buffer_length);
		length -= buffer_length;
		buffer += buffer_length;
	}

	MessageBus::receive(sender_id, buffer, length);
}

IODriver* IOBus::get_driver() {
	return driver;
}

uint32_t IOBus::append(uint8_t* buffer, uint32_t length) {
	uint32_t remaining_length = buffer_length - buffer_index;

	if(length > remaining_length) {
		length = remaining_length;
	}

	memcpy(packet_buffer + buffer_index, buffer, length);

	buffer_index += length;

	return length;
}

void IOBus::transmit() {
	driver->transmit(packet_buffer, buffer_index);
	buffer_index = 0;
}