/*
 *  NUS Calibur Robotics
 *  protocol.h
 *
 *  Created on: 2 Nov 2024
 *      Author: JL
 *      Edited by : ______
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_

#include "../build/build.h"


#ifdef PROTOCOL_24
#include "protocol_calibur24.h"
#endif

#ifdef PROTOCOL_25
#include "protocol_calibur25.h"
#endif

#endif /* PROTOCOL_PROTOCOL_H_ */