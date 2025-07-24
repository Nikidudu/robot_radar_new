/*
 *  NUS Calibur Robotics
 *  protocol_registers.h
 *
 *  Created on: 2 Nov 2024
 *      Author: Yassine
 *      Edited by : JL
 */

#include "message_bus.h"

#include <cstring>
//#include <string>
#include "stdio.h"

// #ifndef PACKET_REGISTER

// #define PACKET_REGISTER(P)                                                              \
//     extern template bool MessageBus::define<P>(uint8_t);                         \
//     extern template bool MessageBus::send<P>(P*);                                \
//     extern template bool MessageBus::handle<P>(std::function<void(uint8_t, P*)>);

// #endif

// This file may only be included by message_bus.cpp
#include "protocol.h"

#ifdef PROTOCOL_24
PACKET_REGISTER(IMUPacket)
PACKET_REGISTER(gimbalJointsPacket)
PACKET_REGISTER(chassisJointsPacket)
PACKET_REGISTER(dummyPacket)
PACKET_REGISTER(imuPacket)
PACKET_REGISTER(chassisSpeedCommandPacket)
PACKET_REGISTER(gimbalAngleCommandPacket)
PACKET_REGISTER(LeftTriggerPositionPacket)
PACKET_REGISTER(RightTriggerPositionPacket)
PACKET_REGISTER(ChassisSpinCommandPacket)
PACKET_REGISTER(gimbalAngleYawCommandPacket)
PACKET_REGISTER(gimbalAnglePitchCommandPacket)
PACKET_REGISTER(FrontFiringPacket)
PACKET_REGISTER(BackFiringPacket)
PACKET_REGISTER(RobotStatusPacket)
#endif

#ifdef PROTOCOL_25
//REGISTER(IMUPacket)
//REGISTER(gimbalJointsPacket)
//REGISTER(chassisJointsPacket)
PACKET_REGISTER(dummyPacket)
//REGISTER(imuPacket)
PACKET_REGISTER(chassisSpeedCommandPacket)
PACKET_REGISTER(chassisSpinCommandPacket)
PACKET_REGISTER(isNavigatingPacket)
//REGISTER(gimbalAngleCommandPacket)
//REGISTER(LeftTriggerPositionPacket)
//REGISTER(RightTriggerPositionPacket)
//REGISTER(ChassisSpinCommandPacket)
//REGISTER(gimbalAngleYawCommandPacket)
//REGISTER(gimbalAnglePitchCommandPacket)
//REGISTER(FrontFiringPacket)
//REGISTER(BackFiringPacket)
//REGISTER(RobotStatusPacket)
//REGISTER(SideDialPacket)
//PACKET_REGISTER(SuperCapDataPacket)
//PACKET_REGISTER(MaxChassisPowerPacket)

// Aimbot
PACKET_REGISTER(competitionStatusPacket)
//PACKET_REGISTER(occupationStatusPacket)
//PACKET_REGISTER(winStatusPacket)
PACKET_REGISTER(leftTriggerPositionPacket)

PACKET_REGISTER(cvGimbalCommandPacket)
PACKET_REGISTER(firingCommandPacket)
PACKET_REGISTER(aimCommandPacket)
#endif
