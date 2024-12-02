/*
 * ProtocolRegisters.h
 *
 *  Created on: 8 May 2020
 *      Author: Arion
 */

#ifndef REGISTER
    #error "This file may only be included by MessageBus.cpp"
#endif

// This file may only be included by MessageBus.cpp (which exports the REGISTER macro)
#include "Protocol.h"

#ifdef PROTOCOL_24
REGISTER(IMUPacket)
REGISTER(gimbalJointsPacket)
REGISTER(chassisJointsPacket)
REGISTER(dummyPacket)
REGISTER(imuPacket)
REGISTER(chassisSpeedCommandPacket)
REGISTER(gimbalAngleCommandPacket)
REGISTER(LeftTriggerPositionPacket)
REGISTER(RightTriggerPositionPacket)
REGISTER(ChassisSpinCommandPacket)
REGISTER(gimbalAngleYawCommandPacket)
REGISTER(gimbalAnglePitchCommandPacket)
REGISTER(FrontFiringPacket)
REGISTER(BackFiringPacket)
REGISTER(RobotStatusPacket)
REGISTER(SideDialPacket)
REGISTER(SuperCapDataPacket)
REGISTER(MaxChassisPowerPacket)
#endif
