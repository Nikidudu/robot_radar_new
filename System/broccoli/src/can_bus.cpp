/*
 *  NUS Calibur Robotics
 *  can_bus.h
 *
 *  Created on: 20 Dec 2023
 *      Author: Yassine
 *      Edited by : JL
 */

#include "build.h"

#ifdef BUILD_WITH_CAN_BUS

#include "can_bus.h"
#include "protocol.h"

CANBus::CANBus(IODriver* driver) : IOBus(driver, can_frame, sizeof(can_frame)) {

	// Sentry

	define<IMUPacket>(1);
	define<ChassisJointsPacket>(2);
	define<GimbalJointsPacket>(3);
	define<DummyPacket>(4);
	define<ImuPacket>(5);
	define<ChassisSpeedCommandPacket>(6);
	define<GimbalAngleCommandPacket>(7);
	define<LeftTriggerPositionPacket>(8);
	define<RightTriggerPositionPacket>(9);
	define<ChassisSpinCommandPacket>(10);
	define<FrontFiringPacket>(11);
	define<BackFiringPacket>(12);
	define<RobotStatusPacket>(13);
    define<GimbalAngleYawCommandPacket>(14);
	define<GimbalAnglePitchCommandPacket>(15);
}

#endif /* BUILD_WITH_CAN_BUS */
