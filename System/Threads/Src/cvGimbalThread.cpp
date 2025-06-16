/*
 * cvGimbalThread.cpp
 *
 *  Created on: Dec 15, 2024
 *      Author: cw
 */
#ifdef BUILD_WITH_STMUART

#include <cvGimbalThread.h>
#include <Telemetry.h>

#include "robot_config.h"
extern motor_data_t g_can_motors[24];
extern motor_data_t g_pitch_motor;

extern gimbal_control_t gimbal_ctrl_data;
extern uint8_t control_mode;

cvGimbalThread* cvGimbalInstance = nullptr;

cvGimbalThread::~cvGimbalThread(){
}

void cvGimbalThread::init() {}

void cvGimbalThread::loop() {}

void cvGimbalThread::handle_cv_gimbal(uint8_t sender_id, cvGimbalCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		return;
	} else if (control_mode == SBC_CTRL_MODE) {
	    gimbal_ctrl_data.yaw = packet->yaw + g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
	    gimbal_ctrl_data.pitch = packet->pitch;//+ g_pitch_motor.angle_data.adj_ang;
	}
}

#endif
