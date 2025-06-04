/*
 * cvAimbotCommandThread.c
 *
 *  Created on: Mar 20, 2025
 *      Author: cw
 */

#include <cvAimbotCommandThread.h>
#include <Telemetry.h>
#include "robot_config.h"
#include "INS_task.h"
#include "bsp_damiao.h"
extern motor_data_t g_can_motors[24];
extern motor_data_t g_pitch_motor;

extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

extern INS_t INS;
extern orientation_data_t imu_heading;

extern gimbal_control_t gimbal_ctrl_data;
extern uint8_t control_mode;

extern gun_control_t launcher_ctrl_data;

cvAimbotCommandThread* cvAimbotCommandInstance = nullptr;

cvAimbotCommandThread::~cvAimbotCommandThread(){
}

void cvAimbotCommandThread::init() {
	cvAimbotCommandInstance = this;
	yaw = 0.0f;
	pitch = 0.0f;
	aim_state = true;
	fire_state = false;

	// Optionally: initialize UART telemetry
}

void cvAimbotCommandThread::loop() {
	if (control_mode == SBC_CTRL_MODE) {
		if (aim_state) {
			gimbal_ctrl_data.delta_yaw = yaw*2; // + g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
			gimbal_ctrl_data.pitch = pitch + INS.Pitch;
//			gimbal_ctrl_data.pitch = imu_heading.pit;
			launcher_ctrl_data.firing = fire_state;
		} else {
			gimbal_ctrl_data.pitch = 0;
			launcher_ctrl_data.firing = 0;
			yaw = g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
		}
		// SEND AIM STATE TO MINI PC
	}

	osDelay(2);
	portYIELD();
}

void cvAimbotCommandThread::handle_cv_gimbal(uint8_t sender_id, cvGimbalCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		return;
	}
	if (cvAimbotCommandInstance == nullptr) {
		scanf("cv aimbot command thread instance does not exist yet\r\n");
		return;
	}

	cvAimbotCommandInstance->send_command_gimbal(packet);
}

void cvAimbotCommandThread::send_command_gimbal(cvGimbalCommandPacket* packet) {
	yaw = YAW_INVERT * packet->yaw;
	pitch = PITCH_INVERT * packet->pitch;
}

void cvAimbotCommandThread::handle_cv_firing(uint8_t sender_id, cvFiringCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		return;
	}
	if (cvAimbotCommandInstance == nullptr) {
		scanf("cv aimbot command thread instance does not exist yet\r\n");
		return;
	}

	cvAimbotCommandInstance->send_command_firing(packet);
}

void cvAimbotCommandThread::send_command_firing(cvFiringCommandPacket* packet) {
	fire_state = packet->fire_state;
}

void cvAimbotCommandThread::handle_cv_aim(uint8_t sender_id, cvAimCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		return;
	}
	if (cvAimbotCommandInstance == nullptr) {
		scanf("cv aimbot command thread instance does not exist yet\r\n");
		return;
	}

	cvAimbotCommandInstance->send_command_aim(packet);
}
//
void cvAimbotCommandThread::send_command_aim(cvAimCommandPacket* packet) {
	aim_state = packet->aim_state;
}
