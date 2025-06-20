/*
 * spd_cmd_thread.cpp
 *
 *  Created on: May 16, 2025
 *      Author: Yassine, JL
 */

#include <spd_cmd_thread.h>
#include <Telemetry.h>
#include "typedefs.h"
#include "robot_config.h"
#include <cmath>
#include <algorithm>

extern remote_cmd_t g_remote_cmd;
extern gimbal_control_t	gimbal_ctrl_data;
extern chassis_control_t chassis_ctrl_data;
extern gun_control_t launcher_ctrl_data;
extern uint8_t control_mode;

extern int g_spinspin_mode;
//extern uint8_t set_launcher;
static float test_vals[3];

double global_dt = 0.0;


ChassisSpdCmdThread* chassisSpeedInstance = nullptr;

ChassisSpdCmdThread::~ChassisSpdCmdThread(){
}

void ChassisSpdCmdThread::init(){
	chassisSpeedInstance = this;

	curr_receive_time = 0;
	last_receive_time = 0;

	V_horz = 0;
	V_lat = 0;
	V_yaw = 0;

	beyblade_mode = false;

	gimbal_pitch = 0;
	gimbal_yaw = 0;
	gimbal_ctrl_data.enabled = 1;
	chassis_ctrl_data.enabled = 1;

	front_or_back = 1;


	fire_front_launcher = 0;
	fire_back_launcher = 0;

	gimbal_ctrl_data.imu_mode = true;
	alignChassisAndGimbal = false;
	manual_mode = true; // true for manual control, false for auto (startup) control
}

void ChassisSpdCmdThread::loop() {
//	if (g_remote_cmd.left_switch == ge_RSW_SHUTDOWN && manual_mode){
	if (control_mode == SBC_CTRL_MODE) {
		if (g_remote_cmd.right_switch != ge_RSW_ALL_ON) { // Safety kill
//			manual_mode = false;
			V_horz = 0;
			V_lat = 0;
			V_yaw = 0;
			beyblade_mode = false;
			gimbal_pitch = 0;
			gimbal_yaw = 0;
			front_or_back = 1;
			fire_front_launcher = 0;
			fire_back_launcher = 0;
			control_reset();
//	} else if (g_remote_cmd.left_switch != ge_RSW_SHUTDOWN && !manual_mode){
		}

		else {
			if (!beyblade_mode)
				V_yaw = chassis_center_yaw();
			else
				V_yaw = beyblade_mode;

			chassis_set_ctrl(V_horz, V_lat, V_yaw);

			double dt = (curr_receive_time - last_receive_time) / 1000.0;
			gimbal_ctrl_data.delta_yaw = gimbal_yaw * dt * 4.0;
			gimbal_ctrl_data.delta_yaw = fmaxf(-1.0, fminf(gimbal_ctrl_data.delta_yaw, 1.0));
//			gimbal_set_yaw_speed(gimbal_yaw);
//			gimbal_set_pitch_speed(gimbal_pitch);
//			set_launcher = front_or_back;
//			launcher_ctrl_data.firing = fire_front_launcher;
		}
	}

	osDelay(1);
	portYIELD();
//	else {
//		V_horz = 0;
//		V_lat = 0;
//		V_yaw = 0;
//		beyblade_mode = false;
//		gimbal_pitch = 0;
//		gimbal_yaw = 0;
//		front_or_back = 1;
//		fire_front_launcher = 0;
//		fire_back_launcher = 0;
//		control_reset();
//	}
}


void ChassisSpdCmdThread::handle_chassis_spd_commands(uint8_t sender_id, chassisSpeedCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		scanf("Unreliable chassis command packet");
		return;
	}
	if (chassisSpeedInstance == nullptr) {
		scanf("Speed chassis command thread instance does not exist yet\r\n");
		return;
	}
	else{
//		scanf("Received chassis speed command");
		chassisSpeedInstance->send_commands_chassis(packet);
	}
}

//void ChassisSpdCmdThread::handle_gimbal_spd_commands(uint8_t sender_id, gimbalAngleCommandPacket* packet) {
//	if(!(IS_RELIABLE(*packet))) {
//		scanf("Unreliable gimbal command packet");
//		return;
//	}
//	if (chassisSpeedInstance == nullptr) {
//		scanf("Gimbal command thread instance does not exist yet\r\n");
//		return;
//	}
//	else{
////		scanf("Received chassis speed command");
//		chassisSpeedInstance->send_commands_gimbal(packet);
//	}
//}
//
//void ChassisSpdCmdThread::handle_launcher_front_firing_commands(uint8_t sender_id, FrontFiringPacket* packet) {
//	if(!(IS_RELIABLE(*packet))) {
//		scanf("Unreliable gimbal command packet");
//		return;
//	}
//	if (chassisSpeedInstance == nullptr) {
//		scanf("Gimbal command thread instance does not exist yet\r\n");
//		return;
//	}
//	else{
////		scanf("Received chassis speed command");
//		chassisSpeedInstance->send_commands_launcher_front(packet);
//	}
//}
//
//void ChassisSpdCmdThread::handle_launcher_back_firing_commands(uint8_t sender_id, BackFiringPacket* packet) {
//	if(!(IS_RELIABLE(*packet))) {
//		scanf("Unreliable gimbal command packet");
//		return;
//	}
//	if (chassisSpeedInstance == nullptr) {
//		scanf("Gimbal command thread instance does not exist yet\r\n");
//		return;
//	}
//	else{
////		scanf("Received chassis speed command");
//		chassisSpeedInstance->send_commands_launcher_back(packet);
//	}
//}
//
//void ChassisSpdCmdThread::handle_chassis_spin_command(uint8_t sender_id, ChassisSpinCommandPacket* packet) {
//	if(!(IS_RELIABLE(*packet))) {
//		scanf("Unreliable gimbal command packet");
//		return;
//	}
//	if (chassisSpeedInstance == nullptr) {
//		scanf("Gimbal command thread instance does not exist yet\r\n");
//		return;
//	}
//	else{
////		scanf("Received chassis speed command");
//		chassisSpeedInstance->send_commands_chassis_spin(packet);
//	}
//}
//
//void ChassisSpdCmdThread::handle_gimbal_yaw_command(uint8_t sender_id, gimbalAngleYawCommandPacket* packet) {
//	if(!(IS_RELIABLE(*packet))) {
//		scanf("Unreliable gimbal command packet");
//		return;
//	}
//	if (chassisSpeedInstance == nullptr) {
//		scanf("Gimbal command thread instance does not exist yet\r\n");
//		return;
//	}
//	else{
////		scanf("Received chassis speed command");
//		chassisSpeedInstance->send_commands_yaw_gimbal(packet);
//	}
//}
//
//void ChassisSpdCmdThread::handle_gimbal_pitch_command(uint8_t sender_id, gimbalAnglePitchCommandPacket* packet){
//	if(!(IS_RELIABLE(*packet))) {
//		scanf("Unreliable gimbal command packet");
//		return;
//	}
//	if (chassisSpeedInstance == nullptr) {
//		scanf("Gimbal command thread instance does not exist yet\r\n");
//		return;
//	}
//	else{
////		scanf("Received chassis speed command");
//		chassisSpeedInstance->send_commands_pitch_gimbal(packet);
//	}
//}

void ChassisSpdCmdThread::send_commands_chassis(chassisSpeedCommandPacket* packet){
	if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
//		V_horz = 0;
//		V_lat = 0;
//		gimbal_yaw = 0;
//		control_reset();
	} else {
//		V_horz = -0.7071 * (packet->V_horz) + 0.7071 * (packet->V_lat);
//		V_lat = -0.7071 + (packet->V_horz) - 0.7071 * (packet->V_lat);
		V_horz = packet->V_horz;
		V_lat = packet->V_lat;
		gimbal_yaw = packet->V_yaw;

		last_receive_time = curr_receive_time;
		curr_receive_time = HAL_GetTick();

		test_vals[0] = V_horz;
		test_vals[1] = V_lat;
		test_vals[2] = gimbal_yaw;
		portYIELD();
	}
}

//void ChassisSpdCmdThread::send_commands_gimbal(gimbalAngleCommandPacket* packet){
//	if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
////		gimbal_pitch = 0;
////		gimbal_yaw = 0;
////		chassis_ctrl_data.forward = 0;
////		chassis_ctrl_data.horizontal = 0;
////		chassis_ctrl_data.yaw = 0;
////		chassis_ctrl_data.enabled = 0;
////		gimbal_ctrl_data.pitch = gimbal_pitch;
////		gimbal_ctrl_data.pitch_speed = gimbal_pitch;
////		gimbal_ctrl_data.yaw = imu_heading.yaw;
////		gimbal_ctrl_data.yaw_speed = gimbal_yaw;
////		gimbal_ctrl_data.enabled = 0;
////		launcher_ctrl_data.firing = 0;
////		launcher_ctrl_data.projectile_speed = 0;
////		launcher_ctrl_data.enabled = 0;
////		g_spinspin_mode = 0;
//	} else {
//		gimbal_pitch = packet->pitch;
//		gimbal_yaw = packet->yaw;
//		portYIELD();
//	}
//}
//
//void ChassisSpdCmdThread::send_commands_pitch_gimbal(gimbalAnglePitchCommandPacket* packet){
//	if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
////		gimbal_pitch = 0;
////		control_reset();
//	} else {
//		gimbal_pitch = packet->pitch;
//		portYIELD();
//	}
//}
//
//void ChassisSpdCmdThread::send_commands_yaw_gimbal(gimbalAngleYawCommandPacket* packet){
//	if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
////		gimbal_pitch = 0;
////		gimbal_yaw = 0;
////		control_reset();
//	} else {
//		gimbal_yaw = packet->yaw;
//		portYIELD();
//	}
//}
//
//void ChassisSpdCmdThread::send_commands_launcher_front(FrontFiringPacket* packet){
//	if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
////		fire_front_launcher = 0;
////		control_reset();
//	} else {
////		if (front_or_back == 0)
////			launcher_ctrl_data.enabled = 0;
//		launcher_ctrl_data.projectile_speed = 1;
//		launcher_ctrl_data.enabled = 1;
//		fire_front_launcher = packet->fire_state;
//		front_or_back = 1;
//		portYIELD();
//	}
//}
//
//void ChassisSpdCmdThread::send_commands_launcher_back(BackFiringPacket* packet){
//	if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
////		fire_front_launcher = 0;
////		control_reset();
//	} else {
//		if (front_or_back == 1)
//			launcher_ctrl_data.enabled = 0;
//		launcher_ctrl_data.projectile_speed = 1;
//		launcher_ctrl_data.enabled = 1;
//		fire_front_launcher = packet->fire_state;
//		front_or_back = 0;
//		portYIELD();
//	}
//}
//
//void ChassisSpdCmdThread::send_commands_chassis_spin(ChassisSpinCommandPacket* packet){
//	if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
////		beyblade_mode = 0;
////		control_reset();
//	} else {
//		beyblade_mode = packet->spining_state;
//		portYIELD();
//	}
//}

void ChassisSpdCmdThread::set_spinspin(bool beyblade_mode) {
	this->beyblade_mode = beyblade_mode;
}
