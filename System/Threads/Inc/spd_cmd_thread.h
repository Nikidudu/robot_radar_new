/*
 * spd_cmd_thread.h
 *
 *  Created on: May 16, 2025
 *      Author: Yassine, JL
 */

#ifndef THREADS_INC_SPD_CMD_THREAD_H_
#define THREADS_INC_SPD_CMD_THREAD_H_


#include <stm32f4xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"
#include "Telemetry.h"
#include "control_input_task.h"
#include "Protocol.h"


class ChassisSpdCmdThread : public Thread {
public:


	ChassisSpdCmdThread(): Thread("chassis_speed_command") {};
	~ChassisSpdCmdThread();


	static void handle_chassis_spd_commands(uint8_t sender_id, chassisSpeedCommandPacket* packet);
//	static void handle_gimbal_spd_commands(uint8_t sender_id, gimbalAngleCommandPacket* packet);
//	static void handle_launcher_front_firing_commands(uint8_t sender_id, FrontFiringPacket* packet);
//	static void handle_launcher_back_firing_commands(uint8_t sender_id, BackFiringPacket* packet);
//	static void handle_chassis_spin_command(uint8_t sender_id, ChassisSpinCommandPacket* packet);
//	static void handle_gimbal_yaw_command(uint8_t sender_id, gimbalAngleYawCommandPacket* packet);
//	static void handle_gimbal_pitch_command(uint8_t sender_id, gimbalAnglePitchCommandPacket* packet);
	void send_commands_chassis(chassisSpeedCommandPacket* packet);
//	void send_commands_gimbal(gimbalAngleCommandPacket* packet);
//	void send_commands_launcher_front(FrontFiringPacket* packet);
//	void send_commands_launcher_back(BackFiringPacket* packet);
//	void send_commands_chassis_spin(ChassisSpinCommandPacket* packet);
//	void send_commands_pitch_gimbal(gimbalAnglePitchCommandPacket* packet);
//	void send_commands_yaw_gimbal(gimbalAngleYawCommandPacket* packet);

	void set_spinspin(bool beyblade_mode);

	void init();
	void loop();


private:




	bool beyblade_mode;
	float V_horz; // Horizontal speed. In the direction of the X axis.
	float V_lat;  // Lateral speed. In the direction of the Y axis.
	float V_yaw; // Chassis yaw axis. Not used in beyblade mode.


	float gimbal_pitch; // Pitch angle orientation of the gimbal.
	float gimbal_yaw;	  // Yaw angle orientation of the gimbal.

	bool fire_front_launcher;
	bool fire_back_launcher;
	bool front_or_back;

	bool alignChassisAndGimbal;
	bool manual_mode;

};

extern ChassisSpdCmdThread* chassisSpeedInstance;


#endif /* THREADS_INC_SPD_CMD_THREAD_H_ */
