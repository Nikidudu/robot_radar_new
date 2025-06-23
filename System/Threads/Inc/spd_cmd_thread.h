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
	static void handle_chassis_spin_commmands(uint8_t sender_id, chassisSpinCommandPacket* packet);
	static void handle_navigate_commands(uint8_t sender_id, isNavigatingPacket* packet);

	void send_chassis_spd_commands(chassisSpeedCommandPacket* packet);
	void send_chassis_spin_commands(chassisSpinCommandPacket* packet);
	void send_navigate_commands(isNavigatingPacket* packet);

	void set_spinspin(bool beyblade_mode);

	void init();
	void loop();


private:
	uint32_t curr_receive_time;
	uint32_t last_receive_time;
	bool updated_yaw;

	bool beyblade_mode;
	float V_horz; // Horizontal speed. In the direction of the X axis.
	float V_lat;  // Lateral speed. In the direction of the Y axis.
	float V_yaw; // Chassis yaw axis. Not used in beyblade mode.
};

extern ChassisSpdCmdThread* chassisSpeedInstance;


#endif /* THREADS_INC_SPD_CMD_THREAD_H_ */
