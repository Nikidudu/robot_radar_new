/*
 * status_thread.cpp
 *
 *  Created on: Mar 19, 2025
 *      Author: cw
 */
#ifdef BUILD_WITH_STMUART

#include <status_thread.h>
#include "referee_msgs.h"

#include "robot_config.h"

#include "cvAimbotCommandThread.h"

extern ref_game_state_t ref_game_state;
extern uint32_t ref_game_state_txno;
extern ref_game_robot_data2_t ref_robot_data;
extern uint32_t ref_robot_data_txno;
extern ref_game_robot_HP_t ref_robot_hp;
extern uint32_t ref_robot_hp_txno;

extern cvAimbotCommandThread* cvAimbotCommandInstance;

statusThread* statusInstance = nullptr;

// Declare your data with the proper data structure defined in DataStructures.h
static cvCompetitionStatusData status_data;
static cvCompetitionStatusPacket status_packet;

static cvRobotModeData robot_mode_data;
static cvRobotModePacket robot_mode_packet;

static cvAimSendPacket aimsend_packet;
static cvAimSendData aimsend_data;

extern uint8_t control_mode;

statusThread::~statusThread(){}

void statusThread::init(){}

void statusThread::loop()
{
	if (last_ref_game_state_txno != ref_game_state_txno) {
		status_data.game_progress = ref_game_state.game_progress;
		status_data.time_left = ref_game_state.stage_remain_time;
		last_ref_game_state_txno = ref_game_state_txno;
	}

	if (last_ref_robot_data_txno != ref_robot_data_txno) {
		status_data.robot_id = ref_robot_data.robot_id;
		status_data.current_hp = ref_robot_data.current_HP;
		last_ref_robot_data_txno = ref_robot_data_txno;
	}

	if (last_ref_robot_hp_txno != ref_robot_hp_txno) {
		status_data.red_base_hp = ref_robot_hp.red_base_HP;
		status_data.blue_base_hp = ref_robot_hp.blu_base_HP;
		status_data.red_outpost_hp = 0; // To wait till referee system upgrade
		status_data.blue_outpost_hp = 0; // To wait till referee system upgrade
	}

	if (control_mode == SBC_CTRL_MODE) {
		robot_mode_data.robot_mode = true;
	} else {
		robot_mode_data.robot_mode = false;
	}

	robot_mode_data.toArray((uint8_t*) &robot_mode_packet);
	MAKE_RELIABLE(robot_mode_packet);
	UART_network->send(&robot_mode_packet);

	status_data.toArray((uint8_t*) &status_packet);
	MAKE_RELIABLE(status_packet);
	UART_network->send(&status_packet);

	aimsend_data.aim_send = true;
	// Send aim_state directly

	aimsend_data.toArray((uint8_t*) &aimsend_packet);
	MAKE_RELIABLE(aimsend_packet);
	UART_network->send(&aimsend_packet);


	osDelay(500);

	portYIELD();
}

#endif
