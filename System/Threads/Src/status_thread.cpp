/*
 * status_thread.cpp
 *
 *  Created on: Mar 19, 2025
 *      Author: cw
 */

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
extern ref_game_event_data_t ref_event_data;
extern uint32_t ref_event_data_txno;
extern ref_game_result_t ref_game_result_data;
extern uint32_t ref_game_result_txno;

extern cvAimbotCommandThread* cvAimbotCommandInstance;

statusThread* statusInstance = nullptr;

// Declare your data with the proper data structure defined in DataStructures.h
static cvCompetitionStatusData status_data;
static cvCompetitionStatusPacket status_packet;

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

		// Determining team colour (red if id < 100, else blue )
		team_colour = (status_data.robot_id < 100) ? RED_TEAM : BLUE_TEAM;
	}

	if (last_ref_robot_hp_txno != ref_robot_hp_txno) {
	    status_data.red_hero_hp = ref_robot_hp.red_1_HP;
	    status_data.red_standard_hp = ref_robot_hp.red_3_HP;
	    status_data.red_sentry_hp = ref_robot_hp.red_7_HP;

	    status_data.blue_hero_hp = ref_robot_hp.blu_1_HP;
	    status_data.blue_standard_hp = ref_robot_hp.blu_3_HP;
	    status_data.blue_sentry_hp = ref_robot_hp.blu_7_HP;

	    last_ref_robot_hp_txno = ref_robot_hp_txno;
	}

	if (last_ref_event_txno != ref_event_data_txno) {
	    status_data.resupply_occupation = ref_event_data.event_type & RMUL_RESUPPLY_MASK;
	    status_data.central_occupation = ref_event_data.event_type & RMUL_CENTRAL_MASK;
	    last_ref_event_txno = ref_event_data_txno;
	}

	if (last_game_result_txno != ref_game_result_txno) {
		status_data.win_state = false;
		// during competition result calculation period
		if (status_data.game_progress == 5) {
			if ((team_colour == RED_TEAM && ref_game_result_data.winner == RED_WIN) ||
				(team_colour == BLUE_TEAM && ref_game_result_data.winner == BLUE_WIN)) {
				status_data.win_state = true;
			}
		}

		last_game_result_txno++;
	}

	status_data.toArray((uint8_t*) &status_packet);
	MAKE_RELIABLE(status_packet);
	UART_network->send(&status_packet);

	osDelay(500);

	portYIELD();
}
