/*
 * referee_processing_task.c
 *
 * RTOS task responsible for:
 * - Receiving raw referee system data via UART/DMA
 * - Decoding protocol frames using state machine
 * - Storing decoded data in global structures
 * - Managing UART error recovery
 *
 *  Created on: Jun 18, 2021
 *      Author: wx
 */

/* Private includes ----------------------------------------------------------*/
#include "board_lib.h"
#include "referee_processing_task.h"
#include "referee_msgs.h"
#include "robot_config.h"
#include "usb_task.h"

/* Private variables ---------------------------------------------------------*/
static ref_msg_t g_ref_msg_buffer; /* Static message buffer for protocol decoding */

/* External variables --------------------------------------------------------*/
queue_t referee_uart_q; /* Queue for raw UART data from referee system */

ref_game_state_t ref_game_state;
uint32_t ref_game_state_txno = 0;

ref_game_robot_HP_t ref_robot_hp;
uint32_t ref_robot_hp_txno = 0;

ref_game_robot_data2_t ref_robot_data;
uint32_t ref_robot_data_txno = 0;

ref_robot_power_data_t ref_power_data;
uint32_t ref_power_data_txno = 0;

ref_game_robot_pos_t ref_robot_pos;
uint32_t ref_robot_pos_txno = 0;

ref_buff_data_t ref_buff_data;
uint32_t ref_buff_data_txno = 0;

ref_robot_dmg_t ref_dmg_data;
uint32_t ref_dmg_data_txno = 0;

ref_shoot_data_t ref_shoot_data;
uint32_t ref_shoot_data_txno = 0;

ref_magazine_data_t ref_mag_data;
uint32_t ref_mag_data_txno = 0;
uint8_t g_ref_tx_seq = 0;

/* Full UART reset when stuck (2 consecutive INSUFFICIENT_DATA = aggressive recovery) */
#define REF_CONSECUTIVE_INSUFFICIENT_THRESHOLD 2
static uint8_t s_ref_consecutive_insufficient = 0;

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  Referee System Processing Task
 * @note   Initializes referee UART and enters infinite
 * 		   processing loop. Triggered by task
 * 		   notifications from UART idle line interrupt.
 * @param  argument: Task argument (unused)
 */
void referee_processing_task(void *argument) {
	ref_processing_status_t proc_status;

	status_led(7, on_led);
	status_led(8, off_led);
	ref_robot_data.robot_id = 0;

	memset(&ref_robot_data, 0, sizeof(ref_robot_data));

    /* Initialize referee UART with DMA and IDLE detection */
	ref_usart_start(&REFEREE_UART, &referee_uart_q);

	while (1) {

		ulTaskNotifyTake(pdTRUE, 1000);	/* Wait for task notification from UART RxEventCallback */

		status_led(5, on_led);

        /* Process all available frames in the queue
         * Keep processing until insufficient data remains */
		while (1) {
            /* Decode next frame from raw UART data */
			proc_status = ref_process_data(&referee_uart_q, &g_ref_msg_buffer);

			if (proc_status == PROCESS_SUCCESS) {
                /* Complete frame received and decoded successfully */
				s_ref_consecutive_insufficient = 0;
				switch (g_ref_msg_buffer.cmd_id) {
				case REF_ROBOT_SHOOT_DATA_CMD_ID:
					memcpy(&ref_shoot_data, &g_ref_msg_buffer.data,
							sizeof(ref_shoot_data_t));
					ref_shoot_data_txno++;
					break;
				case REF_GAME_STATE_CMD_ID:
					memcpy(&ref_game_state, &g_ref_msg_buffer.data,
							sizeof(ref_game_state_t));
					ref_game_state_txno++;
					break;
				case REF_ROBOT_DATA_CMD_ID:
					memcpy(&ref_robot_data, &g_ref_msg_buffer.data,
							sizeof(ref_game_robot_data2_t));
					ref_robot_data_txno++;
					break;
				case REF_ROBOT_POS_DATA_CMD_ID:
					memcpy(&ref_robot_pos, &g_ref_msg_buffer.data,
							sizeof(ref_game_robot_pos_t));
					ref_robot_pos_txno++;
					break;
				case REF_ROBOT_POWER_DATA_CMD_ID:
					memcpy(&ref_power_data, &g_ref_msg_buffer.data,
							sizeof(ref_robot_power_data_t));
					ref_power_data_txno++;
					break;
				case REF_ROBOT_DMG_DATA_CMD_ID:
					memcpy(&ref_dmg_data, &g_ref_msg_buffer.data,
							sizeof(ref_robot_dmg_t));
					ref_dmg_data_txno++;
					break;
				case REF_ROBOT_HP_CMD_ID:
					if (g_ref_msg_buffer.data_length >= sizeof(ref_game_robot_HP_t)) {
						/* Legacy: 28 bytes, both teams */
						memcpy(&ref_robot_hp, &g_ref_msg_buffer.data.robot_hp,
								sizeof(ref_game_robot_HP_t));
					} else if (g_ref_msg_buffer.data_length >= REF_ROBOT_HP_2026_DATA_LEN) {
						/* 2026 protocol: 16 bytes, ally only - map to ref_robot_hp by team */
						ref_game_robot_HP_ally_t *ally = &g_ref_msg_buffer.data.robot_hp_ally;
						uint16_t robot_id = ref_robot_data.robot_id;
						if (robot_id >= 1 && robot_id <= 9) {
							/* Red team: ally = red */
							ref_robot_hp.red_1_HP = ally->ally_1_robot_HP;
							ref_robot_hp.red_2_HP = ally->ally_2_robot_HP;
							ref_robot_hp.red_3_HP = ally->ally_3_robot_HP;
							ref_robot_hp.red_4_HP = ally->ally_4_robot_HP;
							ref_robot_hp.red_5_HP = 0;
							ref_robot_hp.red_7_HP = ally->ally_7_robot_HP;
							ref_robot_hp.red_base_HP = ally->ally_base_HP;
						} else if (robot_id >= 101 && robot_id <= 109) {
							/* Blue team: ally = blue */
							ref_robot_hp.blu_1_HP = ally->ally_1_robot_HP;
							ref_robot_hp.blu_2_HP = ally->ally_2_robot_HP;
							ref_robot_hp.blu_3_HP = ally->ally_3_robot_HP;
							ref_robot_hp.blu_4_HP = ally->ally_4_robot_HP;
							ref_robot_hp.blu_5_HP = 0;
							ref_robot_hp.blu_7_HP = ally->ally_7_robot_HP;
							ref_robot_hp.blu_base_HP = ally->ally_base_HP;
						}
						/* Opponent HP not in 0x0003; would need radar 0x0A02 */
					}
					ref_robot_hp_txno++;
					break;
				case REF_ROBOT_MAGAZINE_DATA_CMD_ID:
					memcpy(&ref_mag_data, &g_ref_msg_buffer.data,
							sizeof(ref_magazine_data_t));
					ref_mag_data_txno++;
					// add in the memcpys here
					break;
				default:
					break;
				}

				 /* No complete frame available in queue
				  * Exit processing loop and wait for more data */

			} else if (proc_status == INSUFFICIENT_DATA) {
				s_ref_consecutive_insufficient++;
				if (s_ref_consecutive_insufficient >= REF_CONSECUTIVE_INSUFFICIENT_THRESHOLD) {
					/* Stuck: full UART + parser reset to recover from sync loss */
					ref_parser_reset();
					ref_usart_full_reset(&referee_uart_q);
					s_ref_consecutive_insufficient = 0;
				}
				break; // Not enough data for a complete frame
			}
		}
	}
}

