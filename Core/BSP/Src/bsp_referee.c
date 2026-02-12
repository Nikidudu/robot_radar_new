/*
 * bsp_referee.c
 * Board Support Package - Referee System Communication
 *
 * Handles decoding of DJI RoboMaster referee system protocol frames
 * Processes UART data and extracts structured referee messages
 *
 *  Created on: Jun 28, 2021
 *      Author: wx
 */

/* Private includes ----------------------------------------------------------*/
#include "board_lib.h"
#include "bsp_referee.h"
#include "master_task.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE]; // buffer for storing received protocol frame

/* External variables --------------------------------------------------------*/
queue_t *ref_UART_queue; // Global queue for referee UART data

/* Private function prototypes -----------------------------------------------*/
ref_msg_t ref_get_msg_from_buffer(ref_frame_header_t header, uint8_t *frame_buffer, uint16_t len);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Process incoming referee system UART data
 * @param uart_queue: Queue containing raw UART bytes from referee system
 * @param proc_msg: Pointer to structure where decoded message will be stored
 * @return Processing status: PROCESS_SUCCESS if complete frame received, INSUFFICIENT_DATA otherwise
 *
 * State machine implementation for DJI Referee System protocol decoding
 * Protocol frame format: SOF(1) + DATA_LEN(2) + SEQ(1) + CRC8(1) + CMD_ID(2) + DATA(n) + CRC16(2)
 */
ref_processing_status_t ref_process_data(queue_t *uart_queue, ref_msg_t *proc_msg) {
    static uint8_t unpack_step = STEP_HEADER_SOF;	/* Current state in protocol parsing */
    static uint16_t index = 0;						/* Current position in frame buffer */
    static uint16_t data_len = 0;					/* Expected data payload length */
    static ref_frame_header_t header;				/* Extracted frame header */
    static uint8_t sof = 0xA5;  // Define SOF

    /* Process all available bytes in queue */
    while (queue_get_size(uart_queue) > 0) {
        uint8_t byte = queue_pop_element(uart_queue);

        switch (unpack_step) {
            case STEP_HEADER_SOF:
                /* Look for Start of Frame byte */
                if (byte == sof) {
                    protocol_packet[index++] = byte;
                    unpack_step = STEP_LENGTH_LOW;
                } else {
                    index = 0;  // Reset index on invalid byte
                }
                break;

            case STEP_LENGTH_LOW:
                /* Get lower byte of data length */
                protocol_packet[index++] = byte;
                data_len = byte;
                unpack_step = STEP_LENGTH_HIGH;
                break;

            case STEP_LENGTH_HIGH:
                /* Get higher byte of data length */
                protocol_packet[index++] = byte;
                data_len |= (byte << 8);

                /* Validate data length doesn't exceed buffer capacity */
                if (data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
                    unpack_step = STEP_FRAME_SEQ;
                } else {
                    /* Invalid length, reset state machine */
                    unpack_step = STEP_HEADER_SOF;
                    index = 0;
                    data_len = 0;
                }
                break;

            case STEP_FRAME_SEQ:
                /* Get frame sequence number */
                protocol_packet[index++] = byte;
                unpack_step = STEP_HEADER_CRC8;
                break;

            case STEP_HEADER_CRC8:
                /* Get header CRC8 checksum */
                protocol_packet[index++] = byte;

                /* Verify complete header (5 bytes) */
                if (index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                        /* Extract header fields */
                        header.start_frame = protocol_packet[0];
                        header.data_length = (uint16_t)protocol_packet[1] | ((uint16_t)protocol_packet[2] << 8);
                        header.seq = protocol_packet[3];
                        header.crc = protocol_packet[4];
                        unpack_step = STEP_CMD_ID_LOW;
                    } else {
                        /* CRC8 verification failed; reset state machine */
                        unpack_step = STEP_HEADER_SOF;
                        index = 0;
                        data_len = 0;
                    }
                }
                break;

            case STEP_CMD_ID_LOW:
                /* Get lower byte of command ID */
                protocol_packet[index++] = byte;
                header.cmd_id = byte;
                unpack_step = STEP_CMD_ID_HIGH;
                break;

            case STEP_CMD_ID_HIGH:
                /* Get higher byte of command ID */
                protocol_packet[index++] = byte;
                header.cmd_id |= (byte << 8);
                unpack_step = STEP_DATA;
                break;

            case STEP_DATA:
                /* Accumulate data payload */
                protocol_packet[index++] = byte;

                /* Check if all data bytes received */
                if (index >= (REF_HEADER_CRC_CMDID_LEN + data_len)) {
                    unpack_step = STEP_CRC16;
                }
                break;

            case STEP_CRC16:
                /* Get CRC16 checksum bytes (2 bytes) */
                protocol_packet[index++] = byte;

                /* Verify complete frame */
                if (index >= (REF_HEADER_CRC_CMDID_LEN + data_len + 2)) {
                    if (verify_CRC16_check_sum(protocol_packet, index)) {
                        /* Frame verified successfully - extract message */
                        *proc_msg = ref_get_msg_from_buffer(header, protocol_packet, index);

                        /* Reset state machine for next frame */
                        unpack_step = STEP_HEADER_SOF;
                        index = 0;
                        data_len = 0;
                        return PROCESS_SUCCESS;
                    } else {
                        /* CRC16 verification failed; reset state machine */
                        unpack_step = STEP_HEADER_SOF;
                        index = 0;
                        data_len = 0;
                    }
                }
                break;
        }
    }
    return INSUFFICIENT_DATA;     /* Not enough data for complete frame */
}

/**
 * @brief Extract and parse referee message from raw frame buffer
 * @param header: Extracted frame header containing command ID and metadata
 * @param frame_buffer: Complete raw frame buffer including header and CRC
 * @param len: Total frame length
 *
 * Maps frame data to appropriate structure based on command ID
 */
ref_msg_t ref_get_msg_from_buffer(ref_frame_header_t header, uint8_t *frame_buffer, uint16_t len) {
    ref_msg_t buffer_msg;
    memset(&buffer_msg, 0, sizeof(ref_msg_t));  // Initialize to zero

    /* Data payload starts after header (5 bytes) */
    uint8_t* data_buffer = frame_buffer + REF_HEADER_CRC_CMDID_LEN;
    buffer_msg.cmd_id = header.cmd_id;

    /* Parse data based on command ID */
    switch (header.cmd_id) {
    case REF_GAME_STATE_CMD_ID:
        if (header.data_length >= sizeof(ref_game_state_t)) {
            memcpy(&buffer_msg.data.game_state, data_buffer, sizeof(ref_game_state_t));
        }
        break;
    case REF_GAME_RESULT_ID:
        if (header.data_length >= sizeof(ref_game_result_t)) {
            memcpy(&buffer_msg.data.game_result, data_buffer, sizeof(ref_game_result_t));
        }
        break;
    case REF_ROBOT_HP_CMD_ID:
        if (header.data_length >= sizeof(ref_game_robot_HP_t)) {
            memcpy(&buffer_msg.data.robot_hp, data_buffer, sizeof(ref_game_robot_HP_t));
        }
        break;
    case REF_GAME_EVENT_CMD_ID:
        if (header.data_length >= sizeof(ref_game_event_data_t)) {
            memcpy(&buffer_msg.data.game_event, data_buffer, sizeof(ref_game_event_data_t));
        }
        break;
    case REF_ROBOT_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_game_robot_data2_t)) {
            memcpy(&buffer_msg.data.robot_state, data_buffer, sizeof(ref_game_robot_data2_t));
        }
        break;
    case REF_ROBOT_POWER_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_robot_power_data_t)) {
            memcpy(&buffer_msg.data.power_data, data_buffer, sizeof(ref_robot_power_data_t));
        }
        break;
    case REF_ROBOT_POS_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_game_robot_pos_t)) {
            memcpy(&buffer_msg.data.robot_pos, data_buffer, sizeof(ref_game_robot_pos_t));
        }
        break;
    case REF_ROBOT_BUFF_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_buff_data_t)) {
            memcpy(&buffer_msg.data.robot_buff, data_buffer, sizeof(ref_buff_data_t));
        }
        break;
    case REF_ROBOT_DMG_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_robot_dmg_t)) {
            memcpy(&buffer_msg.data.damage_data, data_buffer, sizeof(ref_robot_dmg_t));
        }
        break;
    case REF_ROBOT_SHOOT_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_shoot_data_t)) {
            memcpy(&buffer_msg.data.shooting_data, data_buffer, sizeof(ref_shoot_data_t));
        }
        break;
    case REF_ROBOT_MAGAZINE_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_magazine_data_t)) {
            memcpy(&buffer_msg.data.magazine_data, data_buffer, sizeof(ref_magazine_data_t));
        }
        break;
    case REF_CUSTOM_DATA_CMD_ID:
        if (header.data_length >= sizeof(ref_custom_data_t)) {
            memcpy(&buffer_msg.data.custom_data, data_buffer, sizeof(ref_custom_data_t));
        }
        break;
    default:
        /* Unknown command ID */
        buffer_msg.cmd_id = 0xFFFF;
        break;
    }

    return buffer_msg;
}
