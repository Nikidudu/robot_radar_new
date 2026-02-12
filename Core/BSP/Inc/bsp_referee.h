/*
 * bsp_referee.h
 *
 *  Created on: Jun 28, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BSP_REFEREE_H_
#define BSP_INC_BSP_REFEREE_H_
#include "referee_msgs.h"

#define REF_DMA_BUF_SIZE 64
#define REF_PROTOCOL_HEADER_SIZE    5
#define REF_HEADER_CRC_CMDID_LEN    7  // Header(5) + cmd_id(2)
#define REF_PROTOCOL_FRAME_MAX_SIZE 128

// Unpack states
#define STEP_HEADER_SOF     0 // Looking for 0xA5
#define STEP_LENGTH_LOW     1 // Data length low byte
#define STEP_LENGTH_HIGH    2 // Data length high byte
#define STEP_FRAME_SEQ      3 // Sequence number
#define STEP_HEADER_CRC8    4 // Header CRC8
#define STEP_CMD_ID_LOW     5 // Command ID low byte
#define STEP_CMD_ID_HIGH    6 // Command ID high byte
#define STEP_DATA           7 // Data field
#define STEP_CRC16          8 // CRC16

/*frame header 	5 bytes
 * cmd_id		2 bytes
 * data			28 bytes		| assuming we're not transmitting data from robot to robot, maximum is 28 bytes for robot hp data
 * frame tail	2 bytes
 * total: 37 bytes
 */
typedef enum {
	PROCESS_SUCCESS=0,
	INSUFFICIENT_DATA=1,
} ref_processing_status_t;

ref_processing_status_t ref_process_data(queue_t *uart_queue, ref_msg_t *proc_msg);
ref_frame_header_t ref_get_header(queue_t *data_buffer);
ref_msg_t ref_get_msg(ref_frame_header_t header,queue_t *uart_queue);

#endif /* BSP_INC_BSP_REFEREE_H_ */
