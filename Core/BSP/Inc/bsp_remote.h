/*
 * bsp_remote.h
 *
 *  Created on: Dec 18, 2025
 *      Author: gskang
 */

#ifndef BSP_INC_BSP_REMOTE_H_
#define BSP_INC_BSP_REMOTE_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

#define REMOTE_DATA_SIZE 21

/* Size in bytes of data sent from remote */

/* Pointer to the received raw data from the remote */
void remote_ISR();
HAL_StatusTypeDef remote_uart_start();

typedef __packed struct
{
    uint8_t sof_1;		//0xA9
    uint8_t sof_2;		//0x53

    uint64_t  ch_0:11;  		    //The horizontal direction of the right joystick  Min:364; Mid:1024; Max:1684
    uint64_t  ch_1:11;           //The vertical direction of the right joystick		 Min:364; Mid:1024; Max:1684
    uint64_t  ch_2:11;           //The vertical direction of the left joystick		 Min:364; Mid:1024; Max:1684
    uint64_t  ch_3:11;           //The horizontal direction of the left joystick   Min:364; Mid:1024; Max:1684

    uint64_t mode_sw:2;			// All off (Left):1; Gimbal (Center):2 All on (Right):3
    uint64_t pause:1;			// Remote vs RC: Unpress:0   Press:1
    uint64_t fn_1:1;			// Remote/RC vs SBC: Unpress:0   Press:1
    uint64_t fn_2:1;			// Unused: Unpress:0   Press:1
    uint64_t wheel:11;			// Min:364; Mid:1024; Max:1684
    uint64_t trigger:1;			// To shoot: Unpress:0   Press:1

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
	uint16_t crc16;
} remote_data_t;

extern remote_cmd_t g_remote_cmd;

#endif /* BSP_INC_BSP_REMOTE_H_ */
