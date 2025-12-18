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

#define REMOTE_DATA_SIZE 23

/* Size in bytes of data sent from remote */

/* Pointer to the received raw data from the remote */
extern uint8_t remote_raw_data[REMOTE_DATA_SIZE];

/**
 * Internal callback function that also converts raw data from the controller to cleaned
 * data that can be used by the system. This function in turn called the ISR,
 * dbus_remote_ISR()
 */
void dbus_remote_ISR(DMA_HandleTypeDef *hdma);

/**
 * This function starts the circular DMA that reads from the USART1 port to memory.
 * This function is blocking, so it is advised to only call this inside a freeRTOS
 * task so that execution flow can continue thanks to preemption.
 *
 * @param huart  Pointer to the UART port handle. This should be the DBUS UART1 port.
 * @param pData  Pointer to the buffer where the received data will be stored. Ideally
 *               this should be the remote_raw_data buffer defined in this header file.
 */
HAL_StatusTypeDef dbus_remote_start();

typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}remote_data_t;

#endif /* BSP_INC_BSP_REMOTE_H_ */
