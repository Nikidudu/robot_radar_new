/**
 * bsp_usart.h
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */

/** Instructions
 * 1) Configure USARTx and the corresponding DMA in CubeMX. Refer to instructions on GitHub.
 * 2) Include this header file.
 * 3) Start circular DMA process for detecting received data using usart_start(). Note
 *    that this function is blocking so this should be started in a freeRTOS task. Call
 *    this after all peripherals have been initialized.
 * 4) Define usart_ISR() somewhere in the code to respond to received data on USARTx.
 * 5) Whenever the DMA received data, the ISR will be triggered. To get the buffer of
 *    raw received data, use the usart_get_data() function.
 * 6) To sent data on the USARTx port, use the usart_send_data() function.
 */

#ifndef INC_BSP_USART_H_
#define INC_BSP_USART_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

HAL_StatusTypeDef remote_uart_start(void);
HAL_StatusTypeDef ref_usart_start(UART_HandleTypeDef *huart, queue_t *uart_queue);

/** Full reset of referee UART: abort DMA, clear buffers, re-arm. Use for recovery from sync loss. */
HAL_StatusTypeDef ref_usart_full_reset(queue_t *uart_queue);

#endif
