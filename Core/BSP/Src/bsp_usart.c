/**
 * bsp_usart.c
 *
 * Created on: Mar 2 2020
 *     Author: wx
 */

#include <stdbool.h>
#include "board_lib.h"
#include "bsp_usart.h"

/* From other tasks (extern) */
extern queue_t *ref_UART_queue;
extern uint8_t ref_dma_buf[REF_DMA_BUF_SIZE];
extern uint8_t remote_raw_data[REMOTE_DATA_SIZE];

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  UART receive complete callback.
 * @note   This function is called by the HAL library when a UART receive
 *         operation (via DMA or interrupt) has completed.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &REMOTE_UART)
    {
    	remote_ISR();  // REMOTE UART ISR handler

    } else if (huart == &REFEREE_UART) {
    	referee_ISR(); // REFEREE UART ISR handler
    }
}

/**
* @brief  UART receive half-complete callback.
* @note   This function is called by the HAL library when a UART DMA
*         reception has filled half of the buffer.
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &REFEREE_UART) {
		referee_half_ISR(); // REFEREE UART ISR handler
	}
}

/**
 * This function starts the circular DMA for remote UART port
 */
HAL_StatusTypeDef remote_uart_start(void)
{
	memset(remote_raw_data, 0, REMOTE_DATA_SIZE);
    UART_HandleTypeDef *huart = &REMOTE_UART;

    if (huart->RxState != HAL_UART_STATE_READY)
        return HAL_BUSY;

    /* Start DMA reception with HAL helper */
    if (HAL_UART_Receive_DMA(huart, remote_raw_data, REMOTE_DATA_SIZE) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * This function starts the circular DMA for referee UART port
 */
HAL_StatusTypeDef ref_usart_start(UART_HandleTypeDef *huart,uint8_t *pData, uint16_t Size,queue_t *uart_queue)
{
    /* Store & init queue (same behavior as before) */
    ref_UART_queue = uart_queue;
    queue_init(ref_UART_queue);

    if (huart->RxState != HAL_UART_STATE_READY)
        return HAL_BUSY;

    /* Start DMA reception */
    if (HAL_UART_Receive_DMA(&REFEREE_UART, ref_dma_buf, REF_DMA_BUF_SIZE)) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


