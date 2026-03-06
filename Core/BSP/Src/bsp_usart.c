/**
 * bsp_usart.c
 * Board Support Package - USART Communication
 *
 * Handles UART/DMA initialization and interrupt callbacks for:
 * - Remote controller receiver (SBUS protocol)
 * - DJI RoboMaster referee system communication
 *
 * Created on: Mar 2 2020
 *     Author: wx
 */
/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include "board_lib.h"
#include "bsp_referee.h"
#include "master_task.h"

/* External variables --------------------------------------------------------*/
uint8_t ref_dma_buf[REF_DMA_BUF_SIZE];

/* Exported variables -------------------------------------------------------*/
extern uint8_t remote_raw_data[REMOTE_DATA_SIZE];
queue_t *ref_UART_queue;
extern queue_t referee_uart_q; /* Queue for raw UART data from referee system */
extern TaskHandle_t referee_processing_task_handle;
extern DMA_HandleTypeDef hdma_usart6_rx;
#define HDMA_REFEREE_RX hdma_usart6_rx

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  UART receive complete callback (legacy, non-IDLE mode)
 * @note   This function is called by the HAL library when a UART receive
 *         operation (via DMA or interrupt) has completed.
 *         Currently used only for remote controller UART.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &REMOTE_UART) {
    	remote_ISR();  // REMOTE UART ISR handler
    }
}

/**
 * @brief  UART receive event callback with IDLE line detection
 * @note   Called when DMA receives data until IDLE line is detected
 *         Used for referee system protocol where frame boundaries are
 *         determined by idle line state
 * @param  huart: UART handle
 * @param  size: Number of bytes received since last IDLE
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart == &REFEREE_UART) {
    	// Process each received byte individually for protocol parsing
        for (uint16_t i = 0; i < size; i++) {
            queue_append_byte(ref_UART_queue, ref_dma_buf[i]);
        }

        // Notify referee processing task
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(referee_processing_task_handle,
                               &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); /* Force context switch if needed */

        // Re-arm DMA for next reception period
        HAL_UARTEx_ReceiveToIdle_DMA(huart, ref_dma_buf, REF_DMA_BUF_SIZE);
    }
}

/**
 * @brief  Initialize and start remote controller UART with circular DMA
 * @note   Configures DMA to continuously receive SBUS data frames
 *         Remote controller operates at 100Hz with 25-byte frames
 * @retval HAL status: HAL_OK on success, HAL_BUSY if UART not ready,
 *                     HAL_ERROR if DMA start fails
 */
HAL_StatusTypeDef remote_uart_start(void)
{
    /* Clear remote data buffer for initial state */
	memset(remote_raw_data, 0, REMOTE_DATA_SIZE);
    UART_HandleTypeDef *huart = &REMOTE_UART;

    /* Verify UART peripheral is ready for operation */
    if (huart->RxState != HAL_UART_STATE_READY)
        return HAL_BUSY;

    /* Start circular DMA reception */
    if (HAL_UART_Receive_DMA(huart, remote_raw_data, REMOTE_DATA_SIZE) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief  Initialize and start referee system UART with IDLE line DMA
 * @note   Configures one-shot DMA with IDLE detection for variable-length frames
 *         DMA is re-armed in the RxEventCallback after each reception
 * @param  huart: UART handle for referee communication
 * @param  size: Maximum reception size
 * @param  uart_queue: Queue for storing raw bytes for protocol decoder
 * @retval HAL status: HAL_OK on success, HAL_ERROR if DMA start fails
 */
HAL_StatusTypeDef ref_usart_start(UART_HandleTypeDef *huart, queue_t *uart_queue) {
    /* Store queue reference for ISR callback and initialize */
    ref_UART_queue = uart_queue;
    queue_init(ref_UART_queue);

    /* Clear DMA buffer */
    memset(ref_dma_buf, 0, REF_DMA_BUF_SIZE);

    if (huart->RxState != HAL_UART_STATE_READY) {
        HAL_UART_AbortReceive(huart);
    }

    /* Start DMA reception with IDLE line detection */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, ref_dma_buf, REF_DMA_BUF_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    // Disables half-transfer interrupt
    __HAL_DMA_DISABLE_IT(&HDMA_REFEREE_RX, DMA_IT_HT);


    return HAL_OK;
}



/**
 * @brief  UART Abort Complete Callback
 * @note   Called when HAL_UART_AbortReceive completes
 *         Handles error recovery and reinitialization of UART peripherals
 * @param  huart: UART handle that triggered the callback
 */
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &REMOTE_UART) {
		HAL_UART_DMAStop(&REMOTE_UART);
		remote_uart_start();
	} else if (huart == &REFEREE_UART) {
		__HAL_DMA_DISABLE(&HDMA_REFEREE_RX);
		ref_usart_start(&REFEREE_UART, &referee_uart_q);
	}
}

