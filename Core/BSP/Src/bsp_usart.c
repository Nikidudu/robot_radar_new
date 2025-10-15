/**
 * bsp_usart.c
 *
 * Created on: Mar 2 2020
 *     Author: wx
 */

#include <bsp_queue.h>
#include "board_settings.h"
#include "bsp_usart.h"
#include "control_input_task.h"
#include "board_lib.h"

static queue_t *xvr_UART_queue;
static queue_t *ref_UART_queue;
extern TaskHandle_t referee_processing_task_handle;

static uint16_t ref_rx_size;
extern uint8_t ref_buffer[2];
extern queue_t referee_uart_q;

// Handles UART communication with mini-PC

void init_xvr_usart(uint8_t *pData){
	xvr_usart_start(&SBC_UART, pData, 15, NULL);
}

void xvr_half_cplt_isr(DMA_HandleTypeDef *hdma){
	//check which buffer is in use
//	if (hdma->Instance->CR &= DMA_SxCR_CT == 0){
		queue_append_byte(xvr_UART_queue, *(uint8_t*)hdma->Instance->M0AR);
//	} else {
//		queue_append_byte(xvr_UART_queue, *(uint8_t*)hdma->Instance->M1AR);
//	}
};
void xvr_full_cplt_isr(DMA_HandleTypeDef *hdma){
//	sbc_process_data();
};

HAL_StatusTypeDef xvr_usart_start(UART_HandleTypeDef *huart,uint8_t *pData, uint16_t Size,queue_t *uart_queue)
{
	//queue to be stored in the original caller function
	xvr_UART_queue = uart_queue;
	queue_init(xvr_UART_queue);
	uint32_t *tmp;

	/* Check that a Rx process is not already ongoing */
	if (huart->RxState == HAL_UART_STATE_READY) {
		if ((pData == NULL) || (Size == 0U))
		{
			return HAL_ERROR;
		}

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;

		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_BUSY_RX;
		huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

		/* Set the UART DMA transfer complete callback */
		huart->hdmarx->XferCpltCallback = xvr_full_cplt_isr;
//		huart->hdmarx->XferHalfCpltCallback = xvr_half_cplt_isr;


		/* Set the DMA abort callback */
		huart->hdmarx->XferAbortCallback = NULL;

		/* Enable the DMA stream */
		tmp = (uint32_t *)&pData;
		HAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t *)tmp, Size);

		/* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
		__HAL_UART_CLEAR_OREFLAG(huart);

		/* Process Unlocked */
		__HAL_UNLOCK(huart);

		/* Enable the UART Parity Error Interrupt */
		SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
	    in the UART CR3 register */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	} else {
		return HAL_BUSY;
	}
}

HAL_StatusTypeDef referee_usart_init(UART_HandleTypeDef *huart,uint8_t *pData, uint16_t Size,queue_t *uart_queue)
{
    if (pData == NULL || Size == 0 || uart_queue == NULL) {
        return HAL_ERROR;
    }

    // store the queue globally so the ISR can access it
    ref_UART_queue = uart_queue;
    queue_init(ref_UART_queue);
    ref_rx_size = Size;

    // start DMA reception with IDLE detection
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size) != HAL_OK) {
        return HAL_ERROR;
    }

    // optionally disable half-transfer interrupt to simplify
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    return HAL_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == REFEREE_UART.Instance) {
        // huart->pRxBuffPtr points to the buffer used in referee_usart_init()
        uint8_t *buf = huart->pRxBuffPtr;

        // push received bytes into queue
        for (uint16_t i = 0; i < Size; i++)
        {
            queue_append_byte(ref_UART_queue, buf[i]);
        }

        // notify FreeRTOS task
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(referee_processing_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        // restart DMA reception on the same buffer
        HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, ref_rx_size);
    }
}

