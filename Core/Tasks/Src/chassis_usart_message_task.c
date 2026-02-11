
#include <stdio.h>
#include <string.h>
#include "main.h"

extern UART_HandleTypeDef huart6;

// Private define
#define UART_TX_PERIOD_MS 5
#define UART_FRAME_HEADER 0xA5
#define UART_PACKET_SIZE 26
#define MEMSET_INTERVAL 100

uint8_t uart_tx_buf[UART_PACKET_SIZE];
uint8_t send_count = 0;
uint8_t rx_data[UART_PACKET_SIZE];
uint8_t rx_counter = 0;

void chassis_usart_message_task(void *argument) {
	HAL_UART_Receive_DMA(&huart6, rx_data, UART_PACKET_SIZE);

	// Initial Trigger: Only done ONCE to start the chain
	sprintf((char*)uart_tx_buf, "Hello from TOP devc %03u\r\n", (unsigned int)send_count);
	HAL_UART_Transmit_DMA(&huart6, uart_tx_buf, UART_PACKET_SIZE); // Send exactly 25 bytes


	while(1) {
		// We leave this loop empty or for other logic.
		// The UART is now handled entirely by the Callback "Chain".
		vTaskDelay(1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        send_count = send_count + 1;
//        if (send_count > 999) send_count = 0;


        sprintf((char*)uart_tx_buf, "Hello from TOP devc %03u\r\n", send_count);
        HAL_UART_Transmit_DMA(huart, uart_tx_buf, sizeof(uart_tx_buf));
    }
}


