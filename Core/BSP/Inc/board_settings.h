/*
 * board_settings.h
 *
 *  Created on: Jul 5, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BOARD_SETTINGS_H_
#define BSP_INC_BOARD_SETTINGS_H_

#define REMOTE_UART 	huart1
#define HDMA_REMOTE_RX 	hdma_usart1_rx

#define SBC_UART		huart3

#define REFEREE_UART	huart6
#define HDMA_REFEREE_RX hdma_usart6_rx
#define HDMA_REFEREE_TX hdma_usart6_tx

#define IMU_HSPI 		hspi1
#define IST_I2C			hi2c3

#endif /* BSP_INC_BOARD_SETTINGS_H_ */
