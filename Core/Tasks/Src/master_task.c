/*
 * master_task.c
 *
 *  Created on: 14 Sep 2023
 *      Author: cwx
 */

/* Private includes ----------------------------------------------------------*/
#include "board_lib.h"
#include "master_task.h"
#include "gimbal_control_task.h"
#include "launcher_control_task.h"
#include "chassis_can_message_task.h"
#include "referee_processing_task.h"
#include "control_input_task.h"
#include "imu_processing_task.h"
#include "buzzing_task.h"
#include "usb_task.h"
#include "INS_task.h"
#include "hud_new.h"
#include "error_handler_task.h"
#include "startup.h"
#include "chassis_usart_message_task.h"

/* External variables --------------------------------------------------------*/
TaskHandle_t referee_processing_task_handle;
TaskHandle_t control_input_task_handle;
TaskHandle_t imu_processing_task_handle;
QueueHandle_t g_buzzing_task_msg;

/* Exported variables -------------------------------------------------------*/
extern TaskHandle_t master_task_handle;
extern gimbal_control_t gimbal_ctrl_data;

void master_task(void *argument) {
	system_init();
	vTaskDelay(1000);

	g_buzzing_task_msg = xQueueCreate(48, sizeof(uint8_t));

	gimbal_ctrl_data.yaw_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);
	gimbal_ctrl_data.pitch_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(gimbal_ctrl_data.pitch_semaphore);

	/* add threads, ... */
	//todo: adjust priorities
	//Threads creation
#ifdef SENTRY
	xTaskCreate(INS_task, "INS_task",
	        configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4, NULL);
#endif

	xTaskCreate(imu_processing_task, "IMU_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 13,
			&imu_processing_task_handle);

	xTaskCreate(control_input_task, "RC_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
			&control_input_task_handle);

//	xTaskCreate(referee_processing_task, "referee_task", 512, (void*) 1,
//			(UBaseType_t) 2, &referee_processing_task_handle);

	xTaskCreate(buzzing_task, "buzzer_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 1, NULL);

	xTaskCreate(new_hud_task, "new_hud_task", 512, (void*) 3, (UBaseType_t) 5, NULL);

	xTaskCreate(error_handler_task, "error_handler_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 9, NULL);

    xTaskCreate(UsbParserTask, "UsbParser", 512, NULL, 12, NULL);

	xTaskCreate(chassis_can_message_task, "chassis_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4, NULL);

	xTaskCreate(launcher_control_task, "launcher_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4, NULL);

	xTaskCreate(gimbal_control_task, "gimbal_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 7, NULL);

	xTaskCreate(chassis_usart_message_task, "usart_chassis_task",
	            256, (void*) 1, (UBaseType_t) 4, NULL);


//	vTaskDelete(master_task_handle);
	while (1) {
		vTaskDelay(1000);
	}

}
