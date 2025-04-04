/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "stdint.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "master_task.h"
#include "Telemetry.h"
#include <cstring>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//FreeRTOS definitions
#define ISR_SEMAPHORE_COUNT 1
#define QUEUE_SIZE 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

TaskHandle_t master_task_handle;
//Input control task definitions

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
	return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask,
		signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */

	//Controls when different task can execute

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	Telemetry::setup();


  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	xTaskCreate(master_task, "master_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 12,
			&master_task_handle);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
//CAN_TxHeaderTypeDef TxHeader; //structure for transmitting messages
//CAN_RxHeaderTypeDef RxHeader; //structure for message reception
//CAN_FilterTypeDef FilterConfig; //declare CAN filter structure
//
//uint32_t TxMailbox;
//uint8_t rxbuf[8];	//receive buffer
//uint8_t txbuf[8];	//transmit buffer
//uint8_t button_pressed;
//uint8_t mailbox_free_level;
//
//unsigned int read_time;
//unsigned int updated_time;
//unsigned int count;
//unsigned char toggle_value;
//
//union{
//	unsigned char data[2];
//	unsigned int number;
//}conv;
/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
//	  txbuf[0] = 0x00;
//	  toggle_value = 0;
	while(1){
		;;
	}
//
//	  TxHeader.DLC=1; //give message size of 1 byte
//	  TxHeader.IDE=CAN_ID_STD; //set identifier to standard
//	  TxHeader.RTR=CAN_RTR_DATA; //RTR bit is set to data
//	  TxHeader.TransmitGlobalTime = DISABLE;
//
//	  FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //set fifo assignment
//	  FilterConfig.FilterIdHigh = 0; //0x245<<5; //the ID that the filter looks for
//	  FilterConfig.FilterIdLow = 0;
//	  FilterConfig.FilterMaskIdHigh = 0;
//	  FilterConfig.FilterMaskIdLow = 0;
//	  FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
//	  //FilterConfig.FilterActivation = ENABLE;
//	  FilterConfig.FilterActivation = ENABLE;
//
//	  HAL_CAN_ConfigFilter(&hcan1, &FilterConfig); //configure CAN filter
//
//
//	  HAL_CAN_Start(&hcan1); //start CAN
//	  /* USER CODE END 2 */
//
//	  /* Infinite loop */
//	  /* USER CODE BEGIN WHILE */
//	  while (1)
//	  {
//
//		  mailbox_free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
//
//		  read_time = HAL_GetTick();
//		  if((read_time - updated_time) >= 1000){
//			  updated_time = read_time;
//			  count++;
//			  TxHeader.StdId = 0x101;
//			  TxHeader.DLC = 2;
//			  //txbuf[0] = count;
//			  conv.number = count;
//			  if( mailbox_free_level > 0){
//				  memcpy(txbuf, conv.data, sizeof(conv.data));
//				  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txbuf, &TxMailbox);
//			  }
//		  }
//	  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

