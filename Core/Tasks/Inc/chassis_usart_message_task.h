/*
 * chassis_usart_message_task.h
 *
 * Created on: Feb 25, 2026
 * Author: bed
 */

#ifndef TASKS_INC_CHASSIS_USART_MESSAGE_TASK_H_
#define TASKS_INC_CHASSIS_USART_MESSAGE_TASK_H_

#include "main.h"
#include "typedefs.h"

// scaling factor to pack float into 2 bytes
// should be the same on both top and bottom dev C
#define SCALE 1000.0f

extern supercap_data supercap;

void chassis_usart_message_task(void *argument);

#endif /* TASKS_INC_CHASSIS_USART_MESSAGE_TASK_H_ */