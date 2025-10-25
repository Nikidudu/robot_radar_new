/*
 * chassis_can_message_task.h
 *
 *  Created on: Oct 25, 2025
 *      Author: zhan-hao
 */

#ifndef TASKS_INC_CHASSIS_CAN_MESSAGE_TASK_H_
#define TASKS_INC_CHASSIS_CAN_MESSAGE_TASK_H_

void chassis_can_message_task(void *argument);
extern CAN_HandleTypeDef hcan1;

#endif /* TASKS_INC_CHASSIS_CAN_MESSAGE_TASK_H_ */
