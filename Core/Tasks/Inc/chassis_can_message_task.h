/*
 * chassis_can_message_task.h
 *
 *  Created on: Oct 25, 2025
 *      Author: zhan-hao
 */

#ifndef TASKS_INC_CHASSIS_CAN_MESSAGE_TASK_H_
#define TASKS_INC_CHASSIS_CAN_MESSAGE_TASK_H_

// scaling factor to pack float into 2 bytes
// should be the same on both top and bottom dev C
#define SCALE 1000.0f

extern supercap_data supercap;
void chassis_can_message_task(void *argument);

#endif /* TASKS_INC_CHASSIS_CAN_MESSAGE_TASK_H_ */
