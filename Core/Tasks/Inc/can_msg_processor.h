/*
 * actuator_feedback_task.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_CAN_MSG_PROCESSOR_H_
#define TASKS_INC_CAN_MSG_PROCESSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

// definitions
#define DEV_C_TOP_TO_BOT_ID 	0x100
#define DEV_C_BOT_TO_TOP_ID 	0x101

// low-pass-filters: between 0(no filtering) and 1(frozen value)
#define SPEED_LPF 0

#ifdef __cplusplus
}
#endif

void can_ISR(CAN_HandleTypeDef *hcan);

#endif /* TASKS_INC_CAN_MSG_PROCESSOR_H_ */
