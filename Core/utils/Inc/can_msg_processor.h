/*
 * actuator_feedback_task.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef UTILS_INC_CAN_MSG_PROCESSOR_H_
#define UTILS_INC_CAN_MSG_PROCESSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DEV_C_TOP_TO_BOT_ID 	0x100
#define DEV_C_BOT_TO_TOP_ID 	0x101

void can_ISR(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif
#endif /* UTILS_INC_CAN_MSG_PROCESSOR_H_ */
