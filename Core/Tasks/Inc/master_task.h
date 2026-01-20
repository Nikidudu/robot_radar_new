/*
 * master_task.h
 *
 *  Created on: 14 Sep 2023
 *      Author: cwx
 */

#ifndef TASKS_INC_MASTER_TASK_H_
#define TASKS_INC_MASTER_TASK_H_

#include "task.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

extern TaskHandle_t referee_processing_task_handle;
extern TaskHandle_t control_input_task_handle;
extern TaskHandle_t imu_processing_task_handle;
extern QueueHandle_t g_buzzing_task_msg;

void master_task(void* argument);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_INC_MASTER_TASK_H_ */
