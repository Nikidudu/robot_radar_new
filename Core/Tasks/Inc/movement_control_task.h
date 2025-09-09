/*
 * movement_control_task.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_MOVEMENT_CONTROL_TASK_H_
#define TASKS_INC_MOVEMENT_CONTROL_TASK_H_

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "typedefs.h"

#include "board_lib.h"
#include "robot_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"
#include "bsp_hall.h"

void movement_control_task(void *argument);

#endif /* TASKS_INC_MOVEMENT_CONTROL_TASK_H_ */
