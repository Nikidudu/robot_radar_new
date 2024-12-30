/*
 * balancing_chassis_task.h
 *
 *  Created on: Dec 10, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_BALANCING_CHASSIS_TASK_H_
#define TASKS_INC_BALANCING_CHASSIS_TASK_H_

void balancing_chassis_task(void *argument);
void Ctrl_Init();
void Ctrl_TargetUpdateTask();
float shortest_angle_diff(float target_angle, float current_angle);
float normalize_angle(float angle);
int ground_detect(float LF, float LTP,float Ltheta,float LL0, float RF, float RTP,float Rtheta,float RL0);

#endif /* TASKS_INC_BALANCING_CHASSIS_TASK_H_ */
