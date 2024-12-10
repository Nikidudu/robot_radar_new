/*
 * leg_function.h
 *
 *  Created on: Dec 10, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_LEG_TASK_H_
#define TASKS_INC_LEG_TASK_H_


void leg_task(void *argument);
void leg_pos(float phi1, float phi4, float pos[2]);
void leg_conv(float F, float Tp, float phi1, float phi4, float T[2]);
void leg_spd(float dphi1, float dphi4, float phi1, float phi4,
             float spd[2]);

#endif /* TASKS_INC_LEG_TASK_H_ */
