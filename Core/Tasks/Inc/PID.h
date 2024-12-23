/*
 * PID.h
 *
 *  Created on: Jul 14, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_PID_H_
#define TASKS_INC_PID_H_
#include "typedefs.h"
void PID_Init(PID *pid, float kp, float ki, float kd, float min_output, float max_output);
float PID_Compute(PID *pid, float setpoint, float measured_value, float dt, float deadzone);
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb, float dt);

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#endif /* TASKS_INC_PID_H_ */
