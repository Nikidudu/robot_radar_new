/*
 * motor_control.h
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#ifndef UTILS_INC_MOTOR_CONTROL_H_
#define UTILS_INC_MOTOR_CONTROL_H_


void yangle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_data, float *prev_imu_data,uint8_t loopback);
void angle_pid(double setpoint, double curr_pt, motor_data_t *motor, uint8_t loopback);
void speed_pid(double setpoint, double curr_pt, pid_data_t *pid);
void imu_angle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_rpm, uint8_t loopback);
void float_minmax(float *motor_in, float motor_max, float motor_min);

#endif /* UTILS_INC_MOTOR_CONTROL_H_ */
