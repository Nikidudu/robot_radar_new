/*
 * motor_control.c
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "motor_control.h"
#include "robot_config.h"

/* Function for angle PID (i.e. aiming for a target angle rather than RPM)
 * Function calculates target RPM, then calls the speed PID
 * function to set the motor's rpm until it reaches the target angle
 *
 * Possible angle wrapping (if loopback is set); uses imu data
 *
 * @param setpoint target value
 * @param curr_pt current angle
 * @param *motor pointer to the struct that contain the data for target motor
 */
void yangle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_data, float *prev_imu_data, uint8_t loopback) {

	double ang_diff = (setpoint - curr_pt);
	if (loopback){
		if (ang_diff > PI) {
			ang_diff -= 2 * PI;
		} else if (ang_diff < -PI) {
			ang_diff += 2 * PI;
		}
	}
//	float est_next_ang = ang_diff + (motor->raw_data.rpm * 2 * PI) * 0.01 / 60; //estimated angle in 10ms
//	if (est_next_ang > PI){
//		ang_diff -= 2 * PI;
//	} else if (est_next_ang < -PI){
//		ang_diff += 2 * PI;
//	}

	if (*prev_imu_data == imu_data) {
		return;}
	uint32_t time_mult;
	motor->angle_pid.last_time[1] = motor->angle_pid.last_time[0];
	motor->angle_pid.last_time[0] = get_microseconds();
	if (motor->angle_pid.last_time[0] <= motor->angle_pid.last_time[1]){
		time_mult = 1000 * 60 / GIMBAL_DELAY;

	} else {
		time_mult = TIMER_FREQ * 60 /
			(float) (motor->angle_pid.last_time[0] - motor->angle_pid.last_time[1]);
	}
	motor->angle_pid.error[1] = motor->angle_pid.error[0];
	motor->angle_pid.error[0] = ang_diff;
	float rpm_pOut = motor->angle_pid.kp * ang_diff;
	float rpm_dOut = motor->angle_pid.kd * (motor->angle_pid.error[0] - motor->angle_pid.error[1]);

	float imu_ang_diff = imu_data - *prev_imu_data;
	imu_ang_diff = (imu_ang_diff > PI) ? imu_ang_diff - (2 * PI) :
			((imu_ang_diff < -PI) ? imu_ang_diff + (2*PI) : imu_ang_diff);
	float imu_rpm = (imu_ang_diff  * time_mult)/(2 * PI);
	*prev_imu_data = imu_data;
	motor->angle_pid.integral += motor->angle_pid.error[0]  * motor->angle_pid.ki;
	float_minmax(&motor->angle_pid.integral, motor->angle_pid.int_max, 0);
	float rpm_iOut = motor->angle_pid.ki;

	motor->angle_pid.output = rpm_pOut + rpm_dOut + rpm_iOut;
	float_minmax(&motor->angle_pid.output, motor->angle_pid.max_out,0);
	speed_pid(motor->angle_pid.output,imu_rpm, &motor->rpm_pid);
}

/* Function for angle PID (i.e. aiming for a target angle rather than RPM)
 * Function calculates target RPM, then calls the speed PID
 * function to set the motor's rpm until it reaches the target angle
 *
 * Possible angle wrapping (if loopback is set)
 *
 * @param setpoint target value
 * @param curr_pt current angle
 * @param *motor pointer to the struct that contain the data for target motor
 */
void angle_pid(double setpoint, double curr_pt, motor_data_t *motor, uint8_t loopback) {
	double ang_diff = (setpoint - curr_pt);

	if (loopback) {
		if (ang_diff > PI) {
			ang_diff -= 2 * PI;
		} else if (ang_diff < -PI) {
			ang_diff += 2 * PI;
		}
	}

	motor->angle_pid.error[1] = motor->angle_pid.error[0];
	motor->angle_pid.error[0] = ang_diff;
	float rpm_pOut = motor->angle_pid.kp * ang_diff;
	float rpm_dOut = motor->angle_pid.kd * (motor->angle_pid.error[0] - motor->angle_pid.error[1]);

	motor->angle_pid.integral += motor->angle_pid.error[0] * motor->angle_pid.ki;
	float_minmax(&motor->angle_pid.integral, motor->angle_pid.int_max, 0);
	float rpm_iOut = motor->angle_pid.integral;
	motor->angle_pid.output = rpm_pOut + rpm_dOut + rpm_iOut;
	float_minmax(&motor->angle_pid.output, motor->angle_pid.max_out,0);
	speed_pid(motor->angle_pid.output, motor->raw_data.rpm, &motor->rpm_pid);
}

/*
 * Function for speed PID
 * For motors that might see constant torque, i.e. chassis motors
 * make sure an integral value is initialised (VERY SMALL, like 0.0001 or smaller)
 * as their systems usually have a steady state error
 *
 *
 * @param setpoint target RPM
 * @param motor's current RPM
 * @param *pid pointer to the rpm_pid struct within the motor's data struct
 */
void speed_pid(double setpoint, double curr_pt, pid_data_t *pid) {
	pid->last_time[1] = pid->last_time[0];
	pid->last_time[0] = get_microseconds();

	uint32_t time_mult = 1; // ignores time scaling (aka assume fixed time step btwn PID updates)
	// uint32_t time_mult = TIMER_FREQ / (float) (pid->last_time[0] - pid->last_time[1]);
	float Pout = 0;
	float Iout = 0;
	float Dout = 0;

	pid->error[1] = pid->error[0];
	pid->error[0] = setpoint - curr_pt;
	Pout = pid->error[0] * pid->kp * time_mult;
	Dout = (float)(pid->error[0] - pid->error[1]) * pid->kd * time_mult;
	pid->integral += pid->error[0] * pid->ki * time_mult;
	float_minmax(&pid->integral, pid->int_max, 0);
	Iout = pid->integral;
	pid->output = Pout + Iout + Dout;
	float_minmax(&pid->output, pid->max_out, 0);
}

void kill_can() {

}

/**
 * Limits the input float variable
 * @params motor_in: the pointer to the variable to be limited
 * @params motor_max: the positive maximum value for the variable
 */

void float_minmax(float *motor_in, float motor_max, float motor_min) {
	if (*motor_in > motor_max) {
		*motor_in = motor_max;
	} else if (*motor_in < -motor_max) {
		*motor_in = -motor_max;
	}
}
