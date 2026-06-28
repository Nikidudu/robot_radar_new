/*
 * motor_control.c
 *
 *  Created on: May 23, 2021
 *      Author: wx
 *
 *
 *  @note
 *  	- This files containes the various pid loops for motor control
 *  	- Currently there are cascading pid loops, and single pid loops
 *  	- Cascading pid loops
 *  		- Basically the same; only differs in the way that current rpm is
 *  		calculated for the speed pid loop
 *  		- yangle_pid():
 *  			- position from imu_heading is used to calculate rpm data
 *  		- imu_angle_pid()
 *  			- rpm data is obtained directly from the gyro in the imu
 *  		- angle_pid():
 *  			- raw rpm data from motor is used
 *  	- single pid loops
 *  		-  speed_pid()
 */

#include "board_lib.h"
#include "motor_control.h"
#include "robot_config.h"
#include "imu_processing_task.h"

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
	float rpm_iOut = motor->angle_pid.ki; // todo: change to .integral?

	motor->angle_pid.output = rpm_pOut + rpm_dOut + rpm_iOut;
	float_minmax(&motor->angle_pid.output, motor->angle_pid.max_out,0);
	speed_pid(motor->angle_pid.output,imu_rpm, &motor->rpm_pid);
}

/* Function for angle PID (i.e. aiming for a target angle rather than RPM)
 * Function calculates target RPM, then calls the speed PID
 * function to set the motor's rpm until it reaches the target angle
 *
 * Possible angle wrapping (if loopback is set)
 * Uses imu_rpm data directly instead of calculating from imu_heading
 *
 * @param setpoint target value
 * @param curr_pt current angle
 * @param *motor pointer to the struct that contain the data for target motor
 * @param imu_rpm corresponding gyro raw rpm data from imu
 */
void imu_angle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_rpm, uint8_t loopback) {
    /* ---------- 1. Compute angle error ---------- */
	double ang_err = (setpoint - curr_pt);
	if (loopback){
		if (ang_err > PI) 			ang_err -= 2.0f * PI;
		else if (ang_err < -PI) 	ang_err += 2.0f * PI;
	}

    /* ---------- 2. Compute dt ---------- */
	uint32_t now = get_microseconds();
	uint32_t last = motor->angle_pid.last_time[0];
	motor->angle_pid.last_time[0] = now;

	float dt;
	if (now > last) {
		dt = (now - last) * 1e-6f;   // microseconds → seconds
	} else {
		dt = GIMBAL_DELAY * 1e-3f;   // fallback (ms → s)
	}

    /* ---------- 3. Angle PID (outer loop) ---------- */
    motor->angle_pid.error[1] = motor->angle_pid.error[0];
    motor->angle_pid.error[0] = ang_err;

    /* P */
    float p_out = motor->angle_pid.kp * ang_err;

    /* D */
    float d_out = motor->angle_pid.kd *
        (motor->angle_pid.error[0] - motor->angle_pid.error[1]) / dt;

    /* I */
    motor->angle_pid.integral += ang_err * dt * motor->angle_pid.ki;
    float_minmax(&motor->angle_pid.integral,
                 motor->angle_pid.int_max,
                -motor->angle_pid.int_max);
    float i_out = motor->angle_pid.integral;

    /* Angle PID output = target RPM */
    float target_rpm = p_out + d_out + i_out;
    float_minmax(&target_rpm,
                 motor->angle_pid.max_out,
                -motor->angle_pid.max_out);

    motor->angle_pid.output = target_rpm;

    /* ---------- 4. Speed PID (inner loop) ---------- */
    speed_pid(target_rpm, imu_rpm, &motor->rpm_pid);
}


//COMMENT OUT BELOW direct_angle_pid FOR PREVIOUS LOGIC
/* Function for direct angle PID (single-loop for position control)
 * Outputs torque/current directly from angle error and angular velocity
 * Used when PITCH_SINGLE_PID_LOOP is defined
 *
 * @param setpoint target angle (radians)
 * @param curr_angle current angle from IMU (radians)
 * @param curr_gyro current angular velocity from IMU gyroscope (rad/s)
 * @param *pid pointer to the pid struct (typically rpm_pid)
 */
void direct_angle_pid(double setpoint, double curr_angle, double curr_gyro, pid_data_t *pid) {
    /* ---------- 1. Update timestamps ---------- */
    pid->last_time[1] = pid->last_time[0];
    pid->last_time[0] = get_microseconds();

    /* ---------- 2. Calculate dt ---------- */
    float dt;
    if (pid->last_time[0] > pid->last_time[1]) {
        dt = (pid->last_time[0] - pid->last_time[1]) * 1e-6f;  // microseconds → seconds
    } else {
        dt = GIMBAL_DELAY * 1e-3f;  // fallback to expected loop time (ms → s)
    }

    /* ---------- 3. Calculate angle error ---------- */
    pid->error[1] = pid->error[0];
    pid->error[0] = setpoint - curr_angle;

    /* ---------- 4. P term: Proportional to angle error ---------- */
    float Pout = pid->error[0] * pid->kp;

    /* ---------- 5. D term: Damping based on angular velocity ---------- */
    // CRITICAL: Negative gyro for damping
    float Dout = -curr_gyro * pid->kd;
//    static float filtered_gyro = 0;
//    float alpha = 0.3f;  // Filter strength (0.1-0.5, lower = more filtering)
//    filtered_gyro = alpha * curr_gyro + (1.0f - alpha) * filtered_gyro;
//
//    float Dout = -filtered_gyro * pid->kd;

    /* ---------- 6. I term: Gravity compensation ---------- */
    pid->integral += pid->error[0] * pid->ki * dt;
    float_minmax(&pid->integral, pid->int_max, -pid->int_max);
    float Iout = pid->integral;

    /* ---------- 7. Combine and output ---------- */
    pid->output = Pout + Iout + Dout;
    float_minmax(&pid->output, pid->max_out, -pid->max_out);
}
//END HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


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
	float FFout = 0;


	pid->error[1] = pid->error[0];
	pid->error[0] = setpoint - curr_pt;
	Pout = pid->error[0] * pid->kp * time_mult;
	Dout = (float)(pid->error[0] - pid->error[1]) * pid->kd * time_mult;
	pid->integral += pid->error[0] * pid->ki * time_mult;
	float_minmax(&pid->integral, pid->int_max, 0);
	Iout = pid->integral;
	FFout = pid->kff*setpoint;

	pid->output = Pout + Iout + Dout + FFout;
	float_minmax(&pid->output, pid->max_out, 0);
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
