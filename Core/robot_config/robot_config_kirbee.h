	/*
 * robot_config_kirbee.h
 *
 *  Created on: Mar 20, 2025
 *      Author: cw
 */

#ifndef ROBOT_CONFIG_ROBOT_CONFIG_KIRBEE_H_
#define ROBOT_CONFIG_ROBOT_CONFIG_KIRBEE_H_

#include "hud_kirbee.h"
#include "motor_config.h"

#define BULLET_17
#define HAS_SBC
/********************* DEV C IMU CONFIGURATION ***********/
//#define BOARD_DOWN
//0 for SWDIO port to be roll, 1 for SWDIO port to be pitch, 2 for vertical mount SWDIO port to the right
#define IMU_ORIENTATION 	1

//flip until motor angle and yaw angle matches
#define IMU_YAW_INVERT		-1
#define IMU_PITCH_INVERT	1
//nothing uses roll.....yet
#define IMU_ROLL_INVERT		1
//#define IST8310
#define REF_POWER_LIM
#define G_X_OFFSET			0 //-7 	// todo: should these be set to 0?? is this the cause of our imu drift?
#define G_Y_OFFSET 			0 // -16
#define G_Z_OFFSET	 		0 // -5
#define ZERO_ROLL

/********************* CONTROL SENSITIVITIES ***********/
#define REMOTE_YAW_SPEED 	 	0.1 			//Speed of gimbal yaw turning
#define REMOTE_PITCH_SPEED 	 	-0.1//0.005		//Speed of gimbal pitch turning

#define MOUSE_X_SENSITIVITY		(300 * REMOTE_YAW_SPEED)	//Speed of yaw turning with mouse, dependent on above speed
#define MOUSE_Y_SENSITIVITY 	(150 * REMOTE_PITCH_SPEED)	//Speed of pitch turning with mouse,  dependent on above speed

/*********************** MANUAL CONTROL CONFIGURATION *******************/
//Inverts for both keyboard and mouse controls
#define YAW_INVERT  			-1				// 1 to invert control -1 to disable
#define PITCH_INVERT  			-1				// 1 to invert control -1 to disable

#define MOUSE_X_INVERT			1				// Set to -1 if it needs to be inverted
#define	MOUSE_Y_INVERT			-1				// Set to -1 if it needs to be inverted

#define KEYBD_MAX_SPD 			1//0.8//0.5		// % of max speed

#define GIMBAL_MODE 			1				// 1 for IMU control, 0 for absolute angle based control

/*********************** SUPERCAP CONFIGURATION *******************/
#define SUPERCAP_PRESENT
#define SUPER_CAP_OFFSET		-3

/*********************** REFEREE SYSTEM CONFIGURATION *******************/


/*********************** AIMBOT CONFIGURATION *******************/


/* PID TUNING GUIDE
 * For all motors, there are 2 different PID values, angle PID and speed PID
 * For motors that require position control, both values have to be set
 *
 * Speed PID is the main control loop which determines how much current
 * to send to the motors. i.e. it sets the target speed for the motors
 * Angle PID calculates the RPM the motor should be running at, then runs the
 * target values through the speed PID loop
 *
 * Generally, the speed control loop should be PID,
 * while the angle control loop can just be a P control
 *
 * TO TUNE PID
 * Tune speed loop FIRST, if not the angle loop might resonate and cause it to oscillate instead
 */

/*********************** LAUNCHER CONFIGURATION ***********************/
#define FEEDER_SPEED			1200	// projectiles per minute
#define	PROJECTILE_SPEED		20.5    //19 gives projectiles speed of 28-29m/s

#define PROJECTILE_SPEED_RATIO	310		//rpm per m/s of the friction wheels ish don't think this will work well lmao
#define FEEDER_SPEED_RATIO		-8		//projectiles per round of the feeder

// prevents pilots from overheating when firing
#define OVERHEAT_PROTECTION
#define OVERHEAT_MARGIN 	2
#define OVERHEAT_EXCESS 	2
#define OVERHEAT_OFFSET		20

#define FEEDER_MOTOR_TYPE	TYPE_M3508
// FEEDER PID VALUES
#define FEEDER_KP 			5
#define FEEDER_KI  			0.02
#define FEEDER_KD  			3
#define FEEDER_MAX_INT		10000
// FEEDER_ANGLE PID VALUES
#define FEEDER_ANGLE_KP 		1000
#define FEEDER_ANGLE_KD  		0
#define FEEDER_ANGLE_KI  		0
#define FEEDER_ANGLE_INT_MAX  	0
#define FEEDER_MAX_RPM			15

#define FEEDER_JAM_TORQUE  		12000	//35000	//20000		// Before feeder deemed to be jammed
#define FEEDER_JAM_RPM			20		// if feeder is below this rpm, it is jammed
#define FEEDER_UNJAM_SPD  		10		//100	// Reverse unjam
#define FEEDER_UNJAM_TIME		7000	//30000
#define FEEDER_MAX_CURRENT		60000   //60000
#define FEEDER_INVERT			1

// FRICTION WHEELS PID VALUES
#define FRICTION_SB_SPIN		0.5 // ratio of max flywheel speed
#define FRICTION_KP  			5
#define FRICTION_KI  			0
#define FRICTION_KD  			0//10
#define FRICTION_MAX_CURRENT 	16384
#define FRICTION_MAX_INT		10000
#define FRICTION_INVERT			-1
#define LAUNCHER_MARGIN			300
#define LAUNCHER_DIFF_MARGIN	300
#define FRICTION_OFFSET			0

#define CLEAR_DELAY				1000

/*********************** CHASSIS CONFIGURATION ***********************/
// CHASSIS WHEELS PID VALUES
#define CHASSIS_KP  		6.0
#define CHASSIS_KI  		0
#define CHASSIS_KD  		0.2
#define CHASSIS_INT_MAX  	5000
#define CHASSIS_MAX_CURRENT 4000//6000//9000
#define CHASSIS_MIN_CURRENT 0
#define BUFFER_MIN			1.0	// power buffer minimum, at zero buffer left, motors will draw CHASSIS_MAX_CURRENT * BUFFER_MIN
									// tune this by seeing if pilot likes the speed
#define CHASSIS_CAN_SPINSPIN
#define CHASSIS_SPINSPIN_MAX 0.9

//#define LVL_TUNING		// scales chassis speed as level increases
//#define ONE_VS_ONE		// 1v1 standard configuration (vs 3v3)

#ifdef LVL_TUNING
/* Speed Value Guide:
 * Values are percentage of the max speed.
 * E.g. 0.5 = 50% max speed
 */
#ifdef ONE_VS_ONE
#define LV1_MAX_SPEED 		    0.34//0.20 // for 1v1
#define LV2_MAX_SPEED  			0.34
#define LV3_MAX_SPEED			0.34
#define LV4_MAX_SPEED			0.34
#define LV5_MAX_SPEED			0.34
#define LV6_MAX_SPEED			0.34
#define LV7_MAX_SPEED			0.34
#define LV8_MAX_SPEED			0.34
#define LV9_MAX_SPEED			0.34
#define LV10_MAX_SPEED			0.34
// supercap boosted values
#define LV11_MAX_SPEED			1
#define LV12_MAX_SPEED		    1
#define LV13_MAX_SPEED			1
#define LV14_MAX_SPEED			1
#define LV15_MAX_SPEED			1
#define LV16_MAX_SPEED			1
#define LV17_MAX_SPEED			1
#define LV18_MAX_SPEED			1
#define LV19_MAX_SPEED			1
#define LV20_MAX_SPEED			1
#else
#define SCALE 					0.40
#define LV1_MAX_SPEED 		    0.24 * SCALE
#define LV2_MAX_SPEED  			0.25 * SCALE
#define LV3_MAX_SPEED			0.26 * SCALE
#define LV4_MAX_SPEED			0.27 * SCALE
#define LV5_MAX_SPEED			0.29 * SCALE
#define LV6_MAX_SPEED			0.30 * SCALE
#define LV7_MAX_SPEED			0.32 * SCALE
#define LV8_MAX_SPEED			0.33 * SCALE
#define LV9_MAX_SPEED			0.35 * SCALE
#define LV10_MAX_SPEED			0.36 * SCALE
// supercap boosted values
#define LV11_MAX_SPEED			0.30
#define LV12_MAX_SPEED		    0.32
#define LV13_MAX_SPEED			0.34
#define LV14_MAX_SPEED			0.35
#define LV15_MAX_SPEED			0.36
#define LV16_MAX_SPEED			0.36
#define LV17_MAX_SPEED			0.36
#define LV18_MAX_SPEED			0.36
#define LV19_MAX_SPEED			0.36
#define LV20_MAX_SPEED			0.36
#endif
/*	Acceleration Value Guide:
 * 	0.05 - Very Slow Acceleration
 * 	0.10 - Slow Acceleration
 * 	0.20 - Moderate Acceleration
 *  0.50 - Fast Acceleration
 *  1.00 - Very Fast Acceleration
 *  2.00 - Extremely Fast Acceleration
 */
//Chassis Acceleration
#ifdef ONE_VS_ONE
#define LV1_MAX_ACCEL			1.0//2.5//1.5 // for 1v1
#define LV2_MAX_ACCEL			1.5
#define LV3_MAX_ACCEL			1.5
#define LV4_MAX_ACCEL			1.5
#define LV5_MAX_ACCEL			0.7
#define LV6_MAX_ACCEL			0.5
#define LV7_MAX_ACCEL			0.5
#define LV8_MAX_ACCEL			0.5
#define LV9_MAX_ACCEL			0.5
#define LV10_MAX_ACCEL			2
#else
#define LV1_MAX_ACCEL			1.5//3.5//2.5//1.5 //for 1v1
#define LV2_MAX_ACCEL			1.5
#define LV3_MAX_ACCEL			1.5
#define LV4_MAX_ACCEL			1.5
#define LV5_MAX_ACCEL			1
#define LV6_MAX_ACCEL			1
#define LV7_MAX_ACCEL			1
#define LV8_MAX_ACCEL			1
#define LV9_MAX_ACCEL			1
#define LV10_MAX_ACCEL			2
#endif

// Yaw max rpm - max RPM for chassis centering
#ifdef ONE_VS_ONE
#define LV1_CHASSIS_YAW_MAX_RPM		0.8//0.6
#define LV1_CHASSIS_YAW_KP			0.7
#define LV1_CHASSIS_YAW_KI			0
#define LV1_CHASSIS_YAW_KD			2//30 // for 1v1

#define LV2_CHASSIS_YAW_MAX_RPM		0.4
#define LV2_CHASSIS_YAW_KP			0.7
#define LV2_CHASSIS_YAW_KI			0
#define LV2_CHASSIS_YAW_KD			2

#define LV3_CHASSIS_YAW_MAX_RPM		0.4
#define LV3_CHASSIS_YAW_KP			0.7
#define LV3_CHASSIS_YAW_KI			0
#define LV3_CHASSIS_YAW_KD			0

#define LV4_CHASSIS_YAW_MAX_RPM		0.4
#define LV4_CHASSIS_YAW_KP			0.7
#define LV4_CHASSIS_YAW_KI			0
#define LV4_CHASSIS_YAW_KD			0

#define LV5_CHASSIS_YAW_MAX_RPM		0.4
#define LV5_CHASSIS_YAW_KP			0.7
#define LV5_CHASSIS_YAW_KI			0
#define LV5_CHASSIS_YAW_KD			0

#define LV6_CHASSIS_YAW_MAX_RPM		0.4
#define LV6_CHASSIS_YAW_KP			0.7
#define LV6_CHASSIS_YAW_KI			0
#define LV6_CHASSIS_YAW_KD			0

#define LV7_CHASSIS_YAW_MAX_RPM		0.4
#define LV7_CHASSIS_YAW_KP			0.7
#define LV7_CHASSIS_YAW_KI			0
#define LV7_CHASSIS_YAW_KD			0

#define LV8_CHASSIS_YAW_MAX_RPM		0.4
#define LV8_CHASSIS_YAW_KP			0.7
#define LV8_CHASSIS_YAW_KI			0
#define LV8_CHASSIS_YAW_KD			0

#define LV9_CHASSIS_YAW_MAX_RPM		0.4
#define LV9_CHASSIS_YAW_KP			0.7
#define LV9_CHASSIS_YAW_KI			0
#define LV9_CHASSIS_YAW_KD			0

#define LV10_CHASSIS_YAW_MAX_RPM	0.8
#define LV10_CHASSIS_YAW_KP			0.7
#define LV10_CHASSIS_YAW_KI			0
#define LV10_CHASSIS_YAW_KD			2
#else
#define LV1_CHASSIS_YAW_MAX_RPM		0.25
#define LV1_CHASSIS_YAW_KP			0.7
#define LV1_CHASSIS_YAW_KI			0
#define LV1_CHASSIS_YAW_KD			2//0 //for 1v1

#define LV2_CHASSIS_YAW_MAX_RPM		0.35
#define LV2_CHASSIS_YAW_KP			0.7
#define LV2_CHASSIS_YAW_KI			0
#define LV2_CHASSIS_YAW_KD			8

#define LV3_CHASSIS_YAW_MAX_RPM		0.40
#define LV3_CHASSIS_YAW_KP			0.7
#define LV3_CHASSIS_YAW_KI			0
#define LV3_CHASSIS_YAW_KD			2

#define LV4_CHASSIS_YAW_MAX_RPM		0.40
#define LV4_CHASSIS_YAW_KP			0.7
#define LV4_CHASSIS_YAW_KI			0
#define LV4_CHASSIS_YAW_KD			2

#define LV5_CHASSIS_YAW_MAX_RPM		0.40
#define LV5_CHASSIS_YAW_KP			0.7
#define LV5_CHASSIS_YAW_KI			0
#define LV5_CHASSIS_YAW_KD			2

#define LV6_CHASSIS_YAW_MAX_RPM		0.40
#define LV6_CHASSIS_YAW_KP			0.7
#define LV6_CHASSIS_YAW_KI			0
#define LV6_CHASSIS_YAW_KD			2

#define LV7_CHASSIS_YAW_MAX_RPM		0.40
#define LV7_CHASSIS_YAW_KP			0.7
#define LV7_CHASSIS_YAW_KI			0
#define LV7_CHASSIS_YAW_KD			2

#define LV8_CHASSIS_YAW_MAX_RPM		0.40
#define LV8_CHASSIS_YAW_KP			0.7
#define LV8_CHASSIS_YAW_KI			0
#define LV8_CHASSIS_YAW_KD			2

#define LV9_CHASSIS_YAW_MAX_RPM		0.40
#define LV9_CHASSIS_YAW_KP			0.7
#define LV9_CHASSIS_YAW_KI			0
#define LV9_CHASSIS_YAW_KD			2

#define LV10_CHASSIS_YAW_MAX_RPM	0.40
#define LV10_CHASSIS_YAW_KP			0.7
#define LV10_CHASSIS_YAW_KI			0
#define LV10_CHASSIS_YAW_KD			2
#endif
#else

#define MAX_SPEED 		    0.7
#define MAX_ACCEL			2

#define CHASSIS_YAW_MAX_RPM   0.7
#define CHASSIS_YAW_KP        1
#define CHASSIS_YAW_KI        0
#define CHASSIS_YAW_KD        15
#endif

#define CHASSIS_YAW_MIN			0.05	// value below which chassis yaw movement is ignored
#define SPIN_ACCELERATION		0.50	// Same guideline as chassis acceleration

#define CHASSIS_SPEED_BOOST		0.15	// Increase MAX_SPEED when spinspin mode is deactivated

/* To configure centers, start the boards in debug mode with all motors
 * powered *but in safe mode* (i.e. remotes off)
 * Physically push the motors to the desired centers
 * and put a breakpoint/live expression on their respective real_ang variables
 * from their raw_data structs
 * The centers should be from 0 to 8192 (for non-DM motors), it should be the value directly from
 * the motors
 * Centers for DM motors should be -PI to PI.
 */
/*********************** GIMBAL CONFIGURATION ***********************/
#define PITCH_SINGLE_PID_LOOP	// define to enable single PID loop instead of cascade PID for pitch
#define PITCH_MOTOR_TYPE		TYPE_DM4310_DJI_MODE

#if PITCH_MOTOR_TYPE != TYPE_DM4310_MIT
#ifndef PITCH_SINGLE_PID_LOOP
#define PITCH_ANGLE_KP	  		80
#define PITCH_ANGLE_KI  		0
#define PITCH_ANGLE_KD  		3
#define PITCH_ANGLE_INT_MAX		0
#endif
#define PITCH_MAX_RPM			60
#define PITCHRPM_KP				36
#define PITCHRPM_KI				0
#define PITCHRPM_KD				70
#define PITCHRPM_INT_MAX		4000
#define PITCH_MAX_CURRENT		205000

#else

#define DM_PITCH_KP				9
#define DM_PITCH_KI				0
#define DM_PITCH_KD				28
#define DM_PITCH_INT_MAX		0
#define DM_PITCH_MAX_OUT		3
#define DM_PITCH_MODE			0	// 0 - MIT, 1 - Position, 2 - Speed

#define DM_PITCH_MIT_KP			0
#define DM_PITCH_MIT_KD			0
#define DM_PITCH_MIT_POS		0
#define DM_PITCH_MIT_VEL		0
#define DM_PITCH_MIT_TOR		0

#endif

#define PITCH_CENTER			990 	// irrelevant for imu control
#define PITCH_MAX_ANG			0.75
#define PITCH_MIN_ANG			-0.52
#define PITCH_CONST 			0

//#define YAW_SINGLE_PID_LOOP		// define to enable single PID loop instead of cascade PID for yaw
#define YAW_MOTOR_TYPE 			TYPE_GM6020

#if YAW_MOTOR_TYPE != TYPE_DM4310_MIT
#ifndef YAW_SINGLE_PID_LOOP

#define YAW_ANGLE_KP			5
#define YAW_ANGLE_KI			0	// 0.0001 Should be very small, just to correct run-off or oscillation errors
#define YAW_ANGLE_KD			0.15	//3000
#define YAW_ANGLE_INT_MAX		0.1
#define YAW_MAX_RPM				132		//85

#endif

#define YAWRPM_KP				8000
#define YAWRPM_KI				0
#define YAWRPM_KD				0
#define YAWRPM_INT_MAX			1000
#define YAW_MAX_CURRENT			20000

#else

#define DM_YAW_KP				5
#define DM_YAW_KI				0
#define DM_YAW_KD				0
#define DM_YAW_INT_MAX			0
#define DM_YAW_MAX_OUT			45
#define DM_YAW_MODE				0	// 0 - MIT, 1 - Position, 2 - Speed

#define DM_YAW_MIT_KP			0
#define DM_YAW_MIT_KD			0
#define DM_YAW_MIT_POS			0
#define DM_YAW_MIT_VEL			0
#define DM_YAW_MIT_TOR			0

#endif

#define YAW_SPINSPIN_CONSTANT	5000
#define YAW_CENTER 				6473
#define YAW_MAX_ANG				0 // unused
#define YAW_MIN_ANG				0 // unused

/*********************** MOTOR ID CONFIGURATIONS *******************/
// NOTE: two motors on the same CAN CANNOT have the same flashing number
//#define ACTIVE_GUIDANCE	// define to enable 4 flwheel firing system (for sniping hero)
#define LAUNCHER_MOTOR_CAN	&hcan2
#define LFRICTION_MOTOR_ID	1
#define RFRICTION_MOTOR_ID	2
#ifdef ACTIVE_GUIDANCE
//#define BFRICTION_MOTOR_ID	3
//#define GFRICTION_MOTOR_ID	4
#endif

#define FEEDER_MOTOR_CAN	&hcan1
#define FEEDER_MOTOR_ID		3

#define PITCH_MOTOR_CAN		&hcan1
#define PITCH_MOTOR_ID 		0x1
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
#define DM_PITCH_MOTOR_ID	0x91 // for DM receiving can ID
#endif

#define YAW_MOTOR_CAN		&hcan1
#define YAW_MOTOR_ID 		5

/*********************** OTHERS ***********************/
#define WHEEL_CIRC			47.1	//in CM

#define CONTROL_DELAY 		5
#define GIMBAL_DELAY		2
#define CHASSIS_DELAY 		5
#define LAUNCHER_DELAY		5

//microsecond timer used for PIDs
#define TIMER_FREQ			1000000 //Cannot be too high if not the ISRs overload the CPU

#endif /* ROBOT_CONFIG_ROBOT_CONFIG_KIRBEE_H_ */
