/*
 * robot_config.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_ROBOT_CONFIG_HERO_H_
#define TASKS_INC_ROBOT_CONFIG_HERO_H_



#define BULLET_42
#define PITCH_ARM		// uses 4 bar linkage for pitch control
//#define BOARD_DOWN
//0 for SWDIO port to be roll, 1 for SWDIO port to be pitch
#define IMU_ORIENTATION	0
//doesn't do anything, todo: implement pid for heater
#define IMU_TARGET_TEMP	50

//flip until motor angle and yaw angle matches
#define IMU_YAW_INVERT		-1
#define IMU_PITCH_INVERT	-1
//nothing uses roll.....yet
#define IMU_ROLL_INVERT		1
//#define IST8310
#define REF_POWER_LIM


/********************* CONTROL SENSITIVITIES ***********/
#define REMOTE_YAW_SPEED 	 			0.05 			//Speed of gimbal yaw turning
#define REMOTE_PITCH_SPEED 	 		0.05//0.05		//Speed of gimbal pitch turning

#define MOUSE_X_SENSITIVITY		(400 * REMOTE_YAW_SPEED)				//Speed of yaw turning with mouse, dependent on above speed
#define MOUSE_Y_SENSITIVITY 	(300 * REMOTE_PITCH_SPEED)				//Speed of pitch turning with mouse,  dependent on above speed



/*********************** REFEREE SYSTEM CONFIGURATION *******************/
#define FRICTION_SB_SPIN		1000
#define ANGLE_FEEDER
#define ANGLE_FEEDER_MARGIN		0.05 //margin in radians
#define ANGLE_FEEDER_SPD_MARGIN 10 //rpm after gearbox margin
#define ANGLE_FEEDER_TIMEOUT    100 //if stuck in angle feeder for more than 2s, timeout
#define ANGLE_FEEDER_DELAY		20

#define OVERHEAT_TIME			500

#define GEAR_DEFAULT			3
#define LV1_FEEDER				100//480//480
#define	LV1_PROJECTILE			15//`b0//18//20//12
#define LV1_POWER				55
#define LV1_MAX_SPEED			5000//3000//3500
#define LV1_MAX_CURRENT			5000//3000

#define LV2_FEEDER				100
#define	LV2_PROJECTILE			15//4//12
#define LV2_POWER				60
#define LV2_MAX_SPEED			7000//4000
#define LV2_MAX_CURRENT			5000

#define LV3_FEEDER				100
#define	LV3_PROJECTILE			15//4//12
#define LV3_POWER				65
#define LV3_MAX_SPEED			9000//4500
#define LV3_MAX_CURRENT			5500

#define GEAR1_YAW_MULT			1
#define GEAR1_SPEED_MULT		0.4
#define GEAR1_ACCEL_MULT		1

#define GEAR2_YAW_MULT			1
#define GEAR2_SPEED_MULT		0.7
#define GEAR2_ACCEL_MULT		1

#define GEAR3_YAW_MULT			1
#define GEAR3_SPEED_MULT		1
#define GEAR3_ACCEL_MULT		1

#define GEAR4_YAW_MULT			1.2
#define GEAR4_SPEED_MULT		1.2
#define GEAR4_ACCEL_MULT		1.6

#define GEAR5_YAW_MULT			2
#define GEAR5_SPEED_MULT		2
#define GEAR5_ACCEL_MULT		2


#define GEAR6_YAW_MULT			3
#define GEAR6_SPEED_MULT		5
#define GEAR6_ACCEL_MULT		10


#define CHASSIS_POWER_KP 0.05
#define CHASSIS_POWER_MARGIN 10

#define CHASSIS_POWER_MULT		1
//todo: implement power settings lol
//#define MOTOR_CURRENT_RATIO		(CHASSIS_MAX_CURRENT / 10000) 	// 10000 mA at max current

#define PROJECTILE_SPEED_RATIO	360								//rpm per m/s of the friction wheels ish don't think this will work well lmao
#define FEEDER_SPEED_RATIO		5//8								//projectiles per round of the feeder

/*********************** MANUAL CONTROL CONFIGURATION *******************/
//Inverts for both keyboard and mouse controls
#define YAW_INVERT  			-1				//1 to invert control -1 to disable
#define PITCH_INVERT  			-1				//1 to invert control -1 to disable
#define MOUSE_X_INVERT			1				//Set to -1 if it needs to be inverted
#define	MOUSE_Y_INVERT			-1				//Set to -1 if it needs to be inverted

#define KEYBD_MAX_SPD 			0.9//0.5				//% of max speed
#define GIMBAL_MODE 			1				//1 for IMU control, 0 for absolute angle based control

/*********************** AIMBOT CONFIGURATION *******************/
#define AIMBOT_YAW_MULT 		0.402 //0.602				//FOV of X axis/2 and invert
#define AIMBOT_PIT_MULT 		0.314 //0.4				//FOV of y aaxis/2 and invert
#define XAVIER_TIMEOUT 			100				//Time before robot returns to manual control

#define AIMBOT_Y_OFFSET			0				//Y point for the robot to aim at
#define AIMBOT_Y_KP				1
#define AIMBOT_Y_KI				0.01
#define AIMBOT_Y_KD				0

#define AIMBOT_X_OFFSET			0				//X Point for the robot to aim at
#define AIMBOT_X_KP				1.2
#define AIMBOT_X_KI				0.01
#define AIMBOT_X_KD				0
#define FOV_MULT				(0.747/2)		//FOV of the camera in radians, change depending on lens specs
#define AIMBOT_KI_MAX			1


#define OBC_DATA_SIZE			8				//Packet size


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
#define FEEDER_KP 			8				// |
#define FEEDER_KI  			0.05				// | - FEEDER PID VALUES
#define FEEDER_KD  			2			// |
#define FEEDER_MAX_INT		15000

#define FEEDER_ANGLE_KP 			300			// |
#define FEEDER_ANGLE_KD  			0			// | - FEEDER_ANGLE PID VALUES
#define FEEDER_ANGLE_KI  			0			// |
#define FEEDER_ANGLE_INT_MAX  		0			// |
#define FEEDER_MAX_RPM				120			// |
#define FEEDER_JAM_TORQUE  		15000			//Torque Before feeder deemed to be jammed
#define FEEDER_UNJAM_SPD  		-100				//Reverse unjam speed
#define FEEDER_UNJAM_TIME		800
#define FEEDER_MAX_CURRENT		16384
#define FEEDER_CUTOFF_TEMP  	60
#define FEEDER_INVERT			1
#define STEPPER_ANGLE			1.8

#define OVERHEAT_EXCESS			1


#define FRICTION_KP  			5			// |
#define FRICTION_KI  			0.02			// | - FRICTION WHEELS PID VALUES
#define FRICTION_KD  			10		// |
#define FRICTION_MAX_CURRENT 	16384
#define FRICTION_MAX_INT		10000
#define FRICTION_INVERT			-1
#define LAUNCHER_MARGIN			10000//20//200
#define LAUNCHER_DIFF_MARGIN	10000//20//200
//#define FRICTION_OFFSET			1000

#define CLEAR_DELAY				4000



/*********************** CHASSIS CONFIGURATION ***********************/
#define CHASSIS_KP  		6					// |
#define CHASSIS_KI  		0.02				// | - CHASSIS WHEELS PID VALUES
#define CHASSIS_KD  		1					// |
#define CHASSIS_INT_MAX  	5000				// |
#define CHASSIS_MAX_CURRENT 8000
#define CHASSIS_MIN_CURRENT 0

#define CHASSIS_CAN_SPINSPIN
#define CHASSIS_SPINSPIN_MIN 0.4
#define CHASSIS_SPINSPIN_MAX 0.9
#define CHASSIS_SPINSPIN_ANNOY_STEPS 50
#define CHASSIS_SPINSPIN_MULT 0.03
#define CHASSIS_SPINSPIN_RANGE (CHASSIS_SPINSPIN_MAX - CHASSIS_SPINSPIN_MIN)
#define CHASSIS_SPINSPIN_MIN_RAMP 0.002

#define CHASSIS_YAW_MAX_RPM	1					//max RPM for chassis centering
#define CHASSIS_YAW_KP 		0.7					//PID for chassis centering
#define CHASSIS_YAW_KI		0
#define CHASSIS_YAW_KD 		0
#define CHASSIS_YAW_MIN		0.1
#define CHASSIS_TRANS_PRIO		0.6			//% of chassis speed to be prioritised for translation
#define CHASSIS_YAW_PRIO		(1-CHASSIS_TRANS_PRIO)

#define CHASSIS_MAX_POWER 		400
#define MAX_SPEED 			M3508_MAX_RPM 				//Max speed of robot

/* To configure centers, start the boards in debug mode with all motors
 * powered *but in safe mode* (i.e. remotes off)
 * Physically push the motors to the desired centers
 * and put a breakpoint/live expression on their respective real_ang variables
 * from their raw_data structs
 * The centers should be from 0 to 8192, it should be the value directly from
 * the motors
 */
/*********************** GIMBAL CONFIGURATION ***********************/
#define PITCH_ANGLE_KP	  		400//800//400
#define PITCH_ANGLE_KD  		00
#define PITCH_ANGLE_KI  		0
#define PITCH_ANGLE_INT_MAX		5000

#define PITCHRPM_KP				100
#define PITCHRPM_KI				0.01//0.1
#define PITCHRPM_KD				0
#define PITCHRPM_INT_MAX		10000
#define PITCH_MAX_RPM			200
#define PITCH_MAX_CURRENT		20000

#define PITCH_CENTER			6818
#define PITCH_MAX_ANG			0
#define PITCH_MIN_ANG			-2.0

#define YAW_ANGLE_KP			100//200
#define YAW_ANGLE_KD			0
#define YAW_ANGLE_KI			0
#define YAW_ANGLE_INT_MAX		100

#define YAWRPM_KP				4000
#define YAWRPM_KI				0
#define YAWRPM_KD				0
#define YAWRPM_INT_MAX			5000
#define YAW_MAX_RPM				60
#define YAW_MAX_CURRENT			20000

#define YAW_CENTER 				8180
#define YAW_MAX_ANG				4*PI
#define YAW_MIN_ANG				4*-PI




/*********************** MOTOR CONFIGURATION *******************/
//CAN ids for the motors, for motors on the CAN2 bus, add 12
//ADD 4 TO GM6020 IDS i.e. flashing 5 times = ID 9
//#define CHASSIS_MCU

#ifndef CHASSIS_MCU
#define FR_MOTOR_ID 		13
#define FL_MOTOR_ID 		14
#define BL_MOTOR_ID 		15
#define BR_MOTOR_ID 		16
#endif
#define FEEDER_MOTOR_ID		7
#define LFRICTION_MOTOR_ID	5
#define RFRICTION_MOTOR_ID	6

//NOTE: two motors CANNOT have the same __flashing__ number (i.e. GM6020 id 9 cannot be used
//with any id 6 motors
#define PITCH_MOTOR_ID 		8
#ifndef CHASSIS_MCU
#define YAW_MOTOR_ID 		20
#endif



/* MECANUM WHEEL PROPERTIES */
#define WHEEL_CIRC			7.625	//in CM

#define FR_ANG_X			-PI/4
#define FR_ANG_Y 			-PI/2
#define FR_ANG_PASSIVE		PI/4
#define FR_DIST				312
#define FR_VX_MULT			-1		//-cos(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_VY_MULT			-1		//-sin(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_YAW_MULT			1		//((-FR_DIST * sin(FR_ANG_Y - FR_ANG_PASSIVE - FR_ANG_X)) / (sin(FR_ANG_PASSIVE) * WHEEL_CIRC))

#define FL_ANG_X			PI/4
#define FL_ANG_Y 			PI/2
#define FL_ANG_PASSIVE		-PI/4
#define FL_DIST				312
#define FL_VX_MULT			-1 		//-cos(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_VY_MULT			1		//-sin(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_YAW_MULT			1	//((-FL_DIST * sin(FL_ANG_Y - FL_ANG_PASSIVE - FL_ANG_X)) / (sin(FL_ANG_PASSIVE) * WHEEL_CIRC))

#define BL_ANG_X			(3*PI/4)
#define BL_ANG_Y 			PI/2
#define BL_ANG_PASSIVE		PI/4
#define BL_DIST				312
#define BL_VX_MULT			1		//-cos(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_VY_MULT			1		//-sin(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_YAW_MULT			1	//((-BL_DIST * sin(BL_ANG_Y - BL_ANG_PASSIVE - BL_ANG_X)) / (sin(BL_ANG_PASSIVE) * WHEEL_CIRC))

#define BR_ANG_X			-(3*PI/4)
#define BR_ANG_Y 			-PI/2
#define BR_ANG_PASSIVE		-PI/4
#define	BR_DIST				312
#define BR_VX_MULT			1		//-cos(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_VY_MULT			-1		//-sin(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_YAW_MULT			1	//((-BR_DIST * sin(BR_ANG_Y - BR_ANG_PASSIVE - BR_ANG_X)) / (sin(BR_ANG_PASSIVE) * WHEEL_CIRC))


/*********************** OTHERS ***********************/

#define CONTROL_DELAY 			5
#define GIMBAL_DELAY			10
#define CHASSIS_DELAY 			10

//microsecond timer used for PIDs
#define TIMER_FREQ			1000000 //Cannot be too high if not the ISRs overload the CPU
#endif /* TASKS_INC_ROBOT_CONFIG_H_ */
