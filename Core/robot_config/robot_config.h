#ifndef TASKS_INC_ROBOT_CONFIG_H_
#define TASKS_INC_ROBOT_CONFIG_H_

//#include "pancake_2026_config.h"
//#include "sentry_2026_config.h"
#include "hero_2026_config.h"

// COMMON CONFIGURATION
#define SPINSPIN_RANDOM_DELAY 50 // todo: seems good to implement this eh

/* Spin-compensation (override in robot config if needed) */
#ifndef CHASSIS_GIMBAL_ANGLE_OFFSET
#define CHASSIS_GIMBAL_ANGLE_OFFSET  0.0f
#endif
#ifndef SPIN_ANGLE_LEAD
#define SPIN_ANGLE_LEAD              0.0f
#endif
#ifndef SPIN_DRIFT_COMPENSATION
#define SPIN_DRIFT_COMPENSATION      0.0f
#endif

// 1 for annoying beep sound, 0 for some error beeps every 3s, -1 for absolute peace and tranquility
#define MOTOR_ONLINE_CHECK 	-1

// set to allow flywheels to spin during standby: 0 (disable feature), 1(spin when in comp), 2(always spin)
#define FRICTION_SB_SPIN_ON	0

// if no overrides in the respective configs
#ifndef CONTROL_DEFAULT
//#define CONTROL_DEFAULT 		KEYBOARD_CTRL_MODE
#define CONTROL_DEFAULT			REMOTE_CTRL_MODE
//#define CONTROL_DEFAULT		SBC_CTRL_MODE
#endif

/*********************** OTHERS ***********************/
#define MOTOR_TIMEOUT_MAX	1000000	// time above which a motor is considered to be disconnected
#define RC_LIMITS			660 	// limits for remote controller
#define HITEMP_WARNING  	70		// temp above which the motor starts to beep
#define TIMER_FREQ			1000000 // microsecond timer used for PIDs; Cannot be too high if not the ISRs overload the CPU

#define KEY_OFFSET_W        ((uint16_t)0x01<<0)
#define KEY_OFFSET_S        ((uint16_t)0x01<<1)
#define KEY_OFFSET_A 		((uint16_t)0x01<<2)
#define KEY_OFFSET_D        ((uint16_t)0x01<<3)
#define KEY_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define KEY_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define KEY_OFFSET_Q        ((uint16_t)0x01<<6)
#define KEY_OFFSET_E        ((uint16_t)0x01<<7)
#define KEY_OFFSET_R        ((uint16_t)0x01<<8)
#define KEY_OFFSET_F        ((uint16_t)0x01<<9)
#define KEY_OFFSET_G        ((uint16_t)0x01<<10)
#define KEY_OFFSET_X        ((uint16_t)0x01<<11)
#define KEY_OFFSET_Z        ((uint16_t)0x01<<12)
#define KEY_OFFSET_C        ((uint16_t)0x01<<13)
#define KEY_OFFSET_V        ((uint16_t)0x01<<14)
#define KEY_OFFSET_B        ((uint16_t)0x01<<15)

#endif /* TASKS_INC_ROBOT_CONFIG_H_ */

