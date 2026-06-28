/*
 * typedefs.h
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#ifndef UTILS_INC_TYPEDEFS_H_
#define UTILS_INC_TYPEDEFS_H_

#include <semphr.h>
#include <event_groups.h>

enum sw
{
	SW_SHUTDOWN = 1,
	SW_GIMBAL = 2,
	SW_ALL_ON = 3
};

enum button_press
{
	BUTTON_NOT_PRESSED = 0,
	BUTTON_PRESSED = 1,
};

#define KEYBOARD_CTRL_MODE	1
#define REMOTE_CTRL_MODE	2
#define SBC_CTRL_MODE 		3

enum rb_mode_t{
	ge_RB_KEYBOARD,
	ge_RB_REMOTE,
	ge_RB_SBC_CTRL
};

typedef struct
{
	float kp;
	float ki;
	float kd;
	float kff;
	float error[2];
	float integral;
	float int_max; 			// maximum allowed integral
	float max_out; 			// maximum output of the PID loop
	float output; 			// PID output
	float physical_max; 	// physical maximum the motor can handle
	uint32_t last_time[2]; 	// timestampes of previous PID updates
} pid_data_t;

typedef struct	{
	// processed encoder/odometry data for a motor
	int32_t ticks;
	int32_t center_ang; // reference zero angle of the motor (for pitch and yaw)
	int32_t min_ticks;
	int32_t max_ticks;
	int32_t tick_range;
	int32_t max_raw_ticks;
	int32_t min_raw_ticks;
	int32_t raw_ticks_range;
	float min_ang;
	float max_ang;
	float ang_range;

	float phy_min_ang;
	float phy_max_ang;
	float gearbox_ratio;
	float adj_ang;		// Adjusted angle in radians (centered, scaled from ticks)
	float dist;
	float wheel_circ; 	// in cm
	float hires_rpm;
	uint8_t init;		// Flag to indicate if odometry has been initialized (first run)
} angle_data_t;

typedef struct {
	// raw feedback from CAN motors
	int32_t angle[2];			// raw encoder readings
	int64_t raw_motor_angle;
	int32_t raw_encoder;
	int32_t encoder_offset;
	int16_t rpm;
	int16_t torque;
	uint8_t temp;
} raw_data_t;

typedef struct {
	CAN_HandleTypeDef *can;
	uint16_t id;
	uint8_t motor_type;
	raw_data_t raw_data;
	pid_data_t rpm_pid;
	pid_data_t angle_pid;
	angle_data_t angle_data;
	float output;
	uint32_t last_time[2];
} motor_data_t;

/* Struct containing cleaned data from remote */
typedef struct {
	/* Joysticks - Values range from -660 to 660 */
	int16_t right_x;
	int16_t right_y;
	int16_t left_x;
	int16_t left_y;
	/* Switches / buttons */
    uint8_t sw;  			// 2-bit mode switch (0-2)
    uint8_t control_mode;   // pause button (0-1)
    uint8_t fn_1;     		// function button 1 (0-1)
    uint8_t fn_2;     		// function button 2 (0-1)
    uint8_t trigger;  		// trigger button (0-1)
    /* Side dial - Values range from -660 to 660 */
	int16_t side_dial;
	/* Mouse movement - Values range from -32768 to 32767 */
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int32_t mouse_hori;
	int32_t mouse_vert;
	/* Mouse clicks - Values range from 0 to 1 */
	uint8_t mouse_left;
	uint8_t mouse_right;
    uint8_t mouse_middle;

	/* Keyboard keys mapping
	 * Bit0 -- W 键
	 * Bit1 -- S 键
	 * Bit2 -- A 键
	 * Bit3 -- D 键
	 * Bit4 -- Shift 键
	 * Bit5 -- Ctrl 键
	 * Bit6 -- Q 键
	 * Bit7 -- E 键
	 * Bit8 -- R 键
	 * Bit9 -- F 键
	 * Bit10 -- G 键
	 * Bit11 -- Z 键
	 * Bit12 -- X 键
	 * Bit13 -- C 键
	 * Bit14 -- V 键
	 * Bit15 -- B 键
	 */
	uint16_t keyboard_keys;
	uint32_t last_time;
} remote_cmd_t;

typedef struct
{
	float pit;
	float rol;
	float yaw;

    /* RAW gyro (rad/s) — for speed PID ONLY */
    float gyro_raw_roll;
    float gyro_raw_pitch;
    float gyro_raw_yaw;
} orientation_data_t;

typedef struct
{
	int16_t feeding_speed;
	int16_t projectile_speed;
	float wheel_power_limit;
	float wheel_buffer_limit;
	uint8_t robot_level;
	float chassis_power;
	uint32_t last_update_time;
} referee_limit_t;

typedef struct
{
	float pitch;
	float yaw; 			// only used if GIMBAL_MODE == 0
	float delta_yaw;	// this is the one used for yaw IMU control mode (GIMBAL_MODE == 1)
	uint8_t imu_mode;	// GIMBAL_MODE (1 for IMU control, 0 for absolute angle based control)
	uint8_t enabled;
	SemaphoreHandle_t yaw_semaphore;
	SemaphoreHandle_t pitch_semaphore;
} gimbal_control_t;

typedef struct
{
	float forward;
	float horizontal;
	float yaw;
	uint8_t enabled;
	uint8_t g_spinspin_mode;
	uint32_t last_time[2]; 	// time stamps of previous communication between Dev C's
} chassis_control_t;

typedef struct
{
	int16_t projectile_speed;
	int16_t firing;
	uint8_t override;
	uint8_t enabled;
} gun_control_t;

typedef __PACKED_STRUCT {
    uint8_t header;
    uint8_t cmd_id; //set to 0x80
    uint8_t team;
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remaining_time;
    uint16_t ammo;
    uint8_t padding[5];
    uint8_t end_byte;
} sbc_game_data_t;

// sent to supercap module
typedef struct __attribute__((packed)){
    uint8_t enable_module;	//enable once and leave it (regulation on or off)
    uint8_t reset;			//reset in case got error eg cap voltage too low, UVLO active
    uint8_t pow_limit;		//set power regulation point, ie set to current level power
    uint16_t energy_buffer;	//send over refsys "virtual energy buffer" to abuse
} ref_msg_packet;

// received from supercap module
typedef struct __attribute__((packed)){
	float chassis_power;	//originally meant for feedback,  but not really relevant now, use it however you want eg if exceed too long and sc is dead kill motors for a while??
	uint8_t error;			//any error state
	uint8_t cap_energy;		//normalized energy left in supercap (impt one)
} supercap_msg_packet;

typedef struct {
	float chassis_power;	// idk for now
	uint8_t charging_state; // amount of energy in supercap (0 - 100)
	int supercap_enabled;	// AKA should robot go faster
	uint32_t last_time[2];
} supercap_data;

typedef enum {
	song,
	ok,
	not_ok,
	control_keyboard,
	control_control,
	control_sbc,
	bz_high,
	bz_low,
	bz_debug_low,
	bz_debug_half_rest,
	bz_debug_high,
	bz_debug_rest,
	bz_debug_hi_temp,
	bz_temp_hi,
	bz_temp_low,
} buzzing_type;

enum launcher_state_e {
	WHEEL_STANDBY,
	WHEEL_FIRING,
	WHEEL_CLEARING
};

enum feeder_state_e {
	FEEDER_STANDBY,
	FEEDER_SPINUP,
	FEEDER_STEP,
	FEEDER_JAM,
	FEEDER_OVERHEAT,
	FEEDER_FIRING,
	FEEDER_FREE,
	FEEDER_LOADED,
	FEEDER_FIRING_2,
	FEEDER_FIRING_3
};

#endif /* UTILS_INC_TYPEDEFS_H_ */
