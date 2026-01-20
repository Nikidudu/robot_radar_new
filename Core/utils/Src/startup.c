/*
 * startup.c
 *
 *  Created on: 10 May 2021
 *      Author: wx
 */

#include "board_lib.h"

void system_init() {
	led_on();
	buzzer_init();
	led_green_off();
	start_micros_timer();
	servo_init();

	imu_init();
	can_start(&hcan1, 0x00000000, 0x00000000);
	can_start(&hcan2, 0x00000000, 0x00000000);
}
