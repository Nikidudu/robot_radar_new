/*
 * bsp_microswitch.c
 *
 *  Created on: Dec 8, 2024
 *      Author: cw
 */
#include "bsp_microswitch.h"
uint8_t projectile_loaded;

void microswitch_int() {
	uint8_t pin_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	// Falling edge, projectile loaded
	if (pin_state == 0) {
		projectile_loaded = 1;
	} else if (pin_state == 1) {
		projectile_loaded = 0;
	}
}
