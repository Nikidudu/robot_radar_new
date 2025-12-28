/*
 * bsp_gpio.c
 *
 *  Created on: Sep 7, 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_gpio.h"

// code for the controllable 5V power port on the dev C
// meant for RoboMaster Red Laser Sight (EOL)
void laser_on()
{ //set to reset for open day
	HAL_GPIO_WritePin(LASER_GPIO_GPIO_Port, LASER_GPIO_Pin, GPIO_PIN_RESET);
}

void laser_off()
{
	HAL_GPIO_WritePin(LASER_GPIO_GPIO_Port, LASER_GPIO_Pin, GPIO_PIN_RESET);
}
