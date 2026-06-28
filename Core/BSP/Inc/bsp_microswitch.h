/*
 * bsp_microswitch.h
 *
 *  Created on: Dec 8, 2024
 *      Author: cw
 */

#ifndef BSP_INC_BSP_MICROSWITCH_H_
#define BSP_INC_BSP_MICROSWITCH_H_

#include "board_lib.h"

void microswitch_int();
void microswitch_int1();
void microswitch_int2();

extern uint8_t projectile_loaded;
extern uint8_t gimbal_upper_bound;
extern uint8_t gimbal_lower_bound;

#endif /* BSP_INC_BSP_MICROSWITCH_H_ */
