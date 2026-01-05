#ifndef BOARD_LIB_H_
#define BOARD_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* --- MCU & Standard Library --- */
#include "stm32f4xx.h"
#include "stdint.h"
#include "arm_math.h"
/* --- RTOS & Middleware --- */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usbd_cdc_if.h"
/* --- Project Config --- */
#include "board_settings.h"
#include "crc8_crc16.h"
#include "robot_config.h"
/* --- BSP Drivers --- */
#include "bsp_queue.h"
#include "bsp_remote.h"
#include "bsp_usart.h"
#include "bsp_referee.h"
#include "bsp_can.h"
#include "bsp_led.h"
#include "bsp_oled.h"
#include "bsp_imu.h"
#include "bsp_buzzer.h"
#include "bsp_gpio.h"
#include "bsp_usb_redir.h"
#include "bsp_micros_timer.h"
#include "bsp_damiao.h"
#include "bsp_lk_motor.h"
#include "bsp_microswitch.h"
#include "bsp_servo.h"

#ifdef __cplusplus
}
#endif

#endif
