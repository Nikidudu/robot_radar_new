################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Tasks/Src/INS_task.c \
../Core/Tasks/Src/PID.c \
../Core/Tasks/Src/balancing_chassis_task.c \
../Core/Tasks/Src/buzzing_task.c \
../Core/Tasks/Src/can_msg_processor.c \
../Core/Tasks/Src/can_relay_task.c \
../Core/Tasks/Src/control_input_task.c \
../Core/Tasks/Src/control_keyboard.c \
../Core/Tasks/Src/control_remote.c \
../Core/Tasks/Src/control_sbc.c \
../Core/Tasks/Src/dm4310_drv.c \
../Core/Tasks/Src/error_handler.c \
../Core/Tasks/Src/gimbal_control_task.c \
../Core/Tasks/Src/hud_task.c \
../Core/Tasks/Src/imu_processing_task.c \
../Core/Tasks/Src/launcher_control_task.c \
../Core/Tasks/Src/leg_task.c \
../Core/Tasks/Src/lqr_k.c \
../Core/Tasks/Src/master_task.c \
../Core/Tasks/Src/motor_config.c \
../Core/Tasks/Src/motor_control.c \
../Core/Tasks/Src/motor_control_task.c \
../Core/Tasks/Src/movement_control_task.c \
../Core/Tasks/Src/observe_task.c \
../Core/Tasks/Src/referee_processing_task.c \
../Core/Tasks/Src/rt_nonfinite.c \
../Core/Tasks/Src/startup_task.c \
../Core/Tasks/Src/telemetry_task.c \
../Core/Tasks/Src/usb_task.c 

C_DEPS += \
./Core/Tasks/Src/INS_task.d \
./Core/Tasks/Src/PID.d \
./Core/Tasks/Src/balancing_chassis_task.d \
./Core/Tasks/Src/buzzing_task.d \
./Core/Tasks/Src/can_msg_processor.d \
./Core/Tasks/Src/can_relay_task.d \
./Core/Tasks/Src/control_input_task.d \
./Core/Tasks/Src/control_keyboard.d \
./Core/Tasks/Src/control_remote.d \
./Core/Tasks/Src/control_sbc.d \
./Core/Tasks/Src/dm4310_drv.d \
./Core/Tasks/Src/error_handler.d \
./Core/Tasks/Src/gimbal_control_task.d \
./Core/Tasks/Src/hud_task.d \
./Core/Tasks/Src/imu_processing_task.d \
./Core/Tasks/Src/launcher_control_task.d \
./Core/Tasks/Src/leg_task.d \
./Core/Tasks/Src/lqr_k.d \
./Core/Tasks/Src/master_task.d \
./Core/Tasks/Src/motor_config.d \
./Core/Tasks/Src/motor_control.d \
./Core/Tasks/Src/motor_control_task.d \
./Core/Tasks/Src/movement_control_task.d \
./Core/Tasks/Src/observe_task.d \
./Core/Tasks/Src/referee_processing_task.d \
./Core/Tasks/Src/rt_nonfinite.d \
./Core/Tasks/Src/startup_task.d \
./Core/Tasks/Src/telemetry_task.d \
./Core/Tasks/Src/usb_task.d 

OBJS += \
./Core/Tasks/Src/INS_task.o \
./Core/Tasks/Src/PID.o \
./Core/Tasks/Src/balancing_chassis_task.o \
./Core/Tasks/Src/buzzing_task.o \
./Core/Tasks/Src/can_msg_processor.o \
./Core/Tasks/Src/can_relay_task.o \
./Core/Tasks/Src/control_input_task.o \
./Core/Tasks/Src/control_keyboard.o \
./Core/Tasks/Src/control_remote.o \
./Core/Tasks/Src/control_sbc.o \
./Core/Tasks/Src/dm4310_drv.o \
./Core/Tasks/Src/error_handler.o \
./Core/Tasks/Src/gimbal_control_task.o \
./Core/Tasks/Src/hud_task.o \
./Core/Tasks/Src/imu_processing_task.o \
./Core/Tasks/Src/launcher_control_task.o \
./Core/Tasks/Src/leg_task.o \
./Core/Tasks/Src/lqr_k.o \
./Core/Tasks/Src/master_task.o \
./Core/Tasks/Src/motor_config.o \
./Core/Tasks/Src/motor_control.o \
./Core/Tasks/Src/motor_control_task.o \
./Core/Tasks/Src/movement_control_task.o \
./Core/Tasks/Src/observe_task.o \
./Core/Tasks/Src/referee_processing_task.o \
./Core/Tasks/Src/rt_nonfinite.o \
./Core/Tasks/Src/startup_task.o \
./Core/Tasks/Src/telemetry_task.o \
./Core/Tasks/Src/usb_task.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Tasks/Src/%.o Core/Tasks/Src/%.su Core/Tasks/Src/%.cyclo: ../Core/Tasks/Src/%.c Core/Tasks/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Core/robot_config -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/utils/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/BRoCo" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/Build" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/Protocol" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/Threads/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/User" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/CMSIS/DSP/Include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/Include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm/EKF" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm/kalman" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/CMSIS/DSP/Source" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm/mahony" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Tasks-2f-Src

clean-Core-2f-Tasks-2f-Src:
	-$(RM) ./Core/Tasks/Src/INS_task.cyclo ./Core/Tasks/Src/INS_task.d ./Core/Tasks/Src/INS_task.o ./Core/Tasks/Src/INS_task.su ./Core/Tasks/Src/PID.cyclo ./Core/Tasks/Src/PID.d ./Core/Tasks/Src/PID.o ./Core/Tasks/Src/PID.su ./Core/Tasks/Src/balancing_chassis_task.cyclo ./Core/Tasks/Src/balancing_chassis_task.d ./Core/Tasks/Src/balancing_chassis_task.o ./Core/Tasks/Src/balancing_chassis_task.su ./Core/Tasks/Src/buzzing_task.cyclo ./Core/Tasks/Src/buzzing_task.d ./Core/Tasks/Src/buzzing_task.o ./Core/Tasks/Src/buzzing_task.su ./Core/Tasks/Src/can_msg_processor.cyclo ./Core/Tasks/Src/can_msg_processor.d ./Core/Tasks/Src/can_msg_processor.o ./Core/Tasks/Src/can_msg_processor.su ./Core/Tasks/Src/can_relay_task.cyclo ./Core/Tasks/Src/can_relay_task.d ./Core/Tasks/Src/can_relay_task.o ./Core/Tasks/Src/can_relay_task.su ./Core/Tasks/Src/control_input_task.cyclo ./Core/Tasks/Src/control_input_task.d ./Core/Tasks/Src/control_input_task.o ./Core/Tasks/Src/control_input_task.su ./Core/Tasks/Src/control_keyboard.cyclo ./Core/Tasks/Src/control_keyboard.d ./Core/Tasks/Src/control_keyboard.o ./Core/Tasks/Src/control_keyboard.su ./Core/Tasks/Src/control_remote.cyclo ./Core/Tasks/Src/control_remote.d ./Core/Tasks/Src/control_remote.o ./Core/Tasks/Src/control_remote.su ./Core/Tasks/Src/control_sbc.cyclo ./Core/Tasks/Src/control_sbc.d ./Core/Tasks/Src/control_sbc.o ./Core/Tasks/Src/control_sbc.su ./Core/Tasks/Src/dm4310_drv.cyclo ./Core/Tasks/Src/dm4310_drv.d ./Core/Tasks/Src/dm4310_drv.o ./Core/Tasks/Src/dm4310_drv.su ./Core/Tasks/Src/error_handler.cyclo ./Core/Tasks/Src/error_handler.d ./Core/Tasks/Src/error_handler.o ./Core/Tasks/Src/error_handler.su ./Core/Tasks/Src/gimbal_control_task.cyclo ./Core/Tasks/Src/gimbal_control_task.d ./Core/Tasks/Src/gimbal_control_task.o ./Core/Tasks/Src/gimbal_control_task.su ./Core/Tasks/Src/hud_task.cyclo ./Core/Tasks/Src/hud_task.d ./Core/Tasks/Src/hud_task.o ./Core/Tasks/Src/hud_task.su ./Core/Tasks/Src/imu_processing_task.cyclo ./Core/Tasks/Src/imu_processing_task.d ./Core/Tasks/Src/imu_processing_task.o ./Core/Tasks/Src/imu_processing_task.su ./Core/Tasks/Src/launcher_control_task.cyclo ./Core/Tasks/Src/launcher_control_task.d ./Core/Tasks/Src/launcher_control_task.o ./Core/Tasks/Src/launcher_control_task.su ./Core/Tasks/Src/leg_task.cyclo ./Core/Tasks/Src/leg_task.d ./Core/Tasks/Src/leg_task.o ./Core/Tasks/Src/leg_task.su ./Core/Tasks/Src/lqr_k.cyclo ./Core/Tasks/Src/lqr_k.d ./Core/Tasks/Src/lqr_k.o ./Core/Tasks/Src/lqr_k.su ./Core/Tasks/Src/master_task.cyclo ./Core/Tasks/Src/master_task.d ./Core/Tasks/Src/master_task.o ./Core/Tasks/Src/master_task.su ./Core/Tasks/Src/motor_config.cyclo ./Core/Tasks/Src/motor_config.d ./Core/Tasks/Src/motor_config.o ./Core/Tasks/Src/motor_config.su ./Core/Tasks/Src/motor_control.cyclo ./Core/Tasks/Src/motor_control.d ./Core/Tasks/Src/motor_control.o ./Core/Tasks/Src/motor_control.su ./Core/Tasks/Src/motor_control_task.cyclo ./Core/Tasks/Src/motor_control_task.d ./Core/Tasks/Src/motor_control_task.o ./Core/Tasks/Src/motor_control_task.su ./Core/Tasks/Src/movement_control_task.cyclo ./Core/Tasks/Src/movement_control_task.d ./Core/Tasks/Src/movement_control_task.o ./Core/Tasks/Src/movement_control_task.su ./Core/Tasks/Src/observe_task.cyclo ./Core/Tasks/Src/observe_task.d ./Core/Tasks/Src/observe_task.o ./Core/Tasks/Src/observe_task.su ./Core/Tasks/Src/referee_processing_task.cyclo ./Core/Tasks/Src/referee_processing_task.d ./Core/Tasks/Src/referee_processing_task.o ./Core/Tasks/Src/referee_processing_task.su ./Core/Tasks/Src/rt_nonfinite.cyclo ./Core/Tasks/Src/rt_nonfinite.d ./Core/Tasks/Src/rt_nonfinite.o ./Core/Tasks/Src/rt_nonfinite.su ./Core/Tasks/Src/startup_task.cyclo ./Core/Tasks/Src/startup_task.d ./Core/Tasks/Src/startup_task.o ./Core/Tasks/Src/startup_task.su ./Core/Tasks/Src/telemetry_task.cyclo ./Core/Tasks/Src/telemetry_task.d ./Core/Tasks/Src/telemetry_task.o ./Core/Tasks/Src/telemetry_task.su ./Core/Tasks/Src/usb_task.cyclo ./Core/Tasks/Src/usb_task.d ./Core/Tasks/Src/usb_task.o ./Core/Tasks/Src/usb_task.su

.PHONY: clean-Core-2f-Tasks-2f-Src

