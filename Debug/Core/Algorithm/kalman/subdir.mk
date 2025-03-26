################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Algorithm/kalman/kalman_filter.c 

C_DEPS += \
./Core/Algorithm/kalman/kalman_filter.d 

OBJS += \
./Core/Algorithm/kalman/kalman_filter.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Algorithm/kalman/%.o Core/Algorithm/kalman/%.su Core/Algorithm/kalman/%.cyclo: ../Core/Algorithm/kalman/%.c Core/Algorithm/kalman/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Core/robot_config -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/System/utils/Inc" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/System/BRoCo/include" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/System/BRoCo/include/BRoCo" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/System/BRoCo/include/Build" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/System/BRoCo/include/Protocol" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/System/Threads/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/User" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/Drivers/CMSIS/DSP/Include" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/Drivers/Include" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/Core/Algorithm" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/Core/Algorithm/EKF" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/Core/Algorithm/kalman" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/Drivers/CMSIS/DSP/Source" -I"C:/Users/YI MING/Desktop/Revert/robot_firmware_new/Core/Algorithm/mahony" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Algorithm-2f-kalman

clean-Core-2f-Algorithm-2f-kalman:
	-$(RM) ./Core/Algorithm/kalman/kalman_filter.cyclo ./Core/Algorithm/kalman/kalman_filter.d ./Core/Algorithm/kalman/kalman_filter.o ./Core/Algorithm/kalman/kalman_filter.su

.PHONY: clean-Core-2f-Algorithm-2f-kalman

