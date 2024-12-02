################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.o Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.su Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.cyclo: ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.c Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Core/robot_config -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/ybakk/OneDrive/Documents/Robomaster_EL/standard_bot_v5C/System/utils/Inc" -I"C:/Users/ybakk/OneDrive/Documents/Robomaster_EL/standard_bot_v5C/System/BRoCo/include" -I"C:/Users/ybakk/OneDrive/Documents/Robomaster_EL/standard_bot_v5C/System/BRoCo/include/BRoCo" -I"C:/Users/ybakk/OneDrive/Documents/Robomaster_EL/standard_bot_v5C/System/BRoCo/include/Build" -I"C:/Users/ybakk/OneDrive/Documents/Robomaster_EL/standard_bot_v5C/System/BRoCo/include/Protocol" -I"C:/Users/ybakk/OneDrive/Documents/Robomaster_EL/standard_bot_v5C/System/Threads/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.cyclo ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src

