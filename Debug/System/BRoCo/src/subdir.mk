################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/BRoCo/src/CANBus.cpp \
../System/BRoCo/src/CanSocketDriver.cpp \
../System/BRoCo/src/IOBus.cpp \
../System/BRoCo/src/MessageBus.cpp \
../System/BRoCo/src/NetworkBus.cpp \
../System/BRoCo/src/RoCanDriver.cpp \
../System/BRoCo/src/RoCanFDDriver.cpp \
../System/BRoCo/src/STMUARTDriver.cpp \
../System/BRoCo/src/UDevDriver.cpp 

OBJS += \
./System/BRoCo/src/CANBus.o \
./System/BRoCo/src/CanSocketDriver.o \
./System/BRoCo/src/IOBus.o \
./System/BRoCo/src/MessageBus.o \
./System/BRoCo/src/NetworkBus.o \
./System/BRoCo/src/RoCanDriver.o \
./System/BRoCo/src/RoCanFDDriver.o \
./System/BRoCo/src/STMUARTDriver.o \
./System/BRoCo/src/UDevDriver.o 

CPP_DEPS += \
./System/BRoCo/src/CANBus.d \
./System/BRoCo/src/CanSocketDriver.d \
./System/BRoCo/src/IOBus.d \
./System/BRoCo/src/MessageBus.d \
./System/BRoCo/src/NetworkBus.d \
./System/BRoCo/src/RoCanDriver.d \
./System/BRoCo/src/RoCanFDDriver.d \
./System/BRoCo/src/STMUARTDriver.d \
./System/BRoCo/src/UDevDriver.d 


# Each subdirectory must supply rules for building sources it contributes
System/BRoCo/src/%.o System/BRoCo/src/%.su System/BRoCo/src/%.cyclo: ../System/BRoCo/src/%.cpp System/BRoCo/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Core/robot_config -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/utils/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/BRoCo" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/Build" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/Protocol" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/Threads/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/CMSIS/DSP/Include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/Include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm/EKF" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm/kalman" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/CMSIS/DSP" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/CMSIS/DSP/Source" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-BRoCo-2f-src

clean-System-2f-BRoCo-2f-src:
	-$(RM) ./System/BRoCo/src/CANBus.cyclo ./System/BRoCo/src/CANBus.d ./System/BRoCo/src/CANBus.o ./System/BRoCo/src/CANBus.su ./System/BRoCo/src/CanSocketDriver.cyclo ./System/BRoCo/src/CanSocketDriver.d ./System/BRoCo/src/CanSocketDriver.o ./System/BRoCo/src/CanSocketDriver.su ./System/BRoCo/src/IOBus.cyclo ./System/BRoCo/src/IOBus.d ./System/BRoCo/src/IOBus.o ./System/BRoCo/src/IOBus.su ./System/BRoCo/src/MessageBus.cyclo ./System/BRoCo/src/MessageBus.d ./System/BRoCo/src/MessageBus.o ./System/BRoCo/src/MessageBus.su ./System/BRoCo/src/NetworkBus.cyclo ./System/BRoCo/src/NetworkBus.d ./System/BRoCo/src/NetworkBus.o ./System/BRoCo/src/NetworkBus.su ./System/BRoCo/src/RoCanDriver.cyclo ./System/BRoCo/src/RoCanDriver.d ./System/BRoCo/src/RoCanDriver.o ./System/BRoCo/src/RoCanDriver.su ./System/BRoCo/src/RoCanFDDriver.cyclo ./System/BRoCo/src/RoCanFDDriver.d ./System/BRoCo/src/RoCanFDDriver.o ./System/BRoCo/src/RoCanFDDriver.su ./System/BRoCo/src/STMUARTDriver.cyclo ./System/BRoCo/src/STMUARTDriver.d ./System/BRoCo/src/STMUARTDriver.o ./System/BRoCo/src/STMUARTDriver.su ./System/BRoCo/src/UDevDriver.cyclo ./System/BRoCo/src/UDevDriver.d ./System/BRoCo/src/UDevDriver.o ./System/BRoCo/src/UDevDriver.su

.PHONY: clean-System-2f-BRoCo-2f-src

