################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.c 

C_DEPS += \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.d 

OBJS += \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Source/SupportFunctions/%.o Drivers/CMSIS/DSP/Source/SupportFunctions/%.su Drivers/CMSIS/DSP/Source/SupportFunctions/%.cyclo: ../Drivers/CMSIS/DSP/Source/SupportFunctions/%.c Drivers/CMSIS/DSP/Source/SupportFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Core/robot_config -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/System/utils/Inc" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/System/BRoCo/include" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/System/BRoCo/include/BRoCo" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/System/BRoCo/include/Build" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/System/BRoCo/include/Protocol" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/System/Threads/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/User" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/Drivers/CMSIS/DSP/Include" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/Drivers/Include" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/Core/Algorithm" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/Core/Algorithm/EKF" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/Core/Algorithm/kalman" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/Drivers/CMSIS/DSP/Source" -I"C:/Users/dota2/OneDrive - National University of Singapore/Desktop/BALANCINF/Maple/robot_firmware_new/Core/Algorithm/mahony" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-SupportFunctions

clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-SupportFunctions:
	-$(RM) ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.su ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.cyclo ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.d ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.o ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-SupportFunctions

