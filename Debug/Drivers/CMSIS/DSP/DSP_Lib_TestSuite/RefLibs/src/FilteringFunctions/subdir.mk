################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/FilteringFunctions.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/biquad.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/conv.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/correlate.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_decimate.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_interpolate.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_lattice.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_sparse.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/iir_lattice.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/lms.c 

C_DEPS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/FilteringFunctions.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/biquad.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/conv.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/correlate.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_decimate.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_interpolate.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_lattice.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_sparse.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/iir_lattice.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/lms.d 

OBJS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/FilteringFunctions.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/biquad.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/conv.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/correlate.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_decimate.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_interpolate.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_lattice.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_sparse.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/iir_lattice.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/lms.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/%.o Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/%.su Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/%.cyclo: ../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/%.c Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Core/robot_config -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/utils/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/BRoCo" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/Build" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/BRoCo/include/Protocol" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/System/Threads/Inc" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/User" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/CMSIS/DSP/Include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/Include" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm/EKF" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Core/Algorithm/kalman" -I"C:/Users/YI MING/Desktop/MAPLE/robot_firmware_new/Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-RefLibs-2f-src-2f-FilteringFunctions

clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-RefLibs-2f-src-2f-FilteringFunctions:
	-$(RM) ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/FilteringFunctions.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/FilteringFunctions.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/FilteringFunctions.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/FilteringFunctions.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/biquad.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/biquad.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/biquad.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/biquad.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/conv.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/conv.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/conv.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/conv.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/correlate.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/correlate.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/correlate.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/correlate.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_decimate.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_decimate.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_decimate.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_decimate.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_interpolate.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_interpolate.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_interpolate.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_interpolate.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_lattice.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_lattice.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_lattice.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_lattice.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_sparse.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_sparse.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_sparse.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/fir_sparse.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/iir_lattice.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/iir_lattice.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/iir_lattice.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/iir_lattice.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/lms.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/lms.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/lms.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/FilteringFunctions/lms.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-RefLibs-2f-src-2f-FilteringFunctions

