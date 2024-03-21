################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Tracealyzer/trcKernelPort.c \
../Core/Tracealyzer/trcSnapshotRecorder.c \
../Core/Tracealyzer/trcStreamingRecorder.c 

OBJS += \
./Core/Tracealyzer/trcKernelPort.o \
./Core/Tracealyzer/trcSnapshotRecorder.o \
./Core/Tracealyzer/trcStreamingRecorder.o 

C_DEPS += \
./Core/Tracealyzer/trcKernelPort.d \
./Core/Tracealyzer/trcSnapshotRecorder.d \
./Core/Tracealyzer/trcStreamingRecorder.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Tracealyzer/%.o Core/Tracealyzer/%.su Core/Tracealyzer/%.cyclo: ../Core/Tracealyzer/%.c Core/Tracealyzer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/kmehta/Dropbox/2024 LTU Formula Electric Firmware/RTOS LTU Formula Electric 2023 Code/Core/Tracealyzer/config" -I"C:/Users/kmehta/Dropbox/2024 LTU Formula Electric Firmware/RTOS LTU Formula Electric 2023 Code/Core/Tracealyzer/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Tracealyzer

clean-Core-2f-Tracealyzer:
	-$(RM) ./Core/Tracealyzer/trcKernelPort.cyclo ./Core/Tracealyzer/trcKernelPort.d ./Core/Tracealyzer/trcKernelPort.o ./Core/Tracealyzer/trcKernelPort.su ./Core/Tracealyzer/trcSnapshotRecorder.cyclo ./Core/Tracealyzer/trcSnapshotRecorder.d ./Core/Tracealyzer/trcSnapshotRecorder.o ./Core/Tracealyzer/trcSnapshotRecorder.su ./Core/Tracealyzer/trcStreamingRecorder.cyclo ./Core/Tracealyzer/trcStreamingRecorder.d ./Core/Tracealyzer/trcStreamingRecorder.o ./Core/Tracealyzer/trcStreamingRecorder.su

.PHONY: clean-Core-2f-Tracealyzer

