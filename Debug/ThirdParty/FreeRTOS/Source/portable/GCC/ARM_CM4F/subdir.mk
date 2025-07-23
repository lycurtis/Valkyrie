################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.o ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.su ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.cyclo: ../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.c ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DSTM32F446xx -c -I../Inc -I"C:/Users/cly/Desktop/STM32_WS/Valkyrie/ThirdParty/FreeRTOS/Source" -I"C:/Users/cly/Desktop/STM32_WS/Valkyrie/ThirdParty/FreeRTOS/Source/include" -I"C:/Users/cly/Desktop/STM32_WS/Valkyrie/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/cly/Desktop/STM32_WS/chip_headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/cly/Desktop/STM32_WS/chip_headers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F

clean-ThirdParty-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F:
	-$(RM) ./ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.cyclo ./ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d ./ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o ./ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.su

.PHONY: clean-ThirdParty-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F

