################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/complementary.c \
../Src/i2c.c \
../Src/kalman.c \
../Src/main.c \
../Src/mpu6050.c \
../Src/spi.c \
../Src/syscalls.c \
../Src/uart.c 

OBJS += \
./Src/complementary.o \
./Src/i2c.o \
./Src/kalman.o \
./Src/main.o \
./Src/mpu6050.o \
./Src/spi.o \
./Src/syscalls.o \
./Src/uart.o 

C_DEPS += \
./Src/complementary.d \
./Src/i2c.d \
./Src/kalman.d \
./Src/main.d \
./Src/mpu6050.d \
./Src/spi.d \
./Src/syscalls.d \
./Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DSTM32F446xx -c -I../Inc -I"C:/Users/cly/Desktop/STM32_WS/Valkyrie/ThirdParty/FreeRTOS/Source" -I"C:/Users/cly/Desktop/STM32_WS/Valkyrie/ThirdParty/FreeRTOS/Source/include" -I"C:/Users/cly/Desktop/STM32_WS/Valkyrie/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/cly/Desktop/STM32_WS/chip_headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/cly/Desktop/STM32_WS/chip_headers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/complementary.cyclo ./Src/complementary.d ./Src/complementary.o ./Src/complementary.su ./Src/i2c.cyclo ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/kalman.cyclo ./Src/kalman.d ./Src/kalman.o ./Src/kalman.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/mpu6050.cyclo ./Src/mpu6050.d ./Src/mpu6050.o ./Src/mpu6050.su ./Src/spi.cyclo ./Src/spi.d ./Src/spi.o ./Src/spi.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/uart.cyclo ./Src/uart.d ./Src/uart.o ./Src/uart.su

.PHONY: clean-Src

