################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/FB6Lib.c \
../src/Gps.c \
../src/adc.c \
../src/batterymonitor.c \
../src/cr_startup_lpc176x.c \
../src/i2c.c \
../src/inertial.c \
../src/lcd.c \
../src/motorcontroller.c \
../src/sensorboard.c \
../src/uart.c 

OBJS += \
./src/FB6Lib.o \
./src/Gps.o \
./src/adc.o \
./src/batterymonitor.o \
./src/cr_startup_lpc176x.o \
./src/i2c.o \
./src/inertial.o \
./src/lcd.o \
./src/motorcontroller.o \
./src/sensorboard.o \
./src/uart.o 

C_DEPS += \
./src/FB6Lib.d \
./src/Gps.d \
./src/adc.d \
./src/batterymonitor.d \
./src/cr_startup_lpc176x.d \
./src/i2c.d \
./src/inertial.d \
./src/lcd.d \
./src/motorcontroller.d \
./src/sensorboard.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -D__USE_CMSIS -DDEBUG -D__CODE_RED -I"/home/anagham/Downloads/Courses/ERTS/workspace/FB6Lib_v_01_00/inc" -I"/home/anagham/Downloads/Courses/ERTS/workspace/CMSISv2_LPC17xx/inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


