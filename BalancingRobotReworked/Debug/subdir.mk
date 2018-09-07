################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../kalman.cpp \
../main.cpp \
../motorControl.cpp \
../mpu6050_IIC.cpp \
../pid.cpp \
../timeTracking.cpp \
../uart.cpp 

OBJS += \
./kalman.o \
./main.o \
./motorControl.o \
./mpu6050_IIC.o \
./pid.o \
./timeTracking.o \
./uart.o 

CPP_DEPS += \
./kalman.d \
./main.d \
./motorControl.d \
./mpu6050_IIC.d \
./pid.d \
./timeTracking.d \
./uart.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -std=c++11 -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


