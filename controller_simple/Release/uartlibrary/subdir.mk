################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/media/daniel/Daniel-HDD/daniel-X542UQ/Programovanie/AVRLibraries/uartlibrary/test_uart.c 

C_DEPS += \
./uartlibrary/test_uart.d 

OBJS += \
./uartlibrary/test_uart.o 


# Each subdirectory must supply rules for building sources it contributes
uartlibrary/test_uart.o: /media/daniel/Daniel-HDD/daniel-X542UQ/Programovanie/AVRLibraries/uartlibrary/test_uart.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=attiny2313 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


