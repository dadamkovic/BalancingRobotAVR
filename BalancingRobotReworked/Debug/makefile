################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
BalancingRobotReworked.lss \

FLASH_IMAGE += \
BalancingRobotReworked.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: BalancingRobotReworked.elf secondary-outputs

# Tool invocations
BalancingRobotReworked.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,BalancingRobotReworked.map,--cref -mrelax -Wl,--gc-sections -mmcu=atmega328p -o "BalancingRobotReworked.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

BalancingRobotReworked.lss: BalancingRobotReworked.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S BalancingRobotReworked.elf  >"BalancingRobotReworked.lss"
	@echo 'Finished building: $@'
	@echo ' '

BalancingRobotReworked.hex: BalancingRobotReworked.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex BalancingRobotReworked.elf  "BalancingRobotReworked.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: BalancingRobotReworked.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p BalancingRobotReworked.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(S_DEPS)$(LSS)$(C_DEPS)$(FLASH_IMAGE)$(CC_DEPS)$(ELFS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) BalancingRobotReworked.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
