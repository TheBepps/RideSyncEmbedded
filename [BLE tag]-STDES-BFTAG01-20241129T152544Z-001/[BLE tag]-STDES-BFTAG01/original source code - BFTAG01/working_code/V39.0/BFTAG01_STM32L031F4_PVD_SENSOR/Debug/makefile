################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BFTAG01_STM32L031F4_PVD_SENSOR.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BFTAG01_STM32L031F4_PVD_SENSOR.list \

OBJCOPY_BIN += \
BFTAG01_STM32L031F4_PVD_SENSOR.bin \


# All Target
all: BFTAG01_STM32L031F4_PVD_SENSOR.elf secondary-outputs

# Tool invocations
BFTAG01_STM32L031F4_PVD_SENSOR.elf: $(OBJS) $(USER_OBJS) C:\WORK\CTMS\FINDER\FIRMWARE\DEV\BFTAG01_STM32L031F4_PVD_SENSOR\STM32L031F4PX_FLASH.ld
	arm-none-eabi-gcc -o "BFTAG01_STM32L031F4_PVD_SENSOR.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\WORK\CTMS\FINDER\FIRMWARE\DEV\BFTAG01_STM32L031F4_PVD_SENSOR\STM32L031F4PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BFTAG01_STM32L031F4_PVD_SENSOR.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BFTAG01_STM32L031F4_PVD_SENSOR.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BFTAG01_STM32L031F4_PVD_SENSOR.list"
	@echo 'Finished building: $@'
	@echo ' '

BFTAG01_STM32L031F4_PVD_SENSOR.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BFTAG01_STM32L031F4_PVD_SENSOR.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
