################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/STM32G4xx_Nucleo/subdir.mk
-include Drivers/BSP/Components/wm8994/subdir.mk
-include Drivers/BSP/Components/st7735/subdir.mk
-include Drivers/BSP/Components/mfxstm32l152/subdir.mk
-include Drivers/BSP/Components/hx8347d/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RT_Temp_Control_Software.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RT_Temp_Control_Software.list \

OBJCOPY_BIN += \
RT_Temp_Control_Software.bin \


# All Target
all: RT_Temp_Control_Software.elf secondary-outputs

# Tool invocations
RT_Temp_Control_Software.elf: $(OBJS) $(USER_OBJS) C:\GitRepository\Real-Time-Embedded-Software-Course-Project\RT_Temp_Control_Software\STM32G431RBTX_FLASH.ld
	arm-none-eabi-gcc -o "RT_Temp_Control_Software.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\GitRepository\Real-Time-Embedded-Software-Course-Project\RT_Temp_Control_Software\STM32G431RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RT_Temp_Control_Software.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RT_Temp_Control_Software.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RT_Temp_Control_Software.list"
	@echo 'Finished building: $@'
	@echo ' '

RT_Temp_Control_Software.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RT_Temp_Control_Software.bin"
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
