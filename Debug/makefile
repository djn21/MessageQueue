################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
ELFSIZE += \
MessageQueue.elf.size \

ELFCHECK += \
MessageQueue.elf.elfcheck \


# All Target
all: MessageQueue.elf secondary-outputs

# Tool invocations
MessageQueue.elf: $(OBJS) ../src/lscript.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MicroBlaze gcc linker'
	mb-gcc -Wl,-T -Wl,../src/lscript.ld -L../../freertos_bsp_0/microblaze_0/lib -mlittle-endian -mxl-barrel-shift -mxl-pattern-compare -mcpu=v8.50.b -mno-xl-soft-mul -Wl,--no-relax -Wl,--gc-sections -o "MessageQueue.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

MessageQueue.elf.size: MessageQueue.elf
	@echo 'Invoking: MicroBlaze Print Size'
	mb-size MessageQueue.elf  |tee "MessageQueue.elf.size"
	@echo 'Finished building: $@'
	@echo ' '

MessageQueue.elf.elfcheck: MessageQueue.elf
	@echo 'Invoking: Xilinx ELF Check'
	elfcheck MessageQueue.elf -hw ../../E2LP_hw_platform/system.xml -pe microblaze_0  |tee "MessageQueue.elf.elfcheck"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES)$(ELFSIZE)$(ELFCHECK)$(S_UPPER_DEPS) MessageQueue.elf
	-@echo ' '

secondary-outputs: $(ELFSIZE) $(ELFCHECK)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
