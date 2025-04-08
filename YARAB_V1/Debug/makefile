################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/COTS/TempControlManager/Src/subdir.mk
-include source/COTS/SlaveControlIF/Src/subdir.mk
-include source/COTS/ScreenIF/Src/subdir.mk
-include source/COTS/Public/Src/subdir.mk
-include source/COTS/FuSaSupervisor/Src/subdir.mk
-include source/COTS/DiagnosticManager/Src/subdir.mk
-include source/COTS/DebugInfoManager/Src/subdir.mk
-include source/COTS/BatteryStatusMonitor/Src/subdir.mk
-include source/COTS/BMSDataBase/Src/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include board/subdir.mk
-include CMSIS_driver/subdir.mk
-include CMSIS/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := YARAB_V1
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: YARAB_V1.axf

dependents:
	-cd C:/Users/abdal/OneDrive\ -\ Faculty\ of\ Engineering\ Ain\ Shams\ University/Desktop/nxp/YARAB_V1/Release && $(MAKE) all

# Tool invocations
YARAB_V1.axf: $(OBJS) $(USER_OBJS) C:/Users/abdal/OneDrive\ -\ Faculty\ of\ Engineering\ Ain\ Shams\ University/Desktop/nxp/YARAB_V1/Release/YARAB_V1.axf makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="YARAB_V1.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m0plus -mthumb -T YARAB_V1_Debug.ld -o "YARAB_V1.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) YARAB_V1.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "YARAB_V1.axf"; # arm-none-eabi-objcopy -v -O binary "YARAB_V1.axf" "YARAB_V1.bin" ; # checksum -p MKL25Z128xxx4 -d "YARAB_V1.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build
C:/Users/abdal/OneDrive\ -\ Faculty\ of\ Engineering\ Ain\ Shams\ University/Desktop/nxp/YARAB_V1/Release/YARAB_V1.axf:

-include ../makefile.targets
