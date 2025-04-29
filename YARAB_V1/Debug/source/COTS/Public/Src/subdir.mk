################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/COTS/Public/Src/helpful.c 

C_DEPS += \
./source/COTS/Public/Src/helpful.d 

OBJS += \
./source/COTS/Public/Src/helpful.o 


# Each subdirectory must supply rules for building sources it contributes
source/COTS/Public/Src/%.o: ../source/COTS/Public/Src/%.c source/COTS/Public/Src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MKL25Z128VLK4_cm0plus -DCPU_MKL25Z128VLK4 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Work_Space\NXP\gp\YARAB_V1\drivers" -I"D:\Work_Space\NXP\gp\YARAB_V1\utilities" -I"D:\Work_Space\NXP\gp\YARAB_V1\CMSIS_driver" -I"D:\Work_Space\NXP\gp\YARAB_V1\CMSIS" -I"D:\Work_Space\NXP\gp\YARAB_V1\board" -I"D:\Work_Space\NXP\gp\YARAB_V1\source" -I"D:\Work_Space\NXP\gp\YARAB_V1" -I"D:\Work_Space\NXP\gp\YARAB_V1\startup" -O0 -fno-common -g3 -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source-2f-COTS-2f-Public-2f-Src

clean-source-2f-COTS-2f-Public-2f-Src:
	-$(RM) ./source/COTS/Public/Src/helpful.d ./source/COTS/Public/Src/helpful.o

.PHONY: clean-source-2f-COTS-2f-Public-2f-Src

