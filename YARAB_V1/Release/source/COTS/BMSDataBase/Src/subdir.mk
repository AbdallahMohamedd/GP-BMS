################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/COTS/BMSDataBase/Src/DataBase.c 

C_DEPS += \
./source/COTS/BMSDataBase/Src/DataBase.d 

OBJS += \
./source/COTS/BMSDataBase/Src/DataBase.o 


# Each subdirectory must supply rules for building sources it contributes
source/COTS/BMSDataBase/Src/%.o: ../source/COTS/BMSDataBase/Src/%.c source/COTS/BMSDataBase/Src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MKL25Z128VLK4_cm0plus -DCPU_MKL25Z128VLK4 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -DNDEBUG -D__REDLIB__ -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\drivers" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\utilities" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\CMSIS_driver" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\CMSIS" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\board" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\source" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\startup" -Os -fno-common -g -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source-2f-COTS-2f-BMSDataBase-2f-Src

clean-source-2f-COTS-2f-BMSDataBase-2f-Src:
	-$(RM) ./source/COTS/BMSDataBase/Src/DataBase.d ./source/COTS/BMSDataBase/Src/DataBase.o

.PHONY: clean-source-2f-COTS-2f-BMSDataBase-2f-Src

