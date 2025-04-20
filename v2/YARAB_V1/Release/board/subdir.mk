################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/board.c \
../board/clock_config.c \
../board/peripherals.c \
../board/pin_mux.c 

C_DEPS += \
./board/board.d \
./board/clock_config.d \
./board/peripherals.d \
./board/pin_mux.d 

OBJS += \
./board/board.o \
./board/clock_config.o \
./board/peripherals.o \
./board/pin_mux.o 


# Each subdirectory must supply rules for building sources it contributes
board/%.o: ../board/%.c board/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MKL25Z128VLK4_cm0plus -DCPU_MKL25Z128VLK4 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -DNDEBUG -D__REDLIB__ -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\drivers" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\utilities" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\CMSIS_driver" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\CMSIS" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\board" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\source" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\startup" -Os -fno-common -g -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-board

clean-board:
	-$(RM) ./board/board.d ./board/board.o ./board/clock_config.d ./board/clock_config.o ./board/peripherals.d ./board/peripherals.o ./board/pin_mux.d ./board/pin_mux.o

.PHONY: clean-board

