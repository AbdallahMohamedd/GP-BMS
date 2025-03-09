################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_clock.c \
../drivers/fsl_cmp.c \
../drivers/fsl_common.c \
../drivers/fsl_dma.c \
../drivers/fsl_dmamux.c \
../drivers/fsl_flash.c \
../drivers/fsl_gpio.c \
../drivers/fsl_i2c.c \
../drivers/fsl_i2c_dma.c \
../drivers/fsl_lpsci.c \
../drivers/fsl_lpsci_dma.c \
../drivers/fsl_pit.c \
../drivers/fsl_rcm.c \
../drivers/fsl_rtc.c \
../drivers/fsl_sim.c \
../drivers/fsl_smc.c \
../drivers/fsl_spi.c \
../drivers/fsl_spi_dma.c \
../drivers/fsl_tpm.c \
../drivers/fsl_uart.c \
../drivers/fsl_uart_dma.c 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_cmp.d \
./drivers/fsl_common.d \
./drivers/fsl_dma.d \
./drivers/fsl_dmamux.d \
./drivers/fsl_flash.d \
./drivers/fsl_gpio.d \
./drivers/fsl_i2c.d \
./drivers/fsl_i2c_dma.d \
./drivers/fsl_lpsci.d \
./drivers/fsl_lpsci_dma.d \
./drivers/fsl_pit.d \
./drivers/fsl_rcm.d \
./drivers/fsl_rtc.d \
./drivers/fsl_sim.d \
./drivers/fsl_smc.d \
./drivers/fsl_spi.d \
./drivers/fsl_spi_dma.d \
./drivers/fsl_tpm.d \
./drivers/fsl_uart.d \
./drivers/fsl_uart_dma.d 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_cmp.o \
./drivers/fsl_common.o \
./drivers/fsl_dma.o \
./drivers/fsl_dmamux.o \
./drivers/fsl_flash.o \
./drivers/fsl_gpio.o \
./drivers/fsl_i2c.o \
./drivers/fsl_i2c_dma.o \
./drivers/fsl_lpsci.o \
./drivers/fsl_lpsci_dma.o \
./drivers/fsl_pit.o \
./drivers/fsl_rcm.o \
./drivers/fsl_rtc.o \
./drivers/fsl_sim.o \
./drivers/fsl_smc.o \
./drivers/fsl_spi.o \
./drivers/fsl_spi_dma.o \
./drivers/fsl_tpm.o \
./drivers/fsl_uart.o \
./drivers/fsl_uart_dma.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c drivers/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MKL25Z128VLK4_cm0plus -DCPU_MKL25Z128VLK4 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -DNDEBUG -D__REDLIB__ -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\drivers" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\utilities" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\CMSIS_driver" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\CMSIS" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\board" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\source" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1" -I"C:\Users\abdal\OneDrive - Faculty of Engineering Ain Shams University\Desktop\nxp\YARAB_V1\startup" -Os -fno-common -g -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-drivers

clean-drivers:
	-$(RM) ./drivers/fsl_clock.d ./drivers/fsl_clock.o ./drivers/fsl_cmp.d ./drivers/fsl_cmp.o ./drivers/fsl_common.d ./drivers/fsl_common.o ./drivers/fsl_dma.d ./drivers/fsl_dma.o ./drivers/fsl_dmamux.d ./drivers/fsl_dmamux.o ./drivers/fsl_flash.d ./drivers/fsl_flash.o ./drivers/fsl_gpio.d ./drivers/fsl_gpio.o ./drivers/fsl_i2c.d ./drivers/fsl_i2c.o ./drivers/fsl_i2c_dma.d ./drivers/fsl_i2c_dma.o ./drivers/fsl_lpsci.d ./drivers/fsl_lpsci.o ./drivers/fsl_lpsci_dma.d ./drivers/fsl_lpsci_dma.o ./drivers/fsl_pit.d ./drivers/fsl_pit.o ./drivers/fsl_rcm.d ./drivers/fsl_rcm.o ./drivers/fsl_rtc.d ./drivers/fsl_rtc.o ./drivers/fsl_sim.d ./drivers/fsl_sim.o ./drivers/fsl_smc.d ./drivers/fsl_smc.o ./drivers/fsl_spi.d ./drivers/fsl_spi.o ./drivers/fsl_spi_dma.d ./drivers/fsl_spi_dma.o ./drivers/fsl_tpm.d ./drivers/fsl_tpm.o ./drivers/fsl_uart.d ./drivers/fsl_uart.o ./drivers/fsl_uart_dma.d ./drivers/fsl_uart_dma.o

.PHONY: clean-drivers

