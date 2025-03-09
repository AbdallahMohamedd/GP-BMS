/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    main.c
 * @brief   Application entry point.
 */



#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stdbool.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_device_registers.h"
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>
#include <COTS/Public/Inc/helpful.h>
#include <COTS/BMSDataBase/Inc/DataBase.h>
#include <COTS/BatteryStatusMonitor/Inc/DataMonitor.h>


// External variables for interrupt flags and systick count
extern volatile bool data_interrupt;
extern volatile bool fault_interrupt;
extern volatile uint32_t systick_count;



int main(void)
{
	uint32_t cc = 0;
	uint32_t aa = 0;
	uint32_t ss = 0;



	// Local variables to store measurement and fault data
	MeasurementData current_Data, previous_Data;
	FaultData current_Faults, previous_Faults;

	///////////////////////////////////////////////////////////////////////////////

	// Variables for Non-Blocking Delay (for the initial delay)
	// uint32_t last_delay_1 = 0;

	//// init phase /////////
	/* Initialize Board */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	BOARD_InitDebugConsole();
#endif

	// Initialize I2C communication
	I2C_init();

	// Configure SysTick timer
	configure_systick();

	// Initialize SPI transfer for Slave Interface
	SlaveIF_TransferInit();
	SlaveIF_SystemSetup(1);
	SlaveIF_SystemConfig1();
	SlaveIF_SystemConfig2();
	SlaveIF_EnableOVUV();
	SlaveIF_CfgAllGpiosForTempSensors();

	///////////////////////////////////////////////////////////////////////////////
	/* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};

	// Initialize GPIO pins for LEDs
	GPIO_PinInit(GPIOA, 5U, &led_config);
	GPIO_PinInit(GPIOA, 4U, &led_config);
	GPIO_PinInit(GPIOA, 12U, &led_config);
	// GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);
	// GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
	// GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);

	// Print welcome message
	PRINTF("Hi Team \n\r\r");

	// Initialize LCD data monitor
	DataMonitor_lcd(60, 95, 0.9, 25.8, 0, 0);


	// Initialize Data by first reading for ever
	DataBase_UpdateMeasurementData(); //Update Measurements
	DataBase_GetCurrentMeasurementData(&current_Data);

	DataBase_UpdateFaultData();  	//Update the Faults Status
	DataBase_GetCurrentFaultData(&current_Faults);






	// Main loop
	while (1)
	{
		if (data_interrupt)
		{
			data_interrupt = false; // clear flag for next event
			DataBase_GetPreviousMeasurementData(&previous_Data);
			DataBase_UpdateMeasurementData(); // Update Measurements
			DataBase_GetCurrentMeasurementData(&current_Data);
			//if ((systick_count - ss) >= 100)
			//{
			//GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
			//GPIO_TogglePinsOutput(GPIOA, 1U<<12U);
			//GPIO_WritePinOutput(GPIOA, 5U, 1);
			//ss = systick_count; // Record the last toggle time
			//PRINTF("Amr\n\r\r");
			//}
		}



		// **BLUE LED IMPLEMENTATION (7 seconds)**
		// Check if it's time to toggle the blue LED
		if ((systick_count - cc) >= 7000)
		{
			//GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
			GPIO_TogglePinsOutput(GPIOA, 1U<<12U);
			//GPIO_WritePinOutput(GPIOA, 5U, 1);
			cc = systick_count; // Record the last toggle time
			PRINTF("BLUEEEEEEEE\n\r\r");
		}



		if (fault_interrupt)
		{
			fault_interrupt = false; // clear flag for next event
			DataBase_GetPreviousFaultData(&previous_Faults); 	// take to last data for compare or make a filter
			DataBase_UpdateFaultData();  //Update the Faults Status
			DataBase_GetCurrentFaultData(&current_Faults);

			//if ((systick_count - aa) >= 100)
			{
				//GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
				//GPIO_TogglePinsOutput(GPIOA, 1U<<12U);
				//GPIO_WritePinOutput(GPIOA, 5U, 1);
				//aa = systick_count; // Record the last toggle time
				//PRINTF("Abdullah\n\r\r");
			}
		}

	}
	return 0;

#ifdef DebugInfoManager
	DebugInfo;
#endif
}
