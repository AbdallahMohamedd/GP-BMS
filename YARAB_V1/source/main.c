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


// --- Needed library --- //
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

// --- Platform/MCU Specific Includes --- //
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_device_registers.h"

// --- External Dependencies --- //
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>
#include <COTS/Public/Inc/helpful.h>
#include <COTS/BMSDataBase/Inc/DataBase.h>
#include <COTS/FuSaSupervisor/Inc/FuSa.h>
#include <COTS/BatteryStatusMonitor/Inc/DataMonitor.h>

// --- Global variables for ISR flags --- //
extern volatile bool data_interrupt;
extern volatile bool fault_interrupt;
extern volatile uint32_t systick_count;
#define EN_Transceiver_Port  PORTE   // EN pin port (e.g., PORTE for FRDM-KL25Z)
#define EN_Transceiver_PIN   0U      // EN pin number (e.g., PTB0, common GPIO on KL25Z)


// --- Main Application Entry Point --- //
int main(void)
{
	/////////////////////////////////////////////////////////////////////////////////////////////////
	/* ========================= --- 1. MCU and Board Initialization --- ========================= */
	/////////////////////////////////////////////////////////////////////////////////////////////////
	/* Initialize Board Hardware */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();          // Initializes peripherals configured by MCUXpresso tool
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	BOARD_InitDebugConsole();             // Initialize debug console (for PRINTF)
#endif
	PRINTF("Board Initialized.\n\r\r");


	////////////////////////////////////////////////////////////////////////////////////////////////////
	/* ========================= --- 2. Application Data Initialization --- ========================= */
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Local variables to store current and previous measurement/fault data
	MeasurementData current_Data, previous_Data;
	FaultData current_Faults, previous_Faults;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* ========================= --- 3. GPIO Initialization (Example for LEDs) --- ========================= */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* Define the init structure for the output LED pin */
	gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0}; // Configure as output, initial state low

//GPIO_PinInit(GPIOE, 0U, &led_config); // EN = 1

	// Initialize GPIO pins used for status LEDs
	//GPIO_PinInit(GPIOA, 5U, &led_config);  // Example: Green LED
	GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);
	PRINTF("Status LEDs Initialized.\n\r\r");


	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* ====================== --- 4. Peripheral Initialization (I2C, SPI, Timers) --- ====================== */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize I2C communication (e.g., for an LCD)
	I2C_init(); 				          // This function initializes the I2C peripheral
	PRINTF("I2C Initialized.\n\r\r");

	// Configure SysTick timer for periodic tasks
	configure_systick(); 	              // Configures for 100ms interval
//GPIO_WritePinOutput(GPIOE, 0U, 1);
//delay_seconds(1);
	// Initialize SPI Master for communication with MC33771B
	SlaveIF_initTransfer();               // This function initializes SPI and sets up handles
	PRINTF("SPI Master Initialized.\n\r\r");


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* ==================== --- 5. MC33771B (Slave) Initialization and Configuration --- ==================== */
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	PRINTF("Configuring MC33771B Slave Device...\n\r\r");
	SlaveIF_setupSystem(true);             // This function sets up the system for MC33771B
	SlaveIF_configSystem1();               // This function configures the system settings for MC33771B
	SlaveIF_configSystem2();               // This function configures the system settings for MC33771B
	SlaveIF_configAdc();                   // This function configures the ADC settings for MC33771B
	SlaveIF_configAdcOffset(1);            // This function configures the ADC offset for MC33771B
	SlaveIF_enableOvUv();                  // This function enables over-voltage and under-voltage protection for MC33771B
	SlaveIF_configAllGpiosForTempSensors();// This function configures GPIOs for temperature sensors
	PRINTF("MC33771B Basic Configuration Complete.\n\r\r");


	//////////////////////////////////////////////////////////////////////////////////////////////
	/* ========================= --- 6. Set Protection Thresholds --- ========================= */
	//////////////////////////////////////////////////////////////////////////////////////////////
	bool success;
	// Use placeholder values for demonstration purposes
	float placeholder_global_ov = 4.00f;
	float placeholder_global_uv = 2.80f;
	float placeholder_cell_ov = 4.00f;
	float placeholder_cell_uv = 2.80f;
	float placeholder_ot_v = 0.6f;          // Voltage corresponding to max temp
	float placeholder_ut_v = 2.9f;          // Voltage corresponding to min temp
	float placeholder_oc_a = 1.00f;         // Amperes
    float placeholder_shunt = 100;          // Defined earlier

	PRINTF("\n--- Setting Protection Thresholds ---\n\r\r");

	// a) Set Global OV/UV Threshold
	PRINTF("Setting Global OV/UV Threshold...\n\r\r");
	success = SlaveIf_setGlobalOvUvThreshold(placeholder_global_ov, placeholder_global_uv);
	if (!success) PRINTF("ERROR: Failed to set Global OV/UV!\n\r\r");

	// b) Set Overtemperature Thresholds (Loop through AN0-AN6)
	PRINTF("Setting Overtemperature Thresholds (AN0-AN6)...\n\r\r");
	for (uint8_t i = 0; i <= 6; i++)
	{
		success = SlaveIf_setOverTempThreshold(i, placeholder_ot_v);
		if (!success)  PRINTF("ERROR: Failed to set OT for AN%d!\n", i+1);
	}

	// c) Set Undertemperature Thresholds (Loop through AN0-AN6)
	PRINTF("Setting Undertemperature Thresholds (AN0-AN6)...\n\r\r");
	for (uint8_t i = 0; i <= 6; i++)
	{
		success = SlaveIf_setUnderTempThreshold(i, placeholder_ut_v);
		if (!success) PRINTF("ERROR: Failed to set UT for AN%d!\n", i);
	}

	// d) Set Overcurrent Threshold
	PRINTF("Setting Overcurrent Threshold...\n\r\r");
	success = SlaveIf_setOverCurrentThreshold(placeholder_oc_a, 100);
	if (!success) PRINTF("ERROR: Failed to set Overcurrent!\n\r\r");

	PRINTF("--- Protection Threshold Configuration Attempted ---\n\r\r");


	/////////////////////////////////////////////////////////////////////////////////////////////
	/* ========================= --- 7. Initial Data Acquisition --- ========================= */
	/////////////////////////////////////////////////////////////////////////////////////////////
	PRINTF("Performing initial data acquisition...\n\r\r");
	SlaveIF_startMeasurementCycle(); 						// Trigger with tag 0

	DataBase_UpdateMeasurementData();	                     // Update Measurements first time
	DataBase_GetCurrentMeasurementData(&current_Data);       // Get the initial data

	FuSa_updateFaultData();                                  // Update Faults Status first time
	FuSa_getCurrentFaultData(&current_Faults);               // Get the initial fault status

DataMonitor_lcd(59, 100, 0.25, 25.6, 1, 0);

	//////////////////////////////////////////////////////////////////////////////////////////
	/* ========================= --- 8. Main Application Loop --- ========================= */
	//////////////////////////////////////////////////////////////////////////////////////////
	PRINTF("Setup Complete. Entering main application loop.\n\r\r");
	// This loop continuously checks for data and fault interrupts and processes them.
	while (1)
	{
		// --- Task 1: Process Data Update Request (triggered by SysTick) ---
		if (data_interrupt)
		{
			data_interrupt = false; // Clear the interrupt flag

			// Shift data: current becomes previous
			DataBase_GetPreviousMeasurementData(&previous_Data); // Optional: Store previous before overwriting

		    SlaveIF_startMeasurementCycle();

			// Update measurement database (this likely reads registers from the slave)
			DataBase_UpdateMeasurementData();

			// Get the newly updated data
			DataBase_GetCurrentMeasurementData(&current_Data);

			// Process or display the new data (e.g., update LCD, run algorithms)
			// PRINTF("Data Update: Current = %f A\n", current_Data.Current); // Example print
			// DataMonitor_lcd(current_Data.SOC, current_Data.SOH, current_Data.Current, ...);
		}

		// --- Task 2: Process Fault Update Request (triggered by SysTick) ---
		if (fault_interrupt)
		{
			fault_interrupt = false; // Clear the interrupt flag

			// Shift data: current becomes previous
			FuSa_getPreviousFaultData(&previous_Faults); // Optional: Store previous before overwriting

		    SlaveIF_startMeasurementCycle();

			// Update fault status database (this likely reads fault registers)
			FuSa_updateFaultData();

			// Get the newly updated fault status
			FuSa_getCurrentFaultData(&current_Faults);

			// Process faults (e.g., trigger safety actions, display warnings)
			// if (current_Faults.OV_Flag) { /* Handle Overvoltage */ }
			// PRINTF("Fault Check Completed.\n\r\r"); // Example print
		}

		// --- Other Tasks ---
		// Add other non-blocking tasks here, such as:
		// - Handling user interface inputs
		// - Running state machine logic
		// - Sending data over CAN/other communication bus
		// - Controlling contactors based on state/faults

		// --- Low Power Mode ---
		// If no flags are set and no other tasks need immediate attention,
		// the MCU could potentially enter a low-power sleep mode until the next interrupt.
		// if (!data_interrupt && !fault_interrupt && !other_flags) {
		//   __WFI(); // Wait For Interrupt (example for ARM Cortex-M)
		// }

	} // End of main loop (while(1))

} // End of main

/////////////////////////////////////////////////////////////////////////////
/* ========================= --- End of File --- ========================= */
