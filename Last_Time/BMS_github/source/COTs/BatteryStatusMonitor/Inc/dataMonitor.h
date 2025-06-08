/**
 * @file 		dataMonitor.h
 * @brief 		Public interface header for the Data Monitor driver.
 *
 * @details 	This header defines the public APIs for the Data Monitor module,
 *         		responsible for displaying battery management system parameters
 *         		such as State of Charge (SOC), State of Health (SOH), current,
 *         		temperature, mode, and fault status on an LCD. It interfaces
 *         		with the ScreenIF module for LCD operations.
 *
 * @note 		Project: Graduation Project - Battery Management System
 * @note 		Engineer: Amr Ahmed
 * @note 		Component: Data Monitor driver
 */

#ifndef DATAMONITOR_H
#define DATAMONITOR_H

//=============================================================================
// Includes
//=============================================================================
#include "COTs/DebugInfoManager/Inc/debugInfo.h"
#include "COTs/ScreenIF/Inc/ScreenIF.h"

//=============================================================================
// Enums
//=============================================================================
/**
 * @brief 		Enum for fault status values.
 * @details 	Defines possible fault states for the battery pack to be used
 *         		in fault status display functions.
 */
typedef enum
{
	FaultStatusNone 	= 0,  	//!< No fault detected in the battery pack.
	FaultStatusActive 	= 1 	//!< Active fault detected in the battery pack.
} BMSFaultStatus_t;

/**
 * @brief 		Enum for operating mode values.
 * @details 	Defines possible operating modes for the battery pack to be used
 *         		in mode display functions.
 */
typedef enum
{
	NormalMode 		= 0,       //!< Normal operating mode.
	SleepMode 		= 1,       //!< Sleep mode for low power consumption.
	DiagnosticMode 	= 2        //!< Diagnostics mode for system testing.
} BMSMode_t;

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief      	Clears the LCD display.
 * @details    	Call ScreenIF_Clear Func to clear all data on the LCD.
 */
void DataMonitor_ClearScreen(void);

/**
 * @brief      	Displays the State of Charge (SOC) on the LCD.
 * @details    	Receives the SOC value and formats it for display on the LCD
 *             	using the ScreenIF module.
 * @param 		soc State of Charge of the battery pack (0 to 100).
 */
void DataMonitor_soc_disp(uint8_t soc);

/**
 * @brief      	Displays the State of Health (SOH) on the LCD.
 * @details    	Receives the SOH value and formats it for display on the LCD
 *             	using the ScreenIF module.
 * @param 		soh State of Health of the battery pack (0 to 100).
 */
void DataMonitor_soh_disp(uint8_t soh);

/**
 * @brief      	Displays the current on the LCD.
 * @details    	Receives the current value and formats it for display on the LCD
 *             	with two decimal places.
 * @param 		current Current of the battery pack in amperes.
 */
void DataMonitor_current_disp(float current);

/**
 * @brief      	Displays the fault status on the LCD.
 * @details    	Displays a warning message for fault condition (FAULT_STATUS_ACTIVE) or a
 *             	no-fault message (FAULT_STATUS_NONE) on the LCD using the ScreenIF module.
 * @param 		fault Fault status of the battery pack.
 */
void DataMonitor_Fault_disp(BMSFaultStatus_t fault);

/**
 * @brief      	Displays the temperature on the LCD.
 * @details    	Receives the temperature value and formats it for display on the LCD
 *             	with two decimal places.
 * @param 		temp Temperature of the battery pack in degrees Celsius.
 */
void DataMonitor_Temp_disp(float temp);

/**
 * @brief      	Displays the operating mode on the LCD.
 * @details    	Displays the operating mode (Normal, Sleep, or Diagnostics) based
 *             	on the input mode value.
 * @param 		mode Operating mode of the battery pack
 */
void DataMonitor_Mode_disp(BMSMode_t mode);

/**
 * @brief      	Displays all battery parameters on the LCD.
 * @details    	Initializes the LCD and displays SOC, SOH, current, temperature,
 *             	mode, and fault status using the respective display functions.
 * @param 		soc State of Charge of the battery pack (0 to 100).
 * @param 		soh State of Health of the battery pack (0 to 100).
 * @param 		current Current of the battery pack in amperes.
 * @param 		temp Temperature of the battery pack in degrees Celsius.
 * @param 		mode Operating mode of the battery pack (see DATA_MONITOR_MODE).
 * @param 		fault Fault status of the battery pack (see DATA_MONITOR_FAULT_STATUS).
 */
void DataMonitor_lcd(uint8_t soc, uint8_t soh, float current, float temp, BMSMode_t mode, BMSFaultStatus_t fault);

/**
 * @brief      	Converts a float to a string with specified precision.
 * @details    	Converts a floating-point number to a string representation with
 *             	the specified number of decimal places for LCD display.
 * @param 		num Floating-point number to convert.
 * @param 		str Pointer to the output string buffer.
 * @param 		precision Number of decimal places to include.
 */
void floatToString(float num, char *str, int precision);

#endif /* DATAMONITOR_H */
//=============================================================================
// End of File
//=============================================================================
