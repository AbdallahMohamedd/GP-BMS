/**
 * @file    ChargingManager.c
 * @brief   Charging Manager module for controlling the Solid State Relay (SSR).
 *
 * @details This module provides functions to open and close the SSR, which connects
 *          or disconnects the battery from the charger or load. It uses a GPIO pin
 *          to control the SSR state. Debugging messages are conditionally compiled
 *          based on the CHARGING_DEBUG_SSR macro defined in ChargingManager.h.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Dependencies: Requires FSL GPIO driver for NXP KL25Z and DebugInfo.h for debug support.
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTS/ChargingManager/Inc/ChargingManager.h>

//=============================================================================
// Public Function Implementations
//=============================================================================
/**
 * @brief Initializes the GPIO pin used for SSR control.
 * @details Configures the SSR pin as a digital output with an initial state of low
 *          (SSR closed, battery connected).
 */
void ChargingManager_initSsrPin(void)
{
	gpio_pin_config_t ssr_config = {
			kGPIO_DigitalOutput, // Configure as output
			0                    // Initial state: low (SSR closed)
	};

	// Initialize the SSR GPIO pin
	GPIO_PinInit(SSR_GPIO_PORT, SSR_GPIO_PIN, &ssr_config);
}

/**
 * @brief Opens the SSR to disconnect the battery.
 * @param[in] en Enable flag (true to open SSR, false to do nothing).
 * @details Sets the SSR GPIO pin high to open the relay, disconnecting the battery
 *          from the charger or load. Prints debug information if CHARGING_DEBUG_SSR is defined.
 */
void ChargingManager_ssrOpen(bool en)
{
	if (en)
	{
		GPIO_WritePinOutput(SSR_GPIO_PORT, SSR_GPIO_PIN, 1); // Set high to open SSR

#ifdef CHARGING_DEBUG_SSR
		DebugInfo;
		PRINTF("ChargingManager: SSR opened (GPIO PC5 set HIGH). Battery disconnected.\n\r");
#endif
	}
}

/**
 * @brief Closes the SSR to connect the battery.
 * @param[in] en Enable flag (true to close SSR, false to do nothing).
 * @details Sets the SSR GPIO pin low to close the relay, connecting the battery
 *          to the charger or load. Prints debug information if CHARGING_DEBUG_SSR is defined.
 */
void ChargingManager_ssrClose(bool en)
{
	if (en)
	{
		GPIO_WritePinOutput(SSR_GPIO_PORT, SSR_GPIO_PIN, 0); // Set low to close SSR

#ifdef CHARGING_DEBUG_SSR
		DebugInfo;
		PRINTF("ChargingManager: SSR closed (GPIO PC5 set LOW). Battery connected.\n\r");
#endif
	}
}

//=============================================================================
// End of File
//=============================================================================
