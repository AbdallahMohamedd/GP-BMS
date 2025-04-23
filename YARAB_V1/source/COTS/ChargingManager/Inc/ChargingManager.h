/**
 * @file    ChargingManager.h
 * @brief   Header file for the Charging Manager module.
 *
 * @details Declares functions for controlling the Solid State Relay (SSR) to
 *          connect or disconnect the battery from the charger or load. Includes
 *          debug macro definitions for conditional debug prints compatible with
 *          DebugInfoManager.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Dependencies: Requires DebugInfo.h for debug support.
 */

#ifndef CHARGING_MANAGER_H
#define CHARGING_MANAGER_H

//=============================================================================
// Includes
//=============================================================================
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>

//=============================================================================
// SSR GPIO Configuration
//=============================================================================
#define SSR_GPIO_PORT     (GPIOB)        // GPIO port for SSR control
#define SSR_GPIO_PIN      (18U)           // GPIO pin for SSR control (PC5)

//=============================================================================
// Public Function Declarations
//=============================================================================
/**
 * @brief Initializes the GPIO pin used for SSR control.
 * @details Configures the SSR pin as a digital output with an initial state of low
 *          (SSR closed, battery connected).
 */
void ChargingManager_initSsrPin(void);

/**
 * @brief Opens the SSR to disconnect the battery.
 * @param[in] en Enable flag (true to open SSR, false to do nothing).
 */
void ChargingManager_ssrOpen(bool en);

/**
 * @brief Closes the SSR to connect the battery.
 * @param[in] en Enable flag (true to close SSR, false to do nothing).
 */
void ChargingManager_ssrClose(bool en);

#endif // CHARGING_MANAGER_H

//=============================================================================
// End of File
//=============================================================================
