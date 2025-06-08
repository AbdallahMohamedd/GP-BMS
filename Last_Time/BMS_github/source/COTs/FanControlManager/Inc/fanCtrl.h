/**
 * @file       FanControlManager.h
 * @brief      Public interface header for the Fan Control Manager driver.
 *
 * @details    This header defines the public APIs for controlling two fans
 *             using PWM signals via TPM0 on the MKL25Z4 microcontroller.
 *             It includes initialization and duty cycle adjustment functions
 *             for Fan 1 (PTC2/TPM0_CH1) and Fan 2 (PTC1/TPM0_CH0).
 *
 * @note       Project:     Graduation Project - Battery Management System
 * @note       Engineer:    Abdullah Mohamed
 * @note       Component:   Fan Control Manager driver
 */

#ifndef FANCONTROLMANAGER_H_
#define FANCONTROLMANAGER_H_

//=============================================================================
// Includes
//=============================================================================
#include "COTs/DebugInfoManager/Inc/debugInfo.h"
#include <MKL25Z4.h>
#include <stdint.h>

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief      Initializes Fan 1 (PTC2/TPM0_CH1) for PWM control.
 * @details    Configures the PWM channel for Fan 1 and starts the counter.
 */
void fanCtrl_fan1Init(void);

/**
 * @brief      Sets the duty cycle for Fan 1 (PTC2/TPM0_CH1).
 * @details    Adjusts the PWM duty cycle to the specified percentage.
 * @param      duty Percentage duty cycle (0-100).
 */
void fanCtrl_fan1SetDuty(uint8_t duty);

/**
 * @brief      Initializes Fan 2 (PTC1/TPM0_CH0) for PWM control.
 * @details    Configures the PWM channel for Fan 2 and starts the counter.
 */
void fanCtrl_fan2Init(void);

/**
 * @brief      Sets the duty cycle for Fan 2 (PTC1/TPM0_CH0).
 * @details    Adjusts the PWM duty cycle to the specified percentage.
 * @param      duty Percentage duty cycle (0-100).
 */
void fanCtrl_fan2SetDuty(uint8_t duty);

#endif /* FANCONTROLMANAGER_H_ */
//=============================================================================
// End of File
//=============================================================================
