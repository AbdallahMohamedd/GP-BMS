/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdelrahman Mohamed
 *	Component:   PWM driver
 *	File: 		 PWM.h
 */

#ifndef PWM_H
#define PWM_H

#include "MKL25Z4.h"
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>

/**
 * @brief Initializes PWM on TPM1 Channel 1 (PTA13).
 *
 * Configures the clock, prescaler, and period for PWM generation.
 *
 * @param prescaler The prescaler value (1, 2, 4, 8, 16, 32, 64, or 128).
 * @param period The PWM period.
 */
void PWM_Init(uint8_t prescaler, uint16_t period);

/**
 * @brief Starts PWM with a given duty cycle.
 *
 * Enables the TPM module and sets the duty cycle.
 *
 * @param duty_cycle Duty cycle percentage (0-100).
 */
void PWM_Start(uint8_t duty_cycle);

/**
 * @brief Updates the duty cycle of the running PWM.
 *
 * Modifies the PWM signal without stopping it.
 *
 * @param duty_cycle Duty cycle percentage (0-100).
 */
void PWM_SetDutyCycle(uint8_t duty_cycle);

/**
 * @brief Stops PWM signal generation.
 *
 * Disables the TPM module to stop the PWM output.
 */
void PWM_Stop(void);

#endif // PWM_H
