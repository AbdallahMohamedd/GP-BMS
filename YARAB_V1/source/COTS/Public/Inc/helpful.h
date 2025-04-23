/**
 * @file helpful.h
 * @brief Header file for utility functions including bit manipulation and delay utilities.
 *
 * @details This file provides macros for bit manipulation and function declarations
 *          for delay utilities used in the Battery Management System project.
 *          It includes functions for microsecond, millisecond, and second delays,
 *          as well as bit operations for setting, resetting, toggling, and reading bits.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Utility driver
 */

#ifndef PUBLIC
#define PUBLIC

//=============================================================================
// Includes
//=============================================================================
#include "fsl_common.h"
#include "stdint.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "string.h"                            
#include "MKL25Z4.h"                            
#include "fsl_debug_console.h" 

//=============================================================================
// Bit Manipulation Macros
//=============================================================================
/**
 * @brief Sets a specific bit in a given REG.
 * @param REG The variable or register to modify.
 * @param bit The bit position (0-31) to set.
 */
#define SET_BIT(REG, BIT)       ((REG) |= (1U << (BIT)))

/**
 * @brief Resets (clears) a specific bit in a given REG.
 * @param REG The variable or register to modify.
 * @param bit The bit position (0-31) to clear.
 */
#define RESET_BIT(REG, BIT)     ((REG) &= ~(1U << (BIT)))

/**
 * @brief Toggles a specific bit in a given REG.
 * @param REG The variable or register to modify.
 * @param bit The bit position (0-31) to toggle.
 */
#define TOGGLE_BIT(REG, BIT)    ((REG) ^= (1U << (BIT)))

/**
 * @brief Reads the value of a specific bit in a given REG.
 * @param REG The variable or register to read from.
 * @param bit The bit position (0-31) to read.
 * @return The value of the bit (0 or 1).
 */
#define READ_BIT(REG, BIT)      (((REG) & (1U << (BIT))) >> (BIT))

//=============================================================================
// Function Declarations
//=============================================================================
/**
 * @brief Initializes the SysTick timer for delay functions.
 * @details Configures the SysTick timer to use the core clock and prepares it
 *          for delay operations. Must be called before using delay functions.
 */
void SysTick_Init(void);

/**
 * @brief Delays execution for a specified number of microseconds.
 * @param us Number of microseconds to delay.
 * @details Uses the SysTick timer to create a precise delay. Accounts for
 *          execution overhead to improve accuracy.
 */
void delay_us(uint32_t us);

/**
 * @brief Delays execution for a specified number of milliseconds.
 * @param ms Number of milliseconds to delay.
 * @details Calls delay_us iteratively to achieve the desired delay.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Delays execution for a specified number of seconds.
 * @param seconds Number of seconds to delay (floating-point).
 * @details Converts seconds to milliseconds and calls delay_ms.
 */
void delay_seconds(float seconds);

#endif /* PUBLIC */


//=============================================================================
// End of File
//=============================================================================
