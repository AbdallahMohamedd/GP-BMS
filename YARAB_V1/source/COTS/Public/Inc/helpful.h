/**
 * @file helpful.h
 * @brief Public interface for timing and interrupt utility functions.
 *
 * @details Declares functions for PIT-based timing and interrupt flag management.
 *          Provides delay functions using PIT for precise timing in non-RTOS environments.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Utility functions for timing and interrupts
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
#include "fsl_pit.h"
#include "stdint.h"
#include "stdbool.h"

#define FLAG_DATA_ISR      ///< Enable prints inside SysTick_Handler
// Assuming PRINTF is defined elsewhere (e.g., fsl_debug_console.h)
#define TIMER_DEBUG_PRINTF // Uncomment for debug prints
//=============================================================================
// Defines
//=============================================================================
#define PIT_PERIODIC_CHANNEL kPIT_Chnl_0 // PIT channel used for 1s/10s interrupts
#define PIT_IRQ PIT_IRQn                 // PIT IRQ number for this device
#define SYSTICK_MAX_COUNT    (0x00FFFFFFUL) // Max reload value for 24-bit SysTick
#define SYSTICK_DELAY_OVERHEAD_US (3U)

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
// Global Flags (Set by ISR, Cleared by User)
//=============================================================================
/**
 * @brief Flag set to true every 1 second by the periodic timer ISR.
 * @details User code should check this flag and clear it (set to false) after processing.
 *          Declared volatile because it's modified in an ISR and read in the main loop.
 */
extern volatile bool g_timer_flag_1s;

/**
 * @brief Flag set to true every 10 seconds by the periodic timer ISR.
 * @details User code should check this flag and clear it (set to false) after processing.
 *          Declared volatile because it's modified in an ISR and read in the main loop.
 */
extern volatile bool g_timer_flag_10s;

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief Initializes the necessary timers for accurate delays and periodic interrupts.
 * @details Configures SysTick for microsecond delays (polling mode) and
 *          configures PIT Channel 0 for 1-second periodic interrupts.
 * @note MUST be called once during system initialization *after* core and bus clocks
 *       have been configured and stabilized (e.g., after BOARD_InitBootClocks()).
 *       It is CRITICAL that SystemCoreClock variable (CMSIS standard) is updated
 *       and CLOCK_GetFreq(kCLOCK_BusClk) returns the correct value at the time
 *       this function is called.
 */
void Timer_Init(void);

/**
 * @brief Provides a highly accurate blocking delay in microseconds.
 * @details Uses the SysTick timer in polling mode (busy-wait).
 *          Blocks CPU execution during the delay.
 * @param us Number of microseconds to delay (must be > 0).
 * @note Accuracy depends on the correctness of the SystemCoreClock variable.
 *       Maximum delay in a single call is limited by the SysTick 24-bit counter
 *       and the core clock frequency (approx 0.34 seconds at 48MHz).
 *       For longer delays, use delay_ms() or delay_seconds().
 */
void delay_us(uint32_t us);

/**
 * @brief Provides a blocking delay in milliseconds.
 * @details Calls delay_us() repeatedly. Blocks CPU execution.
 * @param ms Number of milliseconds to delay.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Provides a blocking delay in seconds (can be fractional).
 * @details Calls delay_ms() and potentially delay_us() for the fractional part.
 *          Blocks CPU execution.
 * @param seconds Number of seconds to delay.
 */
void delay_seconds(float seconds);
#endif // HELPFUL_H

//=============================================================================
// End of File
//=============================================================================
