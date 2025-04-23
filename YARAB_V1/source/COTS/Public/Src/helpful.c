/**
 * @file helpful.c
 * @brief Implementation of utility functions for delays and SysTick timer initialization.
 *
 * @details This file contains the implementation of delay functions (microseconds,
 *          milliseconds, and seconds) using the SysTick timer on the NXP KL25Z
 *          microcontroller. It also includes the initialization of the SysTick timer.
 *          The delay functions are optimized for accuracy by accounting for execution
 *          overhead, ensuring reliable timing for the Battery Management System.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Utility driver
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTS/Public/Inc/helpful.h>

//=============================================================================
// Static Constants
//=============================================================================
/**
 * @brief Number of SysTick ticks per microsecond for a 48 MHz core clock.
 * @details Calculated as 48 MHz / 1 MHz = 48 ticks per microsecond.
 */
static const uint32_t TICKS_PER_US = 48U;

/**
 * @brief Overhead compensation for delay_us function.
 * @details Empirically determined to account for execution overhead (e.g., register
 *          writes, loop checks). Set to 384 ticks (~8 µs at 48 MHz) based on
 *          measurements where 20 µs resulted in 20.25-21 µs, aligning with the
 *          required range of 20-24 µs (optimal at 21 µs) per the MC33664 datasheet.
 */
static const uint32_t DELAY_OVERHEAD_TICKS = 384U;

/**
 * @brief Maximum number of ticks supported by SysTick (24-bit counter).
 */
static const uint32_t SYSTICK_MAX_TICKS = 0xFFFFFFU; // 16,777,215 ticks

//=============================================================================
// Public Function Implementations
//=============================================================================
/**
 * @brief Initializes the SysTick timer for delay functions.
 * @details Configures the SysTick timer to use the core clock (48 MHz on KL25Z)
 *          and sets the maximum reload value. The timer is left disabled to avoid
 *          conflicts with other system components and will be enabled only during
 *          delay operations.
 * @note Must be called once during system initialization before using delay functions.
 */
void SysTick_Init(void)
{
	// Disable SysTick to configure
	SysTick->CTRL = 0;

	// Set reload to maximum value (24-bit)
	SysTick->LOAD = SYSTICK_MAX_TICKS;

	// Clear current value
	SysTick->VAL = 0;

	// Configure to use core clock, no interrupt, leave disabled
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
}

/**
 * @brief Delays execution for a specified number of microseconds.
 * @param us Number of microseconds to delay.
 * @details Calculates the required number of SysTick ticks based on a 48 MHz core
 *          clock (48 ticks per microsecond), compensates for execution overhead, and
 *          uses the SysTick timer to wait for the specified duration. Large delays
 *          are split into multiple smaller delays to respect the SysTick counter limit.
 */
void delay_us(uint32_t us)
{
	uint64_t total_ticks = (uint64_t)us * TICKS_PER_US; // 48 ticks per microsecond
	if (total_ticks > DELAY_OVERHEAD_TICKS) // Adjust for ~8 µs overhead (384 ticks)
	{
		total_ticks -= DELAY_OVERHEAD_TICKS;
	}
	else
	{
		total_ticks = 1; // Minimum ticks
	}

	while (total_ticks > 0)
	{
		uint32_t ticks = (total_ticks > SYSTICK_MAX_TICKS) ? SYSTICK_MAX_TICKS : (uint32_t)total_ticks;
		SysTick->LOAD = ticks - 1;
		SysTick->VAL = 0;
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
		total_ticks -= ticks;
	}
}

/**
 * @brief Delays execution for a specified number of milliseconds.
 * @param ms Number of milliseconds to delay.
 * @details Iteratively calls delay_us to achieve the desired delay in milliseconds.
 *          Each iteration delays for 1000 microseconds (1 ms).
 */
void delay_ms(uint32_t ms)
{
	for (uint32_t i = 0; i < ms; i++)
	{
		delay_us(1000U); // 1000 µs = 1 ms
	}
}

/**
 * @brief Delays execution for a specified number of seconds.
 * @param seconds Number of seconds to delay (floating-point).
 * @details Converts the input seconds to milliseconds and calls delay_ms to
 *          achieve the desired delay.
 */
void delay_seconds(float seconds)
{
	// Convert seconds to milliseconds
	uint32_t ms = (uint32_t)(seconds * 1000.0f);

	// Perform the delay
	delay_ms(ms);
}



/*if ((systick_count - cc) >= 7000)
{
	//GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
	GPIO_TogglePinsOutput(GPIOA, 1U<<12U);
	//GPIO_WritePinOutput(GPIOA, 5U, 1);
	cc = systick_count; // Record the last toggle time
	PRINTF("BLUEEEEEEEE\n\r\r");
}
 */



//=============================================================================
// End of File
//=============================================================================
