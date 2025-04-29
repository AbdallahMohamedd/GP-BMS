/**
 * @file helpful.c
 * @brief Implementation of timing and interrupt utility functions.
 *
 * @details This module provides SysTick-based timing functions and interrupt handling
 *          for periodic events. It is used by other modules (e.g., FuSa) for delays
 *          and scheduling.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Utility functions for timing and interrupts
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTS/Public/Inc/helpful.h> // Public header for this module

//=============================================================================
// Static Variables
//=============================================================================
/** @brief Stores the number of SysTick ticks per microsecond. Calculated once during init. */
static uint32_t systick_ticks_per_us = 0;

/** @brief Software counter used within PIT ISR to track 10-second intervals. */
static volatile uint8_t seconds_counter_10s = 0;

static uint32_t systick_overhead_ticks = 0; // Calculated overhead in ticks

//=============================================================================
// Global Flags (Definition)
//=============================================================================
volatile bool data_interrupt = false;
volatile bool fault_interrupt = false;

//=============================================================================
// Public Function Implementations
//=============================================================================
/**
 * @brief Initializes SysTick and PIT Channel 0.
 */
void Timer_Init(void)
{

	if (SystemCoreClock == 0) {
#ifdef TIMER_DEBUG_PRINTF
		PRINTF("FATAL ERROR: SystemCoreClock is 0! Cannot initialize SysTick delays.\r\n");
#endif
		while(1); // Halt - Cannot proceed
	}

#ifdef TIMER_DEBUG_PRINTF
	PRINTF("[Timer Debug] SystemCoreClock = %u Hz\n\r", SystemCoreClock);
#endif

	// Calculate ticks per microsecond for SysTick (based on Core Clock)
	systick_ticks_per_us = SystemCoreClock / 1000000U;

	if (systick_ticks_per_us == 0) {
#ifdef TIMER_DEBUG_PRINTF
		PRINTF("FATAL ERROR: SystemCoreClock too low for microsecond SysTick resolution.\r\n");
#endif
		while(1); // Halt - Cannot proceed
	}

	// *** Calculate overhead in ticks ***
	systick_overhead_ticks = SYSTICK_DELAY_OVERHEAD_US * systick_ticks_per_us;


	// Configure SysTick:
	// 1. Set Reload value to maximum (doesn't matter much for polling mode here)
	SysTick->LOAD = SYSTICK_MAX_COUNT;
	// 2. Clear the Current Value Register
	SysTick->VAL = 0;
	// 3. Configure Control and Status Register:
	//    - Enable counter (ENABLE=1)
	//    - Use Processor Clock (CLKSOURCE=1)
	//    - Disable SysTick Exception request (TICKINT=0) - IMPORTANT for polling mode!
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk; // Use processor clock, counter disabled initially
	// Note: We don't enable the counter here; delay_us will enable/disable it.

#ifdef TIMER_DEBUG_PRINTF
	PRINTF("INFO: SysTick Initialized. Core Clock: %u Hz, Ticks/us: %u\r\n", SystemCoreClock, systick_ticks_per_us);
#endif


	// --- PIT Initialization for Periodic Interrupts ---
	pit_config_t pitConfig;
	uint32_t pit_source_clock_freq;

	// Get default PIT configuration
	PIT_GetDefaultConfig(&pitConfig);
	pitConfig.enableRunInDebug = true; // Keep PIT running during debug

	// Initialize PIT module structure
	PIT_Init(PIT, &pitConfig); // Assumes PIT clock gate is enabled

	// Get the PIT source clock frequency (usually Bus Clock)
	// ** CRITICAL **: This MUST return the correct frequency for the clock source PIT uses.
	pit_source_clock_freq = CLOCK_GetFreq(kCLOCK_BusClk);

	if (pit_source_clock_freq == 0) {
#ifdef TIMER_DEBUG_PRINTF
		PRINTF("FATAL ERROR: PIT Source Clock Frequency is 0! Check Bus Clock config.\r\n");
#endif
		while(1); // Halt
	}
	if (pit_source_clock_freq < 1000000) {
#ifdef TIMER_DEBUG_PRINTF
		PRINTF("WARNING: PIT Source Clock Frequency (%u Hz) seems low.\r\n", pit_source_clock_freq);
#endif
		// Continue, but timing might be inaccurate if clock is wrong
	}


	// Configure PIT Channel 0 for 1-second period
	// PIT_SetTimerPeriod calculates the correct register value (ticks - 1)
	PIT_SetTimerPeriod(PIT, PIT_PERIODIC_CHANNEL, pit_source_clock_freq);

	// Enable interrupts for Channel 0
	PIT_EnableInterrupts(PIT, PIT_PERIODIC_CHANNEL, kPIT_TimerInterruptEnable);

	// Enable the PIT interrupt in the NVIC
	NVIC_SetPriority(PIT_IRQ, 3); // Set appropriate priority (lower number = higher prio)
	EnableIRQ(PIT_IRQ);

	// Start timer Channel 0
#ifdef TIMER_DEBUG_PRINTF
	PRINTF("INFO: PIT Initialized. Source Clock: %u Hz. Ch%d started for 1s period.\r\n",
			pit_source_clock_freq, (uint32_t)PIT_PERIODIC_CHANNEL);
#endif
	PIT_StartTimer(PIT, PIT_PERIODIC_CHANNEL);
}





// NOP-based delay for very small delays (less than 10 µs)
static void delay_nop_us(uint32_t us)
{
#ifdef TIMER_DEBUG_PRINTF
    PRINTF("[Timer Debug] SystemCoreClock = %u Hz\n\r", SystemCoreClock);
#endif
    // For 1.75 µs at 48 MHz: 1.75 * 48 = 84 cycles
    // Each NOP takes ~1 cycle, but we need to account for function call overhead
    // Let's use a fixed number of NOPs for 1.75 µs
    if (us == 1) { // Special case for 1.75 µs (rounded to 2 µs)
        for (volatile uint32_t i = 0; i < 40; i++) { // Adjust this value experimentally
            __asm volatile ("nop");
        }
    } else {
        uint32_t cycles_per_us = SystemCoreClock / 1000000U;
        uint32_t cycles = us * cycles_per_us;
        uint32_t iterations = cycles / 4; // Fallback for other delays
        for (volatile uint32_t i = 0; i < iterations; i++) {
            __asm volatile ("nop");
        }
    }
}





/**
 * @brief SysTick-based busy-wait delay.
 */
void delay_us(uint32_t us)
{
    // Minimum delay accounting for overhead? If requested delay is <= overhead,
    // the setup/teardown time itself provides the delay.
    // Let's handle 0 explicitly. For 1 to OVERHEAD_US, the result might be slightly shorter than requested.
    if (us == 0) return;

    // Calculate target ticks
    uint32_t target_ticks = us * systick_ticks_per_us;

    // *** Apply Calibration: Subtract overhead ticks ***
    // Ensure we don't underflow if the requested delay is very short
    uint32_t calibrated_ticks;
    if (target_ticks > systick_overhead_ticks) {
        calibrated_ticks = target_ticks - systick_overhead_ticks;
    } else {
        // If requested ticks are less than or equal to overhead,
        // a very short delay is needed. Aim for 1 tick minimum load value.
        // The actual delay will likely be dominated by the overhead itself.
        calibrated_ticks = 1;
    }

    // Limit to SysTick maximum count
    uint32_t reload_val;
    if (calibrated_ticks > SYSTICK_MAX_COUNT) {
        reload_val = SYSTICK_MAX_COUNT;
    } else {
        reload_val = calibrated_ticks;
    }

    // Ensure reload value is at least 1 for the counter operation (Load N -> N+1 states)
    if (reload_val == 0) reload_val = 1;

    // --- SysTick Delay Sequence ---
    SysTick->LOAD = reload_val - 1; // Load N-1 for N+1 cycles ? Let's try Load = ticks-1
                                     // The documentation says N+1 states for LOAD=N.
                                     // So load (calibrated_ticks - 1) should give calibrated_ticks duration.
                                     // Ensure calibrated_ticks - 1 doesn't go below 0.

    if (reload_val > 0) { // Only proceed if reload_val is valid
        SysTick->LOAD = reload_val -1;
        SysTick->VAL = 0; // Clear current value
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable counter

        // Wait for COUNTFLAG
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
        {
            __asm volatile ("nop");
        }

        // Disable counter
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    } else {
         // If calibrated_ticks was <= 0 (should be caught by reload_val=1 earlier),
         // we might skip the SysTick part entirely, as overhead dominates.
         // For simplicity, the reload_val=1 case above handles minimum delay.
    }
}



/**
 * @brief Millisecond delay.
 */
void delay_ms(uint32_t ms)
{
	if (ms == 0) return;
	// Call delay_us repeatedly for 1000us chunks
	for (uint32_t i = 0; i < ms; ++i) {
		delay_us(1000U);
	}
}

/**
 * @brief Second delay (float).
 */
void delay_seconds(float seconds)
{
	if (seconds <= 0.0f) return;
	// Calculate total milliseconds first
	uint32_t ms = (uint32_t)(seconds * 1000.0f);
	if (ms > 0) {
		delay_ms(ms);
	}

	// Handle the fractional part of seconds using delay_us for better precision
	float remainder_s = seconds - (float)ms / 1000.0f;
	if (remainder_s > 0.000001f) { // Check if remainder is significant
		delay_us((uint32_t)(remainder_s * 1000000.0f));
	}
}

//=============================================================================
// Interrupt Service Routine (ISR) Implementations
//=============================================================================

/**
 * @brief PIT Interrupt Handler (handles Channel 0 for periodic events).
 */
void PIT_IRQHandler(void)
{
	// Check if Channel 0 interrupt flag is set
	if (PIT_GetStatusFlags(PIT, PIT_PERIODIC_CHANNEL) & kPIT_TimerFlag)
	{
		// Clear the interrupt flag FIRST
		PIT_ClearStatusFlags(PIT, PIT_PERIODIC_CHANNEL, kPIT_TimerFlag);

		// --- Set 1-Second Flag ---
		data_interrupt = true;

		// --- Handle 10-Second Counter ---
		seconds_counter_10s++;
		if (seconds_counter_10s >= 10)
		{
			fault_interrupt = true;
			seconds_counter_10s = 0; // Reset counter
		}
	}

	// It's good practice to add memory barriers after clearing flags/accessing volatiles in ISR
	__DSB(); // Data Synchronization Barrier
	__ISB(); // Instruction Synchronization Barrier
}



//=============================================================================
// End of File
//=============================================================================
