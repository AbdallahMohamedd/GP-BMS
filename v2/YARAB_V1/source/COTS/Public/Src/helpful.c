/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:   bit_operator driver
 *	File: 		 helpful.c
 */

#include <COTS/Public/Inc/helpful.h>

/*if ((systick_count - cc) >= 7000)
{
	//GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
	GPIO_TogglePinsOutput(GPIOA, 1U<<12U);
	//GPIO_WritePinOutput(GPIOA, 5U, 1);
	cc = systick_count; // Record the last toggle time
	PRINTF("BLUEEEEEEEE\n\r\r");
}
 */



// Initialize SysTick for delays (call once during system initialization)
void SysTick_Init(void)
{
	// Ensure SysTick is configured to use the core clock (48 MHz)
	SysTick->CTRL = 0; // Disable SysTick
	SysTick->LOAD = 0xFFFFFF; // Max reload value (24-bit)
	SysTick->VAL = 0; // Clear current value
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Use core clock, enable
}

// Microsecond delay
/*void delay_us(uint32_t us)
{
	// Calculate ticks for desired delay (48 MHz = 48 ticks per µs)
	uint32_t ticks = us * 48; // 48 ticks per microsecond
	ticks -= 12; // Adjust for overhead (experimentally determined, ~12 ticks at 48 MHz)

	if (ticks > 0xFFFFFF)  // SysTick is 24-bit, max 16777215 ticks (~349 ms at 48 MHz)
		ticks = 0xFFFFFF;

	SysTick->LOAD = ticks; // Set reload value
	SysTick->VAL = 0; // Clear current value
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Ensure SysTick is enabled

	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // Wait for count flag

	// No need to disable SysTick, as it will be reconfigured for the next delay
}*/

void delay_us(uint32_t us)
{
    volatile uint32_t count = (us * (CLOCK_GetCoreSysClkFreq() / 1000000U)) / 11.15;
    while (count--) __NOP();
}

// Millisecond delay
void delay_ms(uint32_t ms)
{
	// Call delay_us for each millisecond
	for (uint32_t i = 0; i < ms; i++)
		delay_us(1000); // 1000 µs = 1 ms

}

// Second delay
void delay_seconds(float seconds)
{
	// Convert seconds to milliseconds
	uint32_t ms = (uint32_t)(seconds * 1000.0f);
	delay_ms(ms);
}
