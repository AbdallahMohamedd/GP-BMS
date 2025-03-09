/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:   bit_operator driver
 *	File: 		 helpful.c
 */

#include "stdint.h"
void delay_seconds(float seconds) //Not to used
{
    volatile uint32_t i = 0;
    uint32_t iterations = (uint32_t)(seconds * 800000); // Calculate the number of iterations

    for (i = 0; i < iterations; ++i)
    {
        __asm("NOP"); /* delay */
    }
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
