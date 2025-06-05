/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "SPI.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq;
    /* SIM_SOPT2[27:26]:
     *  00: Clock Disabled
     *  01: IRC48M
     *  10: OSCERCLK
     *  11: MCGIRCCLK
     */
    CLOCK_SetLpsci0Clock(1);

    uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void InitHW(void)  {
	
	// ----------------------------------
	// system and clock setup
	// ----------------------------------
	// port setup
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;			// enable port A clock
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;			// enable port B clock
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;			// enable port C clock
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;			// enable port D clock
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;			// enable port E clock
	
	// uart 0 hw setup
	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1);      							// uart clock is MCGFLLCLK (=2 bus clock)
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);      								// TPM  clock is MCGFLLCLK (=2 bus clock)
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;									// enable uart 0 clock
    PORTA_PCR1 = PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK;
    PORTA_PCR2 = PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK;

    // Timer setup
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK; 			// enable clock for TPM0
	// TPM clock source: use PLLFLL (48MHz)
	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGPLLCLK
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK; 			// enable clock for TPM1
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;            // enable pit clock

	// ----------------------------------
	// PIT
	// ----------------------------------
	// PIT channel 0
	PIT_MCR |= PIT_MCR_FRZ_MASK;										// freeze PIT in debug
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;										// enable clocking
	PIT_LDVAL0 = BUSFREQ/2;												// timer ch0 start value every 0.5 seconds
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;									// enable timer ch0
	SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK; // enable SPI0 clock
	SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK; // enable SPI1 clock

	PORTD_PCR4 = PORT_PCR_MUX(1);
	GPIOD_PDDR &= ~BIT(4);

	PORTA_PCR12 = PORT_PCR_MUX(1);
	GPIOA_PDDR &= ~BIT(12);

	PORTE_PCR0 = PORT_PCR_MUX(1);
	GPIOE_PDDR |= BIT(0);
	GPIOE_PCOR = BIT(0);

	PORTC_PCR8 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(8);
	GPIOC_PSOR = BIT(8);

	PORTC_PCR5 = PORT_PCR_MUX(2);
	PORTD_PCR2 = PORT_PCR_MUX(2);

	PORTB_PCR10 = PORT_PCR_MUX(2);
	PORTB_PCR11 = PORT_PCR_MUX(2);
	PORTD_PCR7 = PORT_PCR_MUX(5);

	PORTC_PCR10 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(10);
	GPIOC_PCOR = BIT(10);
	PORTC_PCR11 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(11);
	GPIOC_PCOR = BIT(11);
	PORTC_PCR12 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(12);
	GPIOC_PCOR = BIT(12);
	PORTC_PCR13 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(13);
	GPIOC_PCOR = BIT(13);

	PORTC_PCR0 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(0);
	GPIOC_PCOR = BIT(0);
	PORTC_PCR3 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(3);
	GPIOC_PCOR = BIT(3);
	PORTC_PCR4 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(4);
	GPIOC_PCOR = BIT(4);
	PORTC_PCR6 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(6);
	GPIOC_PCOR = BIT(6);
	PORTC_PCR7 = PORT_PCR_MUX(1);
	GPIOC_PDDR |= BIT(7);
	GPIOC_PCOR = BIT(7);

	SPITxInit(0); // SPI0 used to transmit (polling)
	SPIRxInit(1); // SPI1 used to receive (interrupt driven)
	NVICEnIrq(SPI1_IRQ);
	SPIRxEnable();
	SPITxEnable();


	InitBoardLED();
}

// ----------------------------------------------------------------------------
/*! \brief Initializes the FRDM-KL25Z board RGB LED

Attention the PTD1 (blue LED) might be shared with the SPI0_SCK signal!
Therefore call InitBoardLED() prior to InitInterface()!
 */
void InitBoardLED(void)
{

	// ----------------------------------
	// RGB LED setup
	// ----------------------------------
	// port B
	PORTB_PCR18 = PORT_PCR_MUX(1); // alt1 = GPIO
	// ptb18 red (RGB LED)
	GPIOB_PDDR |= BIT(18); // output
	LED_RED_Off();
	// ptb19 green (RGB LED)
	PORTB_PCR19 = PORT_PCR_MUX(1); // alt1 = GPIO
	GPIOB_PDDR |= BIT(19);		   // output
	LED_GREEN_Off();

	//	// ptd1 red blue (RGB LED)
	PORTD_PCR1 = PORT_PCR_MUX(1); // alt1 = GPIO
	GPIOD_PDDR |= BIT(1);		  // output
	LED_BLUE_Off();
}


// --------------------------------------------------------------------
/*! \brief Sets the RGB LED on the FRDM-KL25Z board
 *
 * @param color  Color to set (TYPE_LEDcolor)
 */
void LEDHandler(TYPE_LEDcolor color)  {

	switch(color)  {

		case GreenToggle:
			LED_GREEN_Toggle();
			LED_BLUE_Off();
			LED_RED_Off();
			break;

		case Green:
			LED_GREEN_On();
			LED_BLUE_Off();
			LED_RED_Off();
			break;

		case Orange:
			LED_RED_On();
			LED_GREEN_On();
			LED_BLUE_Off();
			break;

		case Blue:
			LED_BLUE_On();
			LED_RED_Off();
			LED_GREEN_Off();
			break;

		case Red:
			LED_BLUE_Off();
			LED_RED_On();
			LED_GREEN_Off();
			break;

		case Off:
		default :
			LED_GREEN_Off();
			LED_BLUE_Off();
			LED_RED_Off();
			break;
	}

}


