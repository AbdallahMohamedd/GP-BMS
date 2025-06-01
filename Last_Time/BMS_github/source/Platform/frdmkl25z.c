// --------------------------------------------------------------------
//  Copyright (c) 2015, NXP Semiconductors.
//  All rights reserved.
// 
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
// 
//  o Redistributions of source code must retain the above copyright notice, this list
//    of conditions and the following disclaimer.
// 
//  o Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
// 
//  o Neither the name of NXP Semiconductors nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// --------------------------------------------------------------------
#include "frdmkl25z.h"


// --------------------------------------------------------------------
//#include "source/drivers/lld3377x.h"
#include "source/COTs/SlaveControlIF/Inc/SlaveIF.h"
// --------------------------------------------------------------------
TYPE_INTERFACE _interface_;														//!< local copy of interface type 
TYPE_EVB _evb_;																	//!< local copy of evb type
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
// ----------------------------------------------------------------------------
//! \brief Initializes the FRDM-KL25Z freedom board hardware. GPIOs, SPI, Clocks etc.

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
//	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK; 			// enable clock for TPM0
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
}

// ----------------------------------------------------------------------------
/*! \brief Initializes the clock system
  
- use external 8MHz quarz
- bus clock 24MHz
- core clock 48MHz

\image html kl25z-clock.png   
 */
void InitBoardClock(void)  {
	
	// ----------------------------------
	// system and clock setup
	// ----------------------------------
	
    /* System clock initialization */
    /* SIM_SCGC5: PORTA=1 */
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;   /* Enable clock gate for ports to enable pin routing */
 
    /* SIM_CLKDIV1: OUTDIV1=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
    SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(0x01) | SIM_CLKDIV1_OUTDIV4(0x01)); /* Update system prescalers */
    
    /* SIM_SOPT2: PLLFLLSEL=1 */
    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK; /* Select PLL as a clock source for various peripherals */
    
    /* SIM_SOPT1: OSC32KSEL=0 */
    SIM_SOPT1 &= (uint32_t)~(uint32_t)(SIM_SOPT1_OSC32KSEL(0x03)); /* System oscillator drives 32 kHz clock for various peripherals */
    
    /* SIM_SOPT2: TPMSRC=1 */
    SIM_SOPT2 = (uint32_t)((SIM_SOPT2 & (uint32_t)~(uint32_t)(
        SIM_SOPT2_TPMSRC(0x02)
    )) | (uint32_t)(
        SIM_SOPT2_TPMSRC(0x01)
    ));                      /* Set the TPM clock */
    
    // ----  PTA18 and PTA19 connected to XTAL
    /* PORTA_PCR18: ISF=0,MUX=0 */
    PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));                                                   
    
    /* PORTA_PCR19: ISF=0,MUX=0 */
    PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));                                                   
    
    /* Switch to FBE Mode */
    /* MCG_C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
    MCG_C2 = (MCG_C2_RANGE0(0x02) | MCG_C2_EREFS0_MASK);                                                   
    
    /* OSC0_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC0_CR = OSC_CR_ERCLKEN_MASK;                                                   
 
    /* MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);                                                   
    
    /* MCG_C4: DMX32=0,DRST_DRS=0 */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));                                                   
 
    /* MCG_C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
    MCG_C5 = MCG_C5_PRDIV0(0x01);                                                   

    /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
    MCG_C6 = 0x00U;                                                   
    while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
    }
    while((MCG_S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
    }
    /* Switch to PBE Mode */
    /* OSC0_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC0_CR = OSC_CR_ERCLKEN_MASK;                                                   
    /* MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);                                                   
    /* MCG_C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
    MCG_C2 = (MCG_C2_RANGE0(0x02) | MCG_C2_EREFS0_MASK);                                                   
    /* MCG_C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
    MCG_C5 = MCG_C5_PRDIV0(0x01);                                                   
    /* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0 */
    MCG_C6 = MCG_C6_PLLS_MASK;                                                   
    while((MCG_S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
    }
    while((MCG_S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
    }
    /* Switch to PEE Mode */
    /* OSC0_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC0_CR = OSC_CR_ERCLKEN_MASK;                                                   
    /* MCG_C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);                                                   
    /* MCG_C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
    MCG_C2 = (MCG_C2_RANGE0(0x02) | MCG_C2_EREFS0_MASK);                                                   
    /* MCG_C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
    MCG_C5 = MCG_C5_PRDIV0(0x01);                                                   
    /* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0 */
    MCG_C6 = MCG_C6_PLLS_MASK;                                                   
    while((MCG_S & 0x0CU) != 0x0CU) {    /* Wait until output of the PLL is selected */
    }
}	
	
// ----------------------------------------------------------------------------
/*! \brief Initializes the FRDM-KL25Z board RGB LED

Attention the PTD1 (blue LED) might be shared with the SPI0_SCK signal!
Therefore call InitBoardLED() prior to InitInterface()!
 */
void InitBoardLED(void)  {

	// ----------------------------------
	// RGB LED setup
	// ----------------------------------
	// port B
	PORTB_PCR18 = PORT_PCR_MUX(1);										// alt1 = GPIO 
	// ptb18 red (RGB LED)
	GPIOB_PDDR |= BIT(18);												// output
	LED_RED_Off();
	// ptb19 green (RGB LED) 
	PORTB_PCR19 = PORT_PCR_MUX(1);										// alt1 = GPIO 
	GPIOB_PDDR |= BIT(19);												// output
	LED_GREEN_Off();

	
//	// ptd1 red blue (RGB LED) 
	PORTD_PCR1 = PORT_PCR_MUX(1);										// alt1 = GPIO 
	GPIOD_PDDR |= BIT(1);												// output
	LED_BLUE_Off();
}





// ----------------------------------------------------------------------------
/*! \brief Initializes the FRDM-KL25Z interface for the specific EVB.

Different interface options for TPL and SPI are supported. 

For TPL 2 different interfaces (TYPE_EVB) are supported.

For SPI 2 different interfaces (TYPE_EVB) are supported.

 */
void InitInterface(TYPE_INTERFACE interface, TYPE_EVB evb)  {

	_interface_ = interface;
	_evb_ = evb;
	
	
	/*!
	1.  \b SPI-Type1  (HWPLATFORM 0)

	\image html hwplatform0.png

	The "old style = type 1" SPI EVB uses the following interface pins:

	| CON   | PIN   | Function  | Pin Alt | Dir  | Signal |
	|-------|-------|-----------|---------|------|--------|
	| J2-03 | PTC13 | GPIO      |    1    | <--- | FAULT  |
	| J2-02 | PTA13 | GPIO      |    1    | ---> | CSB    |
	| J2-04 | PTD5  | SPI1_SCK  |    2    | ---> | CLK    |
	| J2-20 | PTE1  | SPI1_MO   |    2    | ---> | MOSI   |
	| J2-17 | PTD6  | SPI1_MI   |    2    | <--- | MISO   |
	| J2-13 | PTE31 | not used  |    1    |      | RESET  |
	| J2-01 | PTC12 | not used  |    1    |      | SPI_COM_EN |
	 */	    	
	if((_interface_==IntSPI)&&(_evb_==EVB_Type1))  {				
		
		SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;									// enable SPI1 clock

		PORTC_PCR(13) = PORT_PCR_MUX(1);								
		GPIOC_PDDR  &= ~BIT(13);										

		PORTA_PCR(13) = PORT_PCR_MUX(1);								
		GPIOA_PDDR  |= BIT(13);											
		GPIOA_PSOR  = BIT(13);											

		PORTD_PCR(5) = PORT_PCR_MUX(2);									

		PORTE_PCR(1) = PORT_PCR_MUX(2);									

		PORTD_PCR(6) = PORT_PCR_MUX(5);									

		PORTE_PCR(31) = PORT_PCR_MUX(1);								
		GPIOE_PDDR  &= ~BIT(31);										
	//	GPIOE_PDDR  |= BIT(31);											
	//	GPIOE_PCOR  = BIT(31);											

	//	PORTC_PCR12 = PORT_PCR_MUX(1);									
	//	GPIOC_PDDR  |= BIT(12);											
	//	GPIOC_PSOR  = BIT(12);											

		SPIInit(1);															// SPI1 used to transmit and receive (polling)
		SPIEnable();
		NVICEnIrq(SPI1_IRQ);	
	
	
	
/*!
2. \b SPI-TypeArd (HWPLATFORM 1)

	\image html hwplatform1.png

The "new style = type Ard" SPI EVB uses the following interface pins:
| CON   | PIN   | Function  | Pin Alt | Dir  | Signal |
|-------|-------|-----------|---------|------|--------|
| J1-06 | PTD4  | GPIO      |    1    | <--- | FAULT  |
| J1-14 | PTC8  | GPIO      |    1    | ---> | CSB    |
| J1-09 | PTC5  | SPI0_SCK  |    2    | ---> | CLK    |
| J2-08 | PTD2  | SPI0_MO   |    2    | ---> | MOSI   |
| J2-10 | PTD3  | SPI0_MI   |    2    | <--- | MISO   |
|J10-02 | PTB0  | GPIO      |    1    | ---> | RESET  |
|J10-06 | PTB2  | not used  |    1    |      | GPIO2_SOC  |
| J2-20 | PTE0  | not used  |    1    |      | GPIO0_WKU  |
 */	    	
	}else if((_interface_==IntSPI)&&(_evb_==EVB_TypeArd))  {			

	    SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;										// enable SPI0 clock				
		
		PORTD_PCR(4) = PORT_PCR_MUX(1);										
		GPIOD_PDDR  &= ~BIT(4);							
		
		PORTC_PCR(8) = PORT_PCR_MUX(1);					
		GPIOC_PDDR  |= BIT(8);							
		GPIOC_PSOR  = BIT(8);							

		PORTC_PCR(5)  = PORT_PCR_MUX(2);				

		PORTD_PCR(2)  = PORT_PCR_MUX(2);				

		PORTD_PCR(3)  = PORT_PCR_MUX(2);				
		
		PORTB_PCR(0) = PORT_PCR_MUX(1);					
		GPIOB_PCOR = BIT(0);			
		GPIOB_PDDR |= BIT(0);				
		GPIOB_PCOR = BIT(0);		
		
		SPIInit(0);																// SPI0 used to transmit and receive (polling)
		SPIEnable();
		NVICEnIrq(SPI0_IRQ);	
	
	
/*!
3. \b TPL-Type1 (HWPLATFORM 2)

	\image html hwplatform2.png

The "old style = type 1" TPL EVB uses the following interface pins:

| CON   | PIN   | Function  | Pin Alt | Dir  | Signal |
|-------|-------|-----------|---------|------|--------|
| J1-16 | PTD7  | GPIO      |    1    | <--- | FAULT  |
| J2-18 | PTE0  | GPIO      |    1    | <--- | INTB   |
| J2-13 | PTE31 | GPIO      |    1    | ---> | EN     |
| J2-02 | PTA13 | GPIO      |    1    | ---> | TXCSB  |
| J2-04 | PTD5  | SPI1_SCK  |    2    | ---> | TXCLK  |
| J2-20 | PTE1  | SPI1_MO   |    2    | ---> | TXDATA |
| J2-06 | PTD0  | SPI0_PCS0 |    2    | <--- | RXCSB  |
| J2-12 | PTD1  | SPI0_SCK  |    2    | <--- | RXCLK  |
| J2-08 | PTD2  | SPI0_SI   |    2    | <--- | RXDATA |
 */	    

	}else if((_interface_==IntTPL)&&(_evb_==EVB_Type1))  {								// old style EVB

		SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;								
		SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;								

		PORTD_PCR7  = PORT_PCR_MUX(1);									
		GPIOD_PDDR  &= ~BIT(7);											

		PORTE_PCR0  = PORT_PCR_MUX(1);									
		GPIOE_PDDR  &= ~BIT(0);											
		
		PORTE_PCR31 = PORT_PCR_MUX(1);									
		GPIOE_PDDR  |= BIT(31);											
		GPIOE_PCOR  = BIT(31);											

		PORTA_PCR13 = PORT_PCR_MUX(1);									
		GPIOA_PDDR  |= BIT(13);											
		GPIOA_PSOR  = BIT(13);											
		
		PORTD_PCR5  = PORT_PCR_MUX(2);									
		PORTE_PCR1  = PORT_PCR_MUX(2);									

		PORTD_PCR0 = PORT_PCR_MUX(2);									
		PORTD_PCR1 = PORT_PCR_MUX(2);									
		PORTD_PCR2 = PORT_PCR_MUX(2);		

		//! \todo needs to be clean up (only for one modified board!!!
// ------------------------
/*
		// added to access TPL test mode via SPI
		// PTC12  ->  TEST   0: normal mode, 1: Test mode
		// PTD6/SPI1_MISO  <- DATA_RX
		PORTC_PCR12 = PORT_PCR_MUX(1);										// alt1 = GPIO 
		GPIOC_PCOR  = BIT(12);		
		GPIOC_PDDR  |= BIT(12);												// output
		PORTD_PCR6 = PORT_PCR_MUX(5);										// alt5 = SPI1_MISO 
*/
// ------------------------
		SPITxInit(1);																	// SPI1 used to transmit (polling)
		SPIRxInit(0);																	// SPI0 used to receive (interrupt driven)
		NVICEnIrq(SPI0_IRQ);	
		SPITxEnable();
		SPIRxEnable();

/*!
4. \b TPL-TypeArd (HWPLATFORM 3)

	\image html hwplatform3.png

The "new style = type Ard" TPL EVB uses the following interface pins:

| CON   | PIN   | Function  | Pin Alt | Dir  | Signal |StartUp |
|-------|-------|-----------|---------|------|--------|--------|
| J1-06 | PTD4  | GPIO      |    1    | <--- | FAULT  |        |
| J1-08 | PTA12 | GPIO      |    1    | <--- | INTB   |        |
| J2-18 | PTE0  | GPIO      |    1    | ---> | EN     |        |
| J1-14 | PTC8  | GPIO      |    1    | ---> | TXCSB  |        |
| J1-09 | PTC5  | SPI0_SCK  |    2    | ---> | TXCLK  |        |
| J2-08 | PTD2  | SPI0_MO   |    2    | ---> | TXDATA |        |
| J9-05 | PTB10 | SPI1_PCS0 |    2    | <--- | RXCSB  |        |
| J9-07 | PTB11 | SPI1_SCK  |    2    | <--- | RXCLK  |        |
| J2-19 | PTD7  | SPI1_SI   |    5    | <--- | RXDATA |        |

| J1-13 | PTC10 | GPIO      |    1    | ---> | BRD0   | low    |
| J1-15 | PTC11 | GPIO      |    1    | ---> | BRD1   | low    |
| J2-01 | PTC12 | GPIO      |    1    | ---> | BRD2   | low    |
| J2-03 | PTC13 | GPIO      |    1    | ---> | BRD3   | low    |

| J1-03 | PTC0  | GPIO      |    1    | ---> | CH0    | low    |
| J1-05 | PTC3  | GPIO      |    1    | ---> | CH1    | low    |
| J1-07 | PTC4  | GPIO      |    1    | ---> | CH2    | low    |
| J1-11 | PTC6  | GPIO      |    1    | ---> | CH3    | low    |
| J1-01 | PTC7  | GPIO      |    1    | ---> | CH4    | low    |

 */	    
	}else if((_interface_==IntTPL)&&(_evb_==EVB_TypeArd))  {								// new ardunio style EVB
	    SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;										// enable SPI0 clock
	    SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;										// enable SPI1 clock

		PORTD_PCR4  = PORT_PCR_MUX(1); 
		GPIOD_PDDR  &= ~BIT(4);	

		PORTA_PCR12  = PORT_PCR_MUX(1); 
		GPIOA_PDDR  &= ~BIT(12);	
	    
		PORTE_PCR0  = PORT_PCR_MUX(1);	 
		GPIOE_PDDR  |= BIT(0);			
		GPIOE_PCOR  = BIT(0);
		
		PORTC_PCR8 = PORT_PCR_MUX(1);						 
		GPIOC_PDDR  |= BIT(8);								
		GPIOC_PSOR  = BIT(8);								
		
		PORTC_PCR5  = PORT_PCR_MUX(2);									
		PORTD_PCR2  = PORT_PCR_MUX(2);									

		PORTB_PCR10 = PORT_PCR_MUX(2);									
		PORTB_PCR11 = PORT_PCR_MUX(2);									
		PORTD_PCR7  = PORT_PCR_MUX(5);									

		
		PORTC_PCR10  = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(10);			
		GPIOC_PCOR  = BIT(10);
		PORTC_PCR11  = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(11);			
		GPIOC_PCOR  = BIT(11);
		PORTC_PCR12  = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(12);			
		GPIOC_PCOR  = BIT(12);
		PORTC_PCR13  = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(13);			
		GPIOC_PCOR  = BIT(13);

		PORTC_PCR0   = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(0);			
		GPIOC_PCOR  = BIT(0);
		PORTC_PCR3   = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(3);			
		GPIOC_PCOR  = BIT(3);
		PORTC_PCR4   = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(4);			
		GPIOC_PCOR  = BIT(4);
		PORTC_PCR6   = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(6);			
		GPIOC_PCOR  = BIT(6);
		PORTC_PCR7   = PORT_PCR_MUX(1); 
		GPIOC_PDDR  |= BIT(7);			
		GPIOC_PCOR  = BIT(7);
		
		
		SPITxInit(0);																	// SPI0 used to transmit (polling)
		SPIRxInit(1);																	// SPI1 used to receive (interrupt driven)
		NVICEnIrq(SPI1_IRQ);	
		SPIRxEnable();
		SPITxEnable();
	}
	
	

}



// ----------------------------------------------------------------------------
/*! \brief De-Initializes the FRDM-KL25Z interface for the specific EVB.

4 EVBs are supported right now:
- old TYPE1 EVBs for SPI and TPL
- new Arduino EVBs for SPI and TPL

The local copy of _interface_ and _evb_ are use to know which interface has do be 
de-initilaized.
Sets the local copies to: 
	_interface_ = IntUnknown;
	_evb_ = EVB_Unknown;

 */
void DeInitInterface(void)  {

/*! 1
| CON   | PIN   | Function       | Pin Alt |
|-------|-------|----------------|---------|
| J2-03 | PTC13 | disable/analog |    0    |
| J2-02 | PTA13 | disable/analog |    0    |
| J2-04 | PTD5  | disable/analog |    0    |
| J2-20 | PTE1  | disable/analog |    0    |
| J2-17 | PTD6  | disable/analog |    0    |
| J2-13 | PTE31 | disable/analog |    0    |
| J2-01 | PTC12 | disable/analog |    0    |
 */	    	
	if((_interface_==IntSPI)&&(_evb_==EVB_Type1))  {				
		
		NVICDisIrq(SPI1_IRQ);	
		SPIDisable();
		SIM_SCGC4 &= ~SIM_SCGC4_SPI1_MASK;										// disable SPI1 clock

		PORTC_PCR(13) = PORT_PCR_MUX(0);								

		PORTA_PCR(13) = PORT_PCR_MUX(0);								
		GPIOA_PDDR  &= ~BIT(13);											

		PORTD_PCR(5) = PORT_PCR_MUX(0);									

		PORTE_PCR(1) = PORT_PCR_MUX(0);									

		PORTD_PCR(6) = PORT_PCR_MUX(0);									

		PORTE_PCR(31) = PORT_PCR_MUX(0);								

	//	PORTC_PCR12 = PORT_PCR_MUX(1);									
	//	GPIOC_PDDR  |= BIT(12);											
	//	GPIOC_PSOR  = BIT(12);											

	}

/*! 2
| CON   | PIN   | Function       | Pin Alt |
|-------|-------|----------------|---------|
| J1-06 | PTD4  | disable/analog |    0    |
| J1-14 | PTC8  | disable/analog |    0    |
| J1-09 | PTC5  | disable/analog |    0    |
| J2-08 | PTD2  | disable/analog |    0    |
| J2-10 | PTD3  | disable/analog |    0    |
|J10-02 | PTB0  | disable/analog |    0    |
|J10-06 | PTB2  | disable/analog |    0    |
| J2-20 | PTE0  | disable/analog |    0    |
 */	    	
	if((_interface_==IntSPI)&&(_evb_==EVB_TypeArd))  {			

		NVICDisIrq(SPI0_IRQ);	
		SPIDisable();
		SIM_SCGC4 &= ~SIM_SCGC4_SPI0_MASK;										// disable SPI0 clock				
		
		PORTD_PCR(4) = PORT_PCR_MUX(0);										
		
		PORTC_PCR(8) = PORT_PCR_MUX(0);					
		GPIOC_PDDR  &= ~BIT(8);							

		PORTC_PCR(5)  = PORT_PCR_MUX(0);				

		PORTD_PCR(2)  = PORT_PCR_MUX(0);				

		PORTD_PCR(3)  = PORT_PCR_MUX(0);				
		
		PORTB_PCR(0) = PORT_PCR_MUX(0);					
		GPIOB_PDDR &= ~BIT(0);				
	}
	
	
/*! 3

| CON   | PIN   | Function       | Pin Alt | 
|-------|-------|----------------|---------|
| J1-16 | PTD7  | disable/analog |    0    | 
| J2-18 | PTE0  | disable/analog |    0    | 
| J2-13 | PTE31 | disable/analog |    0    | 
| J2-02 | PTA13 | disable/analog |    0    | 
| J2-04 | PTD5  | disable/analog |    0    | 
| J2-20 | PTE1  | disable/analog |    0    | 
| J2-06 | PTD0  | disable/analog |    0    | 
| J2-12 | PTD1  | disable/analog |    0    | 
| J2-08 | PTD2  | disable/analog |    0    | 

 */	    

	if((_interface_==IntTPL)&&(_evb_==EVB_Type1))  {

		NVICDisIrq(SPI0_IRQ);	
		SPIDisable();
	    SIM_SCGC4 &= ~SIM_SCGC4_SPI0_MASK;										// disable SPI0 clock
	    SIM_SCGC4 &= ~SIM_SCGC4_SPI1_MASK;										// disable SPI1 clock

	    PORTD_PCR7  = PORT_PCR_MUX(0);	 
		GPIOD_PDDR  &= ~BIT(7);			

		PORTE_PCR0  = PORT_PCR_MUX(0); 
		GPIOE_PDDR  &= ~BIT(0);		
		
		PORTE_PCR31 = PORT_PCR_MUX(0); 
		GPIOE_PDDR  &= ~BIT(31);		

		PORTA_PCR13 = PORT_PCR_MUX(0);			 
		GPIOA_PDDR  &= ~BIT(13);				
		
		PORTD_PCR5  = PORT_PCR_MUX(0); 
		PORTE_PCR1  = PORT_PCR_MUX(0); 

		PORTD_PCR0 = PORT_PCR_MUX(0); 
		PORTD_PCR1 = PORT_PCR_MUX(0);
		PORTD_PCR2 = PORT_PCR_MUX(0); 

	}

/*!
4. \b TPL-TypeArd (HWPLATFORM 3)

	\image html hwplatform3.png

The "new style = type Ard" TPL EVB uses the following interface pins:

| CON   | PIN   | Function       | Pin Alt |
|-------|-------|----------------|---------|
| J1-06 | PTD4  | disable/analog |    0    |
| J1-08 | PTA12 | disable/analog |    0    |
| J2-18 | PTE0  | disable/analog |    0    |
| J1-14 | PTC8  | disable/analog |    0    |
| J1-09 | PTC5  | disable/analog |    0    |
| J2-08 | PTD2  | disable/analog |    0    |
| J9-05 | PTB10 | disable/analog |    0    |
| J9-07 | PTB11 | disable/analog |    0    |
| J2-19 | PTD7  | disable/analog |    0    |

| J1-13 | PTC10 | disable/analog |    0    |
| J1-15 | PTC11 | disable/analog |    0    |
| J2-01 | PTC12 | disable/analog |    0    |
| J2-03 | PTC13 | disable/analog |    0    |
    
| J1-03 | PTC0  | disable/analog |    0    |
| J1-05 | PTC3  | disable/analog |    0    |
| J1-07 | PTC4  | disable/analog |    0    |
| J1-11 | PTC6  | disable/analog |    0    |
| J1-01 | PTC7  | disable/analog |    0    |

*/
	
	if((_interface_==IntTPL)&&(_evb_==EVB_TypeArd))  {														// new ardunio style EVB

		NVICDisIrq(SPI1_IRQ);	
		SPIDisable();
	    SIM_SCGC4 &= ~SIM_SCGC4_SPI0_MASK;										// disable SPI0 clock
	    SIM_SCGC4 &= ~SIM_SCGC4_SPI1_MASK;										// disable SPI1 clock

		PORTD_PCR4  = PORT_PCR_MUX(0); 
		GPIOD_PDDR  &= ~BIT(4);	

		PORTA_PCR12  = PORT_PCR_MUX(0); 
		GPIOA_PDDR  &= ~BIT(12);	
	    
		PORTE_PCR0  = PORT_PCR_MUX(0);	 
		GPIOE_PDDR  &= ~BIT(0);			
		
		PORTC_PCR8 = PORT_PCR_MUX(0);						 
		GPIOC_PDDR  &= ~BIT(8);								
		
		PORTC_PCR5  = PORT_PCR_MUX(0);									
		PORTD_PCR2  = PORT_PCR_MUX(0);									

		PORTB_PCR10 = PORT_PCR_MUX(0);									
		PORTB_PCR11 = PORT_PCR_MUX(0);									
		PORTD_PCR7  = PORT_PCR_MUX(0);			

		PORTC_PCR10 = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(10);			
		PORTC_PCR11 = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(11);			
		PORTC_PCR12 = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(12);			
		PORTC_PCR13 = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(13);			

		PORTC_PCR0  = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(0);			

		PORTC_PCR3  = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(3);			

		PORTC_PCR4  = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(4);			

		PORTC_PCR6  = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(6);			

		PORTC_PCR7  = PORT_PCR_MUX(0);	 
		GPIOC_PDDR  &= ~BIT(7);			

	}

	_interface_ = IntUnknown;
	_evb_ = EVB_Unknown;
}

// ----------------------------------------------------------------------------
/*! \brief Reads the status of the Fault pin. 
 * 
 * Works with static Fault signal.
 * Does not work with Fault Wave.
 
 *  Mandatory to prior call InitInterface(). 
 * 
 * @return 1   if FAULT
 *         0   in no Fault
 */ 

//! \todo use interrupt instead of polling. Enhance to be used with wave signal.
u8 FaultPinStatus(void)  {
	
	if(_interface_==IntTPL)  {													// EVBs via TPL

		if(_evb_==EVB_TypeArd)  {													// Arduino type connection
			return ((GPIOD_PDIR&BIT(4))?1:0);									// PTD4
		}
	
		if(_evb_==EVB_Type1)  {													// old Type1 type connectio
			return ((GPIOD_PDIR&BIT(7))?1:0);									// PTD7
		}	
	}

	if(_interface_==IntSPI)  {
		
		if(_evb_==EVB_TypeArd)  {													// Arduino type connection
			return ((GPIOD_PDIR&BIT(4))?1:0);									// PTD4
		}
	
		if(_evb_==EVB_Type1)  {													// old Type1 type connectio
			return ((GPIOC_PDIR&BIT(13))?1:0);									// PTC13
		}	
	}
	
	return 1;
}

// ----------------------------------------------------------------------------
/*! \brief Sets the TPL_Enable pin. 
 */ 
void TplEnable(u8 bEnable)  {
	
	if(_interface_==IntTPL)  {													// EVBs via TPL

		if(_evb_==EVB_TypeArd)  {													// Arduino type connection
			if(bEnable==0)
				GPIOE_PCOR  = BIT(0);
			else	
				GPIOE_PSOR  = BIT(0);
		}
	
		if(_evb_==EVB_Type1)  {													// old Type1 type connectio
			if(bEnable==0)
				GPIOE_PCOR  = BIT(31);
			else	
				GPIOE_PSOR  = BIT(31);
		}	
	}
}


// ----------------------------------------------------------------------------
/*! \brief Reads the status of the INTB pin. 
 
  *  Mandatory to prior call InitInterface(). 
 * 
 * 
 * @return 1   if INTB high
 *         0   in INTB low 
 */ 
u8 IntbPinStatus(void)  {
	
	if(_interface_==IntTPL)  {										// EVBs via TPL

		if(_evb_==EVB_TypeArd)  {										// Arduino type connection
			return ((GPIOA_PDIR&BIT(12))?1:0);									// PTA12
		}
	
		if(_evb_==EVB_Type1)  {											// old Type1 type connection
			return ((GPIOE_PDIR&BIT(0))?1:0);									// PTE0
		}	
	}
	return 1;
}


//-----------------------------------------------------------------------------
/*! \brief Sets the CSB level.
 * 
 *  Mandatory to prior call InitInterface(). 
 * 
 * @param  u8Level		0 for low,  <>0 for high
 * @param  drv         \ref TYPE_DRV_SETUP
 */
void SPICSB(u8 u8Level)  {

	if((_interface_==IntSPI)&&(_evb_==EVB_TypeArd))  {
		if(u8Level==0)
			GPIOC_PCOR  = BIT(8); 												// clear PTC8
		else
			GPIOC_PSOR  = BIT(8); 												// set PTC8
	}
	
	if((_interface_==IntSPI)&&(_evb_==EVB_Type1))  {
		if(u8Level==0)
			GPIOA_PCOR  = BIT(13); 												// clear PTA13
		else
			GPIOA_PSOR  = BIT(13); 												// set PTA13
	}
	
	if((_interface_==IntTPL)&&(_evb_==EVB_TypeArd))  {				
		if(u8Level==0)
			GPIOC_PCOR  = BIT(8); 												// clear PTC8
		else
			GPIOC_PSOR  = BIT(8); 												// set PTC8
	}
	
	if((_interface_==IntTPL)&&(_evb_==EVB_Type1))  {				  	
		if(u8Level==0)
			GPIOA_PCOR  = BIT(13); 												// clear PTA13
		else
			GPIOA_PSOR  = BIT(13); 												// set PTA13
	}	
}

void initFIMode(u8 u8enabledDisabled)										// Activate external script trigger (Fault injection mode)
{
	//SM trigger implementation:
	
	if(u8enabledDisabled < 1)												//FI mode disabled
	{
		PORTA_PCR16 |= PORT_PCR_MUX(1);										//PTA16 is configured as GPIO to disable NMI function on this pin
		PORTA_PCR16 &= (~PORT_PCR_IRQC_MASK | PORT_PCR_IRQC(0x0));			//Disable PTA16 interruptions but keep GPIO function
		NVICDisIrq(PORTA_IRQ);												//Disable NVIC PORT A interrupt
		
		GPIOA_PCOR  |= BIT(5);												//PTA5 output is set to a low state
		GPIOA_PDDR  &= ~BIT(5);												//PTA5 is set as default
		PORTA_PCR5 = 0x00;													//Clear PTA5 configuration 
		
		LED_GREEN_Off();
		LED_RED_On();
	}
	
	else if(u8enabledDisabled >= 1)													//FI mode enabled
	{
		PORTA_PCR16 |= PORT_PCR_MUX(1) |PORT_PCR_IRQC(0xA);					//PTA16 is configured as GPIO with interrupt on falling edge
		GPIOA_PDDR &= ~BIT(16);												//PTA16 is an input	
		NVICSetIrqPrio(PORTA_IRQ, IP_PRIO_1);								//Set NVIC priority of PORTA interrupt (priority 1)
		NVICEnIrq(PORTA_IRQ);												//Enable NVIC PORT A interrupt
		
		PORTA_PCR5 = PORT_PCR_MUX(1);										//PTA5 is configured as GPIO 
		GPIOA_PDDR  |= BIT(5);												//PTA5 is an output
		GPIOA_PCOR  |= BIT(5);												//PTA5 output is set to a low state
		
		LED_RED_Off();
		LED_GREEN_On();
	}
}
