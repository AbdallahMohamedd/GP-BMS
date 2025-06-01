// ----------------------------------------------------------------------------
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
// ----------------------------------------------------------------------------
#include <stdarg.h>
#include <stdio.h>
#include <uart0.h>
// ----------------------------------------------------------------------------
// transmitter
#define UART0TXBUFFERSIZE     		256
u8 u8Uart0TxBuf[UART0TXBUFFERSIZE];												// software needs to be changed if <> 256
u16 u16Uart0TxIdxOut;
u16 u16Uart0TxIdxIn;
volatile u16 u16Uart0TxBufFree;
bool bTransmitting;
// ----------------------------------------------------------------------------
// receiver
#define UART0RXBUFFERSIZE     		256
u8 u8Uart0RxBuf[UART0RXBUFFERSIZE];												// software needs to be changed if <> 256
u16 u16Uart0RxIdxOut;
u16 u16Uart0RxIdxIn;
// ----------------------------------------------------------------------------
char u8Buf[200];
// ----------------------------------------------------------------------------
/*! \brief Initializes the Uart0 module
 *
 * Initializes the Uart0 module:
 * - set the transmit and receive buffer pointers to 0
 * - set the baudrate
 * - enables receiver, transmitter
 * - enable receive interrupt
 * - set transmitting flag to false (transmitter inactive)
 *
 * \sa uart0
 *
 */
void Uart0Init(void)  {

	// transmitter
	u16Uart0TxIdxOut = 0;
    u16Uart0TxIdxIn = 0;
    u16Uart0TxBufFree = UART0TXBUFFERSIZE;
    bTransmitting = false;

    // receiver
    u16Uart0RxIdxOut = 0;
    u16Uart0RxIdxIn = 0;

    UART0_BDH = (UART0_BD>>8)&0x1F;
    UART0_BDL = (UART0_BD>>0)&0xFF;
    UART0_C2  = UART0_C2_RIE_MASK|UART0_C2_RE_MASK | UART0_C2_TE_MASK;
}
// ----------------------------------------------------------------------------
/*! \brief Reads one character from receive buffer.
 *
 * Reads and returns one character from the receive buffer, updates the buffer pointers.
 *
 * @param u8Byte
 * @return \b true  if character was read
 * @return \b false if no character was read (buffer is empty)
 *
 *
 * \sa uart0
 */
bool Uart0ReadByte(u8 *u8Byte)  {
	u8 u8Temp;

	*u8Byte = ' ';
	u8Temp = UART0_C2;
	UART0_C2 &= ~ UART0_C2_RIE_MASK;											// stop rx interrupt

	if(u16Uart0RxIdxIn == u16Uart0RxIdxOut)  {
		UART0_C2 = u8Temp;														// restore
		return false;															// rx buffer empty
	}else{
		*u8Byte = u8Uart0RxBuf[u16Uart0RxIdxOut];
		u16Uart0RxIdxOut = (u16Uart0RxIdxOut + 1) % UART0RXBUFFERSIZE;
		UART0_C2 = u8Temp;														// restore
		return true;
	}
}
// ----------------------------------------------------------------------------
/*! \brief Sends one character
 *
 * Waits till there is space available in the transmit buffer and copies the
 * characters into transmit buffer.
 *
 * Activates the transmitter if inactive.
 *
 * \sa Uart0Send, Uart0SendWaiting
 */
void Uart0SendByte(u8 byte)  {
	volatile u16 u16Free;
	u8 u8Temp;

	do {

		u8Temp = UART0_C2;
		UART0_C2 &= ~UART0_C2_TCIE_MASK;											// disable transmit complete IRQ
		// critial (not atomic)
		u16Free = u16Uart0TxBufFree;												// copy because mod. in IRQ
		UART0_C2 = u8Temp;															// restore
	}while(u16Free<10);


	if(u16Free>2)  {
		u8Uart0TxBuf[u16Uart0TxIdxIn++] = byte;
		u16Uart0TxIdxIn = u16Uart0TxIdxIn % UART0TXBUFFERSIZE;

		u8Temp = UART0_C2;
		UART0_C2 &= ~UART0_C2_TCIE_MASK;										// disable transmit complete IRQ
		// critial (not atomic)
		u16Uart0TxBufFree--;
		UART0_C2 = u8Temp;														// restore
	}else{
		while(1) DONOTHING();
	}

	if(!bTransmitting)  {														// queue empty
		bTransmitting = true;
		UART0_C2 |= UART0_C2_TCIE_MASK;											// enable transmit complete IRQ
	}
}
// ----------------------------------------------------------------------------
/*! \brief Sends multiple characters.
 *
 * Waits till there is space available in the transmit buffer and copies the
 * characters into transmit buffer.
 *
 * Activates the transmitter if inactive.
 *
 * * \sa Uart0SendWaiting, Uart0SendByte
 *
 */
void Uart0Send(char *u8ptr, u8 count)  {
   volatile u16 u16Free;
   u8 u8Temp;

   while (count>0) {															// copy buffer
		u8Temp = UART0_C2;
		UART0_C2 &= ~UART0_C2_TCIE_MASK;										// disable transmit complete IRQ
		// critial (not atomic)
		u16Free = u16Uart0TxBufFree;											// copy because modification. in IRQ
		UART0_C2 = u8Temp;														// restore

		if(u16Free>2)  {
			u8Uart0TxBuf[u16Uart0TxIdxIn++] = *u8ptr++;
			u16Uart0TxIdxIn = u16Uart0TxIdxIn % UART0TXBUFFERSIZE;
			count--;

			u8Temp = UART0_C2;
			UART0_C2 &= ~UART0_C2_TCIE_MASK;									// disable transmit complete IRQ
			// critial (not atomic)
			u16Uart0TxBufFree--;
			UART0_C2 = u8Temp;													// restore

		}
	}
	if(!bTransmitting)  {														// queue empty
		bTransmitting = true;
//		UART0_D = u8TxBuf[u16TxIdxOut++];										// transmit first byte
		UART0_C2 |= UART0_C2_TCIE_MASK;											// enable transmit complete IRQ
	}
}
// ----------------------------------------------------------------------------
/*! \brief Send multiple characters.
 *
 * Same functionality than \ref Uart0Send, but waits till transmitting is completed
 * (transmitter is inactive), before it returns.
 *
 * \sa Uart0Send, Uart0SendByte
 */
void Uart0SendWaiting(char *u8ptr, u16 u16Bytes)  {

	Uart0Send(u8ptr, u16Bytes);
	while(bTransmitting)   DONOTHING();											// transmit everything, before finish
}
// ----------------------------------------------------------------------------
/*! \brief Uart0 Interrupt Handler
 *
 * The Uart interrupt handler is handling interrupt driven receiving and
 * transmitting.
 *
 * Handles receive and transmit buffers and pointers.
 * De-activates transmitter when transmit buffer is empty.
 *
 * also see \ref uart0
 *
 */
void UART0_IRQHandler(void)  {

	if(UART0_S1 & UART0_S1_TC_MASK)  {											// transmitting

		if(u16Uart0TxBufFree==UART0TXBUFFERSIZE)  {								// buffer empty
			UART0_C2 &= ~UART0_C2_TCIE_MASK;									// disable transmit complete IRQ
			bTransmitting = false;
		}else{
			UART0_D = u8Uart0TxBuf[u16Uart0TxIdxOut++];							// next byte
			u16Uart0TxIdxOut = u16Uart0TxIdxOut % UART0TXBUFFERSIZE;
			u16Uart0TxBufFree++;
		}
	}

	if(UART0_S1 & UART0_S1_RDRF_MASK)  {										// receiving

		u8Uart0RxBuf[u16Uart0RxIdxIn++] = UART0_D;								// next byte
		u16Uart0RxIdxIn = u16Uart0RxIdxIn % UART0RXBUFFERSIZE;
		if(u16Uart0RxIdxIn==u16Uart0RxIdxOut)  {
			//! \todo overflow handling
			while(1)  DONOTHING();
		}
	}

}
