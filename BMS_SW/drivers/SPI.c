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
// ------------------------------------------------------------------
#include <COTs/SlaveControlIF/Inc/slaveIF.h>
#include <SPI.h>
#include <tpm1.h> // delay
// #include "source/drivers/lld3377x.h"
//-----------------------------------------------------------------------------
// private storage of SPI no
u8 _RxSPI; // stores module number of receive SPI
u8 _TxSPI; // stores module number of transmit SPI
//-----------------------------------------------------------------------------
// private storage for IRQ receive buffer (FIFO)
u8 u8SPIRxBuf[255]; // limits the max read to 250/5 = 50 registers
u8 u8SPIRxIdxOut;	// buffer out pointer (FIFO)
u8 u8SPIRxIdxIn;	// buffer in pointer (FIFO)
//-----------------------------------------------------------------------------
/*! \brief Initialises the SPI. (Single-SPI usage)
 *
 * The SPI is setup as:
 * - Master mode
 * - 1 Mbit/s  (2 MHz SPI clock)
 * - clock idle is low
 * - SCLK edge occurs at start (data valid at falling edge)
 * - the spiNo is stored in _RxSPI and _TxSPI variables
 *
 * The SPI RX buffer pointers are set to 0.
 *
 * @param spiNo SPI module number (0 or 1)
 *
 *
 * \remarks
 * - SPI0 runs on bus clock (24 MHz)
 * - SPI1 runs on system clock (48 MHz)
 *
 */
void SPIInit(u8 spiNo)
{

	_TxSPI = spiNo;
	_RxSPI = spiNo;

	u8SPIRxIdxOut = 0;
	u8SPIRxIdxIn = 0;

	if (_TxSPI == 0)
	{
		SPI0_C1 = SPI_C1_MSTR_MASK | SPI_C1_CPHA_MASK;		  // slave, clock idle=low, SCLK edge occurs at start
		SPI0_BR = SPI_BR_SPPR(3 - 1) | SPI_BR_SPR(4 / 2 - 1); // 24MHz bus   /  (3 * 4) = 2MHz
															  //		SPI0_BR = SPI_BR_SPPR(3-1) | SPI_BR_SPR(2/2-1); 						//24MHz bus   /  (3 * 2) = 4MHz
	}
	if (_TxSPI == 1)
	{
		SPI1_C1 = SPI_C1_MSTR_MASK | SPI_C1_CPHA_MASK; // master, clock idle=low, SCLK edge occurs at start
		SPI1_BR = SPI_BR_SPPR(6 - 1) | SPI_BR_SPR(1);  // 48MHz sysclock   /  (6 * 4) = 2MHz
	}
}
//-----------------------------------------------------------------------------
/*! \brief Enables the SPI and receive interrupt. (Single-SPI usage)
 *
 * Enables the initialised (_RxSPI and _TxSPI variables) SPI and receive interrupt.
 *
 * \b Usage-Example:
 * \code
 * SPIInit(1);
 * SPIEnable();
 * \endcode
 *
 */
void SPIEnable(void)
{

	if (_RxSPI == 0)
	{
		SPI0_C1 |= SPI_C1_SPE_MASK;	 // enable spi
		SPI0_C1 |= SPI_C1_SPIE_MASK; // enable interrupt
	}
	if (_RxSPI == 1)
	{
		SPI1_C1 |= SPI_C1_SPE_MASK;	 // enable spi
		SPI1_C1 |= SPI_C1_SPIE_MASK; // enable interrupt
	}
}
//-----------------------------------------------------------------------------
/*! \brief Disables the SPI and receive interrupt. (Single-SPI usage)
 *
 * Disables the initialised (_RxSPI and _TxSPI variables) SPI and receive interrupt.
 *
 */
void SPIDisable(void)
{

	if (_RxSPI == 0)
	{
		SPI0_C1 &= ~SPI_C1_SPE_MASK;  // disable spi
		SPI0_C1 &= ~SPI_C1_SPIE_MASK; // disable interrupt
	}
	if (_RxSPI == 1)
	{
		SPI1_C1 &= ~SPI_C1_SPE_MASK;  // disable spi
		SPI1_C1 &= ~SPI_C1_SPIE_MASK; // disable interrupt
	}
}
//-----------------------------------------------------------------------------
/*! \brief SPI routine to transfer data. (Single-SPI usage)
 *
 * @param u8TxData  buffer of transmit data
 * @param u8RxData  buffer for receive data
 * @param u8TxLen   amount of data bytes to transfere
 * @return  RETURN_OK if successful
 */
bool SPISendBuffer(u8 *u8TxData, u8 *u8RxData, u8 u8TxLen)
{

	SPIRxFlushBuffer();					// flush receiver
	SPITxSendBuffer(u8TxData, u8TxLen); // transmit read request

	if (MSGLEN != SPIRxReadBytes(u8RxData, u8TxLen)) // read request "echo" and check
		return slaveIF_setError(ERR_TX);

	return true; // Return Data
}

//-----------------------------------------------------------------------------
/*! \brief Initialises the spiNo for transmitting as SPI Master. (Dual-SPI usage)
 *
 * The SPI is setup as:
 * - Master mode
 * - 1 Mbit/s  (2 MHz SPI clock)
 * - clock idle is low
 * - SCLK edge occurs at start (data valid at falling edge)
 * - the spiNo is stored in _TxSPI variable
 *
 * @param spiNo SPI module number (0 or 1)
 *
 * \remarks
 * - SPI0 runs on bus clock (24 MHz)
 * - SPI1 runs on system clock (48 MHz)
 *
 */
void SPITxInit(u8 spiNo)
{

	_TxSPI = spiNo;

	if (_TxSPI == 0)
	{
		// SPI0 runs on bus clock!!!
		SPI0_C1 = SPI_C1_MSTR_MASK | SPI_C1_CPHA_MASK;		  // slave, clock idle=low, SCLK edge occurs at start
		SPI0_BR = SPI_BR_SPPR(3 - 1) | SPI_BR_SPR(4 / 2 - 1); // 24MHz bus   /  (3 * 4) = 2MHz
	}
	if (_TxSPI == 1)
	{
		// SPI1 runs on system clock!!!
		SPI1_C1 = SPI_C1_MSTR_MASK | SPI_C1_CPHA_MASK; // master, clock idle=low, SCLK edge occurs at start
		SPI1_BR = SPI_BR_SPPR(6 - 1) | SPI_BR_SPR(1);  // 48MHz sysclock   /  (6 * 4) = 2MHz
	}
}
//-----------------------------------------------------------------------------
/*! \brief Enables the Transmit SPI. (Dual-SPI usage)
 *
 * Enables the initialised (_TxSPI variables) SPI.
 *
 * \b Usage-Example:
 * \code
 * SPITxInit(1);
 * SPITxEnable();
 * \endcode
 *
 */
void SPITxEnable(void)
{

	if (_TxSPI == 0)
	{
		SPI0_C1 |= SPI_C1_SPE_MASK;
	}
	if (_TxSPI == 1)
	{
		SPI1_C1 |= SPI_C1_SPE_MASK;
	}
}
//-----------------------------------------------------------------------------
/*! \brief Disables the Rransmit SPI. (Dual-SPI usage)
 *
 * Disables the initialised (_TxSPI variables) SPI.
 *
 */
void SPITxDisable(void)
{

	if (_TxSPI == 0)
	{
		SPI0_C1 &= ~SPI_C1_SPE_MASK;
	}
	if (_TxSPI == 1)
	{
		SPI1_C1 &= ~SPI_C1_SPE_MASK;
	}
}
//-----------------------------------------------------------------------------
/*! \brief Transmits u8TxLen bytes from the u8TxData buffer.. (Dual-SPI usage)
 *
 * Handles CS signal and transmitting of the bytes.
 *
 * @param u8TxData  buffer to data to transmit
 * @param u8TxLen   number of bytes to transmit
 * @return  0  always
 */
void SPITxSendBuffer(u8 *u8TxData, u8 u8TxLen)
{

	slaveIF_SPICS(0);
	if (_TxSPI == 0)
	{
		// transmit handling
		while (u8TxLen > 0)
		{ // everything send and received
			if (SPI0_S & SPI_S_SPTEF_MASK)
			{ // transmitter empty
				if (u8TxLen > 0)
				{ // still data to send?
					u8TxLen--;
					SPI0_D = *(u8TxData + u8TxLen);
				}
			}
		}
		while (!(SPI0_S & SPI_S_SPTEF_MASK))
			DONOTHING(); // wait till everything was sent
	}
	if (_TxSPI == 1)
	{
		// transmit handling
		while (u8TxLen > 0)
		{ // everything send and received
			if (SPI1_S & SPI_S_SPTEF_MASK)
			{ // transmitter empty
				if (u8TxLen > 0)
				{ // still data to send?
					u8TxLen--;
					SPI1_D = *(u8TxData + u8TxLen);
				}
			}
		}
		while (!(SPI1_S & SPI_S_SPTEF_MASK))
			DONOTHING(); // wait till everything was sent
	}
	Delay(DELAY_1us); // required to wait till buffer is sent
	slaveIF_SPICS(1);
}
//-----------------------------------------------------------------------------
/*! \brief Initialises the spiNo for receiving as SPI Slave. (Dual-SPI usage)
 *
 * The SPI is setup as:
 * - Slave mode
 * - 1 Mbit/s  (2 MHz SPI clock)
 * - clock idle is low
 * - SCLK edge occurs at start (data valid at falling edge)
 * - the spiNo is stored in _RxSPI variable
 *
 * @param spiNo SPI module number (0 or 1)
 *
 * \note
 * Receiving is handled by interrupt. Reading reads out the buffered data.
 *
 * \remarks
 * - SPI0 runs on bus clock (24 MHz)
 * - SPI1 runs on system clock (48 MHz)
 */
void SPIRxInit(u8 spiNo)
{

	_RxSPI = spiNo;

	if (_RxSPI == 0)
	{
		SPI0_C1 = SPI_C1_CPHA_MASK; // slave, clock idle=low, SCLK edge occurs at start
		// SPI0 runs on bus clock!!!
		SPI0_BR = SPI_BR_SPPR(3 - 1) | SPI_BR_SPR(4 / 2 - 1); // 24MHz bus   /  (3 * 4) = 2MHz
	}
	if (_RxSPI == 1)
	{
		SPI1_C1 = SPI_C1_CPHA_MASK; // slave, clock idle=low, SCLK edge occurs at start
		// SPI1 runs on system clock!!!
		SPI1_BR = SPI_BR_SPPR(6 - 1) | SPI_BR_SPR(1); // 48MHz sysclock   /  (6 * 4) = 2MHz
	}
	u8SPIRxIdxOut = 0;
	u8SPIRxIdxIn = 0;
}
//-----------------------------------------------------------------------------
/*! \brief Enables the Receive SPI. (Dual-SPI usage)
 *
 * Enables the initialised (_RxSPI variables) SPI and interrupt.
 *
 * \b Usage-Example:
 * \code
 * SPIRxInit(1);
 * SPIRxEnable();
 * \endcode
 *
 */
void SPIRxEnable(void)
{

	if (_RxSPI == 0)
	{
		SPI0_C1 |= SPI_C1_SPE_MASK;	 // enable spi
		SPI0_C1 |= SPI_C1_SPIE_MASK; // enable interrupt
	}
	if (_RxSPI == 1)
	{
		SPI1_C1 |= SPI_C1_SPE_MASK;	 // enable spi
		SPI1_C1 |= SPI_C1_SPIE_MASK; // enable interrupt
	}
}
//-----------------------------------------------------------------------------
/*! \brief Disables the Receive SPI. (Dual-SPI usage)
 *
 * Disables the initialised (_RxSPI variables) SPI and interrupt.
 *
 */
void SPIRxDisable(void)
{

	if (_RxSPI == 0)
	{
		SPI0_C1 &= ~SPI_C1_SPE_MASK;  // disable spi
		SPI0_C1 &= ~SPI_C1_SPIE_MASK; // disable interrupt
	}
	if (_RxSPI == 1)
	{
		SPI1_C1 &= ~SPI_C1_SPE_MASK;  // disable spi
		SPI1_C1 &= ~SPI_C1_SPIE_MASK; // disable interrupt
	}
}

//-----------------------------------------------------------------------------
/*! \brief Flush the SPI receive buffer. (Dual-SPI usage)
 *
 */
void SPIRxFlushBuffer(void)
{

	u8SPIRxIdxOut = 0;
	u8SPIRxIdxIn = 0;
}
//-----------------------------------------------------------------------------
/*! \brief Read maxBytes data bytes from receive buffer. (Dual-SPI usage)
 *
 * @param u8Data    pointer to read buffer
 * @param maxBytes  number of bytes to read
 * @return			number of bytes read
 */
u8 SPIRxReadBytes(u8 *u8Data, u8 maxBytes)
{
	u8 u8BytesRead;

	u8BytesRead = 0;
	u8Data = u8Data + maxBytes; // set pointer to end of buffer

	while ((u8SPIRxIdxOut != u8SPIRxIdxIn) && (u8BytesRead < maxBytes))
	{ // data in buffer and not reached maxBytes
		u8Data--;
		*u8Data = u8SPIRxBuf[u8SPIRxIdxOut]; // read out
		if (u8SPIRxIdxOut++ >= sizeof(u8SPIRxBuf))
			u8SPIRxIdxOut = 0;
		u8BytesRead++;
	}
	return u8BytesRead;
}
//-----------------------------------------------------------------------------
#define SPI_SPRF_TIMEOUT 48000
/*! \brief SPI0 Interrupt handler
 *
 * Handles receiving of bytes.
 * The received byte is stored in the u8SPIRxBuf buffer
 *
 *
 *
 */
void SPI0_IRQHandler(void)
{
	u16 u16Timeout;

	u16Timeout = SPI_SPRF_TIMEOUT;

	while ((!(SPI0_S & SPI_S_SPRF_MASK)) && (u16Timeout > 0))
	{ // workaround for SPI RF missing
		u16Timeout--;
	} // Wait for RX Ready bit or timeout

	u8SPIRxBuf[u8SPIRxIdxIn] = SPI0_D;
	if (u8SPIRxIdxIn++ >= sizeof(u8SPIRxBuf))
		u8SPIRxIdxIn = 0; // ring buffer

	//! \todo add buffer overrun detection
}
//-----------------------------------------------------------------------------
void SPI1_IRQHandler(void)
{
	u16 u16Timeout;

	u16Timeout = SPI_SPRF_TIMEOUT;
	while ((!(SPI1_S & SPI_S_SPRF_MASK)) && (u16Timeout > 0))
	{ // workaround for SPI RF missing
		u16Timeout--;
	} // Wait for RX Ready bit or timeout

	u8SPIRxBuf[u8SPIRxIdxIn] = SPI1_D;
	if (u8SPIRxIdxIn++ >= sizeof(u8SPIRxBuf))
		u8SPIRxIdxIn = 0; // ring buffer

	//! \todo add buffer overrun detection
}
//-----------------------------------------------------------------------------
/*! \brief Transmits u8TxLen bytes from the u8TxData buffer.. (Dual-SPI usage)
 *
 * \note
 * This is a function used for a special use case.
 *
 * \todo remove from Driver Library
 *
 *
 * DOES not handles CS signal
 * Handles transmitting of the bytes.
 *
 * @param u8TxData  buffer to data to transmit
 * @param u8TxLen   number of bytes to transmit
 *
 */
void SPITxSendBufferNoCSB(u8 *u8TxData, u8 u8TxLen)
{

	if (_TxSPI == 0)
	{
		// transmit handling
		while (u8TxLen > 0)
		{ // everything send and received

			if (SPI0_S & SPI_S_SPTEF_MASK)
			{ // transmitter empty
				if (u8TxLen > 0)
				{ // still data to send?
					u8TxLen--;
					SPI0_D = *(u8TxData + u8TxLen);
				}
			}
		}
		while (!(SPI0_S & SPI_S_SPTEF_MASK))
			DONOTHING(); // wait till everything was sent
	}
	if (_TxSPI == 1)
	{
		// transmit handling
		while (u8TxLen > 0)
		{ // everything send and received

			if (SPI1_S & SPI_S_SPTEF_MASK)
			{ // transmitter empty
				if (u8TxLen > 0)
				{ // still data to send?
					u8TxLen--;
					SPI1_D = *(u8TxData + u8TxLen);
				}
			}
		}
		while (!(SPI1_S & SPI_S_SPTEF_MASK))
			DONOTHING(); // wait till everything was sent
	}
	Delay(DELAY_1us); // required to wait till buffer is sent
}
//-----------------------------------------------------------------------------
//! \todo remove from Driver Library

/*void PORTA_IRQHandler(void)  {
	u8 idx, rxBytes, rxFrame[5];
	u8 reg;
	u16 data;
	u32 isense;

	if(PORTA_ISFR & BIT(13))  {

		rxBytes = SPIRxReadBytes(rxFrame, 5);									//read
		SPIRxFlushBuffer();														//flush receiver

		idx = u8SPIRxIdxIn;


		if(rxBytes == 0)  {
			u16Counter[1]++;				// no response counter
		}else if (rxBytes != 5)  {
			u16Counter[2]++;				// ResponseLen  counter
		}else if (lld3377x_CrcCalc(rxFrame, 5)!=0)  {
			u16Counter[3]++;				// CRC error  counter
		}else if(1) {
			u16Counter[0]++;				// ok response counter
		}

		reg  = UNPACK_REGADR(rxFrame);
		data = UNPACK_DATA(rxFrame);
		u32RegCopy[reg] = BIT(31)|data;

		if(reg==MEAS_ISENSE2)  {
			isense = (u32RegCopy[MEAS_ISENSE1]&0x7FFF)<<4  |  (u32RegCopy[MEAS_ISENSE2]&0x000F)<<0;
			u32RegCopy[0] = BIT(31)|isense;
		}
		PORTA_PCR13 |= PORT_PCR_ISF_MASK;
	}
}
*/
