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
//! \addtogroup drivers
// @{
/*! \brief SPI driver for the FRDM-KL25Z EVB
 * 
 * 
 * This driver contains two different implementations to fit to the different hardware platforms.
 *  
 * The main difference is if one SPI (\b Single-SPI) is used for transmitting and receiving, or
 * if a separate SPI (\b Dual-SPI) is used - one for transmitting and another one for receiving.
 * In the later case the transmitting SPI is used in \a Master and the receiving one as SPI \a Slave.
 *  
 * \b Single-SPI
 *       
 * Normal SPI usage as master, handling both transmitting and receiving.
 *                                     
 * e.g. used for SPI interface (Arduino type EVB)                                                                             
 * \image html hwplatform1.png                                                                  
 *  
 * \b Usage-Example:
 * 
 * Single \b SPI0 used for transmitting and receiving
 * 
 * \code  
 	SPIInit(0);						
 	SPIEnable();
 	NVICEnIrq(SPI0_IRQ);	
 * \endcode
 * 
 * 
 * \b Dual-SPI
 *  	                                                                                             
 * One SPI used as Master for transmitting (\c SPITx..... functions) 
 *                                                 
 * One SPI used as Slave for receiving (\c SPIRx..... functions)                                                    
 * 
 * e.g. used for TPL interface (Arduino type EVB)
 * \image html hwplatform3.png                                                                  
 *                                                                                                   
 * \b Usage-Example:

 * \b SPI0 (Master) used for transmitting 
 *  
 * \b SPI1 (Slave) used for receiving
 *  
 * \code  
	SPITxInit(0);					
	SPIRxInit(1);					
	NVICEnIrq(SPI1_IRQ);	
	SPITxEnable();
	SPIRxEnable();
 * \endcode
 * 
 * 
 * 
 * \note
 * The \b CS (chip select) signal is handled manually. 
 * To support different hardware platform, the CS functions is in the \ref frdmkl25z module.\n
 * The SPI receiving is handled interrupt driven.
 * The receive buffer is limited to 250 bytes (=50 x 40bit frames) 
 *  
 *  \remarks
 *  - SPI0 runs on bus clock!!!
 *  - SPI1 runs on system clock!!!
 *  - issues running at 2MHz with only receiving 4 bytes instead of 5!!
 *  - 4MHz SPI seems to be too high for CPU performance
*/

//! \addtogroup spi

// @{
// ----------------------------------------------------------------------------
#ifndef SPI_H_
#define SPI_H_
// ----------------------------------------------------------------------------
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include <MKL25Z4.h>
#include "COTs/SlaveControlIF/Inc/SlaveIF.h"
#include "board.h"
#include "clock_config.h"
// ----------------------------------------------------------------------------
// Single-SPI functions
void SPIInit(u8 spiNo);
void SPIEnable(void);
void SPIDisable(void);
bool SPISendBuffer(u8 *u8TxData, u8 *u8RxData, u8 u8Len);
// ----------------------------------------------------------------------------
// Dual-SPI functions
void SPITxInit(u8 spiNo); 
void SPITxEnable(void);
void SPITxDisable(void);
void SPITxSendBuffer(u8 *u8TxData, u8 u8TxLen); 
void SPIRxInit(u8 spiNo);
void SPIRxEnable(void);
void SPIRxDisable(void);
void SPIRxFlushBuffer(void);
u8 SPIRxReadBytes(u8 *u8Data, u8 maxBytes);
void SPI0_IRQHandler(void);
void SPI1_IRQHandler(void);
// ----------------------------------------------------------------------------
// for DAC Board
void SPITxSendBufferNoCSB(u8 *u8TxData, u8 u8TxLen);
// ----------------------------------------------------------------------------
//void PORTA_IRQHandler(void);
// ----------------------------------------------------------------------------
#endif /* SPI_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------
