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
//! \addtogroup drivers 
// @{
/*! \brief Functions to handle UART0 communication
 * 
 * The UART0 is used to communicate to a host application (GUI running on a PC), 
 * indicating provided information (e.g. measurement results) and allowing to 
 * control the embedded application (e.g. enter low power mode).
 * 
 * The uart sources are using an interrupt driven receiving and transmitting.
 * 
 * For the transmitting a FIFO (First In First Out) buffer (256 characters) is 
 * implemented together with a buffer pointer for adding characters to the buffer
 * (\ref u16Uart0TxIdxIn) and a buffer pointer for getting the next character
 * (\ref u16Uart0TxIdxOut) to be transmitted (TxD).
 * 
 * For the receiving also a FIFO buffer (256 characters) is 
 * implemented together with a buffer pointer for adding characters received on 
 * the UART (RxD) to the buffer (\ref u16Uart0RxIdxIn) and a buffer pointer for
 * retrieving the next character (\ref u16Uart0RxIdxOut) from the buffer (e.g. \ref 
 * Uart0ReadByte).
 *   
 * A single interrupt (\ref UART0_IRQHandler) is handling both receiving and
 * transmitting of single characters. 
 * 
 * \note
 * The application has to ensure the transmit buffer is not overflowing
 *  (e.g. \ref Uart0SendWaiting).
 * 
 * \note
 * The application has to ensure that the receiver buffer is not overflowing!
 * 
 */
//! \addtogroup uart0
// @{
// ----------------------------------------------------------------------------
#ifndef UART0_H_
#define UART0_H_
// ----------------------------------------------------------------------------
#include <MKL25Z4.h>
#include "Platform/KL25ZUtil.h"
#include "Platform/frdmkl25z.h"
// ----------------------------------------------------------------------------
#define UART0_BD          (u16) (2*BUSFREQ/(16*115200))							//!< Uart0 Baudrate \n Uart 0 runs on system clock = 2x BUSFREQ    					
// ----------------------------------------------------------------------------
void Uart0Init(void);
bool Uart0ReadByte(u8 *u8Byte);
void Uart0SendByte(u8 byte);
void Uart0Send(char *u8ptr, u8 count);
void Uart0SendWaiting(char *u8ptr, u16 u16Bytes);
// ----------------------------------------------------------------------------
void UART0_IRQHandler(void);
// ----------------------------------------------------------------------------
#endif /* UART0_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------
