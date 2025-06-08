//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
//! \addtogroup platform
// @{
//! \brief Useful macros and inlines for the KL25Z controller
//! \addtogroup kl25zutil
// @{
//-----------------------------------------------------------------------------
#ifndef KL25ZUTIL_H_
#define KL25ZUTIL_H_
//-----------------------------------------------------------------------------
#include <stdbool.h>
#include "MKL25Z4.h"
#include "board.h"
//#define	TRUE	1
//#define	FALSE	0	
#define SET_BIT(address, bit) address |= (1 << bit)
//-----------------------------------------------------------------------------
#include <stdint.h>
//-----------------------------------------------------------------------------
typedef uint8_t u8;
typedef uint16_t  u16;
typedef uint32_t u32;

typedef int8_t s8;
typedef int16_t  s16;
typedef int32_t s32;
typedef int64_t s64;
//-----------------------------------------------------------------------------
#define EVER               (;;)													//!< for better readability
#define DONOTHING()         {;}  												//!< for better readability
#define BIT(x)             (1UL<<(x))											//!< bit value handling
// ----------------------------------------------------------------------------
#define NVICEnIrq(v) 	NVIC_BASE_PTR->ISER = BIT((v))							//!< enable interrupt				
#define NVICDisIrq(v) 	NVIC_BASE_PTR->ICER = BIT((v))							//!< disable interrupt		
// ----------------------------------------------------------------------------
#define NVICSetIrqPrio(irq, prio)   	NVIC_BASE_PTR->IP[(irq)/4] = (prio)<<(((irq)%4)*8+6) //!< set interrupt priority
// ----------------------------------------------------------------------------
#define IP_PRIO_1      0														//!< Interrupt priority (highest)	
#define IP_PRIO_2      1														//!< Interrupt priority 
#define IP_PRIO_3      2														//!< Interrupt priority
#define IP_PRIO_4      3														//!< Interrupt priority (lowest)
// ----------------------------------------------------------------------------
#define SPI0_IRQ		10														//!< Interrupt numbers
#define SPI1_IRQ		11														//!< Interrupt numbers
#define UART0_IRQ		12														//!< Interrupt numbers
#define PIT_IRQ 		22														//!< Interrupt numbers
#define PORTA_IRQ       30														//!< Interrupt numbers
//-----------------------------------------------------------------------------
#define EnableInterrupts() asm ("CPSIE  i")										//!< macro to enable interrupts
#define DisableInterrupts() asm ("CPSID  i")									//!< macro to disable interrupts
#define MCUReset()			{SCB_AIRCR = 0x05FA0000| SCB_AIRCR_SYSRESETREQ_MASK;}//!< macro to perform a software reset (VectKey = 0x05FA) 
//-----------------------------------------------------------------------------
#endif // KL25ZUTIL_H_
//-----------------------------------------------------------------------------
// @}
// @}
//-----------------------------------------------------------------------------
