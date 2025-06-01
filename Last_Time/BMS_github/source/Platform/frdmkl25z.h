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
//! \addtogroup platform 
// @{
//! \brief Macros and functions for the FRDM-KL25Z Board 

//! \addtogroup frdmkl25z
// @{
// ----------------------------------------------------------------------------
#ifndef FRDMKL25Z_H_
#define FRDMKL25Z_H_
// ----------------------------------------------------------------------------
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include <SPI.h>
#include "MKL25Z4.h"




// ----------------------------------------------------------------------------
// status pins 
#define SETSTAT0(v)		if(v) GPIOB_PSOR = BIT(0); else GPIOB_PCOR = BIT(0)
#define SETSTAT1(v)		if(v) GPIOB_PSOR = BIT(1); else GPIOB_PCOR = BIT(1)
#define SETSTAT1Tog()	    ( GPIOB_PTOR = BIT(1))
// ----------------------------------------------------------------------------
void InitBoardClock(void);
//void InitBoardLED(void);-->board.c
void InitInterface(TYPE_INTERFACE interface, TYPE_EVB evb);
void DeInitInterface(void);
u8 FaultPinStatus(void);
//void TplEnable(u8 bEnable); --> to slaveif.h
//u8 IntbPinStatus(void);  --> to slaveif.h
//void SPICSB(u8 u8Level); --> to slaveif.h
void initFIMode(u8 u8enabledDisabled);	


// ----------------------------------------------------------------------------
#endif /* FRDMKL25Z_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------
