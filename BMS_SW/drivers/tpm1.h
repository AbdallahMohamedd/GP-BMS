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
//! \addtogroup  drivers 
// @{
/*! \brief Function used for generating delays us and ms.
 * 
 * Use (add if necessary) the DELAY_xxus macros (due to 1/3us resolution) 
 * 
 */
//! \addtogroup timer
// @{
// ----------------------------------------------------------------------------
#ifndef TPM1_H_
#define TPM1_H_
// ----------------------------------------------------------------------------
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include <MKL25Z4.h>
//#include <Platform/frdmkl25z.h>
//#include "source/COTs/SlaveControlIF/Inc/SlaveIF.h"

// ----------------------------------------------------------------------------
// predefined delays for 48MHz bus, prescaler 16
#define DELAY_1us				(   1*3)										//!< used for readability
#define DELAY_22us				(  22*3)                                        //!< used for readability
#define DELAY_100us				( 100*3)                                        //!< used for readability
#define DELAY_150us				( 150*3)                                        //!< used for readability
#define DELAY_200us				( 200*3)                                        //!< used for readability
#define DELAY_325us				( 325*3)                                        //!< used for readability
#define DELAY_500us				( 500*3)                                        //!< used for readability
#define DELAY_600us				( 600*3)                                        //!< used for readability
#define DELAY_680us				( 680*3)                                        //!< used for readability
#define DELAY_1000us			(1000*3)                                        //!< used for readability
// ----------------------------------------------------------------------------
void DelayInit(void);
void Delay(u16 delay);
void Delayms(u16 msDelay);
// ----------------------------------------------------------------------------
#endif /* TPM1_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------
