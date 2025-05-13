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
//! \addtogroup project
// @{
//! \addtogroup config
//! \brief Initial MC33771/772 Configuration List
// @{
// ----------------------------------------------------------------------------
#ifndef CONFIG_H_
#define CONFIG_H_
// ----------------------------------------------------------------------------
#include "Platform/KL25ZUtil.h"
// ----------------------------------------------------------------------------
/*! \brief structure for MC3377x configuration list entry.
 * 
 * List of register address and register values to be loaded during configuration.
 * 
 * \note
 * The last list entry must indicate the end of the list:
 * \code	{0 , 0 }		// end symbol \endcode
 * 
 */
typedef struct{
	u8 regAdr;																	//!< register address to be written to
	u16 regValue;																//!< register value to write 
}TYPE_BCC_CONF;
// ----------------------------------------------------------------------------
extern const TYPE_BCC_CONF CONF33771SPI[];
extern const TYPE_BCC_CONF CONF33771TPL[];
extern const TYPE_BCC_CONF CONF33772SPI[];
extern const TYPE_BCC_CONF CONF33772TPL[];
// ----------------------------------------------------------------------------
#endif /* CONFIG_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------
