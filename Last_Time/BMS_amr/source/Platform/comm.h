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
//! \addtogroup platform 
// @{
// ----------------------------------------------------------------------------
/*! \brief Functions used to send and receive data to / from GUI
 * 
 * <b>Communication protocol:</b> \n
 * Features of the communication protocol:
 * - fast (binary coding)
 * - exchange is based on CID, INDEX, DATA
 * - variable length data, coding is on a multiple of 8 bit -> chars
 * - decoding (e.g. signed vs. unsigned) needs to be done in the receiver based on the knowledge of the data exchanged (INDEX)
 * - frameing with sync \b <0xAA><0xAA> 
 * - a \b <0xAA> in the data is encoded as \b <0xAA><0x01> 
 * 		- the 0x01 is intended as length and can be also >0x01 for multiple <0xAA>s 
 * 		- this is not yet implemented or used
 * 
 * The following table shows the length [bits] for the 3 fields CID, INDEX and DATA, and gives examples
 * 
 * |   Type  | CID | INDEX | DATA | Example (hex)           | Description (hex)               |
 * |:-------:|:---:|:-----:|:----:|-------------------------|---------------------------------|
 * | no data |  4  |   12  |   0  | 10 1F AA AA             | CID:1 Index: 01F                |
 * |  8 bit  |  4  |   12  |   8  | 10 1F 55 AA AA          | CID:1 Index: 01F Data: 55       |
 * |  16 bit |  4  |   12  |  16  | 10 1F AA 01 55 AA AA    | CID:1 Index: 01F Data: AA55     |
 * |  32 bit |  4  |   12  |  32  | 00 01 12 34 56 78 AA AA | CID:0 Index: 001 Data: 12345678 |
 * |  48 bit |  4  |   12  |  48  | ...                     | ...                             |
 * 
 * 
 * \remarks
 * The old \b <\\n> based communication protocol was replaced for the following reasons:
 * - faster (binary coding, vs. hex char coding)
 * - reduced the no. of bytes by -20% ... -30%
 * - loop time Eval GUI 120ms -> 81ms
 * - loop time EMC GUI for one CID 20ms -> 16
 *                                                              
 * between PC -> PackController (KL25Z)                         
 * \code                                                        
 * 'i'nnnndd'\n'                                                
 * 'i'nnnndddd'\n'                                              
 * 'i'nnnndddddddd'\n'                                          
 * 			nnnn: index of data    (hex)                     
 * 			dd: 8 bit data (hex) or                          
 * 			dddd: 16 bit data (hex) or                       
 * 			dddddddd: 32 bit data (hex)                      
 * \endcode        
 * 
 * 
*/
//! \addtogroup comm
// ----------------------------------------------------------------------------
// @{
// ----------------------------------------------------------------------------
#ifndef COMM_H_
#define COMM_H_
// ----------------------------------------------------------------------------
#include "source/drivers/uart0.h"
#include <stdbool.h>
// ----------------------------------------------------------------------------
#define SYNC 0xAA																//!< SYNC is <AA><AA>
// ----------------------------------------------------------------------------
//! \brief union to easily handle 32 bit data as raw or IEEE-754 float value 
typedef union {																	
	uint32_t  raw;																//!< access as u32
	float     ieee754;															//!< access as float (IEEE-754)
}TYPE_D32;
// ----------------------------------------------------------------------------
//! \brief union to easily handle variable length data for communication. 
typedef union {															
	uint64_t d64;																//!< 64 bit long data	
	TYPE_D32 d32;																//!< 32 bit long data
	uint16_t d16;																//!< 16 bit long data
	uint8_t  d8;																//!< 8 bit long data
}TYPE_VAR_DATA;
// ----------------------------------------------------------------------------
bool SendIdxData0(u8 cid, u16 idx);
bool SendIdxData8(u8 cid, u16 idx, u8 data);
bool SendIdxData16(u8 cid, u16 idx, u16 data);
bool SendIdxData32(u8 cid, u16 idx, u32 data);
bool SendIdxDataFloat(u8 cid, u16 idx, float data);
bool SendIdxData48(u8 cid, u16 idx, uint64_t data);
// ----------------------------------------------------------------------------
bool ReadIdxData(u8 *cid, u16 *commId, TYPE_VAR_DATA *data);

#endif /* COMM_H_ */
//---------------------------------------------------------------------
// @}
// @}
//---------------------------------------------------------------------
