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
#include "mc33771_crc8_test.h"
// ----------------------------------------------------------------------------
/*! \brief Tests crc pattern from mc33771 datasheet.
 * order of bytes is MSB-> LSB
 * returns 0 if all test patterns pass (each failing tests sets a bit)
 */
u32 crc8_test(void)  {
  u32 u32Result;	

  
  u8 u8Pat1[] = { 0x10, 0x08, 0x01, 0x01};
  u8 u8Pat2[] = { 0xA1, 0x01, 0x0A, 0x0A};
  u8 u8Pat3[] = { 0x22, 0x0F, 0xC4, 0x01};
  u8 u8Pat4[] = { 0x53, 0x01, 0x57, 0x72};
  u8 u8Pat5[] = { 0x00, 0x00, 0x00, 0x00};

  u8 u8Pat6[] = { 0xBD, 0x10, 0x09, 0x01, 0x11 };
  u8 u8Pat7[] = { 0x66, 0x50, 0x09, 0x02, 0x20 };
  u8 u8Pat8[] = { 0xFB, 0xA5, 0x09, 0x03, 0x51 };
  u8 u8Pat9[] = { 0xC0, 0x62, 0x09, 0x04, 0xFF };
  u8 u8Pat10[]= { 0xB2, 0x00, 0x00, 0x00, 0x00 };

  u32Result = 0;
  if(lld3377xCrcCalc(&u8Pat1[0], 4) != 0x22)      u32Result |= BIT(1);
  if(lld3377xCrcCalc(&u8Pat2[0], 4) != 0xF6)      u32Result |= BIT(2);
  if(lld3377xCrcCalc(&u8Pat3[0], 4) != 0x6A)      u32Result |= BIT(3);
  if(lld3377xCrcCalc(&u8Pat4[0], 4) != 0x71)      u32Result |= BIT(4);
  if(lld3377xCrcCalc(&u8Pat5[0], 4) != 0xB2)      u32Result |= BIT(5);
   
   
  if(lld3377xCrcCalc(&u8Pat6[0], 5) != 0)      u32Result |= BIT(6);
  if(lld3377xCrcCalc(&u8Pat7[0], 5) != 0)      u32Result |= BIT(7);
  if(lld3377xCrcCalc(&u8Pat8[0], 5) != 0)      u32Result |= BIT(8);
  if(lld3377xCrcCalc(&u8Pat9[0], 5) != 0)      u32Result |= BIT(9);
  if(lld3377xCrcCalc(&u8Pat10[0], 5) != 0)      u32Result |= BIT(10);
   
   
   return u32Result;
   
}
