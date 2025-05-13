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
#include "comm.h"
// ----------------------------------------------------------------------------
#define SYNC       0xAA
// ----------------------------------------------------------------------------
/*! \brief sends data via Uart0 to GUI
 * 
 * In this case the INDEX of the data is the only information sent.
 * There is no data provided.
 * 
 * @param cid	CID (4 bit)
 * @param idx   INDEX (12 bit) of the data
 * 
 * @return \b true  always
 */
bool SendIdxData0(u8 cid, u16 idx)  {

	u8 temp;
	
	temp = (cid<<4) | (idx>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = idx&0xFF;
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	Uart0SendByte(SYNC);
	Uart0SendByte(SYNC);
	return true;
}
// ----------------------------------------------------------------------------
/*! \brief sends 8 bit of data via Uart0 to GUI
 * 
 * @param cid	CID (4 bit)
 * @param idx   INDEX (12 bit) of the data
 * @param data  8 bit of data
 * 
 * @return \b true  always
 */
bool SendIdxData8(u8 cid, u16 idx, u8 data)  {

	u8 temp;
	
	temp = (cid<<4) | (idx>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = idx&0xFF;
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);


	
	Uart0SendByte(data);
	if(data==SYNC)  
		Uart0SendByte(0x01);
	
	Uart0SendByte(SYNC);
	Uart0SendByte(SYNC);
	return true;
}
// ----------------------------------------------------------------------------
/*! \brief sends 16 bit of data via Uart0 to GUI
 * 
 * @param cid	CID (4 bit)
 * @param idx   INDEX (12 bit) of the data
 * @param data  16 bit of data
 * 
 * @return \b true  always
 */
bool SendIdxData16(u8 cid, u16 idx, u16 data)  {

	u8 temp;

	temp = (cid<<4) | (idx>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = idx&0xFF;
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	
	temp = (u8)(data>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = (u8)(data>>0);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	
	Uart0SendByte(SYNC);
	Uart0SendByte(SYNC);
	return true;
}
// ----------------------------------------------------------------------------
/*! \brief sends 32 bit of float data via Uart0 to GUI
 * 
 * @param cid	CID (4 bit)
 * @param idx   INDEX (12 bit) of the data
 * @param data  32 bit of float data
 * 
 * @return \b true  always
 */
bool SendIdxDataFloat(u8 cid, u16 idx, float data)  {
	TYPE_D32 temp;
	
	temp.ieee754 = data;
	
	return SendIdxData32(cid, idx, temp.raw);
}
// ----------------------------------------------------------------------------
/*! \brief sends 32 bit of data via Uart0 to GUI
 * 
 * @param cid	CID (4 bit)
 * @param idx   INDEX (12 bit) of the data
 * @param data  32 bit of data
 * 
 * @return \b true  always
 */
bool SendIdxData32(u8 cid, u16 idx, u32 data)  {

	u8 temp;
	
	temp = (cid<<4) | (idx>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = idx&0xFF;
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	
	temp = (u8)(data>>24);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	temp = (u8)(data>>16);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	temp = (u8)(data>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = (u8)(data>>0);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	
	Uart0SendByte(SYNC);
	Uart0SendByte(SYNC);
	return true;
}
// ----------------------------------------------------------------------------
/*! \brief sends 48 bit of data via Uart0 to GUI
 * 
 * @param cid	CID (4 bit)
 * @param idx   INDEX (12 bit) of the data
 * @param data  48 bit of data
 * 
 * @return \b true  always
 */
bool SendIdxData48(u8 cid, u16 idx, uint64_t data)  {

	u8 temp;
	
	temp = (cid<<4) | (idx>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = idx&0xFF;
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	temp = (u8)(data>>48);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	temp = (u8)(data>>32);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = (u8)(data>>24);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	temp = (u8)(data>>16);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);

	temp = (u8)(data>>8);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	temp = (u8)(data>>0);
	Uart0SendByte(temp);
	if(temp==SYNC)  
		Uart0SendByte(0x01);
	
	
	Uart0SendByte(SYNC);
	Uart0SendByte(SYNC);
	return true;
}

// ----------------------------------------------------------------------------
/*! \brief reads one frame from the uart buffer
 * 
 * Attempts to read a communication frame from uart buffer. \n
 * - checks is a sync \b <0xAA><0xAA> was received
 * - translates escape patterns (e.g. \b <0xAA><0xnn>)
 * - handles variable length data
 * - returns CID, INDEX and DATA  
 * 
 * @param *cid		pointer to cid (return data)
 * @param *commId   pointer to commId (return data)
 * @param *data		pointer to data (return data)
 * 
 * @return \b true   a complete frame is available 
 * @return \b false  no (complete) frame is available
 *  
 * \remarks
 * uses a local buffer to store characters received, but frame not complete yet.          
 */ 
bool ReadIdxData(u8 *cid, u16 *commId, TYPE_VAR_DATA *data)  {
	static u8 syncsCnt=0, n, byte, localBuf[64], bufPos=0;

	
	// -----  receive commands from GUI -----
	while(Uart0ReadByte(&byte))  {
		if(byte==SYNC)  {
			syncsCnt++;
			if(syncsCnt==2)  {													// two syncs -> end of frame
				// decode frame 
				*cid  = 0x0F & (localBuf[0]>>4);
				*commId = 0x0FFF & ((localBuf[0]<<8) | localBuf[1]);

				if(bufPos==2)  {

				}else if(bufPos==3)  {
					data->d8 = localBuf[2];
				}else if(bufPos==4)  {
					data->d16 = localBuf[2]<<8 | localBuf[3];
				}else if(bufPos==6)  {
					data->d32.raw = localBuf[2]<<24 |localBuf[3]<<16 |localBuf[4]<<8 | localBuf[5];
				}else{
					// error
				}
				bufPos = 0;
				syncsCnt = 0;
				return true;
			}
		}else{
			if(syncsCnt==1)  {
				// translate e.g. 0xAA 0xnn to nn times 0xAA
				for(n=0; n<byte; n++)  {
					localBuf[bufPos++] = SYNC;
					if(bufPos>=sizeof(localBuf))  {
						// buffer overrun
						while(1)	DONOTHING();
					}
				}
			}else{
				localBuf[bufPos++] = byte;
				if(bufPos>=sizeof(localBuf))  {
					// buffer overrun
					while(1)	DONOTHING();
				}
			}			
			syncsCnt = 0;
		}
	}
	return false;
}

