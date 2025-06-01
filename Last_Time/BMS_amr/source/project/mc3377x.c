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
#include "source/COTs/SlaveControlIF/Inc/SlaveIF.h"
// ----------------------------------------------------------------------------
extern const u16 _TAGID_BCC14p2[];
extern const u16 _TAGID_BCC14[];
extern const u16 _TAGID_BCC6[];
// ----------------------------------------------------------------------------
#define CheckCID(v)      	(((v)<1) || ((v)>MAX_CLUSTER))						//!< macro to check if CID is in {1..MAX_CLUSTER} 	
// ----------------------------------------------------------------------------
/*! \brief Initialises the BMS system
 * 
 * Initialises the BMS system with NoOfNodes.
 * 
 * <b>For all nodes</b>
 * - Clear errors (\ref lld3377xClearError)
 * - Send wakeup
 * - Check if unassigned node present
 * 		- Assigns CID (starting with 1, counting up)  
 * 		- Close the TPL bus switch (except for last CID in chain) 
 * 
 * @param NoOfNodes	 1..15 number of nodes in the chain
 * @return \b true   if successful
 * @return \b false  in case of errors
 * 
 * \b Note: works for TPL and SPI (only one node) interface
 * 
 */
bool BMSInit(u8 NoOfNodes)  {	
	u8 cid;
	
	if(NoOfNodes==0)	return false;
	if(NoOfNodes>MAX_CLUSTER)	return false;	
	
	for(cid=1; cid<=NoOfNodes; cid++)  {
		lld3377xClearError();
		lld3377xWakeUp();														// send wakeup in case next device is in idle mode
		if(lld3377xReadRegisters(CIDunassiged, INIT, 1, NULL))  {				// check if somebody is there / read Init register
			lld3377xWriteRegister(CIDunassiged, INIT, (u16)cid, NULL);			// assign CID 			
			if(cid<NoOfNodes)  {												// all nodes except last one
				lld3377xWriteRegister(cid, INIT, INIT_BUS_SW, NULL);			// close switch	(except last node)		
			}else{
				lld3377xWriteRegister(cid, INIT, 0x0000, NULL);					// open switch	(last node)			
			}
		}
	}
	return !lld3377xGetError(NULL);
}
// ----------------------------------------------------------------------------
/*! \brief Performs basic MC3377x configuration.
 * 
 * Loads the configuration list conf into the node cid registers.
 *
 * @param  cid        CID to be configured
 * @param  conf       pointer to the configuration list
 * 
 * @return \b true    if successful
 * @return \b false   if not successful
 * 
 * \note 
 * A list entry with register address equal to 0 terminates the list.
 *  
 */
bool MC3377xConfig(u8 cid, const TYPE_BCC_CONF conf[])  {
	u16 n;
	
	if(CheckCID(cid)) 
		return _lld3377xSetError(ERR_WrongParam);

	n = 0;	
	while(conf[n].regAdr != 0)  {	
		lld3377xWriteRegister(cid, conf[n].regAdr, conf[n].regValue, NULL);	
		n++;
	}	
	return !lld3377xGetError(NULL);
}	
// ----------------------------------------------------------------------------
/*! \brief starts a ADC conversion (set SOC bit in ADC_CFG register),
 * 
 * @param cid         cluster ID 
 * @param tagID       TagID for ADC measurement
 * 
 * @return \b true    if successful
 * @return \b false   if not successful
 * 
 */
bool MC3377xADCStartConversion(u8 cid, u8 tagID)    {
	u16 adcCfg;
	
	if(CheckCID(cid)) 
		return _lld3377xSetError(ERR_WrongParam);
	
	if(tagID>15) 
		return _lld3377xSetError(ERR_WrongParam);
	
	lld3377xReadRegisters(cid, ADC_CFG, 1, &adcCfg);
	adcCfg = adcCfg | ADC_CFG_SOC;												// set start of conversion
	adcCfg = (adcCfg&0x0FFF) | (tagID<<12);										// set TagId
	lld3377xWriteRegister(cid, ADC_CFG, adcCfg, NULL);

	
	if(lld3377xGetError(NULL))  {
		return false;
	}else{
		lld3377xSetTagID(cid, tagID);
		return true;
	}
}
// ----------------------------------------------------------------------------
/*! \brief checks status of End Of Conversion.
 * 
 * Checks the status of the conversion of the cluster CID by reading 
 * the SOC/nEOC bit in ADC_CFG register.
 * 
 * \note 
 * Also returns \b false in case of errors:
 * - cid out of range \ref CheckCID  (sets \ref ERR_WrongParam)
 * - read ADC_CFG register failed 
 * 
 * @param cid       cluster ID 

 * @return \b true    if conversion still ongoing 
 * @return \b false   if Conversion complete, or in case of errors
 */
bool MC3377xADCIsConverting(u8 cid)  {
	u16 adcCfg;
	
	if(CheckCID(cid))  return _lld3377xSetError(ERR_WrongParam);

	if(lld3377xReadRegisters(cid, ADC_CFG, 1, &adcCfg))  {
		return adcCfg & ADC_CFG_SOC;									
	}else{	
		return false;
	}	
}
// ----------------------------------------------------------------------------
/*! \brief reads the measurement registers and return them as raw data in RawMeasResults
 * 
 * @param cid        		cluster (CID) to handle
 * @param tagId      		TagID to be used to check reading against  
 * @param NoCTs      		number of cell termianls to handle
 * @param *RawMeasResults   pointer to RawMeasResults (return data)  
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 * 
 * \remarks 
 * The Measurement data is only valid, if successfully executed (true).
 * There is no check of DataReady (Bit15) implemented!
 */  
bool MC3377xGetRawMeasurements(u8 cid, u8 tagId, u8 NoCTs, TYPE_MEAS_RESULTS_RAW *RawMeasResults)    {
	u16 rdData[0x1B];
	u8 u8Idx;
	u32 u19Current;
	
	// -----  burst read data  -----
	if(lld3377xReadRegisters(cid, MEAS_ISENSE1, 0x3, &rdData[0]))  {			// read register 0x30..0x32
		u19Current = ((rdData[0]&0x7FFF)<<4) | ((rdData[1]&0x000F)<<0);			// -----  MEAS_ISENSE  -----
		// -----  sign extend to s32  -----
		if(u19Current & BIT(18))  {												// if current reading is negative
			RawMeasResults->s32Current = u19Current | 0xFFF80000;   			// sign extension 
		}else{
			RawMeasResults->s32Current = u19Current;   							// nothing to do 		
		}
		RawMeasResults->u16StackVoltage = rdData[2]&0x7FFF;						// -----  MEAS_STACK  -----
	}
	
	// -----  MEAS_CELL[14..1]  -----
	if(lld3377xReadRegisters(cid, MEAS_CELL1-NoCTs+1 , NoCTs, &rdData[0]))  {	
		u8Idx = NoCTs;
		while(u8Idx)  {
			u8Idx--;
			RawMeasResults->u16CellVoltage[u8Idx] = rdData[NoCTs-1-u8Idx]&0x7FFF;	// reverse order 
		}
	}		
	// -----  MEAS_AN[6..0]  -----
	if(lld3377xReadRegisters(cid, MEAS_AN6, 0x7, &rdData[0]))  {	
		u8Idx = 7;
		while(u8Idx)  {
			u8Idx--;
			RawMeasResults->u16ANVoltage[u8Idx] = rdData[7-1-u8Idx]&0x7FFF;		// reverse order 
		}
	}
	
	if(lld3377xReadRegisters(cid, MEAS_IC_TEMP, 0x3, &rdData[0]))  {	
		RawMeasResults->u16ICTemp = rdData[0]&0x7FFF;							// MEAS_IC_TEMP
		RawMeasResults->u16VbgADC1A = rdData[1]&0x7FFF;							// MEAS_VBGxx
		RawMeasResults->u16VbgADC1B = rdData[2]&0x7FFF;
	}
	
	// -----  burst read data  -----
	if(lld3377xReadRegisters(cid, CC_NB_SAMPLES, 3, &rdData[0]))  {				// read register 0x2D..0x2F
		RawMeasResults->u16CCSamples = rdData[0];
		RawMeasResults->s32CCCounter = (s32) ((rdData[1]<<16)|rdData[2]);
	}	

	return !lld3377xGetError(NULL);
}
// ------------------------------------------------------------------
/*! \brief Reads the General Universal ID (GUID)  
 * 
 * Reads the GUID (37 bit) of the cluster cid from Fuse Mirror memory (address 0x17..0x19).
 * Data is returned in pCluster.Guid
 * 
 \b MC33771 RevA:
 |  UID [36:30]   |  UID [29:14]   |  UID [13:0]    | 
 |:--------------:|:--------------:|:--------------:|
 | 0x17 [15:9]    | 0x18 [15:0]    | 0x19 [13:0]    |
 |  (7 bit)       |  (16 bit)      |  (14 bit)      |
 
 
 
\b MC33772 RevA:
 |  UID [36:30]   |  UID [29:14]   |  UID [13:0]    | 
 |:--------------:|:--------------:|:--------------:|
 | 0x13 [15:9]    | 0x14 [15:0]    | 0x15 [13:0]    |
 |  (7 bit)       |  (16 bit)      |  (5 bit)       |
  
 
 
 \b MC33771 RevB:
 |  UID [36:21]   |  UID [20:5]    |  UID [4:0]     | 
 |:--------------:|:--------------:|:--------------:|
 | 0x18 [15:0]    | 0x19 [15:0]    | 0x1A [4:0]     |
 |  (16 bit)      |  (16 bit)      |  (5 bit)       |
 
 
 \b MC33772 RevB:
 |  UID [36:21]   |  UID [20:5]    |  UID [4:0]     | 
 |:--------------:|:--------------:|:--------------:|
 | 0x10 [15:0]    | 0x11 [15:0]    | 0x12 [4:0]     |
 |  (16 bit)      |  (16 bit)      |  (5 bit)       |
 
 * 
 * 
 * 
 * 
 * @param cid		 cluster (CID) to handle	
 * @param pCluster   pointer to cluster (only modifies pCluster.Guid)
 *   
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 * 
 * \remarks
 * The clusters type and silicon revision must be known! (MC3377xGetSiliconRevision() must be called \b first!)
  * 
 * In case of not successful execution Guid is set to = 0x0000_0000
 * 
 */
bool MC3377xGetGUID(u8 cid, LLD_TYPE_CLUSTER *pCluster)  {
	u16 rdData[3];
	u8 u8Adr;

	pCluster->Guid = 0L;

	if(CheckCID(cid))  
		return _lld3377xSetError(ERR_WrongParam);
	if(pCluster->Chip == Chip_Unknown) 

		
		return _lld3377xSetError(ERR_WrongParam);
	
	switch(pCluster->Chip)  {
	case Chip_MC33771A:
	case Chip_MC33771BM:
//		 \b MC33771 RevA:
//		 |  UID [36:30]   |  UID [29:14]   |  UID [13:0]    | 
//		 |:--------------:|:--------------:|:--------------:|
//		 | 0x17 [15:9]    | 0x18 [15:0]    | 0x19 [13:0]    |
//		 |  (7 bit)       |  (16 bit)      |  (14 bit)      |
		 		
		for(u8Adr=0x17; u8Adr<=0x19; u8Adr++)  {
			lld3377xWriteRegister(cid, FUSE_MIRROR_CTRL, (u8Adr<<8), NULL);
			lld3377xReadRegisters(cid, FUSE_MIRROR_DATA, 1, &rdData[u8Adr-0x17]);	
		}
		if(lld3377xGetError(NULL))   {
			pCluster->Guid = 0L;
			return false;
		}else{
			pCluster->Guid = ((uint64_t)(rdData[0]&0xFE00)<<21) | ((rdData[1]&0xFFFF)<<14) | ((rdData[2]&0x3FFF)<<0);
			return true;
		}
		break;

	case Chip_MC33771B:
//		 \b MC33771 RevB:
//		 |  UID [36:21]   |  UID [20:5]    |  UID [4:0]     | 
//		 |:--------------:|:--------------:|:--------------:|
//		 | 0x18 [15:0]    | 0x19 [15:0]    | 0x1A [4:0]     |
//		 |  (16 bit)      |  (16 bit)      |  (5 bit)       |
		for(u8Adr=0x18; u8Adr<=0x1A; u8Adr++)  {
			lld3377xWriteRegister(cid, FUSE_MIRROR_CTRL, (u8Adr<<8), NULL);
			lld3377xReadRegisters(cid, FUSE_MIRROR_DATA, 1, &rdData[u8Adr-0x18]);	
		}
		if(lld3377xGetError(NULL))   {
			pCluster->Guid = 0L;
			return false;
		}else{
			pCluster->Guid = ((uint64_t)(rdData[0]&0xFFFF)<<21) | ((rdData[1]&0xFFFF)<<5) | ((rdData[2]&0x001F)<<0);
			return true;
		}
		break;

	case Chip_MC33772A:
	case Chip_MC33772BM:
//		\b MC33772 RevA:
//		 |  UID [36:30]   |  UID [29:14]   |  UID [13:0]    | 
//		 |:--------------:|:--------------:|:--------------:|
//		 | 0x13 [15:9]    | 0x14 [15:0]    | 0x15 [13:0]    |
//		 |  (7 bit)       |  (16 bit)      |  (14 bit)       |
		
		for(u8Adr=0x13; u8Adr<=0x15; u8Adr++)  {
			lld3377xWriteRegister(cid, FUSE_MIRROR_CTRL, (u8Adr<<8), NULL);
			lld3377xReadRegisters(cid, FUSE_MIRROR_DATA, 1, &rdData[u8Adr-0x13]);	
		}
		if(lld3377xGetError(NULL))   {
			pCluster->Guid = 0L;
			return false;
		}else{
			pCluster->Guid = ((uint64_t)(rdData[0]&0xFE00)<<21) | ((rdData[1]&0xFFFF)<<14) | ((rdData[2]&0x3FFF)<<0);
			return true;
		}
		break;
		
	case Chip_MC33772B:
//		\b MC33772 RevB:
//		 |  UID [36:21]   |  UID [20:5]    |  UID [4:0]     | 
//		 |:--------------:|:--------------:|:--------------:|
//		 | 0x10 [15:0]    | 0x11 [15:0]    | 0x12 [4:0]     |
//		 |  (16 bit)      |  (16 bit)      |  (5 bit)       |
		for(u8Adr=0x10; u8Adr<=0x12; u8Adr++)  {
			lld3377xWriteRegister(cid, FUSE_MIRROR_CTRL, (u8Adr<<8), NULL);
			lld3377xReadRegisters(cid, FUSE_MIRROR_DATA, 1, &rdData[u8Adr-0x10]);	
		}
		if(lld3377xGetError(NULL))   {
			pCluster->Guid = 0L;
			return false;
		}else{
			pCluster->Guid = ((uint64_t)(rdData[0]&0xFFFF)<<21) | ((rdData[1]&0xFFFF)<<5) | ((rdData[2]&0x001F)<<0);
			return true;
		}
		break;
		
		
	case Chip_Unknown:
	default:
		return _lld3377xSetError(ERR_WrongParam);
		break;
	}
	
	
	for(u8Adr=0x17; u8Adr<=0x19; u8Adr++)  {
		lld3377xWriteRegister(cid, FUSE_MIRROR_CTRL, (u8Adr<<8), NULL);
		lld3377xReadRegisters(cid, FUSE_MIRROR_DATA, 1, &rdData[u8Adr-0x17]);	
	}
	if(lld3377xGetError(NULL))   {
		pCluster->Guid = 0L;
		return false;
	}else{
		pCluster->Guid = ((uint64_t)(rdData[0]&0xFE00)<<21) | ((rdData[1]&0xFFFF)<<14) | ((rdData[2]&0x3FFF)<<0);
		return true;
	}
}
// ----------------------------------------------------------------------------
/*! \brief Reads the silicon revision of the device
 * 
 * Reads the \ref SILICON_REV register and retrieves full and mask revisions of the device.
 * 
 * @param cid         cluster (CID) to handle
 * @param pCluster    fills the pCluster FRev and MRev 
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 * 
 * \remarks
 * In case of not successful execution Frev and Mrev will be set to 0
*/
bool MC3377xGetSiliconRevision(u8 cid, LLD_TYPE_CLUSTER *pCluster)    {
	u16 rdData;
	
	pCluster->FRev = 0;
	pCluster->MRev = 0;

	if(cid>15) 
		return _lld3377xSetError(ERR_WrongParam);

	if(lld3377xReadRegisters(cid, SILICON_REV, 1, &rdData))  {
		pCluster->FRev = (u8) (rdData>>3)&0x07;
		pCluster->MRev = (u8) (rdData>>0)&0x07;
	}
	return !lld3377xGetError(NULL);
}
// ----------------------------------------------------------------------------
/*! \brief Reads the silicon type of the cid.
 * 
 * Evaluates the silicon type of the cid. The routine is using a different
 * algorithm depending if the device is in reset state (registers are in reset state) 
 * or if it has to be assumed the the registers might already be modified.
 * 
 * <b>Register in Reset state (cid = 0)</b> \n 
 * Reads \ref OV_UV_EN and \ref SYS_CFG2 registers.
 * - if \ref OV_UV_EN = 0x3FFF => MC33771 (otherwise MC33772)
 * - if \ref SYS_CFG2 bit14 = 1 => Rev A (otherwise Rev B)
 * 
 * <b>Register state unknown</b> \n
 * store \ref OV_UV_EN, attempt to write 0x3FFF, read and compare with 0x3FFF, restore \ref OV_UV_EN  
 * - if \ref OV_UV_EN = 0x3FFF => MC33771 (otherwise MC33772)
 * 
 * store \ref INIT, attempt to set bit 5, read and compare with 0x3FFF, restore \ref INIT 
 * - if \ref INIT bit5 = 0 => Rev A (otherwise Rev B)
 * 
 * @param cid         cluster (CID) to handle
 * @param pCluster    fills the pCluster.Chip, pCluster.tagIDlist and pCluster.NoCTS 
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 * 
 * \remarks 
 * The clusters silicon revision must be known! (MC3377xGetSiliconRevision() must be called \b first!)
 * 
 * Also translates engineering versions to normal versions:\n
 * | device  | engineering version | translated version |
 * |:-------:|:-------------------:|:------------------:|
 * | MC33771 |       7.7           |        1.0         |
 * | MC33771 |       7.2, 7.3      |        2.0         |
 * | MC33771 |       6.2, 6.6      |        3.1         |
 * | MC33771 |       6.1           |        3.2         |
 * 
 */
bool MC3377xGetSiliconType(u8 cid, LLD_TYPE_CLUSTER *pCluster)    {
	LLD_TYPE_RETURN res;
	u16 rdData, bakData;
	bool bcc14;
	bool revA;
	
	// default return values in case of errors	
	pCluster->Chip = Chip_Unknown;
	pCluster->NoCTs= 14;
	pCluster->pTagIdList = NULL;

	if(cid==0)  {
		// works with CID-00 => reset state
		lld3377xReadRegisters(cid, OV_UV_EN, 1, &rdData);				
		bcc14 = (rdData == 0x3FFF);												// 14cells 0x3FFF  ; 6cells  0x003F
		lld3377xReadRegisters(cid, SYS_CFG2, 1, &rdData);				
		revA = (rdData&0x4000);													// bit14=1 => Rev A
	}else{	
		if(CheckCID(cid)) 
			return _lld3377xSetError(ERR_WrongParam);
		// assigned CID => need to test
		if(lld3377xReadRegisters(cid, OV_UV_EN, 1, &bakData))  {				// to keep state
			lld3377xWriteRegister(cid, OV_UV_EN, 0x3FFF, NULL);			
			lld3377xReadRegisters(cid, OV_UV_EN, 1, &rdData);					// now check
			lld3377xWriteRegister(cid, OV_UV_EN, bakData, NULL);				// restore
			bcc14 = (rdData == 0x3FFF);
		}
		if(lld3377xReadRegisters(cid, INIT, 1, &bakData))  {					// to keep state
			lld3377xWriteRegister(cid, INIT, bakData|(1UL<<5), NULL);			// attempt to set Bit5			
			lld3377xReadRegisters(cid, INIT, 1, &rdData);						// now check
			lld3377xWriteRegister(cid, INIT, bakData, NULL);					// restore
			revA = (rdData&(1UL<<5))==0;										// bit5=0 => Rev A
		}
	}

	pCluster->pTagIdList = NULL;
		
	if(!lld3377xGetError(&res))  {
		if(bcc14)	{	
			pCluster->NoCTs= 14;
			pCluster->pTagIdList = _TAGID_BCC14;
			if(revA) {
				pCluster->Chip = Chip_MC33771A;
				// translate engineering chip revisions
				if((pCluster->FRev==7)&&(pCluster->MRev==7))  {
					pCluster->FRev = 1; pCluster->MRev = 0;
				}
				if((pCluster->FRev==7)&&(pCluster->MRev==2))  {
					pCluster->FRev = 2; pCluster->MRev = 0;
				}
				if((pCluster->FRev==7)&&(pCluster->MRev==3))  {
					pCluster->FRev = 2; pCluster->MRev = 0;
				}
				if((pCluster->FRev==6)&&(pCluster->MRev==2))  {
					pCluster->FRev = 3; pCluster->MRev = 1;
				}
				if((pCluster->FRev==6)&&(pCluster->MRev==6))  {
					pCluster->FRev = 3; pCluster->MRev = 1;
				}
				if((pCluster->FRev==6)&&(pCluster->MRev==1))  {
					pCluster->FRev = 3; pCluster->MRev = 2;
				}

				if(pCluster->FRev==2)  {
					pCluster->pTagIdList = _TAGID_BCC14p2;
				}
			}else{
				pCluster->Chip = Chip_MC33771B;
			}
			
		}else{
			pCluster->NoCTs= 6;
			pCluster->pTagIdList = NULL;
			pCluster->pTagIdList = _TAGID_BCC6;
			if(revA)  {
				pCluster->Chip = Chip_MC33772A;
				// workaround ......
				// translate engineering chip revisions
				if((pCluster->FRev==7)&&(pCluster->MRev==0))  {
//					pCluster->u8FRev = 2; pCluster->u8MRev = 0;					// V2.99 workaround
					pCluster->Chip = Chip_MC33772BM;
				}
			}else{
		  		pCluster->Chip = Chip_MC33772B;
			}
		}
	}
	return !lld3377xGetError(NULL);
}
// ----------------------------------------------------------------------------
/*! \brief reads the status registers and return them in Status
 * 
 * @param cid        cluster (CID) to handle
 * @param *Status    pointer to status (return data)  
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 * 
 * \remarks 
 * The Status data is only value if successfully executed (true).
 */
bool MC3377xGetStatus(u8 cid, TYPE_STATUS *Status)    {
	u16 rdData[13];

	// burst read data
	if(lld3377xReadRegisters(cid, CELL_OV_FLT, 2, &rdData[0]))  {	
		Status->u16CellOV = rdData[0];
		Status->u16CellUV = rdData[1];
	}
	if(lld3377xReadRegisters(cid, CB_OPEN_FLT, 13, &rdData[0]))  {	
		Status->u16CBOpen     = rdData[0];
		Status->u16CBShort    = rdData[1];
		Status->u16CBStatus   = rdData[2];
		
		Status->u16GPIOStatus = rdData[5];
		Status->u16ANOtUt     = rdData[6];
		Status->u16GPIOOpen   = rdData[7];
		Status->u16IStatus    = rdData[8];
		Status->u16Comm   	  = rdData[9];
		Status->u16Fault1 	  = rdData[10];
		Status->u16Fault2     = rdData[11];
		Status->u16Fault3     = rdData[12];
	}
	if(lld3377xReadRegisters(cid, MEAS_ISENSE2, 1, &rdData[0]))  {
		Status->u16MeasIsense2 = rdData[0];
	}
	
	return !lld3377xGetError(NULL);
}
// ----------------------------------------------------------------------------
/*! \brief reads the threshold registers and return them in Threshold
 * 
 * @param cid         cluster (CID) to handle
 * @param NoCTs       number of cell terminals to handle 
 * @param *Threshold  pointer to thresholds (return data)  
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 * 
 * \remarks 
 * The Threshold data is only value if successfully executed (true).
 */
bool MC3377xGetThresholds(u8 cid, u8 NoCTs, TYPE_THRESHOLDS *Threshold)    {
	u16 rdData[29];
	u8 u8Idx;

	// -----  TH_ALL_OV_UV  -----
	if(lld3377xReadRegisters(cid, TH_ALL_CT, 1, &rdData[0]))  {
		Threshold->u8ThAllOv = (u8) (rdData[0]>>8);
		Threshold->u8ThAllUv = (u8) (rdData[0]>>0);
	}
	// -----  TH_CT[14..1]_OV_UV  -----
	if(lld3377xReadRegisters(cid, TH_CT1-NoCTs+1, NoCTs, &rdData[0]))  {
		u8Idx = NoCTs;
		while(u8Idx)  {
			u8Idx--;
			Threshold->u8ThCTxOv[u8Idx] = (u8) (rdData[NoCTs-1-u8Idx]>>8);	// reverse order 
			Threshold->u8ThCTxUv[u8Idx] = (u8) (rdData[NoCTs-1-u8Idx]>>0);	// reverse order 
		}
	}
	// -----  TH_ANxOT[6..0] -----
	if(lld3377xReadRegisters(cid, TH_AN6_OT, 7, &rdData[0]))  {
		u8Idx = 7;
		while(u8Idx)  {
			u8Idx--;
			Threshold->u10ThANxOT[u8Idx] = rdData[7-1-u8Idx];	// reverse order 
		}
	}
	// -----  TH_ANxUT[6..0]-----
	if(lld3377xReadRegisters(cid, TH_AN6_UT, 7, &rdData[0]))  {
		u8Idx = 7;
		while(u8Idx)  {
			u8Idx--;
			Threshold->u10ThANxUT[u8Idx] = rdData[7-1-u8Idx];	// reverse order 
		}
	}	
	// -----  TH_ISENSE_OC, TH_COULOMB)CNT -----
	if(lld3377xReadRegisters(cid, TH_ISENSE_OC, 3, &rdData[0]))  {
		Threshold->u12ThIsenseOC   = rdData[0];	
		Threshold->u32ThCoulombCnt = ((u32)rdData[1]<<16) | ((u32)rdData[2]<<0);	 
	}
	return !lld3377xGetError(NULL);
}
// ----------------------------------------------------------------------------
/*! \brief reads the configuration registers and return them in Config
 * 
 * @param cid         cluster (CID) to handle
 * @param NoCTs       number of cell terminals to handle 
 * @param *Config     pointer to Config (return data)  
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 * 
 * \remarks 
 * The Configuration data is only value if successfully executed (true).
 */
bool MC3377xGetConfig(u8 cid, u8 NoCTs, TYPE_CONFIG *Config)    {
	u16 rdData[0x1A];
	u8 u8Idx;

	// burst read data
	if(lld3377xReadRegisters(cid, INIT, 0xF, &rdData[0]))  {
		Config->u16Init = rdData[0];
		Config->u16SysCfgGlobal = rdData[1];
		Config->u16SysCfg1 = rdData[2];
		Config->u16SysCfg2 = rdData[3];
		Config->u16SysDiag = rdData[4];
		Config->u16AdcCfg = rdData[5];
		Config->u16Adc2Comp = rdData[6];
		Config->u16OvUvEn = rdData[7];
	}
	// burst read data
	if(lld3377xReadRegisters(cid, GPIO_CFG1, 3, &rdData[0]))  {	
		Config->u16GPIOCfg1 = rdData[0];
		Config->u16GPIOCfg2 = rdData[1];
		Config->u16GPIOSts  = rdData[2];
	}	
	// burst read data
	if(lld3377xReadRegisters(cid, FAULT_MASK1, 6, &rdData[0]))  {	
		Config->u16FaultMask1 = rdData[0];
		Config->u16FaultMask2 = rdData[1];
		Config->u16FaultMask3 = rdData[2];
		Config->u16WakeupMask1 = rdData[3];
		Config->u16WakeupMask2 = rdData[4];
		Config->u16WakeupMask3 = rdData[5];
	}
	if(lld3377xReadRegisters(cid, CB1_CFG, NoCTs, &rdData[0]))  {
		for(u8Idx=0; u8Idx<NoCTs; u8Idx++)  {
			Config->u16CBCfg[u8Idx] = rdData[u8Idx]; 
		}
	}
	return !lld3377xGetError(NULL);
}
// ----------------------------------------------------------------------------
/*! \brief Transitions to sleep mode.
 * 
 * Implementation is interface dependent.
 * 
 * <b>SPI interface</b> \n
 * - Write an 1 to Go2Sleep in SYS_CFG_GLOBAL register (CID = CIDassigned)
 * 
 * <b>TPL interface</b> \n
 * - Global Write an 1 to Go2Sleep in SYS_CFG_GLOBAL register (CID = GLOBAL_CID)
 * - Disables TPL
 * 
 * @param interface  Interface used (SPI or TPL)
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 *  
 */
bool MC3377xSleepMode(TYPE_INTERFACE interface)  {

	if(interface==IntSPI) {												
		return lld3377xWriteRegister(CIDassiged, SYS_CFG_GLOBAL, SYS_CFG_GLOBAL_GO2SLEEP, NULL);				// sleep command
	}else if(interface==IntTPL) {												
		if(lld3377xWriteGlobalRegister(SYS_CFG_GLOBAL, SYS_CFG_GLOBAL_GO2SLEEP))  {	// sleep command (must be global write for TPL)
			lld3377xTPLDisable();									// disable TPL
			return true;
		}
		return !lld3377xGetError(NULL);
	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief Transitions to normal mode. 
 * 
 * Implementation is interface dependent.
 * 
 * <b>SPI interface</b> \n
 * - Calls \ref lld3377xWakeUp
 * 
 * <b>TPL interface</b> \n
 * - Calls \ref lld3377xTPLEnable
 * - Calls \ref lld3377xWakeUp
 * 
 * @param interface  Interface used (SPI or TPL)
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed 
 *  
 * \remarks
 * Does not handle the WAIT_AFTER_WAKEUP time! \n
 * Sets \ref ERR_WrongInterface in case interface is unknown.
 */
bool MC3377xNormalMode(TYPE_INTERFACE interface)  {

	if(interface==IntSPI) {												
		lld3377xWakeUp();																							
//		Delayms(WAIT_AFTER_WAKEUP);						  
		return true;
	}else if(interface==IntTPL) {												
		lld3377xTPLEnable();
		lld3377xWakeUp();
//		Delayms(WAIT_AFTER_WAKEUP);						  
		return true;
	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief Checks if a MC3377x has woken up. 
 * 
 * Implementation is interface dependent.
 * 
 * <b>SPI interface</b> \n
 * - Wake-up has occurred if FAULT pin is high (\ref FaultPinStatus)
 * 
 * <b>TPL interface</b> \n
 * - Wake-up has occurred if INTB pin is low (\ref IntbPinStatus)
 * 		- waits for 150us
 * 		- enables TPL (\ref lld3377xTPLEnable)
 * 		- send wakeup pattern (\ref lld3377xWakeUp)
 * 
 * @param interface  Interface used (SPI or TPL)
 * 
 * @return \b true   if wakeup was detected
 * @return \b false  if no wakeup was detected or in case of errors 
 *  
 * \remarks
 * Sets \ref ERR_WrongInterface in case interface is unknown.
 */
bool MC3377xCheck4Wakeup(TYPE_INTERFACE interface)  {

	if(interface==IntSPI) {												
		if(FaultPinStatus())  {
	//		lld3377xWakeUp();													// wakeup BCC e.g. required if FAULT is enabled but WAKEUP is not
			return true;
		}
		return false;
	}else if(interface==IntTPL) {												
		if(IntbPinStatus()==0)  {												//check wakeup = INTB pin low  
			Delay(DELAY_150us);
			lld3377xTPLEnable();
			lld3377xWakeUp();
			return true;
		}
		return false;
	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief reads the FUSE Mirror data.
 * 
 * @param cid		CID to read from	
 * @param *fusedata  pointer to fuse data
 * 
 * @return \b true   if successful executed
 * @return \b false  if not successful executed
 *  
 */
bool MC3377xReadFuseMirror(u8 cid, TYPE_FUSE_DATA *fusedata)  {
	u16 rdData;
	u8 u8Adr;
	
#define FUSE_MIRROR_CTRL_FSTM           BIT(4)	

	for(u8Adr=0; u8Adr<32; u8Adr++)  {
		lld3377xWriteRegister(cid, FUSE_MIRROR_CTRL, (u8Adr<<8), &rdData);
		lld3377xReadRegisters(cid, FUSE_MIRROR_DATA, 1, &(fusedata->u16Data[u8Adr]));	
	}
	return !lld3377xGetError(NULL);
}


