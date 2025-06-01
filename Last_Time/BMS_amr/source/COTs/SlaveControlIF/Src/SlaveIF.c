/*
 * Graduation Project: Battery Management System
 * Engineer: Abdullah Mohamed
 * Component: Slave_Control_IF driver
 * File: SlaveIF.c
 * Description: This file contains the implementation of the Slave Interface driver
 *              for communication with the MC33771A slave via the MC33664 transceiver
 *              using SPI on the KL25Z microcontroller.
 */

#include <COTS/SlaveControlIF/Inc/SlaveIF.h>
#include <COTS/SlaveControlIF/Inc/SlaveIF_Cfg.h>


// ----------------------------------------------------------------------------
volatile const u8 RCVALUELIST[] = {	0x0, 0x1, 0x3, 0x2 };						//!< RC values (in case it should not simply count up)!!!
// ----------------------------------------------------------------------------
const u16 _TAGID_BCC14p2[8]= {0x0000,0x0000,0x0000,0xFFFF,0x07FF,0x0000,0x0000,0x0000}; //!< TagId Register List BCC14 cut 2
const u16 _TAGID_BCC14[8]  = {0x0020,0x0000,0xE070,0xFFFF,0x07FF,0x0000,0x0000,0x0000}; //!< TagId Register List BCC14 
const u16 _TAGID_BCC6[8]   = {0x0020,0x0000,0xE070,0xF807,0x07FF,0x0000,0x0000,0x0000}; //!< TagId Register List BCC6 
// ----------------------------------------------------------------------------
// local global var
u8 gu8RCIdx = 0;																//!< index to access RCVALUES[]
u8 gTagID[15];
// ---------------frdm-----------------------------------------------------
TYPE_INTERFACE _interface_frdm;														//!< local copy of interface type
TYPE_EVB _evb_;	//!< local storage of TagID for each CID
// ----------lld------------------------------------------------------------------
static TYPE_INTERFACE *_interface_;												//!< local structure storing the interface (SPI or TPL)
LLD_TYPE_CLUSTER *_cluster_;													//!< local structure storing the cluster information(e.g. enabledTgID, TagIDLIST)
//!< local copy of evb type
// ----------------------------------------------------------------------------
// local routines
bool bRegIsTagID(u8 regAdr, const u16 *tagIDList);
// ----------------------------------------------------------------------------
/*! \brief Routine is retrieving if the register with address regAdr is using
 *  the TagID format (or the RC format).
 * 
 * @param regAdr	       Register Address
 * @param pTagIDList       TagId Register List 
 * 
 * @return \b true   	if regAdr is using the TagID format.
 * @return \b false 	if regAdr is using the RC format.
 */
bool bRegIsTagID(u8 regAdr, const u16 *pTagIDList)  {

	regAdr &= 0x7F;																// Responses Bit (Master Slave bit) 
	return  0 != (pTagIDList[(regAdr>>4)&0x07] & BIT(regAdr&0x0F));
}
// ----------------------------------------------------------------------------
//! \brief crc table for polynom 0x2F.  
static const u8 crc_table[256] = {
		0x00, 0x2f, 0x5e, 0x71, 0xbc, 0x93, 0xe2, 0xcd, 0x57, 0x78, 0x09, 0x26, 0xeb, 0xc4, 0xb5, 0x9a,
		0xae, 0x81, 0xf0, 0xdf, 0x12, 0x3d, 0x4c, 0x63, 0xf9, 0xd6, 0xa7, 0x88, 0x45, 0x6a, 0x1b, 0x34,
		0x73, 0x5c, 0x2d, 0x02, 0xcf, 0xe0, 0x91, 0xbe, 0x24, 0x0b, 0x7a, 0x55, 0x98, 0xb7, 0xc6, 0xe9,
		0xdd, 0xf2, 0x83, 0xac, 0x61, 0x4e, 0x3f, 0x10, 0x8a, 0xa5, 0xd4, 0xfb, 0x36, 0x19, 0x68, 0x47,
		0xe6, 0xc9, 0xb8, 0x97, 0x5a, 0x75, 0x04, 0x2b, 0xb1, 0x9e, 0xef, 0xc0, 0x0d, 0x22, 0x53, 0x7c,
		0x48, 0x67, 0x16, 0x39, 0xf4, 0xdb, 0xaa, 0x85, 0x1f, 0x30, 0x41, 0x6e, 0xa3, 0x8c, 0xfd, 0xd2,
		0x95, 0xba, 0xcb, 0xe4, 0x29, 0x06, 0x77, 0x58, 0xc2, 0xed, 0x9c, 0xb3, 0x7e, 0x51, 0x20, 0x0f,
		0x3b, 0x14, 0x65, 0x4a, 0x87, 0xa8, 0xd9, 0xf6, 0x6c, 0x43, 0x32, 0x1d, 0xd0, 0xff, 0x8e, 0xa1,
		0xe3, 0xcc, 0xbd, 0x92, 0x5f, 0x70, 0x01, 0x2e, 0xb4, 0x9b, 0xea, 0xc5, 0x08, 0x27, 0x56, 0x79,
		0x4d, 0x62, 0x13, 0x3c, 0xf1, 0xde, 0xaf, 0x80, 0x1a, 0x35, 0x44, 0x6b, 0xa6, 0x89, 0xf8, 0xd7,
		0x90, 0xbf, 0xce, 0xe1, 0x2c, 0x03, 0x72, 0x5d, 0xc7, 0xe8, 0x99, 0xb6, 0x7b, 0x54, 0x25, 0x0a,
		0x3e, 0x11, 0x60, 0x4f, 0x82, 0xad, 0xdc, 0xf3, 0x69, 0x46, 0x37, 0x18, 0xd5, 0xfa, 0x8b, 0xa4,
		0x05, 0x2a, 0x5b, 0x74, 0xb9, 0x96, 0xe7, 0xc8, 0x52, 0x7d, 0x0c, 0x23, 0xee, 0xc1, 0xb0, 0x9f,
		0xab, 0x84, 0xf5, 0xda, 0x17, 0x38, 0x49, 0x66, 0xfc, 0xd3, 0xa2, 0x8d, 0x40, 0x6f, 0x1e, 0x31,
		0x76, 0x59, 0x28, 0x07, 0xca, 0xe5, 0x94, 0xbb, 0x21, 0x0e, 0x7f, 0x50, 0x9d, 0xb2, 0xc3, 0xec,
		0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a, 0x15, 0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42
};
// ----------------------------------------------------------------------------
static LLD_TYPE_RETURN _errLast;												//!< storing of last error (Error handling)
// ----------------------------------------------------------------------------
/*! \brief Calculates and tests the CRC8 (MSB first).
 * 
 * Calculates and returns the 8-bit CRC based on the data provided. The calculation
 * is done on fast algorithm using a (256 byte) lookup table. 
 * 
 * @param data	pointer to data (byte-array)	
 * @param len  	number of data bytes to handle
 * 
 * @return \b crc  calculated crc8
 * 
 * 
 * \remarks
 * For this use-case the number of bytes will be 4 to calculate the CRC to be appended
 * to the frame to be sent, or will be 5 (last byte contains the CRC of a received frame).
 * In the later case the calculated return value will be:
 * - 0   if the CRC was correct
 * - <>0 if the CRC was incorrect  
 *    
 */
u8 lld3377xCrcCalc(u8 *data, u16 len)  {

	u8 tbl_idx;
	u8 crc;

	crc = 0x42;																	// seed
	while(len) {
		len--;
		tbl_idx = (crc ^ *(data+len));
		crc = crc_table[tbl_idx];
	}
	return crc;
}


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


// ----------------------------------------------------------------------------
/*! \brief packs a 40bit message frame and returns it in a 5 byte buffer.
 * 
 * Packs the data, address, CID and CRC into the 5 byte buffer.
 * 
 * The buffer has to be sent in the order Byte4..0 with the MSB bit of each byte first.  
 * The CRC field is calculated an added as Byte 0.
 * The bit23 has to be 0 for transfers initiated by the pack-controller.
 * 
 * The graph below shows the packing into the single bytes of the buffer:            
 * \code                                                                             
 * ------------------------------------------------------------------------------    
 * |  Byte 4  |  Byte 3  |  Byte 2            |       Byte 1         |  Byte 0  |    
 * |----------|----------|--------------------|----------------------|----------|    
 * | bit39:31 | bit30:24 | bit23   | bit22:16 | bit15:12   | bit11:8 | bit7:0   |    
 * |     memory data     | response| Register | cluster id | command |          |    
 * |        data         | bResp   |   Addr   |   CID      | cmd     |  CRC     |    
 * ------------------------------------------------------------------------------    
 * \endcode                                                                          
 *                                                                                   
 *  @param pu8Buf     pointer to 5 byte (40bit) buffer (returned data)                          
 *  @param data    16 bit data field to be packed                                 
 *  @param Addr     7 bit address field to be packed                               
 *  @param CID      4 bit Cluster ID to be packed                                  
 *  @param Cmd      4 bit Command to be packed                                     
 * 
 * 
 */
void lld3377xPackFrame(u8 *pu8Buf, u16 data, u8 Addr, u8 CID, u8 cmd)  {

	*(pu8Buf+4) = (u8) (data>>8);
	*(pu8Buf+3) = (u8) (data>>0);
	*(pu8Buf+2) = (u8) (Addr&0x7F);
	*(pu8Buf+1) = (u8) ((CID&0xF)<<4) | (cmd&0xF);
	*(pu8Buf+0) = lld3377xCrcCalc(pu8Buf+1, 4);
}
// ----------------------------------------------------------------------------
/*! \brief Generates and returns a new RC Rolling Counter Value
 * 
 * @return new RC value (based on RCVALUELIST) 
 */
u8 lld3377xNewRCValue(void)  {

	gu8RCIdx++;
	gu8RCIdx = gu8RCIdx % sizeof(RCVALUELIST);									//get next RC value from list
	return RCVALUELIST[gu8RCIdx];	
}
// ----------------------------------------------------------------------------
/*! \brief Returns the number of cell terminals.
 * 
 * Returns the number of cell terminals for the chip type. E.g. returns 14 for MC33771.
 * 
 * \remarks
 * Returns the number of available terminals (not the number of used terminals)! 
 * 
 * 
 * @param chip  see TYPE_CHIP
 * @return <b>number of CTs </b> (or 0 in case of errors)
 */
u8 lld3377xNoOfCTs(TYPE_CHIP chip)  {

	if(chip==Chip_MC33771A)  return 14;
	if(chip==Chip_MC33771BM) return 14;
	if(chip==Chip_MC33771B)  return 14;

	if(chip==Chip_MC33772A)  return 6;
	if(chip==Chip_MC33772BM) return 6;
	if(chip==Chip_MC33772B)  return 6;

	return 0;	
}
// ----------------------------------------------------------------------------
/*! \brief Initialise the low level driver
 * 
 * Initialises the driver:
 * - clears global error (errLast)
 * - stores interface information in _interface_ variable (used in other functions)  
 * 
 * @return \b true  always
 */
bool lld3377xInitDriver(TYPE_INTERFACE *interface)  {

	_errLast = RETURN_OK;
	_interface_ = interface;

	return true;
}
// ----------------------------------------------------------------------------
/*! \brief Initialise the cluster structure
 * 
 * Initialises the cluster information: 
 * - stores the provided cluster data in _cluster_ variable (used in other functions)
 * 
 * @return \b true  always
 */
bool lld3377xInitCluster(LLD_TYPE_CLUSTER *cluster)  {

	_cluster_ = cluster;
	return true;
}
// ----------------------------------------------------------------------------
/*! \brief Sets the TagID for CID to the new value. 
 * 
 * Sets the used TagID to a new value.
 * 
 * \remarks 
 * The following parameters will set a \ref ERR_WrongParam error:
 * - NewTagId is > 15
 * - CID is outside range of 1..14 
 * 
 * \note
 * Each CID has its own TagID.
 * 
 * @return \b true  if successful
 * @return \b false in case of errors 
 * 
 * \sa _lld3377xSetError
 * 
 */
bool lld3377xSetTagID(u8 CID, u8 NewTagId)  {

	if(NewTagId>15)
		return _lld3377xSetError(ERR_WrongParam);
	if(CID>14)
		return _lld3377xSetError(ERR_WrongParam);
	if(CID==0)
		return _lld3377xSetError(ERR_WrongParam);

	gTagID[CID] = NewTagId;
	return true;
}
// ----------------------------------------------------------------------------
/*! \brief sets the internal errors information..
 * 
 * Allows to simulate errors / set an errorCode 
 * 
 * @param errorCode  errorCode to be set
 * 
 * @return \b false  always\n 
 * (for easier handling e.g. \code return _lld3377xSetError(ERR_WrongParam); \endcode)
 * 
 * \remarks
 * An attempt to set the errorCode \ref RETURN_OK will generate (set) \ref ERR_WrongParam.  
 *
 * \sa lld3377xGetError, lld3377xClearError 
 */
bool _lld3377xSetError(LLD_TYPE_RETURN errorCode)  {

	if(errorCode==RETURN_OK)  {
		errorCode = ERR_WrongParam;
	}
	_errLast = errorCode;
	return false;
}
// ----------------------------------------------------------------------------
/*! \brief gets the internal error information.
 * 
 * Gets the error information and returns it.
 *       
 * @param res      pointer to errorCode\n 
 *                 pass \c NULL to ignore return value 
 *                 
 * @return  \b true   if error was detected \n
 *          \b false  if no error was detected \n
 *                    (since last \ref lld337xClearError call)
 *                    
 * \sa lld3377xClearError, _lld3377xSetError         
 */
bool lld3377xGetError(LLD_TYPE_RETURN *errorCode)  {

	if(NULL!=errorCode) 
		*errorCode = _errLast;

	return (_errLast!=RETURN_OK);
}
// ----------------------------------------------------------------------------
/*! \brief clears the internal error information.
 * 
 * Sets the internal error status to \ref RETURN_OK 
 * 
 * \sa lld3377xGetError, _lld3377xSetError         
 */
void lld3377xClearError(void)  {

	_errLast = RETURN_OK;
}
// ----------------------------------------------------------------------------
/*! \brief Wakes-up the MC3377x
 * 
 * Issues a wake-up pattern to wakeup connected MC3377x device(s) and waits for 
 * the WAIT_AFTER_WAKEUP before returning.
 * 
 * For <b>SPI interface</b> the uC is generating a wake-up sequence by toggling of CSB signal once
 * with a given timing. 
 * 
 * 
 * For <b>TPL interface</> the uC is generating a wake-up sequence by toggling of CSB signal twice
 * with a given timing. 
 * 
 * \image html tplwakeuppattern.png
 * 
 * \remarks  
 * Depending on the state of the connected MC3377x device(s) only the device(s)
 * with bus switch closed will see the wakeup pattern on the TPL bus. 

 * \note
 * The bus switch is open in low power mode! 
 * 
 * @return \b false  if error present (\ref lld3377xGetError) or interface is not valid / unknown  
 * @return \b true   otherwise
 *
 */
bool lld3377xWakeUp(void)  {

	if(*_interface_==IntTPL) {
		SPICSB(0);
		Delay(DELAY_22us);				
		SPICSB(1);
		Delay(DELAY_600us);				
		SPICSB(0);
		Delay(DELAY_22us);
		SPICSB(1);
		Delayms(WAIT_AFTER_WAKEUP);						  
		return true;

	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief Read Register(s) from MC3377x  
 * 
 * Requests noRegs2Read number of registers to be read. Its possible to read 1 or more
 * registers in a burst. 
 * The number of register which can be read at once is limited to \b 50, as a result 
 * of the receiving SPI buffer size (50 frames a 5 byte + 5 bytes echo = 255 bytes) 
 * 
 * 
 * @param gTagId 		Tag Id expected to be returned 
 * 
 * @param CID			cluster ID (node address)\n
 * 						must be in range 0..15 
 * @param Register	register address\n
 * 						must be in range 0..0x7F 
 * @param noRegs2Read     number of registers to read
 * 						must be in range 1..50
 * @param *readData		pointer to read data (array of size noRegs2Read)\n   
 *                      if \a NULL is passed, then no data is returned 
 *
 * @return true        if successfully executed
 * @return false       if error present when entering function or error occurs during execution
 * 
 * \note
 * Execution of function is ceased
 * - when an error (lld3377xGetError()==true) is already present
 * - attempting to read 0 or more then 50 registers (\ref ERR_WrongParam)
 * - _interface_ not defined (\ref ERR_WrongInterface)
 * 
 * \note
 * The response is checked against the following criteria and an error code is 
 * captured (\ref lld3377xGetError) and false is returned.\n
 * - for SPI interface
 * 	 -# \ref ERR_ResponseCRC is signalled in case the response frame was corrupted (CRC incorrect)
 * 	 -# \ref ERR_NullResponse is signalled in case of a Null response (not for the first frame)
 * 	 -# \ref ERR_ResponseRC or \ref ERR_ResponseTagId is signalled in case the RC or TagID is incorrect
 *   
 * \note
 * - for TPL interface
 * 	 -# \ref ERR_TX is signalled in case the TPL bus echo does not match the transmitted frame
 * 	 -# \ref ERR_NoResponse is signalled in case no MC3377x data is received
 * 	 -# \ref ERR_ResponseLen is signalled in case less than 5 bytes were received
 * 	 -# \ref ERR_ResponseCRC is signalled in case the response frame was corrupted (CRC incorrect)
 * 	 -# \ref ERR_ResponseAdr is signalled in case response address is incorrect
 * 	 -# \ref ERR_ResponseRC or \ref ERR_ResponseTagId is signalled in case the RC or TagID is incorrect
 * 
 * 
 * \remarks
 * For the IntSPI the read data is returned with the following SPI transfer, 
 * here a read "ahead" is performed to the next address, like this its possible 
 * to read multiple sequential addresses w/o overhead. (similar to burst read of IntTPL)	
 * 
 * \sa lld3377xWriteRegister, lld3377xWriteGlobalRegister, lld3377xNOPRegister*
 */
bool lld3377xReadRegisters(u8 CID, u8 Register, u8 noRegs2Read, u16 *readData)  {
	u8 txFrame[MSGLEN];
	u8 rxFrame[MSGLEN];
	u8 idx;
	u8 RC;
	u8 u8RxBytes;

	if(lld3377xGetError(NULL))		return false;								// only execute if no errors are pending  
	if((noRegs2Read==0)||(noRegs2Read>50))  return _lld3377xSetError(ERR_WrongParam);

	if(*_interface_==IntSPI) {
		RC = lld3377xNewRCValue();											//get next RC value from list
		idx = 0;
		lld3377xPackFrame(txFrame, 1, Register, CID, (RC<<2)|CmdRdLocal);	//pack data (read 1 register)
		SPISendBuffer(txFrame, rxFrame, MSGLEN);							//1st SPI transfer -> transmit

		if(lld3377xCrcCalc(&rxFrame[0], MSGLEN) != 0 )  {						//check response
			return _lld3377xSetError(ERR_ResponseCRC);
		}
		// ignore null response here (initial SPI transfere)
		//	if(IS_NULL_RESPONSE(rxFrame))
		//		return ERR_NullResponse;

		while(noRegs2Read-- >0)  {
			//! \todo new RC?
			lld3377xPackFrame(txFrame, 1, Register+1, CID, (RC<<2)|CmdRdLocal);	//pack data (read 1 register)
			SPISendBuffer(txFrame, rxFrame, MSGLEN);						//next SPI transfere -> read

			if(lld3377xCrcCalc(&rxFrame[0], MSGLEN) != 0 )  					//check response
				return _lld3377xSetError(ERR_ResponseCRC);

			if(IS_NULL_RESPONSE(rxFrame))  
				return _lld3377xSetError(ERR_NullResponse);

			if(CID>0)  {														// no checks for unassigned CIDs reads
				if(_cluster_[CID-1].pTagIdList!=NULL)  {							// use of TagIDs "enabled"? 
					if(bRegIsTagID(Register, _cluster_[CID-1].pTagIdList)) {
						if( UNPACK_TAGID(rxFrame) != gTagID[CID])  {			// check TagID
							return _lld3377xSetError(ERR_ResponseTagId);
						}
					}else{				
						if( UNPACK_RC(rxFrame) != RC)  { 							// check RC
							return _lld3377xSetError(ERR_ResponseRC);
						}
					}
				}
			}
			if(readData!=NULL)  {
				readData[idx] = UNPACK_DATA(rxFrame);  						//read data
			}	
			idx++;
			Register++;														//read next register
		}

		return true;

	}else if(*_interface_==IntTPL) {
		RC = lld3377xNewRCValue();											//get next RC value from list
		idx = 0;
		lld3377xPackFrame(txFrame, noRegs2Read, Register, CID, (RC<<2)|CmdRdLocal);	//pack data (read noRegs2Read registers)
		SPIRxFlushBuffer();														//flush receiver
		SPITxSendBuffer(txFrame, MSGLEN);							//transmit read request

		if(MSGLEN !=  SPIRxReadBytes(rxFrame, MSGLEN))  						//read request "echo" and check 
			return _lld3377xSetError(ERR_TX);
		if(memcmp(txFrame, rxFrame, MSGLEN))  
			return _lld3377xSetError(ERR_TX);

		while(noRegs2Read-- >0) {
			u8RxBytes = SPIRxReadBytes(rxFrame, MSGLEN);						//read response
			if(u8RxBytes == 0)	 						
				return _lld3377xSetError(ERR_NoResponse);
			if(u8RxBytes != MSGLEN)  					
				return _lld3377xSetError(ERR_ResponseLen);
			if(lld3377xCrcCalc(rxFrame, MSGLEN)!=0)		
				return _lld3377xSetError(ERR_ResponseCRC);

			if(UNPACK_REGADR(rxFrame) != Register)	 						// check response address
				return _lld3377xSetError(ERR_ResponseAdr);
			if(CID>0)  {														// no checks for unassigned CIDs reads
				if(_cluster_[CID-1].pTagIdList!=NULL)  {							// use of TagIDs "enabled"? 
					if(bRegIsTagID(Register, _cluster_[CID-1].pTagIdList)) {
						if( UNPACK_TAGID(rxFrame) != gTagID[CID])  { 			// check TagID
							return _lld3377xSetError(ERR_ResponseTagId);
						}
					}else{				
						if( UNPACK_RC(rxFrame) != RC)  {							// check RC
							return _lld3377xSetError(ERR_ResponseRC);
						}
					}
				}
			}

			if(readData!=NULL)  {
				readData[idx] = UNPACK_DATA(rxFrame);  					//read data
			}
			idx++;
			Register++;
		}
		return true;

	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}

// ----------------------------------------------------------------------------
/*! \brief Write register to MC3377x  
 * 
 * Writes the writeData to the register address on the cluster with the CID.
 * 
 * 
 * @param CID			cluster ID (node address)\n
 * 						must be in range 0..15 
 * @param register	register address\n
 * 						must be in range 0..0x7F 
 * @param writeData     data to be written 
 * @param *returnData	pointer to returned data\n   
 *                      if \a NULL is passed, then returned data is ignored 
 *
 * @return true        if successfully executed
 * @return false       if error present when entering function or error occurs during execution
 * 
 * 
 * \note
 * Execution of function is ceased
 * - when an error (lld3377xGetError()==true) is already present
 * - _interface_ not defined (\ref ERR_WrongInterface)
 * 
 * \note
 * The response is checked against the following criteria and an error code is 
 * captured (\ref lld3377xGetError) and false is returned.
 * - for SPI interface
 * 	 -# \ref ERR_ResponseCRC is signalled in case the response frame was corrupted (CRC incorrect)
 * 	 -# \ref ERR_NullResponse is signalled in case of a Null response (not for the first frame)
 *   
 * \note
 * - for TPL interface
 * 	 -# \ref ERR_TX is signalled in case the TPL bus echo does not match the transmitted frame
 * 	 -# \ref ERR_NoResponse is signalled in case no MC3377x data is received
 * 	 -# \ref ERR_ResponseLen is signalled in case less than 5 bytes were received
 * 	 -# \ref ERR_ResponseCRC is signalled in case the response frame was corrupted (CRC incorrect)
 * 	 -# \ref ERR_ResponseRC or \ref ERR_ResponseTagId is signalled in case the RC or TagID is incorrect
 * 
 * 
 * \remarks
 * For the IntSPI the returnData is from previous command! 
 * 
 * 
 * \sa lld3377xReadRegisters, lld3377xWriteGlobalRegister, lld3377xNOPRegister
 */
bool lld3377xWriteRegister(u8 CID, u8 Register, u16 writeData, u16 *returnData)  {
	u8 txFrame[MSGLEN];
	u8 rxFrame[MSGLEN];
	u8 u8RxBytes;
	u8 RC;	

	if(lld3377xGetError(NULL))		return false;								// only execute if no errors are pending  

	if(*_interface_==IntSPI) {
		lld3377xPackFrame(txFrame, writeData, Register, CID, CmdWrLocal);// pack data
		SPISendBuffer(txFrame, rxFrame, MSGLEN);							// transmit

		if(lld3377xCrcCalc(&rxFrame[0], MSGLEN) != 0)  						// check response
			return _lld3377xSetError(ERR_ResponseCRC);
		if(IS_NULL_RESPONSE(rxFrame))	 		
			return _lld3377xSetError(ERR_NullResponse);

		if(returnData!=NULL)
			*returnData = UNPACK_DATA(rxFrame);  								//return data
		return true;

	}else if(*_interface_==IntTPL)  {
		RC = lld3377xNewRCValue();											//get next RC value from list

		lld3377xPackFrame(txFrame, writeData, Register, CID, (RC<<2)|CmdWrLocal);	//pack data
		SPIRxFlushBuffer();														//flush receiver
		SPITxSendBuffer(txFrame, MSGLEN);										//transmit

		if(MSGLEN !=  SPIRxReadBytes(rxFrame, MSGLEN))  						//read request "echo" and check 
			return _lld3377xSetError(ERR_TX);
		if(memcmp(txFrame, rxFrame, MSGLEN))  
			return _lld3377xSetError(ERR_TX);

		u8RxBytes = SPIRxReadBytes(rxFrame, MSGLEN);							//read response
		if(u8RxBytes == 0)					
			return _lld3377xSetError(ERR_NoResponse);
		if(u8RxBytes != MSGLEN)  				
			return _lld3377xSetError(ERR_ResponseLen);
		if(lld3377xCrcCalc(rxFrame, MSGLEN)!=0)  
			return _lld3377xSetError(ERR_ResponseCRC);
		if(UNPACK_RC(rxFrame)!=RC) 	
			return _lld3377xSetError(ERR_ResponseRC);
		if(returnData!=NULL)  {
			*returnData = UNPACK_DATA(rxFrame);  								//return data
		}

		return RETURN_OK;		

	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief Global write register request to all CID  (TPL Version)  
 * 
 * Writes the writeData to the register address on the cluster with the CID.
 * 
 * @param Register	register address\n
 * 						must be in range 0..0x7F 
 * @param writeData     data to be written 
 *
 * @return true        if successfully executed
 * @return false       if error present when entering function or error occurs during execution
 * 
 * 
 * \note
 * Execution of function is ceased
 * - when an error (lld3377xGetError()==true) is already present
 * - _interface_ is \ref IntSPI or not defined (\ref ERR_WrongInterface)
 * 
 * \note
 * The response is checked against the following criteria and an error code is 
 * captured (\ref lld3377xGetError) and false is returned.
 *   
 * \note
 * - for TPL interface
 * 	 -# \ref ERR_TX is signalled in case the TPL bus echo does not match the transmitted frame
 * 
 * 
 * \remarks
 * For the GlobalWrite command does not generate and response! 
 * 
 * \sa lld3377xReadRegisters, lld3377xWriteRegister, lld3377xNOPRegister
 */
bool lld3377xWriteGlobalRegister(u8 Register, u16 writeData)  {
	u8 txFrame[MSGLEN];
	u8 rxFrame[MSGLEN];

	if(lld3377xGetError(NULL))		return false;								// only execute if no errors are pending  

	if(*_interface_==IntSPI) {
		return _lld3377xSetError(ERR_WrongInterface);
	}else if(*_interface_==IntTPL) {
		lld3377xPackFrame(txFrame, writeData, Register, GLOBAL_CID, CmdWrGlobal);				//1. pack data

		SPIRxFlushBuffer();
		SPITxSendBuffer(txFrame, MSGLEN);										//2. transmit

		if(MSGLEN !=  SPIRxReadBytes(rxFrame, MSGLEN))  						//3. read back transmit and check
			return _lld3377xSetError(ERR_TX);
		if(memcmp(txFrame, rxFrame, MSGLEN))  
			return _lld3377xSetError(ERR_TX);

		return true;

	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief NOP request to CID  (NOT TESTED)   
 * 
 * Issues a NOP request to the cluster with the CID CID.
 * 
 * @param Register	register address 
 * @param writeData     data to be written 
 * @param *readData		pointer to returned data\n   
 *                      if \a NULL is passed, then returned data is ignored 

 * @return \b true        if successfully executed
 * @return \b false       if error present when entering function or error occurs during execution
 * 
 * 
 * \note
 * Execution of function is ceased
 * - when an error (lld3377xGetError()==true) is already present
 * - _interface_ is not defined (\ref ERR_WrongInterface)
 * 
 * \note
 * The response is checked against the following criteria and an error code is 
 * captured (\ref lld3377xGetError) and false is returned.
 * - for SPI interface
 * 	 -# \ref ERR_ResponseCRC is signalled in case the response frame was corrupted (CRC incorrect)
 * 	 -# \ref ERR_NullResponse is signalled in case of a Null response (not for the first frame)
 *   
 * \note
 * - for TPL interface
 * 	 -# \ref ERR_TX is signalled in case the TPL bus echo does not match the transmitted frame
 * 
 * 
 * \remarks
 * The readData is not compared against the writeData! If this is desired it has to be performed on application level.\n
 * For the IntSPI the readData is from previous command! 
 * 
 * \sa lld3377xReadRegisters, lld3377xWriteRegister, lld3377xWriteGlobalRegister
 */
bool lld3377xNOPRegister(u8 CID, u8 Register, u16 writeData, u16 *readData)  {
	u8 txFrame[MSGLEN];
	u8 rxFrame[MSGLEN];
	u8 u8RxBytes;
	u8 RC;	

	if(lld3377xGetError(NULL))		return false;								// only execute if no errors are pending  


	if(*_interface_==IntSPI) {
		lld3377xPackFrame(txFrame, writeData, Register, CID, CmdNOP);// pack data
		SPISendBuffer(txFrame, rxFrame, MSGLEN);							// transmit

		if(lld3377xCrcCalc(&rxFrame[0], MSGLEN) != 0)  						// check response
			return _lld3377xSetError(ERR_ResponseCRC);
		if(IS_NULL_RESPONSE(rxFrame))	 		
			return _lld3377xSetError(ERR_NullResponse);

		if(readData!=NULL)
			*readData = UNPACK_DATA(rxFrame);  								//return data
		// SPI version behaves different to TPL version: return data is from previous access

		return true;

	}else if(*_interface_==IntTPL)  {
		RC = lld3377xNewRCValue();											//get next RC value from list

		lld3377xPackFrame(txFrame, writeData, Register, CID, (RC<<2)|CmdNOP);	//pack data
		SPIRxFlushBuffer();														//flush receiver
		SPITxSendBuffer(txFrame, MSGLEN);										//transmit

		if(MSGLEN !=  SPIRxReadBytes(rxFrame, MSGLEN))  						//read request "echo" and check 
			return _lld3377xSetError(ERR_TX);
		if(memcmp(txFrame, rxFrame, MSGLEN))  
			return _lld3377xSetError(ERR_TX);

		u8RxBytes = SPIRxReadBytes(rxFrame, MSGLEN);							//read response
		if(u8RxBytes == 0)					
			return _lld3377xSetError(ERR_NoResponse);
		if(u8RxBytes != MSGLEN)  				
			return _lld3377xSetError(ERR_ResponseLen);
		if(lld3377xCrcCalc(rxFrame, MSGLEN)!=0)  
			return _lld3377xSetError(ERR_ResponseCRC);
		if(UNPACK_RC(rxFrame)!=RC) 	
			return _lld3377xSetError(ERR_ResponseRC);
		if(readData!=NULL)  {
			*readData = UNPACK_DATA(rxFrame);  								//return data
		}

		return RETURN_OK;		

	}else{
		return _lld3377xSetError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief Sets the TPL enable signal 
 * 
 * Sets the TPL enable signal and waits for an acknowledge on INTB (low pulse).
 * -# disables TPL signal for 150us
 * -# enables TPL signal
 * -# waits for acknowledge from MC33664 (INTB low pulse)
 * \image html intbverificationpulse.png 
 * 
 * -# delay of 100us before returning  (tready)
 * 
 * @return \b true        if successfully executed
 * @return \b false       if error present when entering function (\ref lld3377xGetError) or in case of a missing acknowledgement (\ref ERR_Timeout)
 * 
 * \sa lld3377xTPLDisable and TplEnable, IntbPinStatus
 */
bool lld3377xTPLEnable(void)  {
	u32 u32Cnt;

	if(lld3377xGetError(NULL))		return false;								// only execute if no errors are pending  

	TplEnable(0);		//disable
	Delay(DELAY_150us);
	// start timeout >>110us
	TplEnable(1);		//enable

	u32Cnt = 0x10000;
	while(IntbPinStatus()==1)  {												// wait for a low
		if(u32Cnt--==0)  {
			return _lld3377xSetError(ERR_Timeout);
		}
	}

	u32Cnt = 0x10000;
	while(IntbPinStatus()==0)	{												// wait for a high
		if(u32Cnt--==0)  {
			return _lld3377xSetError(ERR_Timeout);
		}
	}

	//	Delay(DELAY_100us);
	return true;
}
// ----------------------------------------------------------------------------
/*! \brief Clears the TPL enable signal 
 * 
 * Clears the TPL enable signal 
 * 
 * @return \b true        if successfully executed
 * @return \b false       if error present when entering function (\ref lld3377xGetError)
 * 
 * \sa lld3377xTPLEnable and TplEnable
 */
bool lld3377xTPLDisable(void)  {

	if(lld3377xGetError(NULL))		return false;								// only execute if no errors are pending  

	TplEnable(0);																// disable	
	return true;
}


#define ADC_CFG_DEFAULT     (PGA_GAIN_AUTO |ADC1_A_14bit|ADC1_B_14bit|ADC2_16bit)				// reset status
#define ADC_CFG_SETTING     (PGA_GAIN_AUTO |ADC1_A_16bit|ADC1_B_16bit|ADC2_16bit)


// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33771 (14 cells) via SPI. 

Array of Configuration Register Values.  

For SPI its important to enable the CSB wakeup in the WAKEUP_MASK1 register!

The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
 */
const TYPE_BCC_CONF CONF33771SPI[] = {
		// register      	data
		// set OV & UV thresholds
		//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
		{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT7,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT8,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT9,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT10,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT11,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT12,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT13,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT14,        	TH_OVUV_VALUE(3.0, 1.8)   },

		// set OT & UT thresholds
		{ TH_AN0_OT,        DEG30C              },
		{ TH_AN1_OT,		DEG30C              },
		{ TH_AN2_OT,		DEG30C              },
		{ TH_AN3_OT,		DEG30C              },
		{ TH_AN4_OT,		DEG30C              },
		{ TH_AN5_OT,		DEG30C              },
		{ TH_AN6_OT,		DEG30C              },

		{ TH_AN0_UT,        DEG0C              },
		{ TH_AN1_UT,		DEG0C              },
		{ TH_AN2_UT,		DEG0C              },
		{ TH_AN3_UT,		DEG0C              },
		{ TH_AN4_UT,		DEG0C              },
		{ TH_AN5_UT,		DEG0C              },
		{ TH_AN6_UT,		DEG0C              },

		// define Fault Handling (mask = 1 "disable")
		{ FAULT_MASK1, 		0x1FF0				},// CT OT/UT/OV/UV enabled
		{ FAULT_MASK2, 		0xFE7F				},// 0xFE7F all masked
		{ FAULT_MASK3, 		0xFFFF 				},// 0xFFFF all masked

		// define Wakeup sources
		{ WAKEUP_MASK1, 	0x189F,				},	// 0x199F all masked   enabled: CSB WU
		{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
		{ WAKEUP_MASK3, 	0xBFFF,         	},	// 0xBFFF all masked

		// configuration (bits)
		{ OV_UV_EN,         0x3FFF				},	// enable OV & UV handling
		{ SYS_CFG1, 		0x9000  			},
		{ SYS_CFG2, 		0x6330	         	},
		{ FAULT1_STATUS, 	0xC000	 			},	// clear all bits, except POR, Reset
		{ FAULT2_STATUS, 	0 					},	// clear all bits
		{ FAULT3_STATUS, 	0 					},	// clear all bits
		{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
		{0 , 0 }		// end symbol
};

// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33771 (14 cells) via TPL. 

Array of Configuration Register Values.  

For TPL typically the CSB wakeup is disabled (in the WAKEUP_MASK1 register)!

The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
 */
const TYPE_BCC_CONF CONF33771TPL[] = {
		// register      	data
		// set OV & UV thresholds
		//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
		{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT7,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT8,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT9,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT10,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT11,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT12,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT13,        	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT14,        	TH_OVUV_VALUE(3.0, 1.8)   },


		// set OT & UT thresholds
		{ TH_AN0_OT,        DEG30C              },
		{ TH_AN1_OT,		DEG30C              },
		{ TH_AN2_OT,		DEG30C              },
		{ TH_AN3_OT,		DEG30C              },
		{ TH_AN4_OT,		DEG30C              },
		{ TH_AN5_OT,		DEG30C              },
		{ TH_AN6_OT,		DEG30C              },

		{ TH_AN0_UT,        DEG0C              },
		{ TH_AN1_UT,		DEG0C               },
		{ TH_AN2_UT,		DEG0C               },
		{ TH_AN3_UT,		DEG0C               },
		{ TH_AN4_UT,		DEG0C               },
		{ TH_AN5_UT,		DEG0C               },
		{ TH_AN6_UT,		DEG0C               },

		//	{ TH_ISENSE_OC,		0x000,              },
		//	{ TH_COULOMB_H,		0x0000,             },
		//	{ TH_COULOMB_L,		0x0000,             },


		// define Fault Handling (mask = 1 "disable")
		{ FAULT_MASK1, 		0x1FF0				},// CT OT/UT/OV/UV enabled
		{ FAULT_MASK2, 		0xFE7F				},// 0xFE7F all masked
		{ FAULT_MASK3, 		0xFFFF 				},// 0xFFFF all masked

		// define Wakeup sources
		{ WAKEUP_MASK1, 	0x199F,				},	// 0x199F all masked
		{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
		{ WAKEUP_MASK3, 	0xBFFF,         	},	// 0xBFFF all masked

		// configuration (bits)
		{ OV_UV_EN,         0x3FFF				},	// enable OV & UV handling
		{ SYS_CFG1, 		0x9000	 			},
		{ SYS_CFG2, 		0x6330	         	},
		{ FAULT1_STATUS, 	0xC000	 			},	// clear all bits, except POR, Reset
		{ FAULT2_STATUS, 	0 					},	// clear all bits
		{ FAULT3_STATUS, 	0 					},	// clear all bits
		{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
		{0 , 0 }		// end symbol
};

// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33772 (6 cells) via SPI. 

Array of Configuration Register Values.  

For SPI its important to enable the CSB wakeup in the WAKEUP_MASK1 register!

The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
 */
const TYPE_BCC_CONF CONF33772SPI[] = {
		//	// register      	data
		// set OV & UV thresholds
		//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
		{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },

		// set OT & UT thresholds
		{ TH_AN0_OT,        DEG30C,                   },
		{ TH_AN1_OT,		DEG30C,                   },
		{ TH_AN2_OT,		DEG30C,                   },
		{ TH_AN3_OT,		DEG30C,                   },
		{ TH_AN4_OT,		DEG30C,                   },
		{ TH_AN5_OT,		DEG30C,                   },
		{ TH_AN6_OT,		DEG30C,                   },

		{ TH_AN0_UT,        DEG0C,                    },
		{ TH_AN1_UT,		DEG0C,                    },
		{ TH_AN2_UT,		DEG0C,                    },
		{ TH_AN3_UT,		DEG0C,                    },
		{ TH_AN4_UT,		DEG0C,                    },
		{ TH_AN5_UT,		DEG0C,                    },
		{ TH_AN6_UT,		DEG0C,                    },

		//	{ TH_ISENSE_OC,		0x000,                    },
		//	{ TH_COULOMB_H,		0x0000,                    },
		//	{ TH_COULOMB_L,		0x0000,                    },


		// define Fault Handling (mask = 1 "disable")
		{ FAULT_MASK1, 		0x1FF0,				},// CT OT/UT/OV/UV enabled
		{ FAULT_MASK2, 		0xFE7F,				},// 0xFE7F all masked
		{ FAULT_MASK3, 		0xFFFF, 			},// 0xFFFF all masked

		// define Wakeup sources
		{ WAKEUP_MASK1, 	0x189F,				},	// 0x199F all masked   enabled: CSB WU
		{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
		{ WAKEUP_MASK3, 	0xA03F,         	},	// 0xA03F all masked

		// configuration (bits)
		{ OV_UV_EN,         0x003F,			},	// enable OV & UV handling
		{ SYS_CFG1, 		0x9000, 		},
		{ SYS_CFG2, 		0x6330,         },
		{ FAULT1_STATUS, 	0xC000, 		},	// clear all bits, except POR, Reset
		{ FAULT2_STATUS, 	0, 				},	// clear all bits
		{ FAULT3_STATUS, 	0, 				},	// clear all bits
		{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
		{0 , 0 }		// end symbol
};
// ----------------------------------------------------------------------------
/*! \brief Example Configuration for MC33772 (6 cells) via TPL. 

Array of Configuration Register Values.  

For TPL typically the CSB wakeup is disabled (in the WAKEUP_MASK1 register)!

The values are written into the BCC registers at Configuration. The writing 
order is top to bottom. So e.g. clearing of Fault conditions (FAULT_STATUS) should be done after
setting thresholds and masking. 
 */
const TYPE_BCC_CONF CONF33772TPL[] = {
		//	// register      	data
		// set OV & UV thresholds
		//	{ TH_ALL_CT,       	TH_OVUV_VALUE(3.0, 2.0)   },
		{ TH_CT1,      		TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT2,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT3,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT4,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT5,          	TH_OVUV_VALUE(3.0, 1.8)   },
		{ TH_CT6,          	TH_OVUV_VALUE(3.0, 1.8)   },

		// set OT & UT thresholds
		{ TH_AN0_OT,        DEG30C,                   },
		{ TH_AN1_OT,		DEG30C,                   },
		{ TH_AN2_OT,		DEG30C,                   },
		{ TH_AN3_OT,		DEG30C,                   },
		{ TH_AN4_OT,		DEG30C,                   },
		{ TH_AN5_OT,		DEG30C,                   },
		{ TH_AN6_OT,		DEG30C,                   },

		{ TH_AN0_UT,        DEG0C,                    },
		{ TH_AN1_UT,		DEG0C,                    },
		{ TH_AN2_UT,		DEG0C,                    },
		{ TH_AN3_UT,		DEG0C,                    },
		{ TH_AN4_UT,		DEG0C,                    },
		{ TH_AN5_UT,		DEG0C,                    },
		{ TH_AN6_UT,		DEG0C,                    },

		// define Fault Handling (mask = 1 "disable")
		{ FAULT_MASK1, 		0x1FF0,				},// CT OT/UT/OV/UV enabled
		{ FAULT_MASK2, 		0xFE7F,				},// 0xFE7F all masked
		{ FAULT_MASK3, 		0xFFFF, 			},// 0xFFFF all masked

		// define Wakeup sources
		{ WAKEUP_MASK1, 	0x199F,				},	// 0x199F all masked
		{ WAKEUP_MASK2, 	0xFF36,         	},	// 0xFF36 all masked
		{ WAKEUP_MASK3, 	0xA03F,         	},	// 0xA03F all masked

		// configuration (bits)
		{ OV_UV_EN,         0x003F,			},	// enable OV & UV handling
		{ SYS_CFG1, 		0x9000, 		},
		{ SYS_CFG2, 		0x6330,         },
		{ FAULT1_STATUS, 	0xC000, 		},	// clear all bits, except POR, Reset
		{ FAULT2_STATUS, 	0, 				},	// clear all bits
		{ FAULT3_STATUS, 	0, 				},	// clear all bits
		{ ADC_CFG,         	ADC_CFG_SETTING	    },     // set ADC
		{0 , 0 }		// end symbol
}; 


// ----------------------------------------------------------------------------
/*! \brief Initializes the FRDM-KL25Z interface for the specific EVB.

Different interface options for TPL and SPI are supported.

For TPL 2 different interfaces (TYPE_EVB) are supported.

For SPI 2 different interfaces (TYPE_EVB) are supported.

 */
void InitInterface(TYPE_INTERFACE interface, TYPE_EVB evb)
{
	_interface_frdm = interface;
	_evb_ = evb;


	/*!
		4. \b TPL-TypeArd (HWPLATFORM 3)

			\image html hwplatform3.png

		The "new style = type Ard" TPL EVB uses the following interface pins:

		| CON   | PIN   | Function  | Pin Alt | Dir  | Signal |StartUp |
		|-------|-------|-----------|---------|------|--------|--------|
		| J1-06 | PTD4  | GPIO      |    1    | <--- | FAULT  |        |
		| J1-08 | PTA12 | GPIO      |    1    | <--- | INTB   |        |
		| J2-18 | PTE0  | GPIO      |    1    | ---> | EN     |        |
		| J1-14 | PTC8  | GPIO      |    1    | ---> | TXCSB  |        |
		| J1-09 | PTC5  | SPI0_SCK  |    2    | ---> | TXCLK  |        |
		| J2-08 | PTD2  | SPI0_MO   |    2    | ---> | TXDATA |        |
		| J9-05 | PTB10 | SPI1_PCS0 |    2    | <--- | RXCSB  |        |
		| J9-07 | PTB11 | SPI1_SCK  |    2    | <--- | RXCLK  |        |
		| J2-19 | PTD7  | SPI1_SI   |    5    | <--- | RXDATA |        |

		| J1-13 | PTC10 | GPIO      |    1    | ---> | BRD0   | low    |
		| J1-15 | PTC11 | GPIO      |    1    | ---> | BRD1   | low    |
		| J2-01 | PTC12 | GPIO      |    1    | ---> | BRD2   | low    |
		| J2-03 | PTC13 | GPIO      |    1    | ---> | BRD3   | low    |

		| J1-03 | PTC0  | GPIO      |    1    | ---> | CH0    | low    |
		| J1-05 | PTC3  | GPIO      |    1    | ---> | CH1    | low    |
		| J1-07 | PTC4  | GPIO      |    1    | ---> | CH2    | low    |
		| J1-11 | PTC6  | GPIO      |    1    | ---> | CH3    | low    |
		| J1-01 | PTC7  | GPIO      |    1    | ---> | CH4    | low    |

	 */

	if ((_interface_frdm == IntTPL) && (_evb_ == EVB_TypeArd))
	{									  // new ardunio style EVB
		SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK; // enable SPI0 clock
		SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK; // enable SPI1 clock

		PORTD_PCR4 = PORT_PCR_MUX(1);
		GPIOD_PDDR &= ~BIT(4);

		PORTA_PCR12 = PORT_PCR_MUX(1);
		GPIOA_PDDR &= ~BIT(12);

		PORTE_PCR0 = PORT_PCR_MUX(1);
		GPIOE_PDDR |= BIT(0);
		GPIOE_PCOR = BIT(0);

		PORTC_PCR8 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(8);
		GPIOC_PSOR = BIT(8);

		PORTC_PCR5 = PORT_PCR_MUX(2);
		PORTD_PCR2 = PORT_PCR_MUX(2);

		PORTB_PCR10 = PORT_PCR_MUX(2);
		PORTB_PCR11 = PORT_PCR_MUX(2);
		PORTD_PCR7 = PORT_PCR_MUX(5);

		PORTC_PCR10 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(10);
		GPIOC_PCOR = BIT(10);
		PORTC_PCR11 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(11);
		GPIOC_PCOR = BIT(11);
		PORTC_PCR12 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(12);
		GPIOC_PCOR = BIT(12);
		PORTC_PCR13 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(13);
		GPIOC_PCOR = BIT(13);

		PORTC_PCR0 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(0);
		GPIOC_PCOR = BIT(0);
		PORTC_PCR3 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(3);
		GPIOC_PCOR = BIT(3);
		PORTC_PCR4 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(4);
		GPIOC_PCOR = BIT(4);
		PORTC_PCR6 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(6);
		GPIOC_PCOR = BIT(6);
		PORTC_PCR7 = PORT_PCR_MUX(1);
		GPIOC_PDDR |= BIT(7);
		GPIOC_PCOR = BIT(7);

		SPITxInit(0); // SPI0 used to transmit (polling)
		SPIRxInit(1); // SPI1 used to receive (interrupt driven)
		NVICEnIrq(SPI1_IRQ);
		SPIRxEnable();
		SPITxEnable();
	}
}


/*! \brief De-Initializes the FRDM-KL25Z interface for the specific EVB.

4 EVBs are supported right now:
- old TYPE1 EVBs for SPI and TPL
- new Arduino EVBs for SPI and TPL

The local copy of _interface_ and _evb_ are use to know which interface has do be
de-initilaized.
Sets the local copies to:
	_interface_ = IntUnknown;
	_evb_ = EVB_Unknown;

 */
void DeInitInterface(void)
{
	/*!
	4. \b TPL-TypeArd (HWPLATFORM 3)

		\image html hwplatform3.png

	The "new style = type Ard" TPL EVB uses the following interface pins:

	| CON   | PIN   | Function       | Pin Alt |
	|-------|-------|----------------|---------|
	| J1-06 | PTD4  | disable/analog |    0    |
	| J1-08 | PTA12 | disable/analog |    0    |
	| J2-18 | PTE0  | disable/analog |    0    |
	| J1-14 | PTC8  | disable/analog |    0    |
	| J1-09 | PTC5  | disable/analog |    0    |
	| J2-08 | PTD2  | disable/analog |    0    |
	| J9-05 | PTB10 | disable/analog |    0    |
	| J9-07 | PTB11 | disable/analog |    0    |
	| J2-19 | PTD7  | disable/analog |    0    |

	| J1-13 | PTC10 | disable/analog |    0    |
	| J1-15 | PTC11 | disable/analog |    0    |
	| J2-01 | PTC12 | disable/analog |    0    |
	| J2-03 | PTC13 | disable/analog |    0    |

	| J1-03 | PTC0  | disable/analog |    0    |
	| J1-05 | PTC3  | disable/analog |    0    |
	| J1-07 | PTC4  | disable/analog |    0    |
	| J1-11 | PTC6  | disable/analog |    0    |
	| J1-01 | PTC7  | disable/analog |    0    |

	 */
	if ((_interface_frdm == IntTPL) && (_evb_ == EVB_TypeArd))
	{ // new ardunio style EVB

		NVICDisIrq(SPI1_IRQ);
		SPIDisable();
		SIM_SCGC4 &= ~SIM_SCGC4_SPI0_MASK; // disable SPI0 clock
		SIM_SCGC4 &= ~SIM_SCGC4_SPI1_MASK; // disable SPI1 clock

		PORTD_PCR4 = PORT_PCR_MUX(0);
		GPIOD_PDDR &= ~BIT(4);

		PORTA_PCR12 = PORT_PCR_MUX(0);
		GPIOA_PDDR &= ~BIT(12);

		PORTE_PCR0 = PORT_PCR_MUX(0);
		GPIOE_PDDR &= ~BIT(0);

		PORTC_PCR8 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(8);

		PORTC_PCR5 = PORT_PCR_MUX(0);
		PORTD_PCR2 = PORT_PCR_MUX(0);

		PORTB_PCR10 = PORT_PCR_MUX(0);
		PORTB_PCR11 = PORT_PCR_MUX(0);
		PORTD_PCR7 = PORT_PCR_MUX(0);

		PORTC_PCR10 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(10);
		PORTC_PCR11 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(11);
		PORTC_PCR12 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(12);
		PORTC_PCR13 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(13);

		PORTC_PCR0 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(0);

		PORTC_PCR3 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(3);

		PORTC_PCR4 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(4);

		PORTC_PCR6 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(6);

		PORTC_PCR7 = PORT_PCR_MUX(0);
		GPIOC_PDDR &= ~BIT(7);
	}

	_interface_frdm = IntUnknown;
	_evb_ = EVB_Unknown;
}

// ----------------------------------------------------------------------------
/*! \brief Reads the status of the Fault pin.
 *
 * Works with static Fault signal.
 * Does not work with Fault Wave.

 *  Mandatory to prior call InitInterface().
 *
 * @return 1   if FAULT
 *         0   in no Fault
 */

//! \todo use interrupt instead of polling. Enhance to be used with wave signal.
u8 FaultPinStatus(void)
{
	if (_interface_frdm == IntTPL)
	{ // EVBs via TPL
		if (_evb_ == EVB_TypeArd)
		{											// Arduino type connection
			return ((GPIOD_PDIR & BIT(4)) ? 1 : 0); // PTD4
		}
		return 1;
	}
}
// ----------------------------------------------------------------------------
/*! \brief Sets the TPL_Enable pin.
 */
void TplEnable(u8 bEnable)
{

	if (_interface_frdm == IntTPL)
	{ // EVBs via TPL

		if (_evb_ == EVB_TypeArd)
		{ // Arduino type connection
			if (bEnable == 0)
				GPIOE_PCOR = BIT(0);
			else
				GPIOE_PSOR = BIT(0);
		}
	}
}

// ----------------------------------------------------------------------------
/*! \brief Reads the status of the INTB pin.

 *  Mandatory to prior call InitInterface().
 *
 *
 * @return 1   if INTB high
 *         0   in INTB low
 */
u8 IntbPinStatus(void)
{

	if (_interface_frdm == IntTPL)
	{ // EVBs via TPL

		if (_evb_ == EVB_TypeArd)
		{											 // Arduino type connection
			return ((GPIOA_PDIR & BIT(12)) ? 1 : 0); // PTA12
		}
	}
	return 1;
}

//-----------------------------------------------------------------------------
/*! \brief Sets the CSB level.
 *
 *  Mandatory to prior call InitInterface().
 *
 * @param  u8Level		0 for low,  <>0 for high
 * @param  drv         \ref TYPE_DRV_SETUP
 */
void SPICSB(u8 u8Level)
{
	if(u8Level==0)
		GPIOC_PCOR  = BIT(8); 												// clear PTC8
	else
		GPIOC_PSOR  = BIT(8); 												// set PTC8
}

void initFIMode(u8 u8enabledDisabled) // Activate external script trigger (Fault injection mode)
{
	// SM trigger implementation:

	if (u8enabledDisabled < 1) // FI mode disabled
	{
		PORTA_PCR16 |= PORT_PCR_MUX(1);							   // PTA16 is configured as GPIO to disable NMI function on this pin
		PORTA_PCR16 &= (~PORT_PCR_IRQC_MASK | PORT_PCR_IRQC(0x0)); // Disable PTA16 interruptions but keep GPIO function
		NVICDisIrq(PORTA_IRQ);									   // Disable NVIC PORT A interrupt

		GPIOA_PCOR |= BIT(5);  // PTA5 output is set to a low state
		GPIOA_PDDR &= ~BIT(5); // PTA5 is set as default
		PORTA_PCR5 = 0x00;	   // Clear PTA5 configuration

		LED_GREEN_Off();
		LED_RED_On();
	}

	else if (u8enabledDisabled >= 1) // FI mode enabled
	{
		PORTA_PCR16 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xA); // PTA16 is configured as GPIO with interrupt on falling edge
		GPIOA_PDDR &= ~BIT(16);								 // PTA16 is an input
		NVICSetIrqPrio(PORTA_IRQ, IP_PRIO_1);				 // Set NVIC priority of PORTA interrupt (priority 1)
		NVICEnIrq(PORTA_IRQ);								 // Enable NVIC PORT A interrupt

		PORTA_PCR5 = PORT_PCR_MUX(1); // PTA5 is configured as GPIO
		GPIOA_PDDR |= BIT(5);		  // PTA5 is an output
		GPIOA_PCOR |= BIT(5);		  // PTA5 output is set to a low state

		LED_RED_Off();
		LED_GREEN_On();
	}
}












































