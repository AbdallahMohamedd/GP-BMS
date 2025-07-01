
#include "source/COTs/BMSDataBase/Inc/database.h"


// ----------------------------------------------------------------------------
extern const uint16_t _TAGID_BCC14p2[];
extern const uint16_t _TAGID_BCC14[];
extern const uint16_t _TAGID_BCC6[];
// ----------------------------------------------------------------------------
#define CheckCID(v) (((v) < 1) || ((v) > MAX_CLUSTER)) //!< macro to check if CID is in {1..MAX_CLUSTER}
// ----------------------------------------------------------------------------
/*! \brief Initialises the BMS system
 *
 * Initialises the BMS system with NoOfNodes.
 *
 * <b>For all nodes</b>
 * - Clear errors (\ref slaveIF_clearError)
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

//---------------------------ADC-----------------------------------------------------

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

bool BMSInit(uint8_t NoOfNodes)
{
	uint8_t cid;

	if (NoOfNodes == 0)
		return false;
	if (NoOfNodes > MAX_CLUSTER)
		return false;

	for (cid = 1; cid <= NoOfNodes; cid++)
	{
		slaveIF_clearError();
		slaveIF_wakeUp(); // send wakeup in case next device is in idle mode
		if (slaveIF_readReg(CIDunassiged, INIT, 1, NULL))
		{															   // check if somebody is there / read Init register
			slaveIF_writeReg(CIDunassiged, INIT, (uint16_t)cid, NULL); // assign CID
			if (cid < NoOfNodes)
			{													// all nodes except last one
				slaveIF_writeReg(cid, INIT, INIT_BUS_SW, NULL); // close switch	(except last node)
			}
			else
			{
				slaveIF_writeReg(cid, INIT, 0x0000, NULL); // open switch	(last node)
			}
		}
	}
	return !slaveIF_getError(NULL);
}
// ----------------------------------------------------------------------------
/*! \brief Performs basic MC33771B configuration.
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
bool MC3377xConfig(uint8_t cid, const SsysConf_t conf[])
{
	uint16_t n;

	if (CheckCID(cid))
		return slaveIF_setError(ERR_WrongParam);

	n = 0;
	while (conf[n].regAdr != 0)
	{
		slaveIF_writeReg(cid, conf[n].regAdr, conf[n].regValue, NULL);
		n++;
	}
	return !slaveIF_getError(NULL);
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
bool MC3377xADCStartConversion(uint8_t cid, uint8_t tagID)
{
	uint16_t adcCfg;

	if (CheckCID(cid))
		return slaveIF_setError(ERR_WrongParam);

	if (tagID > 15)
		return slaveIF_setError(ERR_WrongParam);

	slaveIF_readReg(cid, ADC_CFG, 1, &adcCfg);
	adcCfg = adcCfg | ADC_CFG_SOC;				// set start of conversion
	adcCfg = (adcCfg & 0x0FFF) | (tagID << 12); // set TagId
	slaveIF_writeReg(cid, ADC_CFG, adcCfg, NULL);

	if (slaveIF_getError(NULL))
	{
		return false;
	}
	else
	{
		slaveIF_setTagID(cid, tagID);
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
bool MC3377xADCIsConverting(uint8_t cid)
{
	uint16_t adcCfg;

	if (CheckCID(cid))
		return slaveIF_setError(ERR_WrongParam);

	if (slaveIF_readReg(cid, ADC_CFG, 1, &adcCfg))
	{
		return adcCfg & ADC_CFG_SOC;
	}
	else
	{
		return false;
	}
}
// ----------------------------------------------------------------------------

bool Abdullah_Temp(TYPE_MEAS_RESULTS_RAW *RawMeasResults)
{
	RawMeasResults->u16ANVoltage[0] = thermalManager_readRawData(PTB0_Channel);
	RawMeasResults->u16ANVoltage[1] = thermalManager_readRawData(PTB1_Channel);
	RawMeasResults->u16ANVoltage[2] = thermalManager_readRawData(PTD1_Channel);
	RawMeasResults->u16ANVoltage[3] = thermalManager_readRawData(PTD5_Channel);
	RawMeasResults->u16ANVoltage[4] = thermalManager_readRawData(PTD6_Channel);
	RawMeasResults->u16ANVoltage[5] = thermalManager_readRawData(PTD6_Channel);
	RawMeasResults->u16ANVoltage[6] = thermalManager_readRawData(PTD0_Channel);
}

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
bool MC3377xGetRawMeasurements(uint8_t cid, uint8_t tagId, uint8_t NoCTs, TYPE_MEAS_RESULTS_RAW *RawMeasResults)
{
	uint16_t rdData[0x1B];
	uint8_t u8Idx;
	uint32_t u19Current;

	// -----  burst read data  -----
	if (slaveIF_readReg(cid, MEAS_ISENSE1, 0x3, &rdData[0]))
	{																			// read register 0x30..0x32
		u19Current = ((rdData[0] & 0x7FFF) << 4) | ((rdData[1] & 0x000F) << 0); // -----  MEAS_ISENSE  -----
		// -----  sign extend to int32_t  -----
		if (u19Current & BIT(18))
		{														  // if current reading is negative
			RawMeasResults->s32Current = u19Current | 0xFFF80000; // sign extension
		}
		else
		{
			RawMeasResults->s32Current = u19Current; // nothing to do
		}
		RawMeasResults->u16StackVoltage = rdData[2] & 0x7FFF; // -----  MEAS_STACK  -----
	}

	// -----  MEAS_CELL[14..1]  -----
	if (slaveIF_readReg(cid, MEAS_CELL1 - NoCTs + 1, NoCTs, &rdData[0]))
	{
		u8Idx = NoCTs;
		while (u8Idx)
		{
			u8Idx--;
			RawMeasResults->u16CellVoltage[u8Idx] = rdData[NoCTs - 1 - u8Idx] & 0x7FFF; // reverse order
		}
	}

	if (slaveIF_readReg(cid, MEAS_IC_TEMP, 0x3, &rdData[0]))
	{
		RawMeasResults->u16ICTemp = rdData[0] & 0x7FFF;	  // MEAS_IC_TEMP
		RawMeasResults->u16VbgADC1A = rdData[1] & 0x7FFF; // MEAS_VBGxx
		RawMeasResults->u16VbgADC1B = rdData[2] & 0x7FFF;
	}

	// -----  burst read data  -----
	if (slaveIF_readReg(cid, CC_NB_SAMPLES, 3, &rdData[0]))
	{ // read register 0x2D..0x2F
		RawMeasResults->u16CCSamples = rdData[0];
		RawMeasResults->s32CCCounter = (int32_t)((rdData[1] << 16) | rdData[2]);
	}

	return !slaveIF_getError(NULL);
}
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
bool MC3377xGetStatus(uint8_t cid, TYPE_STATUS *Status)
{
	uint16_t rdData[13];

	// burst read data
	if (slaveIF_readReg(cid, CELL_OV_FLT, 2, &rdData[0]))
	{
		Status->u16CellOV = rdData[0];
		Status->u16CellUV = rdData[1];
	}
	if (slaveIF_readReg(cid, CB_OPEN_FLT, 13, &rdData[0]))
	{
		Status->u16CBOpen = 0;
		Status->u16CBShort = 0;
		Status->u16CBStatus = rdData[2];

		Status->u16GPIOStatus = 0;
		Status->u16ANOtUt = falg_temp;
		Status->u16GPIOOpen = 0;
		Status->u16IStatus = 0;
		Status->u16Comm = 0;
		Status->u16Fault1 = rdData[10];
		Status->u16Fault2 = rdData[11];
		Status->u16Fault3 = 0;
	}
	if (slaveIF_readReg(cid, MEAS_ISENSE2, 1, &rdData[0]))
	{
		Status->u16MeasIsense2 = rdData[0];
	}

	return !slaveIF_getError(NULL);
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
bool MC3377xGetSiliconRevision(uint8_t cid, SclusterInfo_t *pCluster)
{
	uint16_t rdData;

	pCluster->FRev = 0;
	pCluster->MRev = 0;

	if (cid > 15)
		return slaveIF_setError(ERR_WrongParam);

	if (slaveIF_readReg(cid, SILICON_REV, 1, &rdData))
	{
		pCluster->FRev = (uint8_t)(rdData >> 3) & 0x07;
		pCluster->MRev = (uint8_t)(rdData >> 0) & 0x07;
	}
	return !slaveIF_getError(NULL);
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
bool MC3377xGetSiliconType(uint8_t cid, SclusterInfo_t *pCluster)
{
	TypeReturn_t res;
	uint16_t rdData, bakData;
	bool bcc14;
	bool revA;

	// default return values in case of errors
	pCluster->Chip = Chip_Unknown;
	pCluster->NoCTs = 14;
	pCluster->pTagIdList = NULL;

	if (cid == 0)
	{
		// works with CID-00 => reset state
		slaveIF_readReg(cid, OV_UV_EN, 1, &rdData);
		bcc14 = (rdData == 0x3FFF); // 14cells 0x3FFF  ; 6cells  0x003F
		slaveIF_readReg(cid, SYS_CFG2, 1, &rdData);
		revA = (rdData & 0x4000); // bit14=1 => Rev A
	}
	else
	{
		if (CheckCID(cid))
			return slaveIF_setError(ERR_WrongParam);
		// assigned CID => need to test
		if (slaveIF_readReg(cid, OV_UV_EN, 1, &bakData))
		{ // to keep state
			slaveIF_writeReg(cid, OV_UV_EN, 0x3FFF, NULL);
			slaveIF_readReg(cid, OV_UV_EN, 1, &rdData);		// now check
			slaveIF_writeReg(cid, OV_UV_EN, bakData, NULL); // restore
			bcc14 = (rdData == 0x3FFF);
		}
		if (slaveIF_readReg(cid, INIT, 1, &bakData))
		{															 // to keep state
			slaveIF_writeReg(cid, INIT, bakData | (1UL << 5), NULL); // attempt to set Bit5
			slaveIF_readReg(cid, INIT, 1, &rdData);					 // now check
			slaveIF_writeReg(cid, INIT, bakData, NULL);				 // restore
			revA = (rdData & (1UL << 5)) == 0;						 // bit5=0 => Rev A
		}
	}

	pCluster->pTagIdList = NULL;

	if (!slaveIF_getError(&res))
	{
		if (bcc14)
		{
			pCluster->NoCTs = 14;
			pCluster->pTagIdList = _TAGID_BCC14;
			if (revA)
			{
				pCluster->Chip = Chip_MC33771A;
				// translate engineering chip revisions
				if ((pCluster->FRev == 7) && (pCluster->MRev == 7))
				{
					pCluster->FRev = 1;
					pCluster->MRev = 0;
				}
				if ((pCluster->FRev == 7) && (pCluster->MRev == 2))
				{
					pCluster->FRev = 2;
					pCluster->MRev = 0;
				}
				if ((pCluster->FRev == 7) && (pCluster->MRev == 3))
				{
					pCluster->FRev = 2;
					pCluster->MRev = 0;
				}
				if ((pCluster->FRev == 6) && (pCluster->MRev == 2))
				{
					pCluster->FRev = 3;
					pCluster->MRev = 1;
				}
				if ((pCluster->FRev == 6) && (pCluster->MRev == 6))
				{
					pCluster->FRev = 3;
					pCluster->MRev = 1;
				}
				if ((pCluster->FRev == 6) && (pCluster->MRev == 1))
				{
					pCluster->FRev = 3;
					pCluster->MRev = 2;
				}

				if (pCluster->FRev == 2)
				{
					pCluster->pTagIdList = _TAGID_BCC14p2;
				}
			}
			else
			{
				pCluster->Chip = Chip_MC33771B;
			}
		}
		else
		{
			pCluster->NoCTs = 6;
			pCluster->pTagIdList = NULL;
			pCluster->pTagIdList = _TAGID_BCC6;
			if (revA)
			{
				pCluster->Chip = Chip_MC33772A;
				// workaround ......
				// translate engineering chip revisions
				if ((pCluster->FRev == 7) && (pCluster->MRev == 0))
				{
					//					pCluster->u8FRev = 2; pCluster->u8MRev = 0;					// V2.99 workaround
					pCluster->Chip = Chip_MC33772BM;
				}
			}
			else
			{
				pCluster->Chip = Chip_MC33772B;
			}
		}
	}
	return !slaveIF_getError(NULL);
}
// ----------------------------------------------------------------------------

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
bool MC3377xGetThresholds(uint8_t cid, uint8_t NoCTs, TYPE_THRESHOLDS *Threshold)
{
	uint16_t rdData[29];
	uint8_t u8Idx;

	// -----  TH_ALL_OV_UV  -----
	if (slaveIF_readReg(cid, TH_ALL_CT, 1, &rdData[0]))
	{
		Threshold->u8ThAllOv = (uint8_t)(rdData[0] >> 8);
		Threshold->u8ThAllUv = (uint8_t)(rdData[0] >> 0);
	}
	// -----  TH_CT[14..1]_OV_UV  -----
	if (slaveIF_readReg(cid, TH_CT1 - NoCTs + 1, NoCTs, &rdData[0]))
	{
		u8Idx = NoCTs;
		while (u8Idx)
		{
			u8Idx--;
			Threshold->u8ThCTxOv[u8Idx] = (uint8_t)(rdData[NoCTs - 1 - u8Idx] >> 8); // reverse order
			Threshold->u8ThCTxUv[u8Idx] = (uint8_t)(rdData[NoCTs - 1 - u8Idx] >> 0); // reverse order
		}
	}
	// -----  TH_ANxOT[6..0] -----
	if (slaveIF_readReg(cid, TH_AN6_OT, 7, &rdData[0]))
	{
		u8Idx = 7;
		while (u8Idx)
		{
			u8Idx--;
			Threshold->u10ThANxOT[u8Idx] = rdData[7 - 1 - u8Idx]; // reverse order
		}
	}
	// -----  TH_ANxUT[6..0]-----
	if (slaveIF_readReg(cid, TH_AN6_UT, 7, &rdData[0]))
	{
		u8Idx = 7;
		while (u8Idx)
		{
			u8Idx--;
			Threshold->u10ThANxUT[u8Idx] = rdData[7 - 1 - u8Idx]; // reverse order
		}
	}
	// -----  TH_ISENSE_OC, TH_COULOMB)CNT -----
	if (slaveIF_readReg(cid, TH_ISENSE_OC, 3, &rdData[0]))
	{
		Threshold->u12ThIsenseOC = rdData[0];
		Threshold->u32ThCoulombCnt = ((uint32_t)rdData[1] << 16) | ((uint32_t)rdData[2] << 0);
	}
	return !slaveIF_getError(NULL);
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
bool MC3377xSleepMode(TYPE_INTERFACE interface)
{

	if (interface == IntSPI)
	{
		return slaveIF_writeReg(CIDassiged, SYS_CFG_GLOBAL, SYS_CFG_GLOBAL_GO2SLEEP, NULL); // sleep command
	}
	else if (interface == IntTPL)
	{
		if (slaveIF_writeGlobalReg(SYS_CFG_GLOBAL, SYS_CFG_GLOBAL_GO2SLEEP))
		{								  // sleep command (must be global write for TPL)
			slaveIF_transceiverDisable(); // disable TPL
			return true;
		}
		return !slaveIF_getError(NULL);
	}
	else
	{
		return slaveIF_setError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief Transitions to normal mode.
 *
 * Implementation is interface dependent.
 *
 * <b>SPI interface</b> \n
 * - Calls \ref slaveIF_wakeUp
 *
 * <b>TPL interface</b> \n
 * - Calls \ref slaveIF_transceiverEnable
 * - Calls \ref slaveIF_wakeUp
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
bool MC3377xNormalMode(TYPE_INTERFACE interface)
{

	if (interface == IntSPI)
	{
		slaveIF_wakeUp();
		//		Delayms(WAIT_AFTER_WAKEUP);
		return true;
	}
	else if (interface == IntTPL)
	{
		slaveIF_transceiverEnable();
		slaveIF_wakeUp();
		//		Delayms(WAIT_AFTER_WAKEUP);
		return true;
	}
	else
	{
		return slaveIF_setError(ERR_WrongInterface);
	}
}
// ----------------------------------------------------------------------------
/*! \brief Checks if a MC33771B has woken up.
 *
 * Implementation is interface dependent.
 *
 * <b>SPI interface</b> \n
 * - Wake-up has occurred if FAULT pin is high (\ref slaveIF_faultPinStatus)
 *
 * <b>TPL interface</b> \n
 * - Wake-up has occurred if INTB pin is low (\ref slaveIF_IntbPinStatus)
 * 		- waits for 150us
 * 		- enables TPL (\ref slaveIF_transceiverEnable)
 * 		- send wakeup pattern (\ref slaveIF_wakeUp)
 *
 * @param interface  Interface used (SPI or TPL)
 *
 * @return \b true   if wakeup was detected
 * @return \b false  if no wakeup was detected or in case of errors
 *
 * \remarks
 * Sets \ref ERR_WrongInterface in case interface is unknown.
 */
bool MC3377xCheck4Wakeup(TYPE_INTERFACE interface)
{

	if (interface == IntSPI)
	{
		if (slaveIF_faultPinStatus())
		{
			//		slaveIF_wakeUp();													// wakeup BCC e.g. required if FAULT is enabled but WAKEUP is not
			return true;
		}
		return false;
	}
	else if (interface == IntTPL)
	{
		if (slaveIF_IntbPinStatus() == 0)
		{ // check wakeup = INTB pin low
			Delay(DELAY_150us);
			slaveIF_transceiverEnable();
			slaveIF_wakeUp();
			return true;
		}
		return false;
	}
	else
	{
		return slaveIF_setError(ERR_WrongInterface);
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
bool MC3377xReadFuseMirror(uint8_t cid, TYPE_FUSE_DATA *fusedata)
{
	uint16_t rdData;
	uint8_t u8Adr;

#define FUSE_MIRROR_CTRL_FSTM BIT(4)

	for (u8Adr = 0; u8Adr < 32; u8Adr++)
	{
		slaveIF_writeReg(cid, FUSE_MIRROR_CTRL, (u8Adr << 8), &rdData);
		slaveIF_readReg(cid, FUSE_MIRROR_DATA, 1, &(fusedata->u16Data[u8Adr]));
	}
	return !slaveIF_getError(NULL);
}
