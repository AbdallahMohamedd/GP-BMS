/**
 * @file        dataBase.c
 * @brief       Implementation of the BMS Database module.
 *
 * @details     This file contains the implementation for managing Battery Management System (BMS) data,
 *              including initialization, configuration, measurement reading, and status retrieval
 *              from MC3377x devices.
 *
 * @note        Project: Graduation Project - Battery Management System
 * @note        Engineer: Abdullah Mohamed
 * @note        Component: BMS DataBase module
 */

//=============================================================================
// Includes
//=============================================================================
#include "source/COTs/BMSDataBase/Inc/database.h"

//=============================================================================
// Lookup table
//=============================================================================
typedef struct
{
	float soc; // SoC in percentage (0.0 to 100.0)
	float ocv; // OCV in volts
} SOC_OCV_Pair;

// Lookup Table (50 points from Excel data)
#define TABLE_SIZE 50
SOC_OCV_Pair soc_ocv_table[TABLE_SIZE] = {
	{0.0539138, 2.56124},  // SoC = 0.0539138053%, OCV = 2.56124 V
	{2.0076153, 2.98329},  // SoC = 2.007615325%, OCV = 2.98329 V
	{4.0428615, 3.22326},  // SoC = 4.0428614752%, OCV = 3.22326 V
	{6.0981076, 3.27666},  // SoC = 5.9965629949%, OCV = 3.27666 V
	{8.1133538, 3.30754},  // SoC = 8.1133537757%, OCV = 3.30754 V
	{10.1282620, 3.33135}, // SoC = 10.1282629646%, OCV = 3.33135 V
	{12.1835091, 3.35966}, // SoC = 12.1835091148%, OCV = 3.35966 V
	{14.2387553, 3.38732}, // SoC = 14.238755265%, OCV = 3.38732 V
	{16.2940014, 3.41563}, // SoC = 16.2940014152%, OCV = 3.41563 V
	{18.3492476, 3.44201}, // SoC = 18.3492476347%, OCV = 3.44201 V
	{20.4044938, 3.46710}, // SoC = 20.4044938444%, OCV = 3.46710 V
	{22.4597400, 3.48447}, // SoC = 22.4597405738%, OCV = 3.48447 V
	{24.5149862, 3.50313}, // SoC = 24.5149867241%, OCV = 3.50313 V
	{26.5702323, 3.51985}, // SoC = 26.5702325435%, OCV = 3.51985 V
	{28.6254785, 3.53401}, // SoC = 28.6254787325%, OCV = 3.53401 V
	{30.6807247, 3.54752}, // SoC = 30.6807245519%, OCV = 3.54752 V
	{32.7359709, 3.55910}, // SoC = 32.7359704822%, OCV = 3.55910 V
	{34.7912171, 3.57068}, // SoC = 34.7912171218%, OCV = 3.57068 V
	{36.8464633, 3.58162}, // SoC = 36.846463372%, OCV = 3.58162 V
	{38.9017094, 3.59320}, // SoC = 38.9017091915%, OCV = 3.59320 V
	{40.9569556, 3.60413}, // SoC = 40.956955011%, OCV = 3.60413 V
	{43.0122018, 3.61571}, // SoC = 42.9732688614%, OCV = 3.61571 V
	{45.0674480, 3.62794}, // SoC = 44.9897226809%, OCV = 3.62794 V
	{47.1226942, 3.64145}, // SoC = 47.0249688311%, OCV = 3.64145 V
	{49.1779403, 3.65560}, // SoC = 49.1414226505%, OCV = 3.65560 V
	{51.2331865, 3.67104}, // SoC = 51.1766688008%, OCV = 3.67104 V
	{53.2884327, 3.68713}, // SoC = 53.1303703205%, OCV = 3.68713 V
	{55.3436789, 3.70643}, // SoC = 55.0028641709%, OCV = 3.70643 V
	{57.3989250, 3.73731}, // SoC = 57.6075748896%, OCV = 3.73731 V
	{59.4541712, 3.75854}, // SoC = 59.4800687401%, OCV = 3.75854 V
	{61.5094174, 3.77849}, // SoC = 61.4337702598%, OCV = 3.77849 V
	{63.5646636, 3.80165}, // SoC = 63.8760656401%, OCV = 3.80165 V
	{65.6199098, 3.81837}, // SoC = 65.7482225292%, OCV = 3.81837 V
	{67.6751559, 3.83382}, // SoC = 67.6207163797%, OCV = 3.83382 V
	{69.7304021, 3.85183}, // SoC = 69.6556255686%, OCV = 3.85183 V
	{71.7856483, 3.86727}, // SoC = 71.6093270883%, OCV = 3.86727 V
	{73.8408945, 3.88207}, // SoC = 73.6002763083%, OCV = 3.88207 V
	{75.8961407, 3.89880}, // SoC = 75.517067089%, OCV = 3.89880 V
	{77.9513868, 3.91617}, // SoC = 77.5523132392%, OCV = 3.91617 V
	{80.0066330, 3.94705}, // SoC = 79.8899147488%, OCV = 3.94705 V
	{82.0618792, 3.95927}, // SoC = 82.0295178084%, OCV = 3.95927 V
	{84.1171254, 3.98050}, // SoC = 84.0647639586%, OCV = 3.98050 V
	{86.1723716, 4.00366}, // SoC = 86.1000101088%, OCV = 4.00366 V
	{88.2276177, 4.02618}, // SoC = 88.1349192978%, OCV = 4.02618 V
	{90.2828639, 4.04741}, // SoC = 90.170165448%, OCV = 4.04741 V
	{92.3381101, 4.06800}, // SoC = 92.5309161977%, OCV = 4.06800 V
	{94.3933563, 4.08280}, // SoC = 94.4034100482%, OCV = 4.08280 V
	{96.4486024, 4.09824}, // SoC = 96.3571115679%, OCV = 4.09824 V
	{98.5038486, 4.12076}, // SoC = 98.4735653873%, OCV = 4.12076 V
	{100.0000000, 4.18398} // SoC = 100.9967314756% (clamped), OCV = 4.18398 V
};
//=============================================================================
// Global Variables
//=============================================================================
static uint32_t prev_time = 0;	  //!< Static variable to store previous timestamp for SOC/SOH calculations.
static uint8_t SOC;				  //!< Static variable to store State of Charge.
static long prev_ccounter = 0;	  //!< Static variable to store previous Coulomb counter value.
static uint16_t prev_samples = 0; //!< Static variable to store previous samples value for Coulomb counter.

//=============================================================================
// Function Definitions
//=============================================================================
/**
 * @brief Initializes the BMS system.
 * @details Initializes the BMS system with the specified number of nodes.
 *          For all nodes:
 *          - Clears errors (slaveIF_clearError).
 *          - Sends wakeup command.
 *          - Checks for unassigned nodes and assigns CID (starting with 1, counting up).
 *          - Closes the TPL bus switch (except for the last CID in the chain).
 * @param NoOfNodes Number of nodes in the chain (1..15).
 * @return bool True if successful, false in case of errors.
 * @note Works for TPL and SPI (only one node) interface.
 */
bool dataBase_BMSInit(uint8_t NoOfNodes)
{
	uint8_t cid;

	if (NoOfNodes == 0)
		return false;
	if (NoOfNodes > MAX_CLUSTER)
		return false;

	for (cid = 1; cid <= NoOfNodes; cid++)
	{
		slaveIF_clearError();
		slaveIF_wakeUp(); // Send wakeup in case next device is in idle mode
		if (slaveIF_readReg(CIDunassiged, INIT, 1, NULL))
		{															   // Check if somebody is there / read Init register
			slaveIF_writeReg(CIDunassiged, INIT, (uint16_t)cid, NULL); // Assign CID
			if (cid < NoOfNodes)
			{													// All nodes except last one
				slaveIF_writeReg(cid, INIT, INIT_BUS_SW, NULL); // Close switch	(except last node)
			}
			else
			{
				slaveIF_writeReg(cid, INIT, 0x0000, NULL); // Open switch	(last node)
			}
		}
	}
	return !slaveIF_getError(NULL);
}

/**
 * @brief Performs basic MC33771B configuration.
 * @details Loads the configuration list `conf` into the registers of the node specified by `cid`.
 * @param cid Cluster ID to be configured.
 * @param conf Pointer to the configuration list. A list entry with `regAdr` equal to 0 terminates the list.
 * @return bool True if successful, false if not successful.
 */
bool dataBase_bmsConfig(uint8_t cid, const SsysConf_t conf[])
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

/**
 * @brief Starts an ADC conversion.
 * @details Sets the SOC bit in the ADC_CFG register to initiate an ADC conversion.
 * @param cid Cluster ID.
 * @param tagID TagID for ADC measurement.
 * @return bool True if successful, false if not successful.
 */
bool dataBase_startConvADC(uint8_t cid, uint8_t tagID)
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

/**
 * @brief Checks the status of End Of Conversion (EOC).
 * @details Checks the status of the conversion of the cluster `cid` by reading
 *          the SOC/nEOC bit in the ADC_CFG register.
 * @param cid Cluster ID.
 * @return bool True if conversion is still ongoing, false if conversion is complete or an error occurred.
 * @note Returns false in case of errors:
 *       - `cid` out of range (sets ERR_WrongParam).
 *       - Reading ADC_CFG register failed.
 */
bool dataBase_ADCIsConverting(uint8_t cid)
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

/**
 * @brief Reads temperature data from thermal sensors.
 * @details This function reads raw data from various thermal channels and stores them
 *          in the `u16ANVoltage` array of the `RawMeasResults` structure.
 * @param RawMeasResults Pointer to a `TYPE_MEAS_RESULTS_RAW` structure to store the raw measurement results.
 * @return bool Always returns true, as error handling for `thermalManager_readRawData` is not implemented here.
 */
bool dataBase_getTempRawData(TYPE_MEAS_RESULTS_RAW *RawMeasResults)
{
	RawMeasResults->u16ANVoltage[0] = thermalManager_readRawData(PTB0_Channel);
	RawMeasResults->u16ANVoltage[1] = thermalManager_readRawData(PTB1_Channel);
	RawMeasResults->u16ANVoltage[2] = thermalManager_readRawData(PTD1_Channel);
	RawMeasResults->u16ANVoltage[3] = thermalManager_readRawData(PTD5_Channel);
	RawMeasResults->u16ANVoltage[4] = thermalManager_readRawData(PTD6_Channel);
	RawMeasResults->u16ANVoltage[5] = thermalManager_readRawData(PTD6_Channel);
	RawMeasResults->u16ANVoltage[6] = thermalManager_readRawData(PTD0_Channel);
}

/**
 * @brief Reads measurement registers and returns them as raw data.
 * @details Reads various measurement registers (current, stack voltage, cell voltages, IC temperature, band gap) and
 *          Coulomb counter data from the MC3377x device and populates the `RawMeasResults` structure.
 * @param cid Cluster ID to handle.
 * @param tagId TagID to be used to check reading against.
 * @param NoCTs Number of cell terminals to handle.
 * @param RawMeasResults Pointer to a `TYPE_MEAS_RESULTS_RAW` structure to store the raw measurement results.
 * @return bool True if successful, false if not successful.
 * @remarks The Measurement data is only valid if successfully executed (true). There is no check of DataReady (Bit15) implemented!
 */
bool dataBase_getRawData(uint8_t cid, uint8_t tagId, uint8_t NoCTs, TYPE_MEAS_RESULTS_RAW *RawMeasResults)
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

/**
 * @brief Reads the status registers and returns them in the Status structure.
 * @details Reads various status registers (cell OV/UV, CB faults, GPIO status, AN temp, ISense, Comm, Faults) and
 *          populates the `Status` structure.
 * @param cid Cluster ID to handle.
 * @param Status Pointer to a `TYPE_STATUS` structure to store the status information.
 * @return bool True if successful, false if not successful.
 * @remarks The Status data is only valid if successfully executed (true).
 */
bool dataBase_getStatus(uint8_t cid, TYPE_STATUS *Status)
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

/**
 * @brief Reads the silicon revision of the device.
 * @details Reads the SILICON_REV register and retrieves full and mask revisions of the device.
 * @param cid Cluster ID to handle.
 * @param pCluster Pointer to a `SclusterInfo_t` structure to fill with FRev and MRev.
 * @return bool True if successful, false if not successful.
 * @remarks In case of not successful execution, FRev and MRev will be set to 0.
 */
bool dataBase_getSiliconRevision(uint8_t cid, SclusterInfo_t *pCluster)
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

/**
 * @brief Reads the silicon type of the CID.
 * @details Evaluates the silicon type of the CID. The routine uses a different algorithm
 *          depending on whether the device is in reset state (registers are in reset state)
 *          or if it has to be assumed that the registers might already be modified.
 *
 *          <b>Register in Reset state (cid = 0)</b>
 *          - Reads OV_UV_EN and SYS_CFG2 registers.
 *          - If OV_UV_EN = 0x3FFF => MC33771 (otherwise MC33772).
 *          - If SYS_CFG2 bit14 = 1 => Rev A (otherwise Rev B).
 *
 *          <b>Register state unknown</b>
 *          - Stores OV_UV_EN, attempts to write 0x3FFF, reads and compares with 0x3FFF, restores OV_UV_EN.
 *          - If OV_UV_EN = 0x3FFF => MC33771 (otherwise MC33772).
 *          - Stores INIT, attempts to set bit 5, reads and compares with 0x3FFF, restores INIT.
 *          - If INIT bit5 = 0 => Rev A (otherwise Rev B).
 * @param cid Cluster ID to handle.
 * @param pCluster Pointer to a `SclusterInfo_t` structure to fill with Chip, TagIDlist, and NoCTs.
 * @return bool True if successful, false if not successful.
 * @remarks The cluster's silicon revision must be known! (`dataBase_getSiliconRevision()` must be called first!)
 *          Also translates engineering versions to normal versions:
 *          | device  | engineering version | translated version |
 *          |:-------:|:-------------------:|:-------------------|
 *          | MC33771 |       7.7           |        1.0         |
 *          | MC33771 |       7.2, 7.3      |        2.0         |
 *          | MC33771 |       6.2, 6.6      |        3.1         |
 *          | MC33771 |       6.1           |        3.2         |
 */
bool dataBase_getSiliconType(uint8_t cid, SclusterInfo_t *pCluster)
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

/**
 * @brief Reads the threshold registers and returns them in the Threshold structure.
 * @details Reads various threshold registers (cell OV/UV, ANx OT/UT, ISense OC, Coulomb Counter) and
 *          populates the `Threshold` structure.
 * @param cid Cluster ID to handle.
 * @param NoCTs Number of cell terminals to handle.
 * @param Threshold Pointer to a `TYPE_THRESHOLDS` structure to store the thresholds.
 * @return bool True if successful, false if not successful.
 * @remarks The Threshold data is only valid if successfully executed (true).
 */
bool dataBase_getThresholds(uint8_t cid, uint8_t NoCTs, TYPE_THRESHOLDS *Threshold)
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

/**
 * @brief Reads fuse mirror memory data from the MC3377x device.
 * @details Reads 32 words (16-bit) of fuse mirror memory data from the device.
 * @param cid Cluster ID to handle.
 * @param fusedata Pointer to a `TYPE_FUSE_DATA` structure to store the fuse data.
 * @return bool True if successful, false if not successful.
 */
bool dataBase_readFuseMirror(uint8_t cid, TYPE_FUSE_DATA *fusedata)
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

/**
 * @brief Puts the MC3377x device into sleep mode.
 * @param interface The communication interface (e.g., SPI, TPL).
 * @return bool True if successful, false otherwise.
 */
bool dataBase_sleepMode(TYPE_INTERFACE interface)
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

/**
 * @brief Puts the MC3377x device into normal operating mode.
 * @param interface The communication interface (e.g., SPI, TPL).
 * @return bool True if successful, false otherwise.
 */
bool dataBase_normalMode(TYPE_INTERFACE interface)
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

/**
 * @brief Checks if the MC3377x device has woken up from sleep mode.
 * @param interface The communication interface (e.g., SPI, TPL).
 * @return bool True if the device has woken up, false otherwise.
 */
bool dataBase_check4Wakeup(TYPE_INTERFACE interface)
{

	if (interface == IntSPI)
	{
		if (slaveIF_faultPinStatus())
		{
			//		slaveIF_wakeUp();	// wakeup BCC e.g. required if FAULT is enabled but WAKEUP is not
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

/**
 * @brief Calculates the initial State of Charge (SOC) for the battery pack using the Open Circuit Voltage (OCV) method.
 * @param ocv_values Array of open circuit voltage values for each cell (14 cells).
 * @return float Initial State of Charge (SOC) for the pack.
 */
float dataBase_initialSOC_Pack(float ocv_values[14])
{
	float soc_values[14];
	for (int i = 0; i < 14; i++)
	{
		ocv_values[i] = ocv_values[i] * 0.00015258789;
	}
	// Calculate SOC for each OCV
	for (uint32_t j = 0; j < 14; j++)
	{
		float ocv = ocv_values[j];

		// Handle out-of-bounds OCV
		if (ocv <= soc_ocv_table[0].ocv)
		{
			soc_values[j] = soc_ocv_table[0].soc;
			continue;
		}
		if (ocv >= soc_ocv_table[TABLE_SIZE - 1].ocv)
		{
			soc_values[j] = soc_ocv_table[TABLE_SIZE - 1].soc;
			continue;
		}

		// Find the interval and perform linear interpolation
		for (uint32_t i = 0; i < TABLE_SIZE - 1; i++)
		{
			if (ocv >= soc_ocv_table[i].ocv && ocv < soc_ocv_table[i + 1].ocv)
			{
				float ocv1 = soc_ocv_table[i].ocv;
				float ocv2 = soc_ocv_table[i + 1].ocv;
				float soc1 = soc_ocv_table[i].soc;
				float soc2 = soc_ocv_table[i + 1].soc;
				soc_values[j] = soc1 + (soc2 - soc1) * (ocv - ocv1) / (ocv2 - ocv1);
				break;
			}
		}
	}

	// Calculate average SOC
	float sum = 0.0;
	for (uint32_t j = 0; j < 14; j++)
	{
		sum += soc_values[j];
	}
	return sum / 14.0;
}

/**
 * @brief Calculates the initial State of Charge (SOC) for a single cell using the Open Circuit Voltage (OCV) method.
 * @param ocv Open circuit voltage of the cell.
 * @return float Initial State of Charge (SOC) for the cell.
 */
float dataBase_initialSOC_Cell(float ocv)
{
	// Handle out-of-bounds OCV
	if (ocv <= soc_ocv_table[0].ocv)
		return soc_ocv_table[0].soc;
	if (ocv >= soc_ocv_table[TABLE_SIZE - 1].ocv)
		return soc_ocv_table[TABLE_SIZE - 1].soc;

	// Find the interval
	for (uint32_t i = 0; i < TABLE_SIZE - 1; i++)
	{
		if (ocv >= soc_ocv_table[i].ocv && ocv < soc_ocv_table[i + 1].ocv)
		{
			float ocv1 = soc_ocv_table[i].ocv;
			float ocv2 = soc_ocv_table[i + 1].ocv;
			float soc1 = soc_ocv_table[i].soc;
			float soc2 = soc_ocv_table[i + 1].soc;
			// Linear interpolation
			return soc1 + (soc2 - soc1) * (ocv - ocv1) / (ocv2 - ocv1);
		}
	}
	return 0.0; // Should not reach here
}

/**
 * @brief Calculates the State of Charge (SOC) using Coulomb counting.
 * @details This function estimates the SOC based on current measurements and time.
 * @param rawResults Raw measurement data including Coulomb counter.
 * @param voltage Current voltage reading of the battery.
 * @param current_time Current timestamp.
 * @return float Calculated State of Charge (SOC).
 */
float dataBase_calculateSOC(TYPE_MEAS_RESULTS_RAW rawResults, uint16_t voltage, uint32_t current_time)
{
	float capacityAh;

	if (voltage > 10)
	{
		capacityAh = PACK_BATTERY_CAPACITY_AH;
	}
	else
	{
		capacityAh = CELL_BATTERY_CAPACITY_AH;
	}

	// Calculate charge difference using Coulomb counter
	float delta_charge_coulombs = (float)(rawResults.s32CCCounter) * 0.0001;  // Assuming V2RES = 0.0001V, adjust if different
	float delta_time_seconds = (current_time - prev_time) / 1000.0;			  // Convert ms to seconds
	float deltaSOC = (delta_charge_coulombs / (capacityAh * 3600.0)) * 100.0; // Convert to Ah and percentage

	SOC += deltaSOC;
	prev_time = current_time;

	//     if (SOC > 100.0)
	//         SOC = 100.0;
	//     else if (SOC < 0.0)
	//         SOC = 0.0;

	return SOC;
}

/**
 * @brief Calculates the State of Health (SOH) using Coulomb counter data.
 * @details This function estimates the SOH based on the accumulated charge/discharge and nominal capacity.
 * @param rawResults Raw measurement data including Coulomb counter.
 * @param voltage Current voltage reading of the battery.
 * @param nominal_capacity_mAh Nominal capacity of the battery in mAh.
 * @param v2res Reference voltage resolution (e.g., for converting raw ADC values).
 * @param current_time Current timestamp.
 * @param prev_time Previous timestamp.
 * @return float Calculated State of Health (SOH).
 */
float dataBase_calculateSOH(TYPE_MEAS_RESULTS_RAW rawResults, float voltage, float nominal_capacity_mAh,
							float v2res, uint32_t current_time, uint32_t prev_time)
{
	float capacityAh;

	// Determine nominal capacity based on voltage (pack or cell)
	if (voltage > 10)
	{
		capacityAh = nominal_capacity_mAh / 1000.0; // Convert mAh to Ah for pack (21 Ah for 14 cells)
	}
	else
	{
		capacityAh = nominal_capacity_mAh / (1000.0 * 14.0); // Convert mAh to Ah per cell (1.5 Ah)
	}

	// Initialize prev_time and prev values on first call
	if (prev_time == 0)
	{
		prev_time = current_time;
		prev_ccounter = rawResults.s32CCCounter;
		prev_samples = rawResults.u16CCSamples;
		return 100.0; // Initial SOH assumed 100% until measured
	}

	// Calculate time difference in seconds
	float delta_time_seconds = (current_time - prev_time) / 1000.0;

	// Check if samples are available to avoid division by zero
	if (rawResults.u16CCSamples > 0 && prev_samples > 0)
	{
		// Calculate change in Coulomb counter and samples
		long delta_ccounter = rawResults.s32CCCounter - prev_ccounter;
		uint16_t delta_samples = rawResults.u16CCSamples - prev_samples;

		// Calculate average current in Amps using V2RES
		float average_current = (delta_ccounter * v2res) / delta_samples;
		float charge_ah = (average_current * delta_time_seconds) / 3600.0; // Convert to Ah

		// Calculate current capacity based on accumulated charge
		float current_capacity_ah = capacityAh - (charge_ah / 1000.0); // Adjust based on discharge

		// Calculate SOH as a percentage of nominal capacity
		float soh = (current_capacity_ah / capacityAh) * 100.0;

		// Ensure SOH is within valid range
		if (soh > 100.0)
			soh = 100.0;
		else if (soh < 0.0)
			soh = 0.0;

		// Update previous values
		prev_ccounter = rawResults.s32CCCounter;
		prev_samples = rawResults.u16CCSamples;
		prev_time = current_time;

		// Return SOH
		return soh;
	}
	else
	{
		// Return previous SOH if no valid samples
		return 100.0; // Default to 100% if no change detected
	}
}
