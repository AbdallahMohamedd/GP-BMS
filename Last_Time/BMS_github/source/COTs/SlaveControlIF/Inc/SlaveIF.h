/**
 * @file SlaveIF.h
 * @brief Public interface header for the Slave Interface (SlaveIF) driver.
 *
 * @details This header defines the public APIs, register addresses, bit definitions,
 *          data structures, and enums for interfacing with the SlaveIF module
 *          (MC33771x/MC33772x) via SPI or TPL communication. It provides functions
 *          for initialization, configuration, and data reading/writing.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Slave_Control_IF driver
 */

#ifndef SLAVE_IF_H
#define SLAVE_IF_H

//=============================================================================
// Includes
//=============================================================================
#include <COTs/DebugInfoManager/Inc/debugInfo.h>
#include "tpm1.h" // For delay functions
#include "spi.h"  // For SPI types and functions

//=============================================================================
// External Variables
//=============================================================================
extern TYPE_INTERFACE _interface; //!< Local copy of interface type.
extern TYPE_EVB _evb;			  //!< Local copy of EVB type.

//=============================================================================
// Defines and Macros
//=============================================================================
#define MSGLEN (5U)		 //!< Number of bytes in a message (5*8 = 40 bits).
#define GLOBAL_CID (0U)	 //!< Cluster ID for global write commands.
#define MAX_CLUSTER (2U) //!< Maximum number of clusters (1..15, CID=0 reserved).

// --- Data Unpacking Macros ---
#define UNPACK_REGADR(v) ((uint8_t)((v)[2] & 0x7F))											   //!< Extracts Register Address from received frame.
#define UNPACK_DATA(v) ((uint16_t)(((v)[4] << 8) | (v)[3]))									   //!< Extracts Data field from received frame.
#define UNPACK_TAGID(v) ((uint8_t)((v)[1] & 0x0F))											   //!< Extracts TagID field from received frame.
#define UNPACK_RC(v) ((uint8_t)(((v)[1] & 0x0C) >> 2))										   //!< Extracts RC field from received frame.
#define IS_NULL_RESPONSE(v) (((v)[4] == 0) && ((v)[3] == 0) && ((v)[2] == 0) && ((v)[1] == 0)) //!< Checks for NULL response (all fields except CRC are 0).

// --- Register Bit Definitions ---
#define INIT_BUS_SW (BIT(4))			  //!< Bus Switch Control
#define SYS_CFG_GLOBAL_GO2SLEEP (BIT(0))  //!< Goto sleep bit
#define SYS_CFG1_CB_DRVEN (BIT(7))		  //!< General enable or disable for all cell balance drivers.
#define SYS_CFG1_I_MEAS_EN (BIT(9))		  //!< ISENSE measurement enable bit
#define SYS_CFG1_GODIAG (BIT(6))		  //!< Goto diag bit
#define SYS_CFG1_CB_MANUAL_PAUSE (BIT(5)) //!< Cell balancing manual pause
#define SYS_CFG1_SOFT_RST (BIT(4))		  //!< Soft reset bit
#define SYS_DIAG_DA_DIAG (BIT(7))		  //!< Diagnostics Differential amplifier diagnostic. Diagnostic mode function only
#define SYS_DIAG_POLARITY (BIT(6))		  //!< Control bit used in terminal leakage detection. Controls the polarity between the level shifter and the ADC1-A and ADC1-B converters
#define SYS_DIAG_CT_LEAK_DIAG (BIT(5))	  //!< Control bit used in terminal leakage detection. Commands the MUX to route the CTx/CBx pin to ADC1-A,B converters. This bit must be exclusive vs. DA_DIAG
#define SYS_DIAG_CT_OV_UV (BIT(4))		  //!< Diagnostics Cell Terminal OV/UV bit
#define SYS_DIAG_CT_OL_ODD (BIT(3))		  //!< Diagnostics Cell Terminal Open load Odd bit
#define SYS_DIAG_CT_OL_EVEN (BIT(2))	  //!< Diagnostics Cell Terminal Open load Even bit
#define ADC_CFG_SOC (BIT(11))			  //!< ADC Start of Conversion bit
#define MEAS_DATA_RDY (BIT(15))			  //!< Data Ready Bit
#define CB_CFG_CB_EN (BIT(9))			  //!< Cell Balance enable

//=============================================================================
// Typedefs
//=============================================================================
/**
 * @brief Structure for MC3377x configuration list entry.
 * @details Defines register address and value pairs for configuration loading.
 * @note Last entry must be {0, 0} to indicate end of list.
 */
typedef struct
{
	uint8_t regAdr;	   //!< Register address to be written.
	uint16_t regValue; //!< Register value to write.
} TYPE_BCC_CONF;

/**
 * @brief 128-bit array indicating TagID or RC format for registers.
 * @details Each bit indicates whether a register uses TagID (1) or RC (0) format.
 */
typedef struct
{
	uint16_t w[8]; //!< 128-bit array for TagID/RC indication.
} TYPE_TAGIDLIST;

/**
 * @brief Structure to hold cluster information.
 * @details Contains chip type, GUID, revisions, and TagID list pointer.
 */
typedef struct
{
	int64_t Guid;				//!< Global Unique ID.
	uint8_t Chip;				//!< Chip type (see TYPE_CHIP).
	uint8_t FRev;				//!< Full mask revision.
	uint8_t MRev;				//!< Metal mask revision.
	uint8_t NoCTs;				//!< Number of Cell Terminals (6 or 14 supported).
	const uint16_t *pTagIdList; //!< Pointer to TagID list.
} LLD_TYPE_CLUSTER;

//=============================================================================
// Enums
//=============================================================================
/**
 * @brief Enum for supported chip types.
 */
typedef enum
{
	Chip_Unknown = 0,	//!< Unknown chip.
	Chip_MC33771A = 1,	//!< MC33771A, 14-channel device.
	Chip_MC33772A = 2,	//!< MC33772A, 6-channel device.
	Chip_MC33771BM = 3, //!< MC33771B-M, 14-channel device.
	Chip_MC33772BM = 4, //!< MC33772B-M, 6-channel device.
	Chip_MC33771B = 5,	//!< MC33771B, 14-channel device.
	Chip_MC33772B = 6	//!< MC33772B, 6-channel device.
} TYPE_CHIP;

/**
 * @brief Enum for function return values.
 */
typedef enum
{
	RETURN_OK = 0x00,		   //!< No error.
	ERR_TX = 0x01,			   //!< Transmission error (TPL only).
	ERR_NoResponse = 0x02,	   //!< No bytes received.
	ERR_ResponseLen = 0x03,	   //!< Wrong number of bytes received.
	ERR_ResponseCRC = 0x04,	   //!< CRC error in received data.
	ERR_NullResponse = 0x05,   //!< Null response (all data bytes 0x00).
	ERR_WrongParam = 0x06,	   //!< Invalid parameters passed.
	ERR_ResponseTagId = 0x07,  //!< Response Tag ID mismatch.
	ERR_ResponseRC = 0x08,	   //!< Response RC mismatch.
	ERR_ResponseAdr = 0x09,	   //!< Response register address mismatch.
	ERR_WrongInterface = 0x0A, //!< Unknown or invalid interface.
	ERR_Timeout = 0x0B		   //!< Timeout error.
} LLD_TYPE_RETURN;

/**
 * @brief Enum for Cluster IDs (SPI version).
 */
typedef enum
{
	CIDunassiged = 0, //!< Unassigned node (reacts to CID 0 and INIT register).
	CIDassiged = 1	  //!< Assigned CID for SPI (CID 1).
} LLD_TYPE_CID;

/**
 * @brief Enum for commands (SPI version).
 */
typedef enum
{
	CmdNOP = 0x0,	  //!< No Operation command (not supported).
	CmdRdLocal = 0x1, //!< Local Read command.
	CmdWrLocal = 0x2, //!< Local Write command.
	CmdWrGlobal = 0x3 //!< Global Write command (not allowed for SPI).
} LLD_TYPE_CMD;

/**
 * @brief Enum for SYS_CFG1 register Cyclic Timer settings.
 */
typedef enum
{
	CyclicTimerOff = 0x0000,  //!< Cyclic measurements disabled.
	CyclicTimerCont = 0x2000, //!< Continuous cyclic measurements.
	CyclicTimer0s1 = 0x4000,  //!< Cyclic measurements every 0.1s.
	CyclicTimer0s2 = 0x6000,  //!< Cyclic measurements every 0.2s.
	CyclicTimer1s = 0x8000,	  //!< Cyclic measurements every 1s.
	CyclicTimer2s = 0xA000,	  //!< Cyclic measurements every 2s.
	CyclicTimer4s = 0xC000,	  //!< Cyclic measurements every 4s.
	CyclicTimer8s = 0xE000	  //!< Cyclic measurements every 8s.
} LLD_TYPE_CYCLIC_TIMER;

/**
 * @brief Enum for SYS_CFG1 register Diagnostic Timeout settings.
 */
typedef enum
{
	DiagTimeoutNone = 0x0000, //!< No timeout (diagnostic mode disabled).
	DiagTimeout005 = 0x0400,  //!< Diagnostic timeout 0.05s.
	DiagTimeout01 = 0x0800,	  //!< Diagnostic timeout 0.1s.
	DiagTimeout02 = 0x0C00,	  //!< Diagnostic timeout 0.2s.
	DiagTimeout1 = 0x1000,	  //!< Diagnostic timeout 1s.
	DiagTimeout2 = 0x1400,	  //!< Diagnostic timeout 2s.
	DiagTimeout4 = 0x1800,	  //!< Diagnostic timeout 4s.
	DiagTimeout8 = 0x1C00	  //!< Diagnostic timeout 8s.
} LLD_TYPE_DIAG_TIMEOUT;

/**
 * @brief Enum for ADC_CFG register PGA Gain settings.
 */
typedef enum
{
	PGA_GAIN_4 = 0x0000,   //!< PGA Gain 4.
	PGA_GAIN_16 = 0x0100,  //!< PGA Gain 16.
	PGA_GAIN_64 = 0x0200,  //!< PGA Gain 64.
	PGA_GAIN_256 = 0x0300, //!< PGA Gain 256.
	PGA_GAIN_AUTO = 0x0700 //!< PGA Gain automatic.
} LLD_TYPE_PGA_GAIN;

/**
 * @brief Enum for ADC_CFG register ADC1_A resolution settings.
 */
typedef enum
{
	ADC1_A_13bit = 0x0000, //!< 13-bit resolution.
	ADC1_A_14bit = 0x0010, //!< 14-bit resolution.
	ADC1_A_15bit = 0x0020, //!< 15-bit resolution.
	ADC1_A_16bit = 0x0030  //!< 16-bit resolution.
} LLD_TYPE_ADC1_A_RES;

/**
 * @brief Enum for ADC_CFG register ADC1_B resolution settings.
 */
typedef enum
{
	ADC1_B_13bit = 0x0000, //!< 13-bit resolution.
	ADC1_B_14bit = 0x0004, //!< 14-bit resolution.
	ADC1_B_15bit = 0x0008, //!< 15-bit resolution.
	ADC1_B_16bit = 0x000C  //!< 16-bit resolution.
} LLD_TYPE_ADC1_B_RES;

/**
 * @brief Enum for ADC_CFG register ADC2 resolution settings.
 */
typedef enum
{
	ADC2_13bit = 0x0000, //!< 13-bit resolution.
	ADC2_14bit = 0x0001, //!< 14-bit resolution.
	ADC2_15bit = 0x0002, //!< 15-bit resolution.
	ADC2_16bit = 0x0003	 //!< 16-bit resolution.
} LLD_TYPE_ADC2_RES;

/**
 * @brief Enum for memory address accesses (register addresses).
 */
typedef enum
{
	Reserved = 0x00,		 //!< 0x00 reserved
	INIT = 0x01,			 //!< 0x01 device initialisation	   (Global write is forbidden for CID)
	SYS_CFG_GLOBAL = 0x02,	 //!< 0x02 global system configuration (GLOBAL access only and no ECHO in transformer mode, SPI mode this is operates as a standard write.)
	SYS_CFG1 = 0x03,		 //!< 0x03 system configuration
	SYS_CFG2 = 0x04,		 //!< 0x04 system configuration
	SYS_DIAG = 0x05,		 //!< 0x05 system diagnostics
	ADC_CFG = 0x06,			 //!< 0x06 ADC configuration
	ADC2_OFFSET_COMP = 0x07, //!< 0x07 ADC2 offset compensation
	OV_UV_EN = 0x08,		 //!< 0x08 CT measurement selection
	CELL_OV_FLT = 0x09,		 //!< 0x09 CT over-voltage fault
	CELL_UV_FLT = 0x0A,		 //!< 0x0A CT under-voltage fault
	// Reserved        0x0B
	CB1_CFG = 0x0C,					//!< 0x0C CB configuration for cell 1
	CB2_CFG = 0x0D,					//!< 0x0D CB configuration for cell 2
	CB3_CFG = 0x0E,					//!< 0x0E CB configuration for cell 3
	CB4_CFG = 0x0F,					//!< 0x0F CB configuration for cell 4
	CB5_CFG = 0x10,					//!< 0x10 CB configuration for cell 5
	CB6_CFG = 0x11,					//!< 0x11 CB configuration for cell 6
	CB7_CFG = 0x12,					//!< 0x12 CB configuration for cell 7
	CB8_CFG = 0x13,					//!< 0x13 CB configuration for cell 8
	CB9_CFG = 0x14,					//!< 0x14 CB configuration for cell 9
	CB10_CFG = 0x15,				//!< 0x15 CB configuration for cell 10
	CB11_CFG = 0x16,				//!< 0x16 CB configuration for cell 11
	CB12_CFG = 0x17,				//!< 0x17 CB configuration for cell 12
	CB13_CFG = 0x18,				//!< 0x18 CB configuration for cell 13
	CB14_CFG = 0x19,				//!< 0x19 CB configuration for cell 14
	CB_OPEN_FLT = 0x1A,				//!< 0x1A Open CB fault
	CB_SHORT_FLT = 0x1B,			//!< 0x1B Short CB fault
	CB_DRV_STS = 0x1C,				//!< 0x1C CB driver status
	GPIO_CFG1 = 0x1D,				//!< 0x1D GPIO configuration
	GPIO_CFG2 = 0x1E,				//!< 0x1E GPIO configuration
	GPIO_STS = 0x1F,				//!< 0x1F GPIO diagnostic
	AN_OT_UT_FLT = 0x20,			//!< 0x20 AN over and undertemp
	GPIO_SHORT_Anx_OPEN_STS = 0x21, //!< 0x21 Short GPIO / Open AN diagnostic
	I_STATUS = 0x22,				//!< 0x22 This register contains PGA DAC value
	COM_STATUS = 0x23,				//!< 0x23 Number of CRC error counted
	FAULT1_STATUS = 0x24,			//!< 0x24 Fault status
	FAULT2_STATUS = 0x25,			//!< 0x25 Fault status
	FAULT3_STATUS = 0x26,			//!< 0x26 Fault status
	FAULT_MASK1 = 0x27,				//!< 0x27 Fault pin mask
	FAULT_MASK2 = 0x28,				//!< 0x28 Fault pin mask
	FAULT_MASK3 = 0x29,				//!< 0x29 Fault pin mask
	WAKEUP_MASK1 = 0x2A,			//!< 0x2A Wakeup events mask
	WAKEUP_MASK2 = 0x2B,			//!< 0x2B Wakeup events mask
	WAKEUP_MASK3 = 0x2C,			//!< 0x2C Wakeup events mask
	CC_NB_SAMPLES = 0x2D,			//!< 0x2D Number of samples in coulomb counter
	COULOUMB_CNT1 = 0x2E,			//!< 0x2E..2F Couloumb counter accumulator
	COULOUMB_CNT2 = 0x2F,			//!< 0x2E..2F Couloumb counter accumulator
	MEAS_ISENSE1 = 0x30,			//!< 0x30 ISENSE measurement
	MEAS_ISENSE2 = 0x31,			//!< 0x31 ISENSE measurement
	MEAS_STACK = 0x32,				//!< 0x32 stack  voltage measurement
	MEAS_CELL14 = 0x33,				//!< 0x33 cell14 voltage measurement
	MEAS_CELL13 = 0x34,				//!< 0x34 cell13 voltage measurement
	MEAS_CELL12 = 0x35,				//!< 0x35 cell12 voltage measurement
	MEAS_CELL11 = 0x36,				//!< 0x36 cell11 voltage measurement
	MEAS_CELL10 = 0x37,				//!< 0x37 cell10 voltage measurement
	MEAS_CELL9 = 0x38,				//!< 0x38 cell9  voltage measurement
	MEAS_CELL8 = 0x39,				//!< 0x39 cell8  voltage measurement
	MEAS_CELL7 = 0x3A,				//!< 0x3A cell7  voltage measurement
	MEAS_CELL6 = 0x3B,				//!< 0x3B cell6  voltage measurement
	MEAS_CELL5 = 0x3C,				//!< 0x3C cell5  voltage measurement
	MEAS_CELL4 = 0x3D,				//!< 0x3D cell4  voltage measurement
	MEAS_CELL3 = 0x3E,				//!< 0x3E cell3  voltage measurement
	MEAS_CELL2 = 0x3F,				//!< 0x3F cell2  voltage measurement
	MEAS_CELL1 = 0x40,				//!< 0x40 cell1  voltage measurement
	MEAS_AN6 = 0x41,				//!< 0x41 AN6 voltage measurement
	MEAS_AN5 = 0x42,				//!< 0x42 AN5 voltage measurement
	MEAS_AN4 = 0x43,				//!< 0x43 AN4 voltage measurement
	MEAS_AN3 = 0x44,				//!< 0x44 AN3 voltage measurement
	MEAS_AN2 = 0x45,				//!< 0x45 AN2 voltage measurement
	MEAS_AN1 = 0x46,				//!< 0x46 AN1 voltage measurement
	MEAS_AN0 = 0x47,				//!< 0x47 AN0 voltage measurement
	MEAS_IC_TEMP = 0x48,			//!< 0x48 IC temperature measurement
	MEAS_VBG_DIAG_ADC1A = 0x49,		//!< 0x49 ADCIA Band Gap Reference measurement
	MEAS_VBG_DIAG_ADC1B = 0x4A,		//!< 0x4A ADCIB Band Gap Reference measurement
	TH_ALL_CT = 0x4B,				//!< 0x4B CTx over and undervoltage threshold
	TH_CT14 = 0x4C,					//!< 0x4C CT14 over and undervoltage threshold
	TH_CT13 = 0x4D,					//!< 0x4D CT13 over and undervoltage threshold
	TH_CT12 = 0x4E,					//!< 0x4E CT12 over and undervoltage threshold
	TH_CT11 = 0x4F,					//!< 0x4F CT11 over and undervoltage threshold
	TH_CT10 = 0x50,					//!< 0x50 CT10 over and undervoltage threshold
	TH_CT9 = 0x51,					//!< 0x51 CT9 over and undervoltage threshold
	TH_CT8 = 0x52,					//!< 0x52 CT8 over and undervoltage threshold
	TH_CT7 = 0x53,					//!< 0x53 CT7 over and undervoltage threshold
	TH_CT6 = 0x54,					//!< 0x54 CT6 over and undervoltage threshold
	TH_CT5 = 0x55,					//!< 0x55 CT5 over and undervoltage threshold
	TH_CT4 = 0x56,					//!< 0x56 CT4 over and undervoltage threshold
	TH_CT3 = 0x57,					//!< 0x57 CT3 over and undervoltage threshold
	TH_CT2 = 0x58,					//!< 0x58 CT2 over and undervoltage threshold
	TH_CT1 = 0x59,					//!< 0x59 CT1 over and undervoltage threshold
	TH_AN6_OT = 0x5A,				//!< 0x5A AN6 overtemp threshold
	TH_AN5_OT = 0x5B,				//!< 0x5B AN5 overtemp threshold
	TH_AN4_OT = 0x5C,				//!< 0x5C AN4 overtemp threshold
	TH_AN3_OT = 0x5D,				//!< 0x5D AN3 overtemp threshold
	TH_AN2_OT = 0x5E,				//!< 0x5E AN2 overtemp threshold
	TH_AN1_OT = 0x5F,				//!< 0x5F AN1 overtemp threshold
	TH_AN0_OT = 0x60,				//!< 0x60 AN0 overtemp threshold
	TH_AN6_UT = 0x61,				//!< 0x61 AN6 undertemp threshold
	TH_AN5_UT = 0x62,				//!< 0x62 AN5 undertemp threshold
	TH_AN4_UT = 0x63,				//!< 0x63 AN4 undertemp threshold
	TH_AN3_UT = 0x64,				//!< 0x64 AN3 undertemp threshold
	TH_AN2_UT = 0x65,				//!< 0x65 AN2 undertemp threshold
	TH_AN1_UT = 0x66,				//!< 0x66 AN1 undertemp threshold
	TH_AN0_UT = 0x67,				//!< 0x67 AN0 undertemp threshold
	TH_ISENSE_OC = 0x68,			//!< 0x68 ISENSE over current threshold
	TH_COULOMB_H = 0x69,			//!< 0x69 Over Coulomb counter threshold registers TH_COULOMB_CNT
	TH_COULOMB_L = 0x6A,			//!< 0x6A Over Coulomb counter threshold registers TH_COULOMB_CNT
	SILICON_REV = 0x6B,				//!< 0x6B silicon revision
	EEPROM_CNTL = 0x6C,				//!< 0x6C EEPROM transfere control
	DED_ENCODE1 = 0x6D,				//!< 0x6D ECC signature 1
	DED_ENCODE2 = 0x6E,				//!< 0x6E ECC signature 2
	FUSE_MIRROR_DATA = 0x6F,		//!< 0x6F Fuse Mirror data
	FUSE_MIRROR_CTRL = 0x70,		//!< 0x70 Fuse Mirror control
} LLD_TYPE_REG_NAME;

// ----------------------------------------------------------------------------
//! \brief datatype to hold GUID Global Unique ID.
typedef int64_t TYPE_GUID;

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief Sets the CSB level for SPI/TPL communication.
 * @details Sets the CSB pin to high (non-zero) or low (zero) for Arduino-type EVB.
 * @param u8Level 0 for low, non-zero for high.
 */
void slaveIF_SPISC(uint8_t u8Level);

/**
 * @brief Reads the status of the INTB pin.
 * @details Returns the INTB pin status for TPL interface on Arduino-type EVB.
 * @return uint8_t 1 if INTB is high, 0 if low.
 */
uint8_t IntbPinStatus(void);

/**
 * @brief Sets the TPL enable signal.
 * @details Enables or disables the TPL signal for Arduino-type EVB.
 * @param bEnable 0 to disable, non-zero to enable.
 */
void TplEnable(uint8_t bEnable);

/**
 * @brief Reads the status of the Fault pin.
 * @details Returns the Fault pin status for TPL interface on Arduino-type EVB.
 * @return uint8_t 1 if Fault, 0 if no Fault.
 */
uint8_t FaultPinStatus(void);

/**
 * @brief Calculates the CRC8 for a data array.
 * @details Uses a lookup table with polynomial 0x2F to calculate CRC8.
 * @param data Pointer to data byte array.
 * @param length Number of bytes to process.
 * @return uint8_t Calculated CRC8 value.
 */
uint8_t lld3377xCrcCalc(uint8_t *data, uint16_t length);

/**
 * @brief Packs a 40-bit message frame into a 5-byte buffer.
 * @details Packs data, address, CID, command, and CRC for SPI/TPL transmission.
 * @param pu8Buf Pointer to 5-byte buffer for packed frame.
 * @param data 16-bit data to pack.
 * @param addr 7-bit register address.
 * @param CID 4-bit Cluster ID.
 * @param cmd 4-bit command.
 */
void lld3377xPackFrame(uint8_t *pu8Buf, uint16_t data, uint8_t addr, uint8_t CID, uint8_t cmd);

/**
 * @brief Generates a new Rolling Counter (RC) value.
 * @details Returns the next RC value from RCVALUELIST.
 * @return uint8_t New RC value.
 */
uint8_t lld3377xNewRCValue(void);

/**
 * @brief Sets the TagID for a given CID.
 * @details Assigns a new TagID to the specified CID.
 * @param CID Cluster ID (1..14).
 * @param NewTagId New TagID value (0..15).
 * @return bool True if successful, false on error.
 */
bool lld3377xSetTagID(uint8_t CID, uint8_t NewTagId);

/**
 * @brief Initializes the SlaveIF driver.
 * @details Clears errors and stores interface information.
 * @param interface Pointer to interface type.
 * @return bool True if successful.
 */
bool lld3377xInitDriver(TYPE_INTERFACE *interface);

/**
 * @brief Initializes the cluster structure.
 * @details Stores cluster information for use in other functions.
 * @param cluster Pointer to cluster data.
 * @return bool True if successful.
 */
bool lld3377xInitCluster(LLD_TYPE_CLUSTER *cluster);

/**
 * @brief Wakes up the MC3377x device.
 * @details Issues a wake-up sequence via CSB toggle and waits for response.
 * @return bool True if successful, false on error.
 */
bool slaveIF_wakeUp(void);

/**
 * @brief Reads registers from MC3377x.
 * @details Reads up to 50 registers in a burst via SPI/TPL.
 * @param CID Cluster ID (0..15).
 * @param Register Register address (0..0x7F).
 * @param noRegs2Read Number of registers to read (1..50).
 * @param readData Pointer to array for read data (or NULL).
 * @return bool True if successful, false on error.
 */
bool lld3377xReadRegisters(uint8_t CID, uint8_t Register, uint8_t noRegs2Read, uint16_t *readData);

/**
 * @brief Writes a register to MC3377x.
 * @details Writes data to a specified register via SPI/TPL.
 * @param CID Cluster ID (0..15).
 * @param Register Register address (0..0x7F).
 * @param writeData Data to write.
 * @param returnData Pointer to returned data (or NULL).
 * @return bool True if successful, false on error.
 */
bool lld3377xWriteRegister(uint8_t CID, uint8_t Register, uint16_t writeData, uint16_t *returnData);

/**
 * @brief Enables TPL interface.
 * @details Sets TPL enable signal and waits for INTB acknowledge.
 * @return bool True if successful, false on error.
 */
bool lld3377xTPLEnable(void);

/**
 * @brief Disables TPL interface.
 * @details Clears the TPL enable signal.
 * @return bool True if successful, false on error.
 */
bool lld3377xTPLDisable(void);

/**
 * @brief Writes a global register to all CIDs (TPL only).
 * @details Writes data to a register for all clusters via TPL.
 * @param Register Register address (0..0x7F).
 * @param writeData Data to write.
 * @return bool True if successful, false on error.
 */
bool lld3377xWriteGlobalRegister(uint8_t Register, uint16_t writeData);

/**
 * @brief Sets an internal error code.
 * @details Allows simulation of errors for testing.
 * @param res Error code to set.
 * @return bool Always false for easy error handling.
 */
bool _lld3377xSetError(LLD_TYPE_RETURN res);

/**
 * @brief Retrieves the last error code.
 * @details Returns the current error status.
 * @param errorCode Pointer to store error code (or NULL).
 * @return bool True if error exists, false otherwise.
 */
bool lld3377xGetError(LLD_TYPE_RETURN *errorCode);

/**
 * @brief Clears the internal error status.
 * @details Resets the error status to RETURN_OK.
 */
void lld3377xClearError(void);

/**
 * @brief Tests CRC patterns from MC33771 datasheet.
 * @details Verifies CRC8 calculation with predefined patterns.
 * @return uint32_t 0 if all tests pass, else bitmask of failed tests.
 */
uint32_t crc8_test(void);

#endif /* SLAVE_IF_H */
//=============================================================================
// End of File
//=============================================================================
