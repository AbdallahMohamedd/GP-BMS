/**
 * @file  		SlaveIF.h
 * @brief 		Public interface header for the Slave Interface (SlaveIF) driver.
 *
 * @details 	This header defines the public APIs, register addresses, bit definitions,
 *          	data structures, and enums for interfacing with the SlaveIF module
 *          	(MC33771x/MC33772x) via SPI or TPL communication. It provides functions
 *          	for initialization, configuration, and data reading/writing.
 *
 * @note 		Project: Graduation Project - Battery Management System
 * @note 		Engineer: Abdullah Mohamed
 * @note 		Component: Slave_Control_IF driver
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
#define MSGLEN 				(5U)				//!< Number of bytes in a message (5*8 = 40 bits).
#define GLOBAL_CID 			(0U)	 			//!< Cluster ID for global write commands.
#define MAX_CLUSTER 		(2U) 				//!< Maximum number of clusters (1..15, CID=0 reserved).

// --- Data Unpacking Macros ---
#define UNPACK_REGADR(v) 		((uint8_t)((v)[2] & 0x7F))											   	//!< Extracts Register Address from received frame.
#define UNPACK_DATA(v) 			((uint16_t)(((v)[4] << 8) | (v)[3]))									//!< Extracts Data field from received frame.
#define UNPACK_TAGID(v) 		((uint8_t)((v)[1] & 0x0F))											   	//!< Extracts TagID field from received frame.
#define UNPACK_RC(v) 			((uint8_t)(((v)[1] & 0x0C) >> 2))										//!< Extracts RC field from received frame.
#define IS_NULL_RESPONSE(v) 	(((v)[4] == 0) && ((v)[3] == 0) && ((v)[2] == 0) && ((v)[1] == 0)) 		//!< Checks for NULL response (all fields except CRC are 0).

// --- Register Bit Definitions ---
#define INIT_BUS_SW 				(BIT(4))	//!< Bus Switch Control
#define SYS_CFG_GLOBAL_GO2SLEEP 	(BIT(0))  	//!< Goto sleep bit
#define SYS_CFG1_CB_DRVEN 			(BIT(7))	//!< General enable or disable for all cell balance drivers.
#define SYS_CFG1_I_MEAS_EN 			(BIT(9))	//!< ISENSE measurement enable bit
#define SYS_CFG1_GODIAG 			(BIT(6))	//!< ISENSE measurement enable bitGoto diag bit
#define SYS_CFG1_CB_MANUAL_PAUSE 	(BIT(5)) 	//!< Cell balancing manual pause
#define SYS_CFG1_SOFT_RST 			(BIT(4))	//!< ISENSE measurement enable bitSoft reset bit
#define SYS_DIAG_DA_DIAG 			(BIT(7))	//!< ISENSE measurement enable bitDiagnostics Differential amplifier diagnostic. Diagnostic mode function only
#define SYS_DIAG_POLARITY 			(BIT(6))	//!< ISENSE measurement enable bitControl bit used in terminal leakage detection. Controls the polarity between the level shifter and the ADC1-A and ADC1-B converters
#define SYS_DIAG_CT_LEAK_DIAG 		(BIT(5))	//!< Control bit used in terminal leakage detection. Commands the MUX to route the CTx/CBx pin to ADC1-A,B converters. This bit must be exclusive vs. DA_DIAG
#define SYS_DIAG_CT_OV_UV 			(BIT(4))	//!< ISENSE measurement enable bitDiagnostics Cell Terminal OV/UV bit
#define SYS_DIAG_CT_OL_ODD 			(BIT(3))	//!< ISENSE measurement enable bitDiagnostics Cell Terminal Open load Odd bit
#define SYS_DIAG_CT_OL_EVEN 		(BIT(2))	//!< Diagnostics Cell Terminal Open load Even bit
#define ADC_CFG_SOC 				(BIT(11))	//!< ISENSE measurement enable bitADC Start of Conversion bit
#define MEAS_DATA_RDY 				(BIT(15))	//!< ISENSE measurement enable bitData Ready Bit
#define CB_CFG_CB_EN 				(BIT(9))	//!< ISENSE measurement enable bitCell Balance enable

//=============================================================================
// Typedefs
//=============================================================================
/**
 * @brief 	Structure for MC33771B configuration list entry.
 * @details Defines register address and value pairs for configuration loading.
 * @note 	Last entry must be {0, 0} to indicate end of list.
 */
typedef struct
{
	uint8_t 	regAdr;	   				//!< Register address to be written.
	uint16_t 	regValue; 				//!< Register value to write.
} SsysConf_t;

/**
 * @brief 	128-bit array indicating TagID or RC format for registers.
 * @details Each bit indicates whether a register uses TagID (1) or RC (0) format.
 */
typedef struct
{
	uint16_t w[8]; 						//!< 128-bit array for TagID/RC indication.
} StagIDList_t;

/**
 * @brief 	Structure to hold cluster information.
 * @details Contains chip type, GUID, revisions, and TagID list pointer.
 */
typedef struct
{
	int64_t 		Guid;				//!< Global Unique ID.
	uint8_t 		Chip;				//!< Chip type (see TypeIC_t).
	uint8_t 		FRev;				//!< Full mask revision.
	uint8_t 		MRev;				//!< Metal mask revision.
	uint8_t 		NoCTs;				//!< Number of Cell Terminals (6 or 14 supported).
	const uint16_t 	*pTagIdList; 		//!< Pointer to TagID list.
} SclusterInfo_t;

//=============================================================================
// Enums
//=============================================================================
/**
 * @brief 	Enum for supported chip types.
 */
typedef enum
{
	Chip_Unknown 		= (0U),			//!< Unknown chip.
	Chip_MC33771A 		= (1U),			//!< MC33771A, 14-channel device.
	Chip_MC33772A 		= (2U),			//!< MC33772A, 6-channel device.
	Chip_MC33771BM 		= (3U),			//!< MC33771B-M, 14-channel device.
	Chip_MC33772BM 		= (4U),			//!< MC33772B-M, 6-channel device.
	Chip_MC33771B 		= (5U),			//!< MC33771B, 14-channel device.
	Chip_MC33772B 		= (6U)			//!< MC33772B, 6-channel device.
} TypeIC_t;

/**
 * @brief 	Enum for function return values.
 */
typedef enum
{
	RETURN_OK 			= (0x00U),		//!< No error.
	ERR_TX 				= (0x01U),		//!< Transmission error (TPL only).
	ERR_NoResponse 		= (0x02U),	  	//!< No bytes received.
	ERR_ResponseLen 	= (0x03U),	  	//!< Wrong number of bytes received.
	ERR_ResponseCRC 	= (0x04U),	  	//!< CRC error in received data.
	ERR_NullResponse 	= (0x05U),	  	//!< Null response (all data bytes 0x00).
	ERR_WrongParam 		= (0x06U),	  	//!< Invalid parameters passed.
	ERR_ResponseTagId 	= (0x07U),  	//!< Response Tag ID mismatch.
	ERR_ResponseRC 		= (0x08U),	  	//!< Response RC mismatch.
	ERR_ResponseAdr 	= (0x09U),	  	//!< Response register address mismatch.
	ERR_WrongInterface 	= (0x0AU), 		//!< Unknown or invalid interface.
	ERR_Timeout 		= (0x0BU)		//!< Timeout error.
} TypeReturn_t;

/**
 * @brief 	Enum for Cluster IDs (SPI version).
 */
typedef enum
{
	CIDunassiged 		= 0, 			//!< Unassigned node (reacts to CID 0 and INIT register).
	CIDassiged 			= 1	  			//!< Assigned CID for SPI (CID 1).
} TypeCID_t;

/**
 * @brief 	Enum for commands (SPI version).
 */
typedef enum
{
	CmdNOP 				= (0x0U),	 	//!< No Operation command (not supported).
	CmdRdLocal 			= (0x1U), 		//!< Local Read command.
	CmdWrLocal 			= (0x2U), 		//!< Local Write command.
	CmdWrGlobal 		= (0x3U) 		//!< Global Write command (not allowed for SPI).
} TypeCMD_t;

/**
 * @brief 	Enum for SYS_CFG1 register Cyclic Timer settings.
 */
typedef enum
{
	CyclicTimerOff 		= (0x0000U),	//!< Cyclic measurements disabled.
	CyclicTimerCont 	= (0x2000U), 	//!< Continuous cyclic measurements.
	CyclicTimer0s1 		= (0x4000U),	//!< Cyclic measurements every 0.1s.
	CyclicTimer0s2 		= (0x6000U),	//!< Cyclic measurements every 0.2s.
	CyclicTimer1s 		= (0x8000U),	//!< Cyclic measurements every 1s.
	CyclicTimer2s 		= (0xA000U),	//!< Cyclic measurements every 2s.
	CyclicTimer4s 		= (0xC000U),	//!< Cyclic measurements every 4s.
	CyclicTimer8s 		= (0xE000U)	 	//!< Cyclic measurements every 8s.
} TypeCycleTimer_t;

/**
 * @brief 	Enum for SYS_CFG1 register Diagnostic Timeout settings.
 */
typedef enum
{
	DiagTimeoutNone 	= (0x0000U), 	//!< No timeout (diagnostic mode disabled).
	DiagTimeout005 		= (0x0400U),	//!< Diagnostic timeout 0.05s.
	DiagTimeout01 		= (0x0800U),	//!< Diagnostic timeout 0.1s.
	DiagTimeout02 		= (0x0C00U),	//!< Diagnostic timeout 0.2s.
	DiagTimeout1 		= (0x1000U),	//!< Diagnostic timeout 1s.
	DiagTimeout2 		= (0x1400U),	//!< Diagnostic timeout 2s.
	DiagTimeout4 		= (0x1800U),	//!< Diagnostic timeout 4s.
	DiagTimeout8 		= (0x1C00U)	 	//!< Diagnostic timeout 8s.
} Type_DiagTimeout_t;

/**
 * @brief 	Enum for ADC_CFG register PGA Gain settings.
 */
typedef enum
{
	PGA_GAIN_4 			= (0x0000U),	//!< PGA Gain 4.
	PGA_GAIN_16 		= (0x0100U),  	//!< PGA Gain 16.
	PGA_GAIN_64 		= (0x0200U),  	//!< PGA Gain 64.
	PGA_GAIN_256 		= (0x0300U), 	//!< PGA Gain 256.
	PGA_GAIN_AUTO 		= (0x0700U)  	//!< PGA Gain automatic.
} TypePGAgain_t;

/**
 * @brief 	Enum for ADC_CFG register ADC1_A resolution settings.
 */
typedef enum
{
	ADC1_A_13bit 		= (0x0000U), 	//!< 13-bit resolution.
	ADC1_A_14bit 		= (0x0010U), 	//!< 14-bit resolution.
	ADC1_A_15bit 		= (0x0020U), 	//!< 15-bit resolution.
	ADC1_A_16bit 		= (0x0030U)  	//!< 16-bit resolution.
} ADC1_A_reg_t;

/**
 * @brief 	Enum for ADC_CFG register ADC1_B resolution settings.
 */
typedef enum
{
	ADC1_B_13bit 		= (0x0000U), 	//!< 13-bit resolution.
	ADC1_B_14bit 		= (0x0004U), 	//!< 14-bit resolution.
	ADC1_B_15bit 		= (0x0008U), 	//!< 15-bit resolution.
	ADC1_B_16bit 		= (0x000CU)  	//!< 16-bit resolution.
} ADC1_B_reg_t;

/**
 * @brief 	Enum for ADC_CFG register ADC2 resolution settings.
 */
typedef enum
{
	ADC2_13bit 			= (0x0000U),	//!< 13-bit resolution.
	ADC2_14bit 			= (0x0001U),	//!< 14-bit resolution.
	ADC2_15bit 			= (0x0002U),	//!< 15-bit resolution.
	ADC2_16bit 			= (0x0003U)		//!< 16-bit resolution.
} ADC2reg_t;

/**
 * @brief 	Enum for memory address accesses (register addresses).
 */
typedef enum
{
	Reserved 					= (0x00U),			//!< 0x00 reserved
	INIT 						= (0x01U),			//!< 0x01 device initialisation	   (Global write is forbidden for CID)
	SYS_CFG_GLOBAL 				= (0x02U),			//!< 0x02 global system configuration (GLOBAL access only and no ECHO in transformer mode, SPI mode this is operates as a standard write.)
	SYS_CFG1 					= (0x03U),			//!< 0x03 system configuration
	SYS_CFG2 					= (0x04U),			//!< 0x04 system configuration
	SYS_DIAG 					= (0x05U),			//!< 0x05 system diagnostics
	ADC_CFG 					= (0x06U),			//!< 0x06 ADC configuration
	ADC2_OFFSET_COMP 			= (0x07U),			//!< 0x07 ADC2 offset compensation
	OV_UV_EN 					= (0x08U),			//!< 0x08 CT measurement selection
	CELL_OV_FLT 				= (0x09U),			//!< 0x09 CT over-voltage fault
	CELL_UV_FLT 				= (0x0AU),			//!< 0x0A CT under-voltage fault
	// Reserved         		= (0x0BU),			//!< 0x0B reserved
	CB1_CFG 					= (0x0CU),			//!< 0x0C CB configuration for cell 1
	CB2_CFG 					= (0x0DU),			//!< 0x0D CB configuration for cell 2
	CB3_CFG 					= (0x0EU),			//!< 0x0E CB configuration for cell 3
	CB4_CFG 					= (0x0FU),			//!< 0x0F CB configuration for cell 4
	CB5_CFG 					= (0x10U),			//!< 0x10 CB configuration for cell 5
	CB6_CFG 					= (0x11U),			//!< 0x11 CB configuration for cell 6
	CB7_CFG 					= (0x12U),			//!< 0x12 CB configuration for cell 7
	CB8_CFG 					= (0x13U),			//!< 0x13 CB configuration for cell 8
	CB9_CFG 					= (0x14U),			//!< 0x14 CB configuration for cell 9
	CB10_CFG 					= (0x15U),			//!< 0x15 CB configuration for cell 10
	CB11_CFG 					= (0x16U),			//!< 0x16 CB configuration for cell 11
	CB12_CFG 					= (0x17U),			//!< 0x17 CB configuration for cell 12
	CB13_CFG 					= (0x18U),			//!< 0x18 CB configuration for cell 13
	CB14_CFG 					= (0x19U),			//!< 0x19 CB configuration for cell 14
	CB_OPEN_FLT 				= (0x1AU),			//!< 0x1A Open CB fault
	CB_SHORT_FLT 				= (0x1BU),			//!< 0x1B Short CB fault
	CB_DRV_STS 					= (0x1CU),			//!< 0x1C CB driver status
	GPIO_CFG1 					= (0x1DU),			//!< 0x1D GPIO configuration
	GPIO_CFG2 					= (0x1EU),			//!< 0x1E GPIO configuration
	GPIO_STS 					= (0x1FU),			//!< 0x1F GPIO diagnostic
	AN_OT_UT_FLT 				= (0x20U),			//!< 0x20 AN over and undertemp
	GPIO_SHORT_Anx_OPEN_STS 	= (0x21U),			//!< 0x21 Short GPIO / Open AN diagnostic
	I_STATUS 					= (0x22U),			//!< 0x22 This register contains PGA DAC value
	COM_STATUS 					= (0x23U),			//!< 0x23 Number of CRC error counted
	FAULT1_STATUS 				= (0x24U),		   	//!< 0x24 Fault status
	FAULT2_STATUS 				= (0x25U),		   	//!< 0x25 Fault status
	FAULT3_STATUS 				= (0x26U),		   	//!< 0x26 Fault status
	FAULT_MASK1 				= (0x27U),			//!< 0x27 Fault pin mask
	FAULT_MASK2 				= (0x28U),			//!< 0x28 Fault pin mask
	FAULT_MASK3 				= (0x29U),			//!< 0x29 Fault pin mask
	WAKEUP_MASK1 				= (0x2AU),			//!< 0x2A Wakeup events mask
	WAKEUP_MASK2 				= (0x2BU),			//!< 0x2B Wakeup events mask
	WAKEUP_MASK3 				= (0x2CU),			//!< 0x2C Wakeup events mask
	CC_NB_SAMPLES 				= (0x2DU),		   	//!< 0x2D Number of samples in coulomb counter
	COULOUMB_CNT1 				= (0x2EU),		   	//!< 0x2E..2F Couloumb counter accumulator
	COULOUMB_CNT2 				= (0x2FU),		   	//!< 0x2E..2F Couloumb counter accumulator
	MEAS_ISENSE1 				= (0x30U),			//!< 0x30 ISENSE measurement
	MEAS_ISENSE2 				= (0x31U),			//!< 0x31 ISENSE measurement
	MEAS_STACK 					= (0x32U),			//!< 0x32 stack  voltage measurement
	MEAS_CELL14 				= (0x33U),			//!< 0x33 cell14 voltage measurement
	MEAS_CELL13 				= (0x34U),			//!< 0x34 cell13 voltage measurement
	MEAS_CELL12 				= (0x35U),			//!< 0x35 cell12 voltage measurement
	MEAS_CELL11 				= (0x36U),			//!< 0x36 cell11 voltage measurement
	MEAS_CELL10 				= (0x37U),			//!< 0x37 cell10 voltage measurement
	MEAS_CELL9 					= (0x38U),			//!< 0x38 cell9  voltage measurement
	MEAS_CELL8 					= (0x39U),			//!< 0x39 cell8  voltage measurement
	MEAS_CELL7 					= (0x3AU),			//!< 0x3A cell7  voltage measurement
	MEAS_CELL6 					= (0x3BU),			//!< 0x3B cell6  voltage measurement
	MEAS_CELL5 					= (0x3CU),			//!< 0x3C cell5  voltage measurement
	MEAS_CELL4 					= (0x3DU),			//!< 0x3D cell4  voltage measurement
	MEAS_CELL3 					= (0x3EU),			//!< 0x3E cell3  voltage measurement
	MEAS_CELL2 					= (0x3FU),			//!< 0x3F cell2  voltage measurement
	MEAS_CELL1 					= (0x40U),			//!< 0x40 cell1  voltage measurement
	MEAS_AN6 					= (0x41U),			//!< 0x41 AN6 voltage measurement
	MEAS_AN5 					= (0x42U),			//!< 0x42 AN5 voltage measurement
	MEAS_AN4 					= (0x43U),			//!< 0x43 AN4 voltage measurement
	MEAS_AN3 					= (0x44U),			//!< 0x44 AN3 voltage measurement
	MEAS_AN2 					= (0x45U),			//!< 0x45 AN2 voltage measurement
	MEAS_AN1 					= (0x46U),			//!< 0x46 AN1 voltage measurement
	MEAS_AN0 					= (0x47U),			//!< 0x47 AN0 voltage measurement
	MEAS_IC_TEMP 				= (0x48U),			//!< 0x48 IC temperature measurement
	MEAS_VBG_DIAG_ADC1A 		= (0x49U),			//!< 0x49 ADCIA Band Gap Reference measurement
	MEAS_VBG_DIAG_ADC1B 		= (0x4AU),			//!< 0x4A ADCIB Band Gap Reference measurement
	TH_ALL_CT 					= (0x4BU),			//!< 0x4B CTx over and undervoltage threshold
	TH_CT14 					= (0x4CU),			//!< 0x4C CT14 over and undervoltage threshold
	TH_CT13 					= (0x4DU),			//!< 0x4D CT13 over and undervoltage threshold
	TH_CT12 					= (0x4EU),			//!< 0x4E CT12 over and undervoltage threshold
	TH_CT11 					= (0x4FU),			//!< 0x4F CT11 over and undervoltage threshold
	TH_CT10 					= (0x50U),			//!< 0x50 CT10 over and undervoltage threshold
	TH_CT9 						= (0x51U),			//!< 0x51 CT9 over and undervoltage threshold
	TH_CT8 						= (0x52U),			//!< 0x52 CT8 over and undervoltage threshold
	TH_CT7 						= (0x53U),			//!< 0x53 CT7 over and undervoltage threshold
	TH_CT6 						= (0x54U),			//!< 0x54 CT6 over and undervoltage threshold
	TH_CT5 						= (0x55U),			//!< 0x55 CT5 over and undervoltage threshold
	TH_CT4 						= (0x56U),			//!< 0x56 CT4 over and undervoltage threshold
	TH_CT3 						= (0x57U),			//!< 0x57 CT3 over and undervoltage threshold
	TH_CT2 						= (0x58U),			//!< 0x58 CT2 over and undervoltage threshold
	TH_CT1 						= (0x59U),			//!< 0x59 CT1 over and undervoltage threshold
	TH_AN6_OT 					= (0x5AU),			//!< 0x5A AN6 overtemp threshold
	TH_AN5_OT 					= (0x5BU),			//!< 0x5B AN5 overtemp threshold
	TH_AN4_OT 					= (0x5CU),			//!< 0x5C AN4 overtemp threshold
	TH_AN3_OT 					= (0x5DU),			//!< 0x5D AN3 overtemp threshold
	TH_AN2_OT 					= (0x5EU),			//!< 0x5E AN2 overtemp threshold
	TH_AN1_OT 					= (0x5FU),			//!< 0x5F AN1 overtemp threshold
	TH_AN0_OT 					= (0x60U),			//!< 0x60 AN0 overtemp threshold
	TH_AN6_UT 					= (0x61U),			//!< 0x61 AN6 undertemp threshold
	TH_AN5_UT 					= (0x62U),			//!< 0x62 AN5 undertemp threshold
	TH_AN4_UT 					= (0x63U),			//!< 0x63 AN4 undertemp threshold
	TH_AN3_UT 					= (0x64U),			//!< 0x64 AN3 undertemp threshold
	TH_AN2_UT 					= (0x65U),			//!< 0x65 AN2 undertemp threshold
	TH_AN1_UT 					= (0x66U),			//!< 0x66 AN1 undertemp threshold
	TH_AN0_UT 					= (0x67U),			//!< 0x67 AN0 undertemp threshold
	TH_ISENSE_OC 				= (0x68U),			//!< 0x68 ISENSE over current threshold
	TH_COULOMB_H 				= (0x69U),			//!< 0x69 Over Coulomb counter threshold registers TH_COULOMB_CNT
	TH_COULOMB_L 				= (0x6AU),			//!< 0x6A Over Coulomb counter threshold registers TH_COULOMB_CNT
	SILICON_REV 				= (0x6BU),			//!< 0x6B silicon revision
	EEPROM_CNTL 				= (0x6CU),			//!< 0x6C EEPROM transfere control
	DED_ENCODE1 				= (0x6DU),			//!< 0x6D ECC signature 1
	DED_ENCODE2 				= (0x6EU),			//!< 0x6E ECC signature 2
	FUSE_MIRROR_DATA 			= (0x6FU),			//!< 0x6F Fuse Mirror data
	FUSE_MIRROR_CTRL 			= (0x70U),			//!< 0x70 Fuse Mirror control
} RegName_t;

// ----------------------------------------------------------------------------
//! \brief datatype to hold GUID Global Unique ID.
typedef int64_t TYPE_GUID;

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief 	Sets the CSB level for SPI/TPL communication.
 * @details Sets the CSB pin to high (non-zero) or low (zero) for Arduino-type EVB.
 * @param 	u8Level 0 for low, non-zero for high.
 */
void slaveIF_SPICS(uint8_t u8Level);

/**
 * @brief 	Reads the status of the INTB pin.
 * @details Returns the INTB pin status for TPL interface on Arduino-type EVB.
 * @return 	uint8_t 1 if INTB is high, 0 if low.
 */
uint8_t slaveIF_IntbPinStatus(void);

/**
 * @brief 	Reads the status of the Fault pin.
 * @details Returns the Fault pin status for TPL interface on Arduino-type EVB.
 * @return 	uint8_t 1 if Fault, 0 if no Fault.
 */
uint8_t slaveIF_faultPinStatus(void);

/**
 * @brief 	Generates a new Rolling Counter (RC) value.
 * @details Returns the next RC value from RCVALUELIST.
 * @return 	uint8_t New RC value.
 */
uint8_t slaveIF_generateNewRC(void);

/**
 * @brief 	Sets the TagID for a given CID.
 * @details Assigns a new TagID to the specified CID.
 * @param 	CID Cluster ID (1..14).
 * @param 	NewTagId New TagID value (0..15).
 * @return 	bool True if successful, false on error.
 */
bool slaveIF_setTagID(uint8_t CID, uint8_t NewTagId);

/**
 * @brief 	Initializes the SlaveIF driver.
 * @details Clears errors and stores interface information.
 * @param 	interface Pointer to interface type.
 * @return 	bool True if successful.
 */
bool slaveIF_initDriver(TYPE_INTERFACE *interface);

/**
 * @brief 	Initializes the cluster structure.
 * @details Stores cluster information for use in other functions.
 * @param 	cluster Pointer to cluster data.
 * @return 	bool True if successful.
 */
bool slaveIF_initCluster(SclusterInfo_t *cluster);

/**
 * @brief 	Wakes up the MC33771B device.
 * @details Issues a wake-up sequence via CSB toggle and waits for response.
 * @return 	bool True if successful, false on error.
 */
bool slaveIF_wakeUp(void);

/**
 * @brief 	Reads registers from MC33771B.
 * @details Reads up to 50 registers in a burst via SPI/TPL.
 * @param 	CID Cluster ID (0..15).
 * @param 	Register Register address (0..0x7F).
 * @param 	noRegs2Read Number of registers to read (1..50).
 * @param 	readData Pointer to array for read data (or NULL).
 * @return 	bool True if successful, false on error.
 */
bool slaveIF_readReg(uint8_t CID, uint8_t Register, uint8_t noRegs2Read, uint16_t *readData);

/**
 * @brief 	Writes a register to MC33771B.
 * @details Writes data to a specified register via SPI/TPL.
 * @param 	CID Cluster ID (0..15).
 * @param 	Register Register address (0..0x7F).
 * @param 	writeData Data to write.
 * @param 	returnData Pointer to returned data (or NULL).
 * @return 	bool True if successful, false on error.
 */
bool slaveIF_writeReg(uint8_t CID, uint8_t Register, uint16_t writeData, uint16_t *returnData);

/**
 * @brief 	Enables TPL interface.
 * @details Sets TPL enable signal and waits for INTB acknowledge.
 * @return 	bool True if successful, false on error.
 */
bool slaveIF_transceiverEnable(void);

/**
 * @brief 	Disables TPL interface.
 * @details Clears the TPL enable signal.
 * @return 	bool True if successful, false on error.
 */
bool slaveIF_transceiverDisable(void);

/**
 * @brief 	Writes a global register to all CIDs (TPL only).
 * @details Writes data to a register for all clusters via TPL.
 * @param 	Register Register address (0..0x7F).
 * @param 	writeData Data to write.
 * @return 	bool True if successful, false on error.
 */
bool slaveIF_writeGlobalReg(uint8_t Register, uint16_t writeData);

/**
 * @brief 	Sets an internal error code.
 * @details Allows simulation of errors for testing.
 * @param 	res Error code to set.
 * @return 	bool Always false for easy error handling.
 */
bool slaveIF_setError(TypeReturn_t res);

/**
 * @brief 	Retrieves the last error code.
 * @details Returns the current error status.
 * @param 	errorCode Pointer to store error code (or NULL).
 * @return 	bool True if error exists, false otherwise.
 */
bool slaveIF_getError(TypeReturn_t *errorCode);

/**
 * @brief 	Clears the internal error status.
 * @details Resets the error status to RETURN_OK.
 */
void slaveIF_clearError(void);

#endif /* SLAVE_IF_H */
//=============================================================================
// End of File
//=============================================================================
