/*
 * SlaveIF.h
 *
 *  Created on: Jun 1, 2025
 *      Author: abdal
 */


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
//! \addtogroup drivers
// @{
/*! \brief Low level driver for the MC33771 and MC33772 products.
 *
 * This module contains low level routines to access the MC33771, MC33772 and MC33664
 * products and provides the following functionalities:
 *
 * - read, write and global write to register functions
 * - CRC computation (lookup-table based)
 * - TPL enable/disable functions
 * - error handling (see below)
 * - handling of TagID and RC rolling counter
 *
 * \note
 * Handling of TPL and SPI interface to MC3377x is handled within only one function.
 *
 *
 * <b>TagID and RC counter</b>
 *
 * The MC3377x provide some means to match a response to a requested e.g. read command
 * by using a (rolling) counter value. Like this is possible to verify that the response is
 * the one from the last sent request and not e.g. from the previous request.
 *
 * The RC Rolling Counter is used for register accesses where the matching is directly
 * between register access and response.
 *
 * Some registers, e.g. MEAS_CELL, however need to be matched with the actual triggering
 * of the measurement itself. Here the request to perform a measurement - start of conversion
 * (SoC) is tagged with an TagID and later reading of e.g. MEAS_CELL registers can be matched
 * with the TagID provided for the SoC.
 *
 * \sa \ref lld3377xNewRCValue, \ref lld3377xSetTagID
 *
 *
 * <b>Error handling philosophy</b>
 *
 * To easier handle and detect, e.g. communication errors, a global error "errLast"
 * is used. In case of an previously detected error further e.g. lld3377xReadRegisters() calls
 * will not executed and return false to indicate unsuccessful execution.
 *
 * Like this is possible to propagate errors through hierarchical code up to
 * the level where the error shall be handled.
 *
 * \sa
 * - \ref lld3377xClearError and \ref lld3377xGetError for error handling
 * - \ref _lld3377xSetError() to test error handling (e.g. inject simulated errors) on application level
 *
 * <b>Handling of interface (TPL or SPI)</b>
 *
 * The communication between the pack controller and a MC3377x will be different,
 * if the MC3377x is directly connected via SPI or accessed using the TPL interface
 * (using MC33664 TPL translator). E.g. will a lld3377xReadRegisters() via SPI interface,
 * require 2 SPI transfers as the SPI MISO data returned is related to the previous SPI transfer.
 *
*/
//! \addtogroup lld3377x
// @{
// ----------------------------------------------------------------------------

#ifndef COTS_SLAVECONTROLIF_INC_SLAVEIF_H_
#define COTS_SLAVECONTROLIF_INC_SLAVEIF_H_
// ----------------------------------------------------------------------------
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include <string.h>																// for memcmp()
#include <stdio.h>
//#include "source/COTs/BMSDataBase/Inc/database.h"
//#include "Platform/frdmkl25z.h"
#include "source/COTs/NTC/Inc/ntc.h"
#include <tpm1.h>																// for Delay
#include "MKL25Z4.h"
#include "spi.h"


// ----------------------------------------------------------------------------
#define MSGLEN				5													//!< no. of bytes in a message (5*8byte = 40bit)
// ----------------------------------------------------------------------------
#define GLOBAL_CID        	0													//!< CID for global write commands
// ----------------------------------------------------------------------------
#define MAX_CLUSTER         2													//!< max. number of clusters 1..15  (CID = 0 reserved for unassigned cluster)
// ----------------------------------------------------------------------------
// values from datasheet
#define WAIT_AFTER_RESET                 5 		 								//!< time [ms] its required to wait after a reset (WAIT_AFTER_RESET)	{VPWR(READY) VPWR to Device Ready for Initialization � � 5.0 ms}
#define WAIT_AFTER_WAKEUP                1 				 						//!< time [ms] to wait after wakeup tWAKE-UP\nSleep Mode to Normal Mode Wake-up from Bus communication.
// ----------------------------------------------------------------------------
#define SETTLING_TIME      1000													//!< time [us] to wait for RCs to settle
	// threshold must be set below lowest expected value
	// Uth < Ucell * (Rpd / (2*Rext+Rpd))
#define SCALE_VCELL         (152.5925E-6)   //[V]
#define OPEN_THRESHOLD         (u16) ((0.100)/SCALE_VCELL)    					// value for 150mV
// ----------------------------------------------------------------------------
// macros to unpack data from array
#define UNPACK_REGADR(v)   ((u8)  ( (v)[2] & 0x7F))								//!< macros for unpacking Register Address field from received frame
#define UNPACK_DATA(v)     ((u16) (( (v)[4] << 8) | (v)[3]))  					//!< macros for unpacking Data field from received frame
#define UNPACK_TAGID(v)    ((u8)  ( (v)[1] & 0x0F))  							//!< macros for unpacking TagID field from received frame
#define UNPACK_RC(v)       ((u8)  ( ((v)[1] & 0x0C))>>2 )  						//!< macros for unpacking RC field from received frame
// ----------------------------------------------------------------------------
#define IS_NULL_RESPONSE(v)  (( (v)[4]==0 )&&( (v)[3]==0 )&&( (v)[2]==0 )&&( (v)[1]==0 ))  //!< macro to check for NULL response\n all fields (except CRC) are 0
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
extern TYPE_INTERFACE _interface;   //!< local copy of interface type
extern TYPE_EVB _evb;				//!< local copy of evb type

// ----------------------------------------------------------------------------
//! \brief 128 bit array, where each bit is indicating if a register is using TagID or RC format response \sa \ref bRegIsTagID
typedef struct { u16 w[8]; }TYPE_TAGIDLIST;									    //!< 128 bit array
// ----------------------------------------------------------------------------
//! \brief structures to handle different chips
typedef enum{
	Chip_Unknown   = 0,															//!< unknown
	Chip_MC33771A  = 1,															//!< MC33771A  14-channel device
	Chip_MC33772A  = 2,															//!< MC33772A   6-channel device
	Chip_MC33771BM = 3,															//!< MC33771B- 14-channel device
	Chip_MC33772BM = 4,															//!< MC33772B-  6-channel device
	Chip_MC33771B  = 5,															//!< MC33771B  14-channel device
	Chip_MC33772B  = 6,															//!< MC33772B   6-channel device
}TYPE_CHIP;
// ----------------------------------------------------------------------------
//! \brief datatype to hold GUID Global Unique ID.
typedef int64_t TYPE_GUID;
// ----------------------------------------------------------------------------
//! \brief structure to hold the Cluster information
typedef struct{
	TYPE_GUID Guid;																//!< Global Unique ID
	TYPE_CHIP Chip;																//!< used Chip
	u8 FRev;																	//!< Full mask revision
	u8 MRev;																	//!< Metal mask revision
	u8 NoCTs;																	//!< Number of Cell Terminals (currently only 6 or 14 is supported)
	const u16 *pTagIdList;														//!< pointer to used TagID list
}LLD_TYPE_CLUSTER;
// ----------------------------------------------------------------------------
/*! \brief function return values
*/

typedef enum {
	RETURN_OK  			= 0x00, 	         									//!< no error
	ERR_WrongParam      = 0x06,													//!< Wrong parameter (invalid parameters were passed)

/*! errors related to readback of transmitted data "echo"  - only tpl interface
	\image html	tplerrtx.png
*/
	ERR_TX				= 0x01,
	ERR_NoResponse		= 0x02,         										//!< no bytes were received
	ERR_ResponseLen     = 0x03,    												//!< wrong no of bytes received
	ERR_ResponseCRC		= 0x04,        											//!< CRC of received data is wrong
	ERR_NullResponse    = 0x05,													//!< Null Return (all data bytes a 0x00 and CRC is correct) - only in SPIVERSION
	ERR_ResponseTagId   = 0x07,													//!< Response Tag ID does not match with provided ID
	ERR_ResponseRC      = 0x08,													//!< Response Tag ID does not match with provided ID
	ERR_ResponseAdr     = 0x09,													//!< Response register address is not expected one
	ERR_WrongInterface  = 0x0A,													//!< wrong / unknown interface
	ERR_Timeout         = 0x0B,													//!< timeout error
	//	ERROR = 0xFF	                   											//!< general (no further detailed) error code
}LLD_TYPE_RETURN;

// ------------------------------------------------------------------
//! \brief enum for CIDs (SPI version)
typedef enum{
	CIDunassiged = 0,															//!< unassiged node reacts to CID 0 and INIT register accesses
	CIDassiged = 1																//!< for SPI we assign CID 1
}LLD_TYPE_CID;
// ------------------------------------------------------------------
//! \brief enum for commands (NOP and WrGlobal not used/supported for SPI version)
typedef enum {
	CmdNOP     = 0x0,															//!< No Operation command not supported
	CmdRdLocal = 0x1,															//!< Local Read command
	CmdWrLocal = 0x2,															//!< Local Write command
	CmdWrGlobal= 0x3															//!< Global Write Command not allowed for SPI
}LLD_TYPE_CMD;
// ----------------------------------------------------------------------------
//! \brief enum for SYS_CFG1 register Cyclic Timer setting
typedef enum {
	CyclicTimerOff   = 0x0000,													//!< cyclic measurements disabled
	CyclicTimerCont  = 0x2000,													//!< continuous cyclic measurements
	CyclicTimer0s1   = 0x4000,													//!< cyclic measurements 0.1s
	CyclicTimer0s2   = 0x6000,													//!< cyclic measurements 0.2s
	CyclicTimer1s    = 0x8000,													//!< cyclic measurements 1s
	CyclicTimer2s    = 0xA000,													//!< cyclic measurements 2s
	CyclicTimer4s    = 0xC000,													//!< cyclic measurements 4s
	CyclicTimer8s    = 0xE000													//!< cyclic measurements 8s
}LLD_TYPE_CYCLIC_TIMER;
// ----------------------------------------------------------------------------
//! \brief enum for SYS_CFG1 register Diagnostic Timeout settings
typedef enum {
	DiagTimeoutNone  = 0x0000,													//!< No timeout (not allowed to enter diag mode)
	DiagTimeout005   = 0x0400,													//!< Diag timeout 0.05s
	DiagTimeout01    = 0x0800,													//!< Diag timeout 0.1s
	DiagTimeout02    = 0x0C00,													//!< Diag timeout 0.2s
	DiagTimeout1     = 0x1000,													//!< Diag timeout 1s
	DiagTimeout2     = 0x1400,													//!< Diag timeout 2s
	DiagTimeout4     = 0x1800,													//!< Diag timeout 4s
	DiagTimeout8     = 0x1C00													//!< Diag timeout 8 s
}LLD_TYPE_DIAG_TIMEOUT;
// ----------------------------------------------------------------------------
//! \brief enum for ADC_CFG register PGA_GAIN setting
typedef enum {
	PGA_GAIN_4       = 0x0000,													//!< PGA Gain 4
	PGA_GAIN_16      = 0x0100,													//!< PGA Gain 16
	PGA_GAIN_64      = 0x0200,													//!< PGA Gain 64
	PGA_GAIN_256     = 0x0300,													//!< PGA Gain 256
	PGA_GAIN_AUTO    = 0x0700,													//!< PGA Gain automatic
}LLD_TYPE_PGA_GAIN;
// ----------------------------------------------------------------------------
//! \brief enum for ADC_CFG register ADC1_A resolution
typedef enum {
	ADC1_A_13bit     = 0x0000,													//!< 13bit resolution
	ADC1_A_14bit     = 0x0010,													//!< 14bit resolution
	ADC1_A_15bit     = 0x0020,													//!< 15bit resolution
	ADC1_A_16bit     = 0x0030,													//!< 16bit resolution
}LLD_TYPE_ADC1_A_RES;
// ----------------------------------------------------------------------------
//! \brief enum for ADC_CFG register ADC1_B resolution
typedef enum {
	ADC1_B_13bit     = 0x0000,													//!< 13bit resolution
	ADC1_B_14bit     = 0x0004,													//!< 14bit resolution
	ADC1_B_15bit     = 0x0008,													//!< 15bit resolution
	ADC1_B_16bit     = 0x000C,													//!< 16bit resolution
}LLD_TYPE_ADC1_B_RES;
// ----------------------------------------------------------------------------
//! \brief enum for ADC_CFG register ADC2 resolution
typedef enum {
	ADC2_13bit     = 0x0000,													//!< 13bit resolution
	ADC2_14bit     = 0x0001,													//!< 14bit resolution
	ADC2_15bit     = 0x0002,													//!< 15bit resolution
	ADC2_16bit     = 0x0003,													//!< 16bit resolution
}LLD_TYPE_ADC2_RES;
// ----------------------------------------------------------------------------
// some register bit definitions
#define INIT_BUS_SW		    		(BIT(4))									//!< Bus Switch Control
#define SYS_CFG_GLOBAL_GO2SLEEP    	(BIT(0))									//!< Goto sleep bit

#define SYS_CFG1_CB_DRVEN           (BIT(7)) 									//!< General enable or disable for all cell balance drivers.
#define SYS_CFG1_I_MEAS_EN 			(BIT(9))									//!< ISENSE measurement enable bit
#define SYS_CFG1_GODIAG  			(BIT(6))									//!< Goto diag bit
#define SYS_CFG1_CB_MANUAL_PAUSE	(BIT(5))									//!< Cell balancing manual pause
#define SYS_CFG1_SOFT_RST			(BIT(4))									//!< Soft reset bit

#define SYS_DIAG_DA_DIAG    		(BIT(7))									//!< Diagnostics Differential amplifier diagnostic. Diagnostic mode function only
#define SYS_DIAG_POLARITY    		(BIT(6))									//!< Control bit used in terminal leakage detection. Controls the polarity between the level shifter and the ADC1-A and ADC1-B converters
#define SYS_DIAG_CT_LEAK_DIAG  		(BIT(5))									//!< Control bit used in terminal leakage detection. Commands the MUX to route the CTx/CBx pin to ADC1-A,B converters. This bit must be exclusive vs. DA_DIAG
#define SYS_DIAG_CT_OV_UV    		(BIT(4))									//!< Diagnostics Cell Terminal OV/UV bit
#define SYS_DIAG_CT_OL_ODD    		(BIT(3))									//!< Diagnostics Cell Terminal Open load Odd bit
#define SYS_DIAG_CT_OL_EVEN   		(BIT(2))									//!< Diagnostics Cell Terminal Open load Even bit

#define ADC_CFG_SOC  		  		(BIT(11))									//!< ADC Start of Conversion bit
#define MEAS_DATA_RDY    			(BIT(15))									//!< Data Ready Bit
#define CB_CFG_CB_EN				(BIT(9))									//!< Cell Balance enable
// ----------------------------------------------------------------------------
//! \brief enum for memory address accesses (register addresses)
typedef enum {
	Reserved      	= 0x00,														//!< 0x00 reserved
	INIT          	= 0x01,														//!< 0x01 device initialisation	   (Global write is forbidden for CID)
	SYS_CFG_GLOBAL	= 0x02,														//!< 0x02 global system configuration (GLOBAL access only and no ECHO in transformer mode, SPI mode this is operates as a standard write.)
	SYS_CFG1      	= 0x03,														//!< 0x03 system configuration
	SYS_CFG2      	= 0x04,														//!< 0x04 system configuration
	SYS_DIAG      	= 0x05,														//!< 0x05 system diagnostics
	ADC_CFG       	= 0x06,														//!< 0x06 ADC configuration
	ADC2_OFFSET_COMP= 0x07,   													//!< 0x07 ADC2 offset compensation
	OV_UV_EN        = 0x08,														//!< 0x08 CT measurement selection
	CELL_OV_FLT 	= 0x09,														//!< 0x09 CT over-voltage fault
	CELL_UV_FLT 	= 0x0A,														//!< 0x0A CT under-voltage fault
	//Reserved        0x0B
	CB1_CFG			= 0x0C,														//!< 0x0C CB configuration for cell 1
	CB2_CFG			= 0x0D,														//!< 0x0D CB configuration for cell 2
	CB3_CFG			= 0x0E,														//!< 0x0E CB configuration for cell 3
	CB4_CFG			= 0x0F,														//!< 0x0F CB configuration for cell 4
	CB5_CFG			= 0x10,														//!< 0x10 CB configuration for cell 5
	CB6_CFG			= 0x11,														//!< 0x11 CB configuration for cell 6
	CB7_CFG			= 0x12,														//!< 0x12 CB configuration for cell 7
	CB8_CFG			= 0x13,														//!< 0x13 CB configuration for cell 8
	CB9_CFG			= 0x14,														//!< 0x14 CB configuration for cell 9
	CB10_CFG		= 0x15,														//!< 0x15 CB configuration for cell 10
	CB11_CFG		= 0x16,														//!< 0x16 CB configuration for cell 11
	CB12_CFG		= 0x17,														//!< 0x17 CB configuration for cell 12
	CB13_CFG		= 0x18,														//!< 0x18 CB configuration for cell 13
	CB14_CFG		= 0x19,														//!< 0x19 CB configuration for cell 14
	CB_OPEN_FLT 	= 0x1A,														//!< 0x1A Open CB fault
	CB_SHORT_FLT 	= 0x1B,														//!< 0x1B Short CB fault
	CB_DRV_STS		= 0x1C,														//!< 0x1C CB driver status
	GPIO_CFG1 		= 0x1D,														//!< 0x1D GPIO configuration
	GPIO_CFG2		= 0x1E,														//!< 0x1E GPIO configuration
	GPIO_STS 		= 0x1F,														//!< 0x1F GPIO diagnostic
	AN_OT_UT_FLT 	= 0x20,														//!< 0x20 AN over and undertemp
	GPIO_SHORT_Anx_OPEN_STS = 0x21, 											//!< 0x21 Short GPIO / Open AN diagnostic
	I_STATUS		= 0x22, 													//!< 0x22 This register contains PGA DAC value
	COM_STATUS    	= 0x23,     												//!< 0x23 Number of CRC error counted
	FAULT1_STATUS 	= 0x24,	  													//!< 0x24 Fault status
	FAULT2_STATUS 	= 0x25,	  													//!< 0x25 Fault status
	FAULT3_STATUS 	= 0x26,	  													//!< 0x26 Fault status
	FAULT_MASK1   	= 0x27,	  													//!< 0x27 Fault pin mask
	FAULT_MASK2   	= 0x28,	  													//!< 0x28 Fault pin mask
	FAULT_MASK3   	= 0x29,	  													//!< 0x29 Fault pin mask
	WAKEUP_MASK1  	= 0x2A,	  													//!< 0x2A Wakeup events mask
	WAKEUP_MASK2  	= 0x2B,	  													//!< 0x2B Wakeup events mask
	WAKEUP_MASK3  	= 0x2C,	  													//!< 0x2C Wakeup events mask
	CC_NB_SAMPLES 	= 0x2D,	  													//!< 0x2D Number of samples in coulomb counter
	COULOUMB_CNT1 	= 0x2E,     												//!< 0x2E..2F Couloumb counter accumulator
	COULOUMB_CNT2 	= 0x2F,
	MEAS_ISENSE1  	= 0x30,     												//!< 0x30 ISENSE measurement
	MEAS_ISENSE2  	= 0x31,		    											//!< 0x31 ISENSE measurement
	MEAS_STACK    	= 0x32,     												//!< 0x32 stack  voltage measurement
	MEAS_CELL14   	= 0x33,														//!< 0x33 cell14 voltage measurement
	MEAS_CELL13   	= 0x34,     												//!< 0x34 cell13 voltage measurement
	MEAS_CELL12   	= 0x35,     												//!< 0x35 cell12 voltage measurement
	MEAS_CELL11   	= 0x36,     												//!< 0x36 cell11 voltage measurement
	MEAS_CELL10   	= 0x37,     												//!< 0x37 cell10 voltage measurement
	MEAS_CELL9    	= 0x38,     												//!< 0x38 cell9  voltage measurement
	MEAS_CELL8    	= 0x39,     												//!< 0x39 cell8  voltage measurement
	MEAS_CELL7    	= 0x3A,     												//!< 0x3A cell7  voltage measurement
	MEAS_CELL6    	= 0x3B,     												//!< 0x3B cell6  voltage measurement
	MEAS_CELL5    	= 0x3C,     												//!< 0x3C cell5  voltage measurement
	MEAS_CELL4    	= 0x3D,     												//!< 0x3D cell4  voltage measurement
	MEAS_CELL3    	= 0x3E,     												//!< 0x3E cell3  voltage measurement
	MEAS_CELL2    	= 0x3F,     												//!< 0x3F cell2  voltage measurement
	MEAS_CELL1    	= 0x40,     												//!< 0x40 cell1  voltage measurement
	MEAS_AN6        = 0x41, 													//!< 0x41 AN6 voltage measurement
	MEAS_AN5        = 0x42,         											//!< 0x42 AN5 voltage measurement
	MEAS_AN4        = 0x43,         											//!< 0x43 AN4 voltage measurement
	MEAS_AN3        = 0x44,         											//!< 0x44 AN3 voltage measurement
	MEAS_AN2        = 0x45,         											//!< 0x45 AN2 voltage measurement
	MEAS_AN1        = 0x46,         											//!< 0x46 AN1 voltage measurement
	MEAS_AN0        = 0x47,         											//!< 0x47 AN0 voltage measurement
	MEAS_IC_TEMP    = 0x48,         											//!< 0x48 IC temperature measurement
	MEAS_VBG_DIAG_ADC1A = 0x49,     											//!< 0x49 ADCIA Band Gap Reference measurement
	MEAS_VBG_DIAG_ADC1B = 0x4A,     											//!< 0x4A ADCIB Band Gap Reference measurement
	TH_ALL_CT       = 0x4B,         											//!< 0x4B CTx over and undervoltage threshold
	TH_CT14         = 0x4C,         											//!< 0x4C CT14 over and undervoltage threshold
	TH_CT13         = 0x4D,         											//!< 0x4D CT13 over and undervoltage threshold
	TH_CT12         = 0x4E,         											//!< 0x4E CT12 over and undervoltage threshold
	TH_CT11         = 0x4F,         											//!< 0x4F CT11 over and undervoltage threshold
	TH_CT10         = 0x50,         											//!< 0x50 CT10 over and undervoltage threshold
	TH_CT9          = 0x51,         											//!< 0x51 CT9 over and undervoltage threshold
	TH_CT8          = 0x52,         											//!< 0x52 CT8 over and undervoltage threshold
	TH_CT7          = 0x53,         											//!< 0x53 CT7 over and undervoltage threshold
	TH_CT6          = 0x54,         											//!< 0x54 CT6 over and undervoltage threshold
	TH_CT5          = 0x55,         											//!< 0x55 CT5 over and undervoltage threshold
	TH_CT4          = 0x56,         											//!< 0x56 CT4 over and undervoltage threshold
	TH_CT3          = 0x57,         											//!< 0x57 CT3 over and undervoltage threshold
	TH_CT2          = 0x58,         											//!< 0x58 CT2 over and undervoltage threshold
	TH_CT1          = 0x59,         											//!< 0x59 CT1 over and undervoltage threshold
	TH_AN6_OT       = 0x5A,         											//!< 0x5A AN6 overtemp threshold
	TH_AN5_OT       = 0x5B,         											//!< 0x5B AN5 overtemp threshold
	TH_AN4_OT       = 0x5C,         											//!< 0x5C AN4 overtemp threshold
	TH_AN3_OT       = 0x5D,         											//!< 0x5D AN3 overtemp threshold
	TH_AN2_OT       = 0x5E,         											//!< 0x5E AN2 overtemp threshold
	TH_AN1_OT       = 0x5F,         											//!< 0x5F AN1 overtemp threshold
	TH_AN0_OT       = 0x60,         											//!< 0x60 AN0 overtemp threshold
	TH_AN6_UT       = 0x61,         											//!< 0x61 AN6 undertemp threshold
	TH_AN5_UT       = 0x62,         											//!< 0x62 AN5 undertemp threshold
	TH_AN4_UT       = 0x63,         											//!< 0x63 AN4 undertemp threshold
	TH_AN3_UT       = 0x64,         											//!< 0x64 AN3 undertemp threshold
	TH_AN2_UT       = 0x65,         											//!< 0x65 AN2 undertemp threshold
	TH_AN1_UT       = 0x66,         											//!< 0x66 AN1 undertemp threshold
	TH_AN0_UT       = 0x67,         											//!< 0x67 AN0 undertemp threshold
	TH_ISENSE_OC    = 0x68,         											//!< 0x68 ISENSE over current threshold
	TH_COULOMB_H    = 0x69,         											//!< 0x69 Over Coulomb counter threshold registers�TH_COULOMB_CNT
	TH_COULOMB_L    = 0x6A,         											//!< 0x6A Over Coulomb counter threshold registers�TH_COULOMB_CNT
	SILICON_REV     = 0x6B,         											//!< 0x6B silicon revision
	EEPROM_CNTL     = 0x6C,         											//!< 0x6C EEPROM transfere control
	DED_ENCODE1     = 0x6D,         											//!< 0x6D ECC signature 1
	DED_ENCODE2     = 0x6E,         											//!< 0x6E ECC signature 2
	FUSE_MIRROR_DATA= 0x6F,														//!< 0x6F Fuse Mirror data
	FUSE_MIRROR_CTRL= 0x70,														//!< 0x70 Fuse Mirror control
}LLD_TYPE_REG_NAME;



#define ADC_CFG_DEFAULT     (PGA_GAIN_AUTO |ADC1_A_14bit|ADC1_B_14bit|ADC2_16bit)				// reset status
#define ADC_CFG_SETTING     (PGA_GAIN_AUTO |ADC1_A_16bit|ADC1_B_16bit|ADC2_16bit)
#define FACTOR_THRESHOLD (256.0 / 5.0)																		   //!< resolution [V/lsb]  of thresholds
#define TH_OVUV_VALUE(ov, uv) ((u16)(((u8)(ov * FACTOR_THRESHOLD) << 8) | ((u8)(uv * FACTOR_THRESHOLD) << 0))) //!< macro to handle /  translate OV and UV thresholds


// ----------------------------------------------------------------------------
// local routines
void SPICSB(u8 u8Level);
u8 IntbPinStatus(void);
void TplEnable(u8 bEnable);
u8 FaultPinStatus(void);
void initFIMode(u8 u8enabledDisabled);

u8 lld3377xCrcCalc(u8 *data, u16 length);
void lld3377xPackFrame(u8 *pu8Buf, u16 data, u8 addr, u8 CID, u8 cmd);
u8 lld3377xNewRCValue(void);
bool lld3377xSetTagID(u8 CID, u8 NewTagId);
// ----------------------------------------------------------------------------
bool lld3377xInitDriver(TYPE_INTERFACE *interface);
bool lld3377xInitCluster(LLD_TYPE_CLUSTER *cluster);
u8 lld3377xNoOfCTs(TYPE_CHIP chip);
// ----------------------------------------------------------------------------
// low level routines for BCC access (SPI & TPL)
bool lld3377xWakeUp(void);
bool lld3377xReadRegisters(u8 CID, u8 Register, u8 noRegs2Read, u16 *readData);
bool lld3377xWriteRegister(u8 CID, u8 Register, u16 writeData, u16 *returnData);
// ----------------------------------------------------------------------------
// low level routines for BCC access (TPL only)
bool lld3377xTPLEnable(void);
bool lld3377xTPLDisable(void);
bool lld3377xWriteGlobalRegister(u8 Register, u16 writeData);
// ----------------------------------------------------------------------------
//bool lld3377xNOPRegister(u8 CID, u8 Register, u16 writeData, u16 *readData);
// ----------------------------------------------------------------------------
// low lever driver error handling routines
bool _lld3377xSetError(LLD_TYPE_RETURN res);									//!< can be used to simulate errors
bool lld3377xGetError(LLD_TYPE_RETURN *errorCode);
void lld3377xClearError(void);

u32 crc8_test(void);


// ----------------------------------------------------------------------------
#endif /* COTS_SLAVECONTROLIF_INC_SLAVEIF_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------



