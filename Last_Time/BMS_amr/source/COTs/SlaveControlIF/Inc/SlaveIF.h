/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  Slave_Control_IF driver
 *	File: 		SlaveIF.h
 */

#ifndef SLAVE_CENTER_IF_H
#define SLAVE_CENTER_IF_H

#include <stdio.h>
#include <string.h>
#include <MKL25Z4.h>
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include <COTs/NTC/Inc/ntc.h>
// #include "source/drivers/lld3377x.h"
#include <source/COTs/SlaveControlIF/Inc/SlaveIF_Cfg.h>
#include "drivers/SPI.h"																// for memcmp()
#include "drivers/tpm1.h"																// for Delay
#include "drivers/uart0.h"																// for UART
#include "board.h"




// status pins
#define SETSTAT0(v)		if(v) GPIOB_PSOR = BIT(0); else GPIOB_PCOR = BIT(0)
#define SETSTAT1(v)		if(v) GPIOB_PSOR = BIT(1); else GPIOB_PCOR = BIT(1)
#define SETSTAT1Tog()	    ( GPIOB_PTOR = BIT(1))

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
//-----------------------------------------------------------------------------



// ----------------------------------------------------------------------------
//! \brief structures to hold different interface configurations
typedef enum{
	IntUnknown = 0,																//!< unknown
	IntSPI = 1,																	//!< direct connection \ref secspisetup
	IntTPL = 2																	//!< \ref sectplsetup
}TYPE_INTERFACE;
// ----------------------------------------------------------------------------
//! \brief structures to hold different SPI<>EVB interface configurations
typedef enum{
	EVB_Unknown   = 0,
	EVB_Type1     = 1,															//!< type 1 EVB \ref pageevb
	EVB_TypeArd   = 2,															//!< type arduino EVB \ref pageevb
}TYPE_EVB;

//  ----------------------------------------------------------------------------
#define FACTOR_THRESHOLD (256.0 / 5.0)																		   //!< resolution [V/lsb]  of thresholds
#define TH_OVUV_VALUE(ov, uv) ((u16)(((u8)(ov * FACTOR_THRESHOLD) << 8) | ((u8)(uv * FACTOR_THRESHOLD) << 0))) //!< macro to handle /  translate OV and UV thresholds
// ----------------------------------------------------------------------------
#define NO_CLUSTER (15u) //!< macro for readability
#define NO_CELLS (14u)	 //!< macro for readability
#define NO_AN (7u)		 //!< macro for readability
#define CELL1 (0U)		 //!< macro for readability
#define CELL2 (1U)		 //!< macro for readability
#define CELL3 (2U)		 //!< macro for readability
#define CELL4 (3U)		 //!< macro for readability
#define CELL5 (4U)		 //!< macro for readability
#define CELL6 (5U)		 //!< macro for readability
#define CELL7 (6U)		 //!< macro for readability
#define CELL8 (7U)		 //!< macro for readability
#define CELL9 (8U)		 //!< macro for readability
#define CELL10 (9U)		 //!< macro for readability
#define CELL11 (10U)	 //!< macro for readability
#define CELL12 (11U)	 //!< macro for readability
#define CELL13 (12U)	 //!< macro for readability
#define CELL14 (13U)	 //!< macro for readability
// ----------------------------------------------------------------------------
#define S19EXTENT (0xFFF80000UL)										 //!< macro to extend negative s19 to s32
#define S19SignExtend(s19) ((s19) & BIT(18)) ? (s19) | S19EXTENT : (s19) //!< macro to sign extend s19 bit values to s32
#define S19_NMAX 0x00040000UL											 //!< -2^18   = -262144
#define S19_MAX 0x0003FFFFUL											 //!< 2^18 -1 = 262143
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
typedef struct
{
	u8 regAdr;	  //!< register address to be written to
	u16 regValue; //!< register value to write
} TYPE_BCC_CONF;
extern const TYPE_BCC_CONF CONF33771TPL[];

// ------------------------------------------------------------------
//! \brief enum for CIDs (SPI version) //Abdullah
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
typedef enum
{
	//	BMS_Unknown 	= 0,														//!< used by GUI
	BMS_Init = 1,	  //!< init phase is assigning CID
	BMS_Config = 2,	  //!< config phase applies initial loading of registers
	BMS_Running = 3,  //!< bms is running
	BMS_Sleeping = 4, //!< bms is sleeping
	BMS_Error = 5,	  //!< error phase
	BMS_Idle = 6	  //!< BMS_Idle
} TYPE_BMS_STATUS;

// ----------------------------------------------------------------------------



//! \brief structure to hold the BMS status //Abdullah 
typedef struct
{
	TYPE_BMS_STATUS Status;	  //!< bms state (used by state machine)
	TYPE_INTERFACE Interface; //!< used interface
	TYPE_EVB EVB;			  //!< used EVB
	u8 NoClusters;			  //!< no of clusters attached (1..14)
	u8 CIDcurrent;			  //!< CID(S) which measure current (not used for demo)
} TYPE_BMS;
// ----------------------------------------------------------------------------
//! \brief structure to hold measurement results (read only) information.
typedef struct
{
	s32 s32Current;				  //!< current reading
	u16 u16StackVoltage;		  //!< stack voltage reading
	u16 u16CellVoltage[NO_CELLS]; //!< cell voltage reading
	u16 u16ANVoltage[7];		  //!< ANx readings
	u16 u16ICTemp;				  //!< IC temperature reading
	u16 u16VbgADC1A;			  //!< band gap readings (diagnostics)
	u16 u16VbgADC1B;			  //!< band gap readings (diagnostics)
	u16 u16CCSamples;			  //!< number of CC samples
	s32 s32CCCounter;			  //!< CC counter value
} TYPE_MEAS_RESULTS_RAW;
// ----------------------------------------------------------------------------
//! \brief structure to hold status (read only / clearable) information.
typedef struct
{
	u16 u16CellOV;		//!< cell over voltage
	u16 u16CellUV;		//!< cell under voltage
	u16 u16CBOpen;		//!< cell balancing open
	u16 u16CBShort;		//!< cell balancing short
	u16 u16CBStatus;	//!< cell balancing status
	u16 u16GPIOStatus;	//!< GPIO status
	u16 u16ANOtUt;		//!< AN over/under temperature
	u16 u16GPIOOpen;	//!< GPIO open
	u16 u16IStatus;		//!< ISense Status
	u16 u16Comm;		//!< Comm Status
	u16 u16Fault1;		//!< Fault1 status
	u16 u16Fault2;		//!< Fault2 status
	u16 u16Fault3;		//!< Fault3 status
	u16 u16MeasIsense2; //!< 0x31 register
} TYPE_STATUS;
// ----------------------------------------------------------------------------
//! \brief structure to hold configuration information.
typedef struct
{
	u16 u16Init;			//!<   0x01 register
	u16 u16SysCfgGlobal;	//!<   0x02 register
	u16 u16SysCfg1;			//!<   0x03 register
	u16 u16SysCfg2;			//!<   0x04 register
	u16 u16SysDiag;			//!<   0x05 register
	u16 u16AdcCfg;			//!<   0x06 register
	u16 u16Adc2Comp;		//!<   0x07 register
	u16 u16OvUvEn;			//!<   0x08 register
	u16 u16GPIOCfg1;		//!<   0x1D register
	u16 u16GPIOCfg2;		//!<   0x1E register
	u16 u16GPIOSts;			//!<   0x1F register
	u16 u16FaultMask1;		//!<   0x27 register
	u16 u16FaultMask2;		//!<   0x28 register
	u16 u16FaultMask3;		//!<   0x29 register
	u16 u16WakeupMask1;		//!<   0x2A register
	u16 u16WakeupMask2;		//!<   0x2B register
	u16 u16WakeupMask3;		//!<   0x2C register
	u16 u16CBCfg[NO_CELLS]; //!<   0x0C..0x19  registers
} TYPE_CONFIG;
// ----------------------------------------------------------------------------
//! \brief structure to hold threshold information.
typedef struct
{
	u8 u8ThAllOv;			//!<  0x4B register
	u8 u8ThAllUv;			//!<  0x4B register
	u8 u8ThCTxOv[NO_CELLS]; //!<  0x4C..0x59 registers
	u8 u8ThCTxUv[NO_CELLS]; //!<  0x4C..0x59 registers
	u16 u10ThANxOT[7];		//!<  0x5A..0x60  Over Temperature  (NTC => Undervoltage) registers
	u16 u10ThANxUT[7];		//!<  0x61..0x67  Under Temperature (NTC => Overvoltage)  registers
	u16 u12ThIsenseOC;		//!<  0x68 register
	u32 u32ThCoulombCnt;	//!<  0x69..6A registers
} TYPE_THRESHOLDS;
// ----------------------------------------------------------------------------
//! \brief structure to hold Fuse Mirror Memory data 32 x 16 bits
typedef struct
{
	u16 u16Data[32]; //!< ram buffer to store fuse mirror memory
} TYPE_FUSE_DATA;

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
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// local routines
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
// ----------------------------------------------------------------------------


u32 crc8_test(void);

// ----------------------------------------------------------------------------
// for DAC Board
void SPITxSendBufferNoCSB(u8 *u8TxData, u8 u8TxLen);

// ----------------------------------------------------------------------------
bool MC3377xSleepMode(TYPE_INTERFACE interface);
bool MC3377xNormalMode(TYPE_INTERFACE interface);
bool MC3377xCheck4Wakeup(TYPE_INTERFACE interface);
// ----------------------------------------------------------------------------
bool MC3377xADCStartConversion(u8 cid, u8 u4TagID);
bool MC3377xADCIsConverting(u8 cid);
// ----------------------------------------------------------------------------
bool BMSInit(u8 NoOfNodes);
// ----------------------------------------------------------------------------
bool MC3377xConfig(u8 cid, const TYPE_BCC_CONF conf[]);
// ----------------------------------------------------------------------------
bool MC3377xGetGUID(u8 cid, LLD_TYPE_CLUSTER *pCluster);
bool MC3377xGetSiliconRevision(u8 cid, LLD_TYPE_CLUSTER *pCluster);
bool MC3377xGetSiliconType(u8 cid, LLD_TYPE_CLUSTER *pCluster);
// ----------------------------------------------------------------------------
bool MC3377xGetRawMeasurements(u8 cid, u8 u4TagId, u8 NoCTs, TYPE_MEAS_RESULTS_RAW *RawMeasResults);
bool MC3377xGetStatus(u8 cid, TYPE_STATUS *Status);
bool MC3377xGetThresholds(u8 cid, u8 NoCTs, TYPE_THRESHOLDS *Threshold);
bool MC3377xGetConfig(u8 cid, u8 NoCTs, TYPE_CONFIG *Config);
// ----------------------------------------------------------------------------
bool MC3377xReadFuseMirror(u8 cid, TYPE_FUSE_DATA *fusedata);
// ----------------------------------------------------------------------------
void App(void);
u8 Char2Number(u8 u8Char);
void HandleGUICommands(TYPE_BMS *bms);
void BMSEnableISense(u8 cidex);
// ----------------------------------------------------------------------------
void DebugPrintMeasurements(u8 cid, u8 NoCTs, TYPE_MEAS_RESULTS_RAW *rawResults);
void DebugPrintThresholds(u8 cid, u8 NoCTs, TYPE_THRESHOLDS *Thresholds);
void DebugPrintConfig(u8 cid, u8 u8NoOfCTs, TYPE_CONFIG *ConfigBits);
void DebugPrintStatus(u8 u4CID, TYPE_STATUS *StatusBits);
// ----------------------------------------------------------------------------
void InitInterface(TYPE_INTERFACE interface, TYPE_EVB evb);
void DeInitInterface(void);
u8 FaultPinStatus(void);
void TplEnable(u8 bEnable);
u8 IntbPinStatus(void);
void SPICSB(u8 u8Level);
void initFIMode(u8 u8enabledDisabled);	
// ----------------------------------------------------------------------------
#endif /* SLAVE_CENTER_IF_H */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------








