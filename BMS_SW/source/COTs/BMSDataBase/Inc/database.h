/*
 * database.h
 *
 *  Created on: Jun 1, 2025
 *      Author: abdal
 */

#ifndef COTS_BMSDATABASE_INC_DATABASE_H_
#define COTS_BMSDATABASE_INC_DATABASE_H_
#include <COTs/DebugInfoManager/Inc/debugInfo.h> // for memcmp
#include <COTs/SlaveControlIF/Inc/slaveIF.h>
#include <COTs/ThermalManager/Inc/tempSens.h> // table for NTC resistor characteristics
#include <tpm1.h>							// for Delay

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
#define S19EXTENT (0xFFF80000UL)										 //!< macro to extend negative s19 to int32_t
#define S19SignExtend(s19) ((s19) & BIT(18)) ? (s19) | S19EXTENT : (s19) //!< macro to sign extend s19 bit values to int32_t
#define S19_NMAX 0x00040000UL											 //!< -2^18   = -262144
#define S19_MAX 0x0003FFFFUL											 //!< 2^18 -1 = 262143
extern uint16_t falg_temp ;
// ----------------------------------------------------------------------------
/*! \brief enum for BMS modes

The graph below shows to typical BMS state machine and transistions:

\dot
digraph G {
	  BMS_Unknown -> BMS_Init;
	  BMS_Init -> BMS_Config -> BMS_Running;
	  BMS_Running -> BMS_Sleeping [label = "goto sleep"];
	  BMS_Sleeping -> BMS_Running [label = "wakeup"];
	  BMS_Init -> BMS_Error;
	  BMS_Running -> BMS_Error;
	  BMS_Sleeping -> BMS_Error;
}
\enddot

 */
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
//! \brief structure to hold the BMS status
typedef struct
{
	TYPE_BMS_STATUS Status;	  //!< bms state (used by state machine)
	TYPE_INTERFACE Interface; //!< used interface
	TYPE_EVB EVB;			  //!< used EVB
	uint8_t NoClusters;		  //!< no of clusters attached (1..14)
	uint8_t CIDcurrent;		  //!< CID(S) which measure current (not used for demo)
} TYPE_BMS;
// ----------------------------------------------------------------------------
//! \brief structure to hold measurement results (read only) information.
typedef struct
{
	int32_t s32Current;				   //!< current reading
	uint16_t u16StackVoltage;		   //!< stack voltage reading
	uint16_t u16CellVoltage[NO_CELLS]; //!< cell voltage reading
	uint16_t u16ANVoltage[7];		   //!< ANx readings
	uint16_t u16ICTemp;				   //!< IC temperature reading
	uint16_t u16VbgADC1A;			   //!< band gap readings (diagnostics)
	uint16_t u16VbgADC1B;			   //!< band gap readings (diagnostics)
	uint16_t u16CCSamples;			   //!< number of CC samples
	int32_t s32CCCounter;			   //!< CC counter value
} TYPE_MEAS_RESULTS_RAW;
// ----------------------------------------------------------------------------
//! \brief structure to hold status (read only / clearable) information.
typedef struct
{
	uint16_t u16CellOV;		 //!< cell over voltage
	uint16_t u16CellUV;		 //!< cell under voltage
	uint16_t u16CBOpen;		 //!< cell balancing open
	uint16_t u16CBShort;	 //!< cell balancing short
	uint16_t u16CBStatus;	 //!< cell balancing status
	uint16_t u16GPIOStatus;	 //!< GPIO status
	uint16_t u16ANOtUt;		 //!< AN over/under temperature
	uint16_t u16GPIOOpen;	 //!< GPIO open
	uint16_t u16IStatus;	 //!< ISense Status
	uint16_t u16Comm;		 //!< Comm Status
	uint16_t u16Fault1;		 //!< Fault1 status
	uint16_t u16Fault2;		 //!< Fault2 status
	uint16_t u16Fault3;		 //!< Fault3 status
	uint16_t u16MeasIsense2; //!< 0x31 register
} TYPE_STATUS;
// ----------------------------------------------------------------------------
//! \brief structure to hold configuration information.
typedef struct
{
	uint16_t u16Init;			 //!<   0x01 register
	uint16_t u16SysCfgGlobal;	 //!<   0x02 register
	uint16_t u16SysCfg1;		 //!<   0x03 register
	uint16_t u16SysCfg2;		 //!<   0x04 register
	uint16_t u16SysDiag;		 //!<   0x05 register
	uint16_t u16AdcCfg;			 //!<   0x06 register
	uint16_t u16Adc2Comp;		 //!<   0x07 register
	uint16_t u16OvUvEn;			 //!<   0x08 register
	uint16_t u16GPIOCfg1;		 //!<   0x1D register
	uint16_t u16GPIOCfg2;		 //!<   0x1E register
	uint16_t u16GPIOSts;		 //!<   0x1F register
	uint16_t u16FaultMask1;		 //!<   0x27 register
	uint16_t u16FaultMask2;		 //!<   0x28 register
	uint16_t u16FaultMask3;		 //!<   0x29 register
	uint16_t u16WakeupMask1;	 //!<   0x2A register
	uint16_t u16WakeupMask2;	 //!<   0x2B register
	uint16_t u16WakeupMask3;	 //!<   0x2C register
	uint16_t u16CBCfg[NO_CELLS]; //!<   0x0C..0x19  registers
} TYPE_CONFIG;
// ----------------------------------------------------------------------------
//! \brief structure to hold threshold information.
typedef struct
{
	uint8_t u8ThAllOv;			 //!<  0x4B register
	uint8_t u8ThAllUv;			 //!<  0x4B register
	uint8_t u8ThCTxOv[NO_CELLS]; //!<  0x4C..0x59 registers
	uint8_t u8ThCTxUv[NO_CELLS]; //!<  0x4C..0x59 registers
	uint16_t u10ThANxOT[7];		 //!<  0x5A..0x60  Over Temperature  (NTC => Undervoltage) registers
	uint16_t u10ThANxUT[7];		 //!<  0x61..0x67  Under Temperature (NTC => Overvoltage)  registers
	uint16_t u12ThIsenseOC;		 //!<  0x68 register
	uint32_t u32ThCoulombCnt;	 //!<  0x69..6A registers
} TYPE_THRESHOLDS;
// ----------------------------------------------------------------------------
////! \brief structure to debug diagnostic test results
// typedef struct{
//	// OV/UV functional verification
//	uint16_t u16OvOdd;          	                                    				//!< for debugging
//	uint16_t u16UvOdd;          	                                    				//!< for debugging
//	uint16_t u16OvEven;         	                                    				//!< for debugging
//	uint16_t u16UvEven;         	                                    				//!< for debugging
//	// CTx open detect
//	uint16_t u16CTOpen;  															//!< CT1 -> bit0
//	uint16_t u16CTResults[NO_CELLS];													//!< actual measurements
// }TYPE_DIAGNOSTICS;
//  ----------------------------------------------------------------------------
//! \brief structure to hold Fuse Mirror Memory data 32 x 16 bits
typedef struct
{
	uint16_t u16Data[32]; //!< ram buffer to store fuse mirror memory
} TYPE_FUSE_DATA;
// ----------------------------------------------------------------------------
bool MC3377xSleepMode(TYPE_INTERFACE interface);
bool MC3377xNormalMode(TYPE_INTERFACE interface);
bool MC3377xCheck4Wakeup(TYPE_INTERFACE interface);
// ----------------------------------------------------------------------------
bool MC3377xADCStartConversion(uint8_t cid, uint8_t u4TagID);
bool MC3377xADCIsConverting(uint8_t cid);
// ----------------------------------------------------------------------------
bool BMSInit(uint8_t NoOfNodes);
// ----------------------------------------------------------------------------
bool MC3377xConfig(uint8_t cid, const SsysConf_t conf[]);
// ----------------------------------------------------------------------------
bool MC3377xGetSiliconRevision(uint8_t cid, SclusterInfo_t *pCluster);
bool MC3377xGetSiliconType(uint8_t cid, SclusterInfo_t *pCluster);
// ----------------------------------------------------------------------------
bool MC3377xGetRawMeasurements(uint8_t cid, uint8_t u4TagId, uint8_t NoCTs, TYPE_MEAS_RESULTS_RAW *RawMeasResults);
bool MC3377xGetStatus(uint8_t cid, TYPE_STATUS *Status);
bool MC3377xGetThresholds(uint8_t cid, uint8_t NoCTs, TYPE_THRESHOLDS *Threshold);
// ----------------------------------------------------------------------------
bool MC3377xReadFuseMirror(uint8_t cid, TYPE_FUSE_DATA *fusedata);
bool Abdullah_Temp(TYPE_MEAS_RESULTS_RAW *RawMeasResults);

#endif /* COTS_BMSDATABASE_INC_DATABASE_H_ */
