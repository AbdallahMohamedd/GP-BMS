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
//! \addtogroup project
// @{
/*! \brief Application level MC33771 and MC33772 routines.
 * 
 * This module provides application level functions to access MC3377x.
 * 
 * 
 * \sa lld3377x for low level access routines, like MC3377xReadRegisters
 * 
 * 
 * 
 * 
 * 
 * 
 */ 

//! \addtogroup mc3377x
// @{
// ----------------------------------------------------------------------------
#ifndef MC3377x_H_
#define MC3377x_H_
// ----------------------------------------------------------------------------
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include <MKL25Z4.h>
#include "config.h"
#include "source/drivers/lld3377x.h"           // low level access
#include <string.h>																// for memcmp
#include <tpm1.h>													// for Delay
#include "Platform/ntc.h"																// table for NTC resistor characteristics
// ----------------------------------------------------------------------------
#define FACTOR_THRESHOLD   (256.0/5.0)											//!< resolution [V/lsb]  of thresholds
#define TH_OVUV_VALUE(ov, uv)   ((u16)(((u8)(ov*FACTOR_THRESHOLD)<<8) | ((u8)(uv*FACTOR_THRESHOLD)<<0))) //!< macro to handle /  translate OV and UV thresholds
// ----------------------------------------------------------------------------
#define NO_CLUSTER  	(15u)													//!< macro for readability
#define NO_CELLS       	(14u)	                                                //!< macro for readability
#define NO_AN           (7u)                                                    //!< macro for readability
#define CELL1   		(0U)		                                            //!< macro for readability
#define CELL2   		(1U) 		                                            //!< macro for readability
#define CELL3   		(2U)		                                            //!< macro for readability
#define CELL4   		(3U)		                                            //!< macro for readability
#define CELL5   		(4U)		                                            //!< macro for readability
#define CELL6   		(5U)		                                            //!< macro for readability
#define CELL7   		(6U)		                                            //!< macro for readability
#define CELL8   		(7U)		                                            //!< macro for readability
#define CELL9   		(8U)		                                            //!< macro for readability
#define CELL10  		(9U)		                                            //!< macro for readability
#define CELL11  		(10U)		                                            //!< macro for readability
#define CELL12  		(11U)		                                            //!< macro for readability
#define CELL13  		(12U)		                                            //!< macro for readability
#define CELL14  		(13U)		                                            //!< macro for readability
// ----------------------------------------------------------------------------
#define S19EXTENT			(0xFFF80000UL)										//!< macro to extend negative s19 to s32								 													
#define S19SignExtend(s19)  ((s19)&BIT(18))? (s19)|S19EXTENT : (s19)  			//!< macro to sign extend s19 bit values to s32 													
#define S19_NMAX  		     0x00040000UL										//!< -2^18   = -262144
#define S19_MAX  		     0x0003FFFFUL										//!< 2^18 -1 = 262143
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
typedef enum {
//	BMS_Unknown 	= 0,														//!< used by GUI
	BMS_Init  		= 1,														//!< init phase is assigning CID
	BMS_Config		= 2,														//!< config phase applies initial loading of registers
	BMS_Running    	= 3,														//!< bms is running
	BMS_Sleeping   	= 4,														//!< bms is sleeping
	BMS_Error      	= 5,														//!< error phase
	BMS_Idle        = 6 														//!< BMS_Idle
}TYPE_BMS_STATUS;
// ----------------------------------------------------------------------------
//! \brief structure to hold the BMS status
typedef struct{
	TYPE_BMS_STATUS Status;														//!< bms state (used by state machine) 
	TYPE_INTERFACE Interface;													//!< used interface 
	TYPE_EVB EVB;																//!< used EVB
	u8 NoClusters;																//!< no of clusters attached (1..14)
	u8 CIDcurrent;																//!< CID(S) which measure current (not used for demo)
}TYPE_BMS;
// ----------------------------------------------------------------------------
//! \brief structure to hold measurement results (read only) information.	
typedef struct{
	s32 s32Current;																//!< current reading 
	u16 u16StackVoltage;														//!< stack voltage reading
	u16 u16CellVoltage[NO_CELLS];												//!< cell voltage reading
	u16 u16ANVoltage[7];														//!< ANx readings
	u16 u16ICTemp;																//!< IC temperature reading
	u16 u16VbgADC1A;															//!< band gap readings (diagnostics)
	u16 u16VbgADC1B;															//!< band gap readings (diagnostics)
	u16 u16CCSamples;															//!< number of CC samples
	s32 s32CCCounter;															//!< CC counter value
}TYPE_MEAS_RESULTS_RAW;
// ----------------------------------------------------------------------------
//! \brief structure to hold status (read only / clearable) information.	
typedef struct{
	u16 u16CellOV;																//!< cell over voltage
	u16 u16CellUV;                                                              //!< cell under voltage
	u16 u16CBOpen;                                                              //!< cell balancing open
	u16 u16CBShort;                                                             //!< cell balancing short
	u16 u16CBStatus;                                                            //!< cell balancing status
	u16 u16GPIOStatus;                                                          //!< GPIO status
	u16 u16ANOtUt;                                                              //!< AN over/under temperature 
	u16 u16GPIOOpen;                                                            //!< GPIO open
	u16 u16IStatus;                                                             //!< ISense Status
	u16 u16Comm;                                                                //!< Comm Status
	u16 u16Fault1;                                                              //!< Fault1 status 
	u16 u16Fault2;                                                              //!< Fault2 status
	u16 u16Fault3;                                                              //!< Fault3 status
	u16 u16MeasIsense2;															//!< 0x31 register
}TYPE_STATUS;
// ----------------------------------------------------------------------------
//! \brief structure to hold configuration information. 
typedef struct{
	u16 u16Init;		 	                                        			//!<   0x01 register     
	u16 u16SysCfgGlobal; 	                                        			//!<   0x02 register     
	u16 u16SysCfg1;		 	                                        			//!<   0x03 register     
	u16 u16SysCfg2;		 	                                        			//!<   0x04 register     
	u16 u16SysDiag;			                                        			//!<   0x05 register     
	u16 u16AdcCfg;		 	                                        			//!<   0x06 register     
	u16 u16Adc2Comp;	 	                                        			//!<   0x07 register     
	u16 u16OvUvEn;			                                        			//!<   0x08 register     
	u16 u16GPIOCfg1;		                                        			//!<   0x1D register     
	u16 u16GPIOCfg2;		                                    				//!<   0x1E register     
	u16 u16GPIOSts;			                                        			//!<   0x1F register     
	u16 u16FaultMask1;		                                        			//!<   0x27 register     
	u16 u16FaultMask2;		                                        			//!<   0x28 register     
	u16 u16FaultMask3;		                                        			//!<   0x29 register     
	u16 u16WakeupMask1;		                                        			//!<   0x2A register     
	u16 u16WakeupMask2;		                                        			//!<   0x2B register     
	u16 u16WakeupMask3;		                                        			//!<   0x2C register     
	u16 u16CBCfg[NO_CELLS];	    	                                    		//!<   0x0C..0x19  registers
}TYPE_CONFIG;
// ----------------------------------------------------------------------------
//! \brief structure to hold threshold information. 
typedef struct{
	u8 u8ThAllOv;			 	                                    			//!<  0x4B register
	u8 u8ThAllUv;			 	                                    			//!<  0x4B register
	u8 u8ThCTxOv[NO_CELLS];		 	                                    		//!<  0x4C..0x59 registers
	u8 u8ThCTxUv[NO_CELLS];		 	                                    		//!<  0x4C..0x59 registers
	u16 u10ThANxOT[7];		 	                                    			//!<  0x5A..0x60  Over Temperature  (NTC => Undervoltage) registers
	u16 u10ThANxUT[7];		 	                                    			//!<  0x61..0x67  Under Temperature (NTC => Overvoltage)  registers
	u16 u12ThIsenseOC;		 	                                    			//!<  0x68 register
	u32 u32ThCoulombCnt;	 	                                    			//!<  0x69..6A registers
}TYPE_THRESHOLDS;
// ----------------------------------------------------------------------------
////! \brief structure to debug diagnostic test results
//typedef struct{
//	// OV/UV functional verification
//	u16 u16OvOdd;          	                                    				//!< for debugging
//	u16 u16UvOdd;          	                                    				//!< for debugging
//	u16 u16OvEven;         	                                    				//!< for debugging
//	u16 u16UvEven;         	                                    				//!< for debugging
//	// CTx open detect
//	u16 u16CTOpen;  															//!< CT1 -> bit0
//	u16 u16CTResults[NO_CELLS];													//!< actual measurements	
//}TYPE_DIAGNOSTICS;
// ----------------------------------------------------------------------------
//! \brief structure to hold Fuse Mirror Memory data 32 x 16 bits
typedef struct{
	u16 u16Data[32];															//!< ram buffer to store fuse mirror memory
}TYPE_FUSE_DATA;
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
#endif /* MC3377x_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------

