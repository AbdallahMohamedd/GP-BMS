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
//! \addtogroup platform
// @{
/*! \brief Pack Controller Configuration

	To dynamically (during runtime) handle different hard configurations FLASH 
	memory is used to store different Pack Controller parameters, e.g. EVB and Interface
	type, number of attached clusters.
	The last 1k (of the available 128k) Flash sector is used for the storage.
 
 */
//! \addtogroup pcconf


// @{
// ----------------------------------------------------------------------------
#ifndef PCCONF_H_
#define PCCONF_H_
// ----------------------------------------------------------------------------
#include <cpu/MKL25Z4_extension.h>			//for CACHE disable
#include <C90TFS/drvsrc/include/SSD_FTFx.h>
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include "MKL25Z4.h"
#include <stdio.h>
//#include "frdmkl25z.h"
// ----------------------------------------------------------------------------
#include "MKL25Z4_features.h"			//for NVM driver
// ----------------------------------------------------------------------------
typedef enum {
	MODE_NORMAL_OPERATION =	0,			 										//!< normal operation
	MODE_INIT_AND_QUITE	  = 1,													//!< used for some RE testing
	MODE_INIT_AND_SLEEP	  = 2,													//!< used for FCC testing
	MODE_SINGLE_CLUSTER_01= 3,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_02= 4,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_03= 5,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_04= 6,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_05= 7,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_06= 8,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_07= 9,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_08=10,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_09=11,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_10=12,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_11=13,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_12=14,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_13=15, 													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_14=16,													//!< used for daisy chain evaluations
	MODE_SINGLE_CLUSTER_15=17													//!< used for daisy chain evaluations
}ENUM_PACKCTRL_MODE;
// ----------------------------------------------------------------------------
#define PC_CONFIG_VALID   0x5AA5												//!< pattern to know if Flash configuration is valid
// ----------------------------------------------------------------------------
//! \brief Pack controller configuration (stored in FLASH)
typedef struct {
	uint16_t Initialized;															//!< used for first time init 
	TYPE_INTERFACE IntType;														//!< TPL or SPI
	TYPE_EVB EvbType;															//!< Type1 or TypeArd
	uint8_t NoCluster;																//!< No of clusters connected
	uint8_t NoCells;																	//!< which chip is connected 6:BCC6, 14:BCC14
	ENUM_PACKCTRL_MODE OpMode;													//!< used for special mode handling (e.g. verbose)
	uint8_t CIDcurrent;																//!< CID which performs current measurements
	uint16_t MeasPeriod;																//!< measurement period
	uint8_t Test;																	//!< used for test purposes, e.g. error injection
	// InitConfig
	
	uint32_t AlignDummy;																//!< used for 4 byte alignment
}TYPE_PC_CONFIG;
//---------------------------------------------------------------------
#define PC_CONFIG_SIZE         sizeof(TYPE_PC_CONFIG)
// ----------------------------------------------------------------------------
// FLASH 128K 0x0000_0000..0x0001_FFFF
// 128 sectors (erase size) with  1k (0x400)
// used sector is last sector 127   
#define FLASH_DATA_ADDR        0x1FC00UL   
#define FLASH_SECTOR_SIZE      0x400   
// ----------------------------------------------------------------------------
bool PackCrtlConfigRead(TYPE_PC_CONFIG *conf, TYPE_PC_CONFIG defConfig);

void PcconfUpdate(TYPE_PC_CONFIG *conf, uint8_t idx, uint16_t data);
bool FlashConfigurationRead(TYPE_PC_CONFIG *packConf, uint8_t packConfSize);
bool FlashConfigurationWrite(TYPE_PC_CONFIG *packConf, uint8_t packConfSize);
// ----------------------------------------------------------------------------


// size of array to copy__Launch_Command function to
// It should be at least equal to actual size of __Launch_Command func
// User can change this value based on RAM size availability and actual size of __Launch_Command function
#define LAUNCH_CMD_SIZE           0x100

/* Size of function used for callback.  Change this depending on the size of your function */
#define CALLBACK_SIZE           0x80

#define BUFFER_SIZE_BYTE          0x80

#define DEBUGENABLE               0x00

#define READ_NORMAL_MARGIN        0x00
#define READ_USER_MARGIN          0x01
#define READ_FACTORY_MARGIN       0x02

#define ONE_KB                    1024

#define FTFx_REG_BASE             0x40020000
#define P_FLASH_BASE              0x00000000

#define BACKDOOR_KEY_LOCATION         0x400

// Program flash IFR
#if (FSL_FEATURE_FLASH_IS_FTFE == 1)
#define PFLASH_IFR                0x3C0
#else // FSL_FEATURE_FLASH_IS_FTFL == 1 or FSL_FEATURE_FLASH_IS_FTFA = =1
#define PFLASH_IFR                0xC0
#endif
// Program Flash block information
#define P_FLASH_SIZE            (FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT)
#define P_BLOCK_NUM             FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT
#define P_SECTOR_SIZE           FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
// Data Flash block information
#define FLEXNVM_BASE            FSL_FEATURE_FLASH_FLEX_NVM_START_ADDRESS
#define FLEXNVM_SECTOR_SIZE     FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SECTOR_SIZE
#define FLEXNVM_BLOCK_SIZE      FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SIZE
#define FLEXNVM_BLOCK_NUM       FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_COUNT

// Flex Ram block information
#define EERAM_BASE              FSL_FEATURE_FLASH_FLEX_RAM_START_ADDRESS
#define EERAM_SIZE              FSL_FEATURE_FLASH_FLEX_RAM_SIZE


// Has flash cache control in MCM module
#if (FSL_FEATURE_FLASH_HAS_MCM_FLASH_CACHE_CONTROLS == 1)
#define CACHE_DISABLE             MCM_BWR_PLACR_DFCS(MCM_BASE_PTR, 1);
// Has flash cache control in FMC module
#elif (FSL_FEATURE_FLASH_HAS_FMC_FLASH_CACHE_CONTROLS == 1)
#if defined(FMC_PFB1CR) && defined(FMC_PFB1CR_B1SEBE_MASK)
#define CACHE_DISABLE     FMC_PFB0CR &= ~(FMC_PFB0CR_B0SEBE_MASK | FMC_PFB0CR_B0IPE_MASK | FMC_PFB0CR_B0DPE_MASK | FMC_PFB0CR_B0ICE_MASK | FMC_PFB0CR_B0DCE_MASK);\
                                  FMC_PFB1CR &= ~(FMC_PFB1CR_B1SEBE_MASK | FMC_PFB1CR_B1IPE_MASK | FMC_PFB1CR_B1DPE_MASK | FMC_PFB1CR_B1ICE_MASK | FMC_PFB1CR_B1DCE_MASK);
#else
#define CACHE_DISABLE     FMC_PFB0CR &= ~(FMC_PFB0CR_B0SEBE_MASK | FMC_PFB0CR_B0IPE_MASK | FMC_PFB0CR_B0DPE_MASK | FMC_PFB0CR_B0ICE_MASK | FMC_PFB0CR_B0DCE_MASK);
#endif
#else
// No cache in the device
#define CACHE_DISABLE
#endif

///////////////////////////////////////////////////////////////////////////////
// Prototypes
///////////////////////////////////////////////////////////////////////////////
extern uint32_t RelocateFunction(uint32_t dest, uint32_t size, uint32_t src);

// --------------------------------------------------------------------
// prototypes
// --------------------------------------------------------------------
// --------------------------------------------------------------------
#endif // PCCONF_H_ 
// --------------------------------------------------------------------
// @}
// @}
// --------------------------------------------------------------------
