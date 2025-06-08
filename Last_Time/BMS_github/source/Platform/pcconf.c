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

/*
 * Those defines are required for the "NVM Driver"

CPU_MKL25Z128VLK4
 *
 *
 */

// ----------------------------------------------------------------------------
#include "pcconf.h"
#include <stdbool.h>
// ----------------------------------------------------------------------------
#define CacheDisable() MCM_BWR_PLACR_DFCS(MCM_BASE_PTR, 1)
#define CacheEnable() MCM_BWR_PLACR_DFCS(MCM_BASE_PTR, 0)
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// Flash
uint8_t buffer[BUFFER_SIZE_BYTE]; /*! Buffer for program */
pFLASHCOMMANDSEQUENCE g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)0xFFFFFFFF;

// Array to copy __Launch_Command func to RAM.
uint16_t ramFunc[LAUNCH_CMD_SIZE / 2];

// Flash Standard Software Driver Structure.
FLASH_SSD_CONFIG flashSSDConfig =
	{
		FTFx_REG_BASE, /*! FTFx control register base */
		P_FLASH_BASE,  /*! Base address of PFlash block */
		P_FLASH_SIZE,  /*! Size of PFlash block */
		FLEXNVM_BASE,  /*! Base address of DFlash block */
		0,			   /*! Size of DFlash block */
		EERAM_BASE,	   /*! Base address of EERAM block */
		0,			   /*! Size of EEE block */
		DEBUGENABLE,   /*! Background debug mode enable bit */
		NULL_CALLBACK  /*! Pointer to callback function */
};
// ----------------------------------------------------------------------------
////! \brief add a error enum
// typedef enum {
//	pass = 0,
//	err_init = 1,
//	err_protect = 2,
//	err_erase = 3
//
//
// }TYPE_PCCONF_ERROR;

// ----------------------------------------------------------------------------
/*! \brief Read the pack controller configuration from FLASH memory.

Reads the pack controller configuration from FLASH memory if valid. Otherwise the
default configuration will be stored to Flash and used.

@param conf
@param defConfig
@return FALSE if Flash writing fails, TRUE otherwise
 */
bool PackCrtlConfigRead(TYPE_PC_CONFIG *conf, TYPE_PC_CONFIG defConfig)
{

	FlashConfigurationRead(conf, PC_CONFIG_SIZE);
	if (conf->Initialized == PC_CONFIG_VALID)
	{
	}
	else
	{
		conf->Initialized = defConfig.Initialized;
		conf->IntType = defConfig.IntType;
		conf->EvbType = defConfig.EvbType;
		conf->NoCluster = defConfig.NoCluster;
		conf->NoCells = defConfig.NoCells;
		conf->OpMode = defConfig.OpMode;
		conf->MeasPeriod = defConfig.MeasPeriod;
		conf->Test = defConfig.Test;
		conf->CIDcurrent = defConfig.CIDcurrent;

		DisableInterrupts();
		if (TRUE != FlashConfigurationWrite(conf, PC_CONFIG_SIZE))
			return FALSE;
		EnableInterrupts();
	}
	return TRUE;
}

// ----------------------------------------------------------------------------
/*! \brief Changes the pack controller configuration.
 *
| Idx | Description           | Comment                                 |
|-----|-----------------------|-----------------------------------------|
| 0   | Save PC config        | data must be 0x5AA5                     |
| 1   | change Int Type       |                                         |
| 2   | change EVB Type       |                                         |
| 3   | change No of Clusters |                                         |
| 4   | change No of CTs      |                                         |
| 5   | operating mode        | e.g. Verbose for FCC testing            |
| 6   | CID w. current meas.  | 0 -> no current meas.                   |
| 7   | Meas. period          | n*100ms                                 |
| 8   | test                  | for test purposes, e.g. error injection |


*/
void PcconfUpdate(TYPE_PC_CONFIG *conf, uint8_t idx, uint16_t data)
{

	switch (idx)
	{
	case 0: // save and reset
		if (data == 0x5AA5)
		{ // EVB type changed
			DisableInterrupts();
			FlashConfigurationWrite(conf, PC_CONFIG_SIZE);
			EnableInterrupts();
			SCB_AIRCR = 0x05FA0000 | SCB_AIRCR_SYSRESETREQ_MASK; // VectKey = 0x05FA
		}
		break;

	case 1:
		if (data == 1)
		{ // EVB type changed
			conf->IntType = 1;
		}
		if (data == 2)
		{ // EVB type changed
			conf->IntType = 2;
		}
		break;

	case 2: // change, save and reset
		if (data == 1)
		{ // EVB type changed
			conf->EvbType = 1;
		}
		if (data == 2)
		{ // EVB type changed
			conf->EvbType = 2;
		}
		break;

	case 3: // No of clusters
		conf->NoCluster = data;
		break;

	case 4: // No of CT
		conf->NoCells = data;
		break;

	case 5: // No of CT
		conf->OpMode = data;
		break;

	case 6:
		conf->CIDcurrent = data;
		break;

	case 7:
		conf->MeasPeriod = data;
		break;

	case 8:
		conf->Test = data;
		break;
	}
}

// ----------------------------------------------------------------------------
/*! \brief
 *
 */
bool FlashConfigurationRead(TYPE_PC_CONFIG *packConf, uint8_t packConfSize)
{
	uint8_t i;
	uint8_t *src, *dst;

	src = (uint8_t *)(FLASH_DATA_ADDR);
	dst = (uint8_t *)packConf;

	for (i = 0; i < packConfSize; i++)
	{
		*dst++ = *src++;
	}

	return TRUE;
}
// ----------------------------------------------------------------------------
/*! \brief
 *
 */
bool FlashConfigurationWrite(TYPE_PC_CONFIG *packConf, uint8_t packConfSize)
{

	uint8_t size;

	size = packConfSize / 4 * 4; // 4 byte alignment
	CacheDisable();
	if (FTFx_OK != FlashInit(&flashSSDConfig))
		return FALSE;

	// copy command handler to RAM (can't execute from same FLASH block, than erased/programmed)
	g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)RelocateFunction((uint32_t)ramFunc, LAUNCH_CMD_SIZE, (uint32_t)FlashCommandSequence);

	// attention no interrupts should happen....

	if (FTFx_OK != FlashEraseSector(&flashSSDConfig, FLASH_DATA_ADDR, FTFx_PSECTOR_SIZE, g_FlashLaunchCommand))
		return FALSE;

	if (FTFx_OK != FlashProgram(&flashSSDConfig, FLASH_DATA_ADDR, size, (uint8_t *)(packConf), g_FlashLaunchCommand))
		return FALSE;

	CacheEnable();
	return TRUE;
}
