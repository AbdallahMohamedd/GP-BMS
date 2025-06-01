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

//! \brief MC3377x Demo Software for EvaluationGUI


//! \addtogroup main
// @{
// ----------------------------------------------------------------------------
#ifndef MAIN_H_
#define MAIN_H_
// ----------------------------------------------------------------------------
#include <COTs/KL25ZUtilize/Inc/KL25ZUtil.h>
#include <stdio.h>
#include <tpm1.h>			// delay
#include "source/drivers/lld3377x.h"           // low level access
#include "mc3377x.h"			// high level access BCC14	 
#include "Platform/frdmkl25z.h"
#include "config.h"
// ----------------------------------------------------------------------------
// prototypes
void App(void);
void InitHW(void);
u8 Char2Number(u8 u8Char);
void HandleGUICommands(TYPE_BMS *bms);
void BMSEnableISense(u8 cidex);
// ----------------------------------------------------------------------------
void DebugPrintMeasurements(u8 cid, u8 NoCTs, TYPE_MEAS_RESULTS_RAW *rawResults);
void DebugPrintThresholds(u8 cid, u8 NoCTs, TYPE_THRESHOLDS *Thresholds);
void DebugPrintConfig(u8 cid, u8 u8NoOfCTs, TYPE_CONFIG *ConfigBits);
void DebugPrintStatus(u8 u4CID, TYPE_STATUS *StatusBits);
// ----------------------------------------------------------------------------
#endif /* MAIN_H_ */
// ----------------------------------------------------------------------------
// @}
// @}
// ----------------------------------------------------------------------------
