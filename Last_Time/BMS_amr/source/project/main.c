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
// ----------------------------------------------------------------------------
// for documentation and change log see \docu\documentation.html
// ----------------------------------------------------------------------------
// #include "main.h"
#include "source/COTs/SlaveControlIF/Inc/SlaveIF.h"
#include "Platform/pcconf.h"
#include "source/Platform/commids.h"
#include "source/Platform/swident.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include <COTs/BatteryStatusMonitor/Inc/DataMonitor.h>
#define SW_IDENT SWIDENT_EVALUATION_SW //!< unique ID for Demo SW <> Eval GUI
#define SW_VER 4
#define SW_SUB 0
// ----------------------------------------------------------------------------
#define SW_ID (u32)(SW_IDENT << 16) | (SW_VER << 8) | (SW_SUB) //!< software identification (for GUI)
// ----------------------------------------------------------------------------
// globals vars
u8 gu4CIDSelected;
static TYPE_PC_CONFIG pcconf;
static u32 msTick;
static u32 u32msCntDown;
// ----------------------------------------------------------------------------
/*! \brief Default Pack Controller Configuration
 *
 * will be used only once after first start. Then the configuration will be stored
 * in the Flash and loaded from Flash afterwards.
 */
TYPE_PC_CONFIG const defPCConfig = {
	PC_CONFIG_VALID,
	//! \todo its not allowed to have an unknown EVB or unknown INT selected, otherwise SPI accesses will crash!!!.
	IntTPL,
	EVB_TypeArd,
	1,					   // no of clusters
	6,					   // no of cells
	MODE_NORMAL_OPERATION, // mode
	0,					   // all CIDs measures the current
	0,					   // measurement period 1 = 100ms
	0					   // test features
};
// ----------------------------------------------------------------------------
// this is for SW debugging only!!!
// #define StopOnError()		while(1) DONOTHING();
#define StopOnError() \
	{                 \
	}
// ----------------------------------------------------------------------------
int main(void)
{
	static TYPE_BMS bms;
	static LLD_TYPE_CLUSTER cluster[MAX_CLUSTER];
	static TYPE_MEAS_RESULTS_RAW rawResults[MAX_CLUSTER];
	static TYPE_STATUS StatusBits[MAX_CLUSTER];
	static TYPE_THRESHOLDS Thresholds[MAX_CLUSTER];
	static TYPE_CONFIG ConfigBits[MAX_CLUSTER];
	u8 cid;
	u8 cidRoundRobin;
	u8 u4TagID;
	u16 u16EvalFault;
	bool bBCCFault;
	u32 timeStamp;

	if (FALSE == PackCrtlConfigRead(&pcconf, defPCConfig))
	{
		while (1)
			DONOTHING(); // how to handle this error?
	}
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();

	InitBoardClock();
	InitHW();
	InitBoardLED();
	PRINTF("Board Initialized.\n\r\r");
	bms.Interface = pcconf.IntType;
	bms.EVB = pcconf.EvbType;
	bms.NoClusters = pcconf.NoCluster;
	bms.CIDcurrent = pcconf.CIDcurrent;
	bms.Status = BMS_Init;
	InitInterface(bms.Interface, bms.EVB);

	// PIT set lower prio by setting higher value
	NVICSetIrqPrio(SPI0_IRQ, IP_PRIO_1);
	NVICSetIrqPrio(SPI1_IRQ, IP_PRIO_1);
	NVICSetIrqPrio(PIT_IRQ, IP_PRIO_1);
	//NVICSetIrqPrio(UART0_IRQ, IP_PRIO_4);

	// setup pit for 1ms timing
	PIT_LDVAL0 = BUSFREQ / 1000;						   // timer ch0 every 0.001 second
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // enable IRQ and timer ch0
	NVICEnIrq(PIT_IRQ);

	LED_RED_On();

	DelayInit();
	//NVICEnIrq(UART0_IRQ);
	//Uart0Init();
	//	crc8_test();																// test patterns for CRC calculus
	I2C_init();
	ScreenIF_Init(4);
	// I2C0_Init();
	  ScreenIF_Clear();

DataMonitor_lcd(50, 50, 2, 25, 1, 0); // Abdullah

	lld3377xInitDriver(&(bms.Interface));
	lld3377xInitCluster(&(cluster[0])); // just tag id is needed for lld3377xReadRegisters()

	u4TagID = 1;
	gu4CIDSelected = 1;
	cidRoundRobin = 1;

	for
		EVER
		{
			u4TagID++;
			u4TagID %= 16;

			switch (bms.Status)
			{

			case BMS_Idle:
				bms.Status = BMS_Init;
				break;

			case BMS_Init:
				LEDHandler(Off);
				lld3377xTPLEnable();
				lld3377xWakeUp();
				lld3377xWriteGlobalRegister(SYS_CFG1, 0x9011); // global reset

				for (cid = 1; cid <= bms.NoClusters; cid++)
				{
					cluster[cid - 1].Chip = Chip_Unknown;
					cluster[cid - 1].Guid = 0L;
					cluster[cid - 1].FRev = 0;
					cluster[cid - 1].MRev = 0;
					cluster[cid - 1].NoCTs = 0;
				}

				if (BMSInit(bms.NoClusters))
				{
					for (cid = 1; cid <= bms.NoClusters; cid++)
					{
						lld3377xClearError();
						MC3377xGetSiliconRevision(cid, &(cluster[cid - 1])); // the sequence / order is required
						MC3377xGetSiliconType(cid, &(cluster[cid - 1]));	 // the sequence / order is required
						//MC3377xGetGUID(cid, &(cluster[cid - 1]));			 // the sequence / order is required
					}
					bms.Status = BMS_Config;
				}
				break;

			case BMS_Config:
				lld3377xClearError();
				LEDHandler(Off);
				for (cid = 1; cid <= bms.NoClusters; cid++)
				{
					MC3377xConfig(cid, CONF33771TPL);
				}

				lld3377xClearError();
				//				BMSEnableISense(bms.CIDcurrent);

				if (lld3377xGetError(NULL))
				{
					bms.Status = BMS_Error; // something failed
					u32msCntDown = 1000;
				}
				else
				{
					bms.Status = BMS_Running;
				}
				break;

			case BMS_Running:
				//! \todo how to start conversion? Global versus local? (global might overwrite ADC_CFG content)
				// chosen to start ADCs individual, to not change ADC_CFG register
				// in a normal system a global write to all MC3377x would be used
				timeStamp = msTick;

				lld3377xClearError();

				for (cid = 1; cid <= bms.NoClusters; cid++)
				{
					MC3377xADCStartConversion(cid, u4TagID);
				}
				Delay(DELAY_325us);
				while (MC3377xADCIsConverting(gu4CIDSelected))
					DONOTHING(); // wait till ready

				// all cids are measured and status is requested
				for (cid = 1; cid <= bms.NoClusters; cid++)
				{
					if (!MC3377xGetRawMeasurements(cid, u4TagID, cluster[cid - 1].NoCTs, &(rawResults[cid - 1])))
					{
						PRINTF("MC3377xGetRawMeasurements failed for CID %d\n", cid);
						bms.Status = BMS_Error; // error handling
						u32msCntDown = 1000;
						StopOnError();
					}
					if (!MC3377xGetStatus(cid, &(StatusBits[cid - 1])))
					{
						bms.Status = BMS_Error; // error handling
						u32msCntDown = 1000;
						StopOnError();
					}
				}

				if (FaultPinStatus())
				{ // check FAULT pin status (after measurement, before Diagnostics)

					LEDHandler(Orange);
				}
				else
				{
					LEDHandler(Green);
				}

				// for debugging (output to GUI)
				// only for the selected ID Status, Config and Thresholds are handled
				if (!MC3377xGetStatus(gu4CIDSelected, &(StatusBits[gu4CIDSelected - 1])))
				{
					bms.Status = BMS_Error; // error handling
					u32msCntDown = 1000;
					StopOnError();
				}

				if (!MC3377xGetThresholds(gu4CIDSelected, cluster[gu4CIDSelected - 1].NoCTs, &(Thresholds[gu4CIDSelected - 1])))
				{
					bms.Status = BMS_Error; // error handling
					u32msCntDown = 1000;
					StopOnError();
				}

				// for debugging (output to GUI)
				if (!MC3377xGetConfig(gu4CIDSelected, cluster[gu4CIDSelected - 1].NoCTs, &(ConfigBits[gu4CIDSelected - 1])))
				{
					bms.Status = BMS_Error; // error handling
					u32msCntDown = 1000;
					StopOnError();
				}
				break;

			case BMS_Sleeping:
				LEDHandler(Blue);
				if (MC3377xCheck4Wakeup(bms.Interface))
				{
					bms.Status = BMS_Running;
					//					if(bms->Interface==IntSPI)  {
					//						// SPI-Ard only
					//						SPIEnable();
					//						SPICSB(1);
					//					}
					//					if(bms->Interface==IntTPL)  {
					//						// SPI-Ard only
					//						SPICSB(0);
					//						SPITxEnable();
					//						SPIRxEnable();
					//					}
				}
				break;

			case BMS_Error:
				LEDHandler(Red);
				if (u32msCntDown == 0)
				{
					bms.Status = BMS_Init; // error handling
				}
				break;
			}

			// output cluster details
			cid = cidRoundRobin;

			cidRoundRobin++;
			if (cidRoundRobin > bms.NoClusters)
				cidRoundRobin = 1;

			if (bms.Status == BMS_Running)
			{
				// all cids are measured
				for (cid = 1; cid <= bms.NoClusters; cid++)
				{
					 //DebugPrintMeasurements(cid, cluster[cid-1].NoCTs, &(rawResults[cid-1]));						// output values to host PC
					// DebugPrintMeasurements(1, cluster[0].NoCTs, &(rawResults[0])); // output values to host PC
					u16 TeamStackVoltage = rawResults[0].u16StackVoltage;
					DataMonitor_lcd((TeamStackVoltage * 2.44141 * 0.001), 90, 55, 52, 1, 0);
					//DataMonitor_soc_disp(22);
					// calculate average current based on coulombcounter
					//				ccCount[1] = rawResults[cid-1].u16CCSamples;
					//				ccValue[1] = rawResults[cid-1].s32CCCounter;
					//				rawEval.s32AvCurrent = (ccValue[1]-ccValue[0])/(ccCount[1]-ccCount[0]);
					//				rawEval.s32AvCurrent = (ccValue[1]-ccValue[0]);
					//				rawEval.u16Samples = ccCount[1]-ccCount[0];
					//! \todo IDs must be moved!!!!
					//				SendIdxData32(cid, ID_BASE_EVALS, rawEvals->s32AvCurrent);
					//				SendIdxData16(cid, ID_BASE_EVALS+1, rawEvals->u16Samples);
					//				ccCount[0] = ccCount[1];
					//				ccValue[0] = ccValue[1];

					// for the selected CID perform evaluation of fault
					bBCCFault = FALSE;
					u16EvalFault = StatusBits[cid - 1].u16Fault1 & ~ConfigBits[cid - 1].u16FaultMask1;
					if (u16EvalFault)
						bBCCFault = TRUE;

					u16EvalFault = StatusBits[cid - 1].u16Fault2 & ~ConfigBits[cid - 1].u16FaultMask2;
					if (u16EvalFault)
						bBCCFault = TRUE;

					u16EvalFault = StatusBits[cid - 1].u16Fault3 & ~ConfigBits[cid - 1].u16FaultMask3;
					if (u16EvalFault)
						bBCCFault = TRUE;

					// this message is used for dataloging!!!
				}

				// for debugging (output to GUI)
				// only for the selected ID Status, Config and Thresholds are handled
				// DebugPrintStatus(gu4CIDSelected, &(StatusBits[gu4CIDSelected-1]));
				// DebugPrintThresholds(gu4CIDSelected, cluster[gu4CIDSelected-1].NoCTs, &(Thresholds[gu4CIDSelected-1]));							// output values to host PC
				// DebugPrintConfig(gu4CIDSelected, cluster[gu4CIDSelected-1].NoCTs, &(ConfigBits[gu4CIDSelected-1]));								// output values to host PC
			}
			// HandleGUICommands(&bms);
		}
	return 0;
}

// ----------------------------------------------------------------------------
/*! \brief generates ms tick
 */
void PIT_IRQHandler(void)
{

	if (PIT_TFLG0 & PIT_TFLG_TIF_MASK)
	{
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; // clear flag
		msTick++;

		if (u32msCntDown > 0)
			u32msCntDown--;
	}
}

// ----------------------------------------------------------------------------
/*! \brief used to decode hex coded character based communication
 *
 * @param u8Char	character to decode ('0'..'9', 'a'..'f', 'A'..'F')
 * @return 			value of the decoded character, e.g. "a" => 10
 *
 * \remarks
 * returns 0 in case of unknown characters.
 */
u8 Char2Number(u8 u8Char)
{

	if ((u8Char >= '0') && (u8Char <= '9'))
		return (u8)(u8Char - 0x30);

	if ((u8Char >= 'A') && (u8Char <= 'F'))
		return (u8)(u8Char - 55);

	if ((u8Char >= 'a') && (u8Char <= 'f'))
		return (u8)(u8Char - 87);

	return 0;
}
// ----------------------------------------------------------------------------
/*! \brief
 *
 * cidex:   0 -> none
 *          1..14 -> enable for CID 1...14
 *          15 -> enable for all CIDs
 *
 */
void BMSEnableISense(u8 cidex)
{
	u16 u16Data;
	u8 cid;

	for (cid = 1; cid <= pcconf.NoCluster; cid++)
	{
		// enable current measurements
		if (lld3377xReadRegisters(cid, SYS_CFG1, 1, &u16Data))
		{
			if (cidex == 0)
			{
				u16Data &= ~SYS_CFG1_I_MEAS_EN; // disable all
			}
			else if (cidex >= 15)
			{
				u16Data |= SYS_CFG1_I_MEAS_EN;
			}
			else
			{
				if (cidex == cid)
				{
					u16Data |= SYS_CFG1_I_MEAS_EN; // disable for cid
				}
				else
				{
					u16Data &= ~SYS_CFG1_I_MEAS_EN; // disable for all others
				}
			}
			lld3377xWriteRegister(cid, SYS_CFG1, u16Data, NULL);
		}
	}
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// globals vars
u16 u16Data;
u16 u16Value;
u16 gu16Len;
// ----------------------------------------------------------------------------
#define MAX_NO_OF_CTx 14 // CT1..14
#define NO_OF_ANx 7		 // AN0..6


