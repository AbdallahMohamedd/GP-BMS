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
#include "main.h"
#include "Platform/pcconf.h"
#include "config.h"
#include "source/Platform/comm.h"
#include "source/Platform/commids.h"
// ----------------------------------------------------------------------------
#include "source/drivers/lld3377x.h"
// ----------------------------------------------------------------------------
#include "source/Platform/swident.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_device_registers.h"
#include "LCD.h"
//#include "LCD.h"
#define SW_IDENT  SWIDENT_EVALUATION_SW 										//!< unique ID for Demo SW <> Eval GUI	
#define SW_VER    4
#define SW_SUB    0
// ----------------------------------------------------------------------------
#define SW_ID	(u32)(SW_IDENT<<16)|(SW_VER<<8)|(SW_SUB)						//!< software identification (for GUI)
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
		1,						// no of clusters
		6,						// no of cells
		MODE_NORMAL_OPERATION,  // mode
		0,						// all CIDs measures the current
		0,						// measurement period 1 = 100ms
		0						// test features
};


// ----------------------------------------------------------------------------
// this is for SW debugging only!!!
//#define StopOnError()		while(1) DONOTHING();
#define StopOnError()		{}
// ----------------------------------------------------------------------------
int main(void)  {
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

	if(FALSE==PackCrtlConfigRead(&pcconf, defPCConfig)) {
		while(1)  DONOTHING();													// how to handle this error?
	}
	InitBoardClock();
	InitHW();
	InitBoardLED();
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	bms.Interface = pcconf.IntType;
	bms.EVB = pcconf.EvbType;
	bms.NoClusters = pcconf.NoCluster;
	bms.CIDcurrent = pcconf.CIDcurrent;
	bms.Status = BMS_Init;
	InitInterface(bms.Interface, bms.EVB);

	// PIT set lower prio by setting higher value
	NVICSetIrqPrio(SPI0_IRQ,  IP_PRIO_1);										
	NVICSetIrqPrio(SPI1_IRQ,  IP_PRIO_1);
	NVICSetIrqPrio(PIT_IRQ,   IP_PRIO_2);
	NVICSetIrqPrio(UART0_IRQ, IP_PRIO_4);

	// setup pit for 1ms timing
	PIT_LDVAL0 = BUSFREQ/1000;													// timer ch0 every 0.001 second
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK|PIT_TCTRL_TEN_MASK;						// enable IRQ and timer ch0
	NVICEnIrq(PIT_IRQ);

	LED_RED_On();

	DelayInit();	
	NVICEnIrq(UART0_IRQ);	
	Uart0Init();																// Uart 0 to communicate with PC GUI
	//	crc8_test();																// test patterns for CRC calculus
	I2C_init();
	//I2C0_Init();
	DataMonitor_lcd(50,50,2,25,1,0);

	lld3377xInitDriver(&(bms.Interface));					
	lld3377xInitCluster(&(cluster[0]));											// just tag id is needed for lld3377xReadRegisters()

	u4TagID = 1;	
	gu4CIDSelected = 1;
	cidRoundRobin = 1;

	for EVER {
		u4TagID++; 
		u4TagID %= 16;

		switch(bms.Status)  {

		case BMS_Idle:
			bms.Status = BMS_Init;
			break;

		case BMS_Init:
			LEDHandler(Off);
			if(bms.Interface==IntTPL)  {
				lld3377xTPLEnable();
				lld3377xWakeUp();
				lld3377xWriteGlobalRegister(SYS_CFG1, 0x9011);					// global reset				
			}else{
				lld3377xWakeUp();
				lld3377xWriteRegister(CIDassiged, SYS_CFG1, 0x9011, NULL);		// reset				
			}
			for(cid=1; cid<=bms.NoClusters; cid++)  {
				cluster[cid-1].Chip = Chip_Unknown;
				cluster[cid-1].Guid = 0L;
				cluster[cid-1].FRev = 0;
				cluster[cid-1].MRev = 0;
				cluster[cid-1].NoCTs = 0;
			}					

			if(BMSInit(bms.NoClusters))	{
				// autodetect cluster (chip version, etc.) 
				for(cid=1; cid<=bms.NoClusters; cid++)  {
					lld3377xClearError();
					MC3377xGetSiliconRevision(cid, &(cluster[cid-1]));			// the sequence / order is required				
					MC3377xGetSiliconType(cid, &(cluster[cid-1]));				// the sequence / order is required					
					MC3377xGetGUID(cid, &(cluster[cid-1]));						// the sequence / order is required			
				}
				bms.Status = BMS_Config;
			}
			break;

		case BMS_Config:
			lld3377xClearError();
			LEDHandler(Off);
			for(cid=1; cid<=bms.NoClusters; cid++)  {
				if(bms.Interface==IntSPI)  {
					if( (cluster[cid-1].Chip==Chip_MC33771A) || (cluster[cid-1].Chip==Chip_MC33771BM) ||(cluster[cid-1].Chip==Chip_MC33771B))  {
						MC3377xConfig(cid, CONF33771SPI);						
					}else if( (cluster[cid-1].Chip==Chip_MC33772A) || (cluster[cid-1].Chip==Chip_MC33772BM) || (cluster[cid-1].Chip==Chip_MC33772B))  {
						MC3377xConfig(cid, CONF33772SPI);						
					}	
				}else if(bms.Interface==IntTPL)  {
					if( (cluster[cid-1].Chip==Chip_MC33771A) || (cluster[cid-1].Chip==Chip_MC33771BM) ||(cluster[cid-1].Chip==Chip_MC33771B))  {
						MC3377xConfig(cid, CONF33771TPL);						
					}else if( (cluster[cid-1].Chip==Chip_MC33772A) || (cluster[cid-1].Chip==Chip_MC33772BM) || (cluster[cid-1].Chip==Chip_MC33772B))  {
						MC3377xConfig(cid, CONF33772TPL);						
					}	
				}
			}	
			lld3377xClearError();
			//				BMSEnableISense(bms.CIDcurrent);

			if(lld3377xGetError(NULL))  {
				bms.Status = BMS_Error;											// something failed
				u32msCntDown = 1000;
			}else{
				bms.Status = BMS_Running;
			}
			break;

		case BMS_Running:
			//! \todo how to start conversion? Global versus local? (global might overwrite ADC_CFG content)
			// chosen to start ADCs individual, to not change ADC_CFG register
			// in a normal system a global write to all MC3377x would be used
			timeStamp = msTick;
			SendIdxData32(0, ID_TIMESTAMP, timeStamp);

			lld3377xClearError();

			for(cid=1; cid<=bms.NoClusters; cid++)  {
				MC3377xADCStartConversion(cid, u4TagID);
			}
			Delay(DELAY_325us);
			while(MC3377xADCIsConverting(gu4CIDSelected))  DONOTHING();			// wait till ready

			// all cids are measured and status is requested
			for(cid=1; cid<=bms.NoClusters; cid++)  {
				if(!MC3377xGetRawMeasurements(cid, u4TagID, cluster[cid-1].NoCTs, &(rawResults[cid-1])))  {
					bms.Status = BMS_Error;										// error handling
					u32msCntDown = 1000;
					StopOnError();
				}
				if(!MC3377xGetStatus(cid, &(StatusBits[cid-1])))  {
					bms.Status = BMS_Error;										// error handling
					u32msCntDown = 1000;
					StopOnError();
				}
			}

			if(FaultPinStatus())	{											// check FAULT pin status (after measurement, before Diagnostics)  				 
				SendIdxData8(0, ID_BASE_FAULTPIN, 2);

				LEDHandler(Orange);
			}else{
				SendIdxData8(0, ID_BASE_FAULTPIN, 1);
				LEDHandler(Green);
			}

			// for debugging (output to GUI)
			// only for the selected ID Status, Config and Thresholds are handled
			if(!MC3377xGetStatus(gu4CIDSelected, &(StatusBits[gu4CIDSelected-1])))  {
				bms.Status = BMS_Error;											// error handling
				u32msCntDown = 1000;
				StopOnError();
			}

			if(!MC3377xGetThresholds(gu4CIDSelected, cluster[gu4CIDSelected-1].NoCTs, &(Thresholds[gu4CIDSelected-1])))  {
				bms.Status = BMS_Error;											// error handling
				u32msCntDown = 1000;
				StopOnError();
			}

			// for debugging (output to GUI)
			if(!MC3377xGetConfig(gu4CIDSelected, cluster[gu4CIDSelected-1].NoCTs, &(ConfigBits[gu4CIDSelected-1])))  {
				bms.Status = BMS_Error;											// error handling
				u32msCntDown = 1000;
				StopOnError();
			}
			break;

		case BMS_Sleeping:
			LEDHandler(Blue);
			if(MC3377xCheck4Wakeup(bms.Interface)) { 
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
			SendIdxData8(0, ID_BASE_BMSSTATUS, (u8)bms.Status);
			if(u32msCntDown==0)  {
				bms.Status = BMS_Init;											// error handling
			}
			break;
		}

		// -----  handle debug comms   -----
		SendIdxData32(0, ID_PACKCTRL_SW, SW_ID);
		SendIdxData8(0, ID_PACKCTRL_INTERFACE, bms.Interface);
		SendIdxData8(0, ID_PACKCTRL_EVB, bms.EVB);
		SendIdxData8(0, ID_PACKCTRL_NOCLUSTER, bms.NoClusters);
		SendIdxData8(0, ID_BASE_BMSSTATUS, bms.Status);

		// output cluster details
		cid = cidRoundRobin;
		SendIdxData16(cidRoundRobin, ID_CHIPREV, ( (((u8)cluster[cid-1].Chip)<<8) | ((((u8)cluster[cid-1].FRev)&0x0F)<<4) | (((u8)cluster[cid-1].MRev)&0x0F)   ));
		SendIdxData48(cidRoundRobin, ID_GUID, cluster[cid-1].Guid);

		cidRoundRobin++;
		if(cidRoundRobin > bms.NoClusters) 
			cidRoundRobin = 1;

		if(bms.Status==BMS_Running)  {
			// all cids are measured
			for(cid=1; cid<=bms.NoClusters; cid++)  {
				DebugPrintMeasurements(cid, cluster[cid-1].NoCTs, &(rawResults[cid-1]));						// output values to host PC

				// calculate average current based on coulombcounter
				//				ccCount[1] = rawResults[cid-1].u16CCSamples;
				//				ccValue[1] = rawResults[cid-1].s32CCCounter;
				SendIdxData16(cid, ID_CC_NB_SAMPLE   , rawResults[cid-1].u16CCSamples);
				SendIdxData32(cid, ID_COULOMB_CNT    , rawResults[cid-1].s32CCCounter);

				//				rawEval.s32AvCurrent = (ccValue[1]-ccValue[0])/(ccCount[1]-ccCount[0]);
				//				rawEval.s32AvCurrent = (ccValue[1]-ccValue[0]);
				//				rawEval.u16Samples = ccCount[1]-ccCount[0];
				//! \todo IDs must be moved!!!! 	
				//				SendIdxData32(cid, ID_BASE_EVALS, rawEvals->s32AvCurrent);
				//				SendIdxData16(cid, ID_BASE_EVALS+1, rawEvals->u16Samples);
				//				ccCount[0] = ccCount[1];
				//				ccValue[0] = ccValue[1];

				SendIdxData16(cid, ID_FAULT_STATUS1, StatusBits[cid-1].u16Fault1);	
				SendIdxData16(cid, ID_FAULT_STATUS2, StatusBits[cid-1].u16Fault2);	
				SendIdxData16(cid, ID_FAULT_STATUS3, StatusBits[cid-1].u16Fault3);	

				SendIdxData16(cid, ID_FAULT_MASK1, ConfigBits[cid-1].u16FaultMask1);	
				SendIdxData16(cid, ID_FAULT_MASK2, ConfigBits[cid-1].u16FaultMask2);	
				SendIdxData16(cid, ID_FAULT_MASK3, ConfigBits[cid-1].u16FaultMask3);	

				// for the selected CID perform evaluation of fault  
				bBCCFault = FALSE;
				u16EvalFault = StatusBits[cid-1].u16Fault1 & ~ConfigBits[cid-1].u16FaultMask1;
				SendIdxData16(cid, ID_BASE_EVALS+2, u16EvalFault);	
				if(u16EvalFault)
					bBCCFault = TRUE;

				u16EvalFault = StatusBits[cid-1].u16Fault2 & ~ConfigBits[cid-1].u16FaultMask2;
				SendIdxData16(cid, ID_BASE_EVALS+3, u16EvalFault);	
				if(u16EvalFault)
					bBCCFault = TRUE;

				u16EvalFault = StatusBits[cid-1].u16Fault3 & ~ConfigBits[cid-1].u16FaultMask3;
				SendIdxData16(cid, ID_BASE_EVALS+4, u16EvalFault);	
				if(u16EvalFault)
					bBCCFault = TRUE;

				// this message is used for dataloging!!!
				SendIdxData8(cid, ID_BCCSTATE, bBCCFault);							

			}

			// for debugging (output to GUI)
			// only for the selected ID Status, Config and Thresholds are handled
			SendIdxData8(0, ID_CID_SELECTED, gu4CIDSelected);	
			DebugPrintStatus(gu4CIDSelected, &(StatusBits[gu4CIDSelected-1]));
			DebugPrintThresholds(gu4CIDSelected, cluster[gu4CIDSelected-1].NoCTs, &(Thresholds[gu4CIDSelected-1]));							// output values to host PC 
			DebugPrintConfig(gu4CIDSelected, cluster[gu4CIDSelected-1].NoCTs, &(ConfigBits[gu4CIDSelected-1]));								// output values to host PC 
		}
		HandleGUICommands(&bms);
	}
	return 0;
}
// ----------------------------------------------------------------------------
void HandleGUIRefresh(TYPE_BMS *bms)  {


}
// ----------------------------------------------------------------------------
// declare outside to be static.... todo!!
u8 idx, u8Byte, u8Line[256], u8LineIdx=0;
/*! \brief Handles Commands from GUI (received on serial interface)


\b Protocol: 
- using a line based \b <\\n> protocol
- first char in line represents command


<b>Command list:</b> \n
| Command | Brief         | Parameter                      | Description                            |
|---------|---------------|--------------------------------|----------------------------------------|
| 'g'     | select CID    | cid                            | selects CID for detailed data exchange |
| 'r'     | reset         | none                           | Soft-resets the uC                     |
| 'S'     | sleep mode    | none                           | sets BMS into Sleep mode               |
| 'W'     | wakeup        | none                           | wakes up BMS                           |
| 'w'     | write         | cid, u8Add, u16Data            | writes to register                     |
| 'm'     | masked write  | cid, u8Add, u16Mask, u16Data   | masked write to register               |
| 'e'     | write exor'ed | cid, u8Add, u16Mask            | exor'ed write to register              |
| 'c'     | configuration | u4Idx, u16Data                 | change pack controller configuration   |

 */
void HandleGUICommands(TYPE_BMS *bms)  {

	u8 cid;
	u8 u8Add;
	u16 u16Data;
	u16 u16Mask;
	static u16 u16RdData[256];

	//LLD_RETURN_TYPE res[10];
	//u16 data[2];
	//u16 dummy;

	// -----  receive commands from GUI -----
	if(Uart0ReadByte(&u8Byte))  {												// bytes received?
		u8Line[u8LineIdx++] = u8Byte;
		while(u8Byte!='\n')  {
			if(Uart0ReadByte(&u8Byte))  {
				u8Line[u8LineIdx++] = u8Byte;
			}else{
				u8Byte = ' ';													// dummy not '\n'
				break ;
			}
		}
		// todo handle buffer overflow

		if(u8Byte == '\n')  {													// end of line detected
			// execute command
			switch(u8Line[0]) {

			case 'g':		                           							// change selected CID
				if(u8LineIdx==3)  {												// check length
					gu4CIDSelected   = Char2Number(u8Line[1]);
				}
				break;
			case 'r':		                           							// reset
				MCUReset();
				break;
			case 'S':		                           							// goto sleep (only BCC is put into sleep, uC is still running)
				// clear flags					
				MC3377xSleepMode(bms->Interface);
				//				if(bms->Interface==IntSPI)  {
				//					// SPI-Ard only
				//					SPICSB(0);
				//					SPIDisable();
				//				}
				//				if(bms->Interface==IntTPL)  {
				//					// SPI-Ard only
				//					SPICSB(0);
				//					SPITxDisable();
				//					SPIRxDisable();
				//				}
				bms->Status = BMS_Sleeping;	
				break;
			case 'W':															// wake up BCC (CSB wakeup)
				if(bms->Status==BMS_Sleeping)  {
					// SPI-Ard only
					SPIEnable();
					SPICSB(1);
				}
				MC3377xNormalMode(bms->Interface);
				bms->Status = BMS_Running;	
				break;
			case 'w':															// write command  format "wIaadddd" I=CID, aa=add, dddd= data
				if(u8LineIdx==9)  {												// check length
					cid   = Char2Number(u8Line[1]);
					u8Add   = Char2Number(u8Line[2])*0x10+Char2Number(u8Line[3]);
					u16Data = Char2Number(u8Line[4])*0x1000+Char2Number(u8Line[5])*0x100+Char2Number(u8Line[6])*0x10+Char2Number(u8Line[7]);
					lld3377xWriteRegister(cid, u8Add, u16Data, u16RdData);				  
				}
				break;
			case 'm':															// masked write command  format "mIaammmmdddd" I=CID, aa=add, mmmm=mask, dddd= data
				if(u8LineIdx==13)  {											// check length
					cid   = Char2Number(u8Line[1]);
					u8Add   = Char2Number(u8Line[2])*0x10+Char2Number(u8Line[3]);
					u16Mask = Char2Number(u8Line[4])*0x1000+Char2Number(u8Line[5])*0x100+Char2Number(u8Line[6])*0x10+Char2Number(u8Line[7]);
					u16Data = Char2Number(u8Line[8])*0x1000+Char2Number(u8Line[9])*0x100+Char2Number(u8Line[10])*0x10+Char2Number(u8Line[11]);
					lld3377xReadRegisters(cid,  u8Add, 1, u16RdData);
					u16Data = (u16RdData[0]&~u16Mask)|u16Data;					// read mask and or with new value
					lld3377xWriteRegister(cid, u8Add, u16Data, u16RdData);				  
				}
				break;
			case 'e':															// write command  format "wIaammmm" I=CID, aa=add, mmmmm=exor mask
				if(u8LineIdx==9)  {												// check length
					cid   = Char2Number(u8Line[1]);
					u8Add   = Char2Number(u8Line[2])*0x10+Char2Number(u8Line[3]);
					u16Mask = Char2Number(u8Line[4])*0x1000+Char2Number(u8Line[5])*0x100+Char2Number(u8Line[6])*0x10+Char2Number(u8Line[7]);
					lld3377xReadRegisters(cid, u8Add, 1, u16RdData);
					u16Data = u16RdData[0]^u16Mask;		// exor with mask
					lld3377xWriteRegister(cid, u8Add, u16Data, u16RdData);				  
				}
				break;
				// change pc (pack controller) configuration
			case 'c':															// config command  format "cidddd" i=indx, dddd= data
				if(u8LineIdx!=7)  												// check length
					break;									
				idx = Char2Number(u8Line[1]);
				u16Data = Char2Number(u8Line[2])*0x1000+Char2Number(u8Line[3])*0x100+Char2Number(u8Line[4])*0x10+Char2Number(u8Line[5]);
				PcconfUpdate(&pcconf, idx, u16Data);
				bms->EVB = pcconf.EvbType;
				bms->Interface = pcconf.IntType;
				bms->NoClusters = pcconf.NoCluster;
				DeInitInterface();
				InitInterface(bms->Interface, bms->EVB);
				//! \todo why a pointer?
				lld3377xInitDriver(&(bms->Interface));

				bms->Status = BMS_Init;
				break;
			}
			u8LineIdx = 0;
		}
	}
}


// ----------------------------------------------------------------------------
/*! \brief generates ms tick  
 */
void PIT_IRQHandler(void)  {

	if(PIT_TFLG0&PIT_TFLG_TIF_MASK)   {
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;						//clear flag
		msTick++;

		if(u32msCntDown>0) 
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
u8 Char2Number(u8 u8Char)  {

	if((u8Char>='0')&&(u8Char<='9'))  
		return (u8)(u8Char-0x30);

	if((u8Char>='A')&&(u8Char<='F'))
		return (u8)(u8Char-55);

	if((u8Char>='a')&&(u8Char<='f'))  
		return (u8)(u8Char-87);

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
void BMSEnableISense(u8 cidex)  {	
	u16 u16Data;
	u8 cid;

	for(cid=1; cid<=pcconf.NoCluster; cid++)  {	
		// enable current measurements	
		if(lld3377xReadRegisters(cid, SYS_CFG1, 1, &u16Data))  {	
			if(cidex==0)  {
				u16Data &= ~SYS_CFG1_I_MEAS_EN;									// disable all
			}else if(cidex>=15)  {
				u16Data |= SYS_CFG1_I_MEAS_EN; 
			}else{ 
				if(cidex==cid)  {
					u16Data |= SYS_CFG1_I_MEAS_EN; 								// disable for cid
				}else{
					u16Data &= ~SYS_CFG1_I_MEAS_EN;								// disable for all others
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
#define MAX_NO_OF_CTx   14   //CT1..14
#define NO_OF_ANx   7    //AN0..6
// ----------------------------------------------------------------------------
/*! \brief sends (Uart0) the measurement values to the GUI.
 * 
 * @param cid			cluster CID
 * @param NoCTs			Number of cell terminals
 * @param *rawResults   pointer to raw measurement result
 */
void DebugPrintMeasurements(u8 cid, u8 NoCTs, TYPE_MEAS_RESULTS_RAW *rawResults)  {
	u8 n;

	SendIdxData16(cid, ID_MEAS_VPWR, rawResults->u16StackVoltage);
	for(n=0; n<NoCTs; n++)  {
		SendIdxData16(cid, ID_MEAS_VCT1+n, rawResults->u16CellVoltage[n]);
	}
	for(n=0; n<NO_OF_ANx; n++)  {
		SendIdxData16(cid, ID_MEAS_VAN0+n, rawResults->u16ANVoltage[n]);
	}
	SendIdxData32(cid, ID_MEAS_VCUR      , rawResults->s32Current);
	SendIdxData16(cid, ID_MEAS_ICTEMP    , rawResults->u16ICTemp);
	SendIdxData16(cid, ID_MEAS_VBG_DIAG_ADC1A, rawResults->u16VbgADC1A);
	SendIdxData16(cid, ID_MEAS_VBG_DIAG_ADC1B, rawResults->u16VbgADC1B);
}
// ----------------------------------------------------------------------------
/*! \brief sends (Uart0) the threshold values to the GUI.
 * 
 * @param cid			cluster CID
 * @param NoCTs			Number of cell terminals
 * @param *Thresholds   pointer to Threshold data
 */
void DebugPrintThresholds(u8 cid, u8 NoCTs, TYPE_THRESHOLDS *Thresholds)  {
	u8 n;

	SendIdxData16(cid, ID_TH_ALL_CT_OV, Thresholds->u8ThAllOv);
	SendIdxData16(cid, ID_TH_ALL_CT_UV, Thresholds->u8ThAllUv);
	for(n=0;n< NoCTs; n++)  {
		SendIdxData16(cid, ID_TH_CT1_OV+2*n, Thresholds->u8ThCTxOv[n]);
		SendIdxData16(cid, ID_TH_CT1_UV+2*n, Thresholds->u8ThCTxUv[n]);
	}
	for(n=0; n<NO_OF_ANx; n++)  {
		SendIdxData16(cid, ID_TH_AN0_OT+n, Thresholds->u10ThANxOT[n]);
		SendIdxData16(cid, ID_TH_AN0_UT+n, Thresholds->u10ThANxUT[n]);
	}
	SendIdxData16(cid, ID_TH_ISENSE_OC, Thresholds->u12ThIsenseOC);
	SendIdxData32(cid, ID_TH_COULOMB_CNT, Thresholds->u32ThCoulombCnt);
}
// ----------------------------------------------------------------------------
/*! \brief sends (Uart0) the status registers/bits to the GUI.
 * 
 * @param cid			cluster CID
 * @param NoCTs			Number of cell terminals
 * @param *StatusBits   pointer to Status register bits
 */
void DebugPrintStatus(u8 cid, TYPE_STATUS *StatusBits)  {

	SendIdxData16(cid, ID_CELL_OV        , StatusBits->u16CellOV);	
	SendIdxData16(cid, ID_CELL_UV        , StatusBits->u16CellUV);	
	SendIdxData16(cid, ID_CB_OPEN_FAULT  , StatusBits->u16CBOpen);	
	SendIdxData16(cid, ID_CB_SHORT_FAULT , StatusBits->u16CBShort);	
	SendIdxData16(cid, ID_CB_DRV_STATUS  , StatusBits->u16CBStatus);	
	SendIdxData16(cid, ID_GPIO_STATUS    , StatusBits->u16GPIOStatus);
	SendIdxData16(cid, ID_AN_OT_UT       , StatusBits->u16ANOtUt);	
	SendIdxData16(cid, ID_GPIO_SHORT_OPEN, StatusBits->u16GPIOOpen);	
	SendIdxData16(cid, ID_I_STATUS       , StatusBits->u16IStatus);	
	SendIdxData16(cid, ID_COM_STATUS     , StatusBits->u16Comm);	 
	SendIdxData16(cid, ID_FAULT_STATUS1  , StatusBits->u16Fault1);	
	SendIdxData16(cid, ID_FAULT_STATUS2  , StatusBits->u16Fault2);	
	SendIdxData16(cid, ID_FAULT_STATUS3  , StatusBits->u16Fault3);	
	SendIdxData16(cid, ID_MEAS_ISENSE2   , StatusBits->u16MeasIsense2);
}
// ----------------------------------------------------------------------------
/*! \brief sends (Uart0) the configuration registers/bits to the GUI.
 * 
 * @param cid			cluster CID
 * @param NoCTs			Number of cell terminals
 * @param *ConfigBits	pointer to Configuration register bits
 * 
 */
void DebugPrintConfig(u8 cid, u8 u8NoOfCTs, TYPE_CONFIG *ConfigBits)  {
	u8 n;

	SendIdxData16(cid, ID_INIT             , ConfigBits->u16Init );	 
	SendIdxData16(cid, ID_SYS_CFG_GLOBAL   , ConfigBits->u16SysCfgGlobal);	
	SendIdxData16(cid, ID_SYS_CFG1         , ConfigBits->u16SysCfg1);	
	SendIdxData16(cid, ID_SYS_CFG2         , ConfigBits->u16SysCfg2);
	SendIdxData16(cid, ID_SYS_DIAG         , ConfigBits->u16SysDiag);
	SendIdxData16(cid, ID_ADC_CFG          , ConfigBits->u16AdcCfg);	

	SendIdxData16(cid, ID_OV_UV_EN         , ConfigBits->u16OvUvEn);	
	SendIdxData16(cid, ID_GPIO_CFG1        , ConfigBits->u16GPIOCfg1);
	SendIdxData16(cid, ID_GPIO_CFG2        , ConfigBits->u16GPIOCfg2);

	SendIdxData16(cid, ID_FAULT_MASK1      , ConfigBits->u16FaultMask1);	 
	SendIdxData16(cid, ID_FAULT_MASK2      , ConfigBits->u16FaultMask2);	 
	SendIdxData16(cid, ID_FAULT_MASK3      , ConfigBits->u16FaultMask3);	 
	SendIdxData16(cid, ID_WAKEUP_MASK1     , ConfigBits->u16WakeupMask1);
	SendIdxData16(cid, ID_WAKEUP_MASK2     , ConfigBits->u16WakeupMask2);
	SendIdxData16(cid, ID_WAKEUP_MASK3     , ConfigBits->u16WakeupMask3);

	SendIdxData16(cid, ID_ADC2_OFFSET_COMP  , ConfigBits->u16Adc2Comp);	 

	for(n=0; n<u8NoOfCTs; n++)  {
		SendIdxData16(cid, ID_CB_CFG_1+n, ConfigBits->u16CBCfg[n]);  
	}
}
