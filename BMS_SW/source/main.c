// --- Needed library --- //
#include <COTs/DebugInfoManager/Inc/debugInfo.h>
#include <COTs/BatteryStatusMonitor/Inc/dataMonitor.h>
#include <COTs/FanControlManager/Inc/fanCtrl.h>
#include <COTs/SlaveControlIF/Inc/slaveIF.h>
#include <COTs/ FuSa/Inc/FuSa.h>
#include <source/COTs/CellBalancigManager/BalancingModel_24a_ert_rtw/BalancingModel_24a.h>
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_pit.h"
#include "fsl_device_registers.h"
// ----------------------------------------------------------------------------
#include "Platform/pcconf.h"
#include "source/COTs/BMSDataBase/Inc/database.h"
// ----------------------------------------------------------------------------
uint16_t falg_temp = 0;
uint16_t faultFlag;

#define SW_VER 4
#define SW_SUB 0
void BMSEnableISense(uint8_t cidex);
// ----------------------------------------------------------------------------
// globals vars
uint8_t gu4CIDSelected;
static TYPE_PC_CONFIG pcconf;
static uint32_t msTick;
static uint32_t prev_time;
static uint32_t u32msCntDown;
extern const SsysConf_t CONF33771TPL[];

//---------------------------------------------------------
TYPE_PC_CONFIG const defPCConfig = {
	PC_CONFIG_VALID,
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
#define StopOnError() \
	{                 \
	}
TYPE_INTERFACE _interface = IntTPL; //!< local copy of interface type
TYPE_EVB _evb = EVB_TypeArd;		//!< local copy of evb type
// ----------------------------------------------------------------------------

int main(void)
{
	static TYPE_BMS bms;
	static SclusterInfo_t cluster[MAX_CLUSTER];
	static TYPE_MEAS_RESULTS_RAW rawResults[MAX_CLUSTER];
	static TYPE_STATUS StatusBits[MAX_CLUSTER];
	static TYPE_THRESHOLDS Thresholds[MAX_CLUSTER];
	static TYPE_CONFIG ConfigBits[MAX_CLUSTER];
	uint8_t cid;
	uint8_t cidRoundRobin;
	uint8_t u4TagID;
	uint16_t u16EvalFault;
	bool bBCCFault;
	uint32_t timeStamp;

	if (FALSE == PackCrtlConfigRead(&pcconf, defPCConfig))
	{
		while (1)
			DONOTHING(); // how to handle this error?
	}
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole(); // Initialize debug console for PRINTF
	Board_InitHW();

	DelayInit();
	fanCtrl_fan1Init();
	fanCtrl_fan2Init();
	thermalManager_Init();
	ScreenIF_Init();
	DataMonitor_startUp();

	// PIT set lower prio by setting higher value
	NVICSetIrqPrio(SPI0_IRQ, IP_PRIO_1);
	NVICSetIrqPrio(SPI1_IRQ, IP_PRIO_1);
	NVICSetIrqPrio(PIT_IRQ, IP_PRIO_1);
	// setup pit for 1ms timing
	PIT_LDVAL0 = BUSFREQ / 1000;						   // timer ch0 every 0.001 second
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // enable IRQ and timer ch0
	NVICEnIrq(PIT_IRQ);

	GPIO_PinInit(GPIOE, 20U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0}); // Blue LED
	GPIO_PinInit(GPIOE, 21U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0}); // Blue LED
	GPIO_PinInit(GPIOE, 22U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0}); // Blue LED
	GPIO_PinInit(GPIOE, 23U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0}); // Blue LED
	GPIO_PinInit(GPIOE, 29U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0}); // Blue LED
	GPIO_PinInit(GPIOE, 30U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0}); // Blue LED

	PRINTF("Board Initialized.\n\r\r");
	bms.Interface = pcconf.IntType;
	bms.EVB = pcconf.EvbType;
	bms.NoClusters = pcconf.NoCluster;
	bms.CIDcurrent = pcconf.CIDcurrent;
	bms.Status = BMS_Init;

	LED_RED_On();

	slaveIF_initDriver(&(bms.Interface));
	slaveIF_initCluster(&(cluster[0])); // just tag id is needed for slaveIF_readReg()

	u4TagID = 1;
	gu4CIDSelected = 1;
	cidRoundRobin = 1;
	float test_cell_volrage[14];

	while (1)
	{
		dataBase_getTempRawData((TYPE_MEAS_RESULTS_RAW *)&rawResults);
		if ((bms.Status == BMS_Error) && (falg_temp))
		{
			if (thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[1]) < 40)
			{
				falg_temp = 0;
				bms.Status = BMS_Init;
				fanCtrl_fan1SetDuty(0);
				fanCtrl_fan2SetDuty(0);
			}
		}
		if (thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[1]) >= 40)
		{
			falg_temp = 1;
			fanCtrl_fan1SetDuty(0);
			fanCtrl_fan2SetDuty(51);
			bms.Status = BMS_Error; // something failed
			u32msCntDown = 1000;
		}

		//		DebugInfo_PrintFaultReason(&StatusBits[0]);
		//		//		prev_ccounter = rawResults.s32CCCounter;
		//		prev_samples = rawResults.u16CCSamples;
		//		PRINTF("Prev sample = %d\r\r\n", prev_samples);
		//		PRINTF("prev_ccounter = %d\r\r\n", prev_ccounter);

		u4TagID++;
		u4TagID %= 16;

		switch (bms.Status)
		{

		case BMS_Idle:
			bms.Status = BMS_Init;
			break;

		case BMS_Init:
			LEDHandler(Off);
			slaveIF_transceiverEnable();
			slaveIF_wakeUp();
			slaveIF_writeGlobalReg(SYS_CFG1, 0x9011); // global reset

			for (cid = 1; cid <= bms.NoClusters; cid++)
			{
				cluster[cid - 1].Chip = Chip_Unknown;
				cluster[cid - 1].Guid = 0L;
				cluster[cid - 1].FRev = 0;
				cluster[cid - 1].MRev = 0;
				cluster[cid - 1].NoCTs = 0;
			}

			if (dataBase_BMSInit(bms.NoClusters))
			{
				for (cid = 1; cid <= bms.NoClusters; cid++)
				{
					slaveIF_clearError();
					dataBase_getSiliconRevision(cid, &(cluster[cid - 1])); // the sequence / order is required
					dataBase_getSiliconType(cid, &(cluster[cid - 1]));	   // the sequence / order is required
				}
				bms.Status = BMS_Config;
			}
			break;

		case BMS_Config:
			slaveIF_clearError();
			LEDHandler(Off);
			for (cid = 1; cid <= bms.NoClusters; cid++)
			{
				dataBase_bmsConfig(cid, CONF33771TPL);
			}

			slaveIF_clearError();
			//				BMSEnableISense(bms.CIDcurrent);

			if (slaveIF_getError(NULL))
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
			timeStamp = msTick;

			slaveIF_clearError();

			for (cid = 1; cid <= bms.NoClusters; cid++)
			{
				dataBase_startConvADC(cid, u4TagID);
			}
			Delay(DELAY_325us);
			while (dataBase_ADCIsConverting(gu4CIDSelected))
				DONOTHING(); // wait till ready

			// all cids are measured and status is requested
			for (cid = 1; cid <= bms.NoClusters; cid++)
			{
				if (!dataBase_getRawData(cid, u4TagID, cluster[cid - 1].NoCTs, &(rawResults[cid - 1])))
				{
					// PRINTF("dataBase_getRawData failed for CID %d\n", cid);
					bms.Status = BMS_Error; // error handling
					u32msCntDown = 1000;
					StopOnError();
				}
				if (!dataBase_getStatus(cid, &(StatusBits[cid - 1])))
				{
					bms.Status = BMS_Error; // error handling
					u32msCntDown = 1000;
					StopOnError();
				}
			}

			if (slaveIF_faultPinStatus())
			{ // check FAULT pin status (after measurement, before Diagnostics)
				DebugInfo_PrintFaultReason(&StatusBits[0]);
				LEDHandler(Orange);
			}
			else
			{
				LEDHandler(Green);
			}
			break;

		case BMS_Sleeping:
			LEDHandler(Blue);
			if (dataBase_check4Wakeup(bms.Interface))
			{
				bms.Status = BMS_Running;
			}
			break;

		case BMS_Error:
			DebugInfo_PrintFaultReason(&StatusBits[0]);
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
				uint16_t TeamStackVoltage = (rawResults[0].u16StackVoltage) * 2.44141 * 0.001;

				if (GPIO_ReadPinInput(GPIOE, 30U) == 0)
				{
					dataMonitor_clearScreen();
					ScreenIF_SetCursor(0, 0);
					dataMonitor_packvoltage((float)TeamStackVoltage);
					PRINTF("11\r\r\n");
				}
				if (GPIO_ReadPinInput(GPIOE, 29U) == 0)
				{
					dataMonitor_balancingStatus(StatusBits->u16CBStatus);
					PRINTF("22\r\r\n");
				}
				if (GPIO_ReadPinInput(GPIOE, 23U) == 0)
				{
					for (int i = 0; i < 14; i++)
					{
						test_cell_volrage[i] = rawResults->u16CellVoltage[i];
					}

					dataMonitor_packInfo(dataBase_initialSOC_Pack(test_cell_volrage), 00, 0, 24, NormalMode, FaultStatusNone);
					PRINTF("33\r\r\n");
				}
				if (GPIO_ReadPinInput(GPIOE, 22) == 0)
				{
					PRINTF("44\r\r\n");
					dataMonitor_clearScreen();
					ScreenIF_SetCursor(0, 0);
					// dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[0]));
					// ScreenIF_SetCursor(0, 1);
					dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[1]));
					// ScreenIF_SetCursor(0, 2);
					// dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[2]));
					// ScreenIF_SetCursor(10, 3);
					// dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[3]));
					// ScreenIF_SetCursor(10, 0);
					// dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[4]));
					// ScreenIF_SetCursor(10, 1);
					// dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[5]));
					// ScreenIF_SetCursor(10, 2);
					// dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[6]));
				}
				if (GPIO_ReadPinInput(GPIOE, 21) == 0)
				{
					dataMonitor_fanInfo(0, 0, ON, ON);
					PRINTF("55\r\r\n");
				}
				// PRINTF("ADC Value1: %5d  →  Temperature1: %.2f °C\r\n", rawResults[0].u16ANVoltage[0], thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[0]));
				// PRINTF("ADC Value1: %5d  →  Temperature1: %.2f °C\r\n", rawResults[0].u16ANVoltage[1], thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[1]));

				// SlaveIF_enableCellBalancing(14, false, 0.5, cid);
				SlaveIF_enableCellBalancing(8, false, 0.5, cid);
				BalancingModel_24a_step(&rawResults[0]);
				PRINTF("cell 1 = %f \r\r\n", (float)((rawResults->u16CellVoltage[0])*CT_Resolution));
				PRINTF("cell 2 = %f \r\r\n", (float)((rawResults->u16CellVoltage[1])*CT_Resolution));
				PRINTF("cell 3 = %f \r\r\n", (float)((rawResults->u16CellVoltage[2])*CT_Resolution));
				PRINTF("cell 4 = %f \r\r\n", (float)((rawResults->u16CellVoltage[3])*CT_Resolution));
				PRINTF("cell 5 = %f \r\r\n", (float)((rawResults->u16CellVoltage[4])*CT_Resolution));
				PRINTF("cell 6 = %f \r\r\n", (float)((rawResults->u16CellVoltage[5])*CT_Resolution));
				PRINTF("cell 7 = %f \r\r\n", (float)((rawResults->u16CellVoltage[6])*CT_Resolution));
				PRINTF("cell 8 = %f \r\r\n", (float)((rawResults->u16CellVoltage[7])*CT_Resolution));
				//				dataMonitor_clearScreen();
				//				ScreenIF_SetCursor(0, 0);
				//				dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[0]));
				//				ScreenIF_SetCursor(0, 1);
				//				dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[1]));
				//				ScreenIF_SetCursor(0, 2);
				//				dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[2]));
				//				ScreenIF_SetCursor(10, 3);
				//				dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[3]));
				//				ScreenIF_SetCursor(10, 0);
				//				dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[4]));
				//				ScreenIF_SetCursor(10, 1);
				//				dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[5]));
				//				ScreenIF_SetCursor(10, 2);
				//				dataMonitor_tempDisp(thermalManager_Raw2Celsius(rawResults[0].u16ANVoltage[6]));
				//  dataMonitor_modeDisp(SleepMode);
				//					if (1)
				//					{
				//						//DataMonitor_lcd(99, 90, 55.5, 52, NormalMode, FaultStatusNone);
				//						yarab = 0;
				//					}
				//					yarab++;
				//					// uint16_t temp1 = rawResults[0].u16ANVoltage[1];
				//  dataMonitor_clearScreen();
				//  ScreenIF_SetCursor(0, 0);
				//  DataMonitor_lcd((thermalManager_Raw2Celsius(temp1)), 90, 55, 52, NormalMode, FaultStatusNone);

				// dataMonitor_socDisp(22);
				//  calculate average current based on coulombcounter
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
		}
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
/*! \brief
 *
 * cidex:   0 -> none
 *          1..14 -> enable for CID 1...14
 *          15 -> enable for all CIDs
 *
 */
void BMSEnableISense(uint8_t cidex)
{
	uint16_t u16Data;
	uint8_t cid;

	for (cid = 1; cid <= pcconf.NoCluster; cid++)
	{
		// enable current measurements
		if (slaveIF_readReg(cid, SYS_CFG1, 1, &u16Data))
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
			slaveIF_writeReg(cid, SYS_CFG1, u16Data, NULL);
		}
	}
}
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// globals vars
uint16_t u16Data;
uint16_t u16Value;
uint16_t gu16Len;
// ----------------------------------------------------------------------------
#define MAX_NO_OF_CTx 14 // CT1..14
#define NO_OF_ANx 7		 // AN0..6
