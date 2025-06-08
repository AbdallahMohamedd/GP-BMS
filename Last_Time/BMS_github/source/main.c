// --- Needed library --- //
#include <COTs/DebugInfoManager/Inc/debugInfo.h>
#include <COTs/BatteryStatusMonitor/Inc/dataMonitor.h>
#include <COTs/FanControlManager/Inc/fanCtrl.h>
#include <COTs/SlaveControlIF/Inc/slaveIF.h>
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_device_registers.h"
// ----------------------------------------------------------------------------
#include "Platform/pcconf.h"
#include "source/COTs/BMSDataBase/Inc/database.h"
// ----------------------------------------------------------------------------
#define CELL_BATTERY_CAPACITY_AH 1.5
#define CELL_OCV_FULL 4.2
#define CELL_OCV_EMPTY 3.0

#define PACK_BATTERY_CAPACITY_AH 21.0
#define PACK_OCV_FULL 58.8
#define PACK_OCV_EMPTY 25
// Global variables to store previous values
static uint32_t prev_time = 0;
static uint8_t SOC;
static long prev_ccounter = 0;	  // Static variable to store previous Coulomb counter value
static uint16_t prev_samples = 0; // Static variable to store previous samples value
/*
@ brief calculation of SOC using OCV method at startup and before connection of load
@ param volage voltage reading of battery
 */
float initialSOC(uint16_t voltage);

void SlaveIF_enableCellBalancing(uint8_t cellNumber, bool enable, float timerValueInMinutes, uint8_t cid);

/*
@ brief calculation of SOC using coloumb counting at runtime and after connection of load
@ param currentA current drawn by the load in Amps
@ param deltaTimeH difference in time between current and previous SOC estimation
 */
uint8_t calculateSOC(TYPE_MEAS_RESULTS_RAW rawResults, uint16_t voltage, uint32_t current_time);

/*
@ brief calculation of SOH using Coulomb counter data
@ param rawResults Raw measurement data including Coulomb counter
@ param nominal_capacity_mAh Nominal capacity of the battery in mAh
@ param v2res Reference voltage resolution
@ param delta_t_seconds Time interval between samples in seconds
 */
float calculate_soh(TYPE_MEAS_RESULTS_RAW rawResults, float voltage, float nominal_capacity_mAh,
					float v2res, uint32_t current_time, uint32_t prev_time);
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

// ----------------------------------------------------------------------------
/*! \brief Default Pack Controller Configuration
 *
 * will be used only once after first start. Then the configuration will be stored
 * in the Flash and loaded from Flash afterwards.
 */
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

	// TPM0_CH1_PWM_Init();
	fanCtrl_fan1Init();
	fanCtrl_fan2Init();

	// PWM_Init();
	DelayInit();

	ScreenIF_Init();
	// dataMonitor_clearScreen();
	//   GPIO_PinInit(GPIOC, 2U, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0}); // Blue LED
	//   GPIO_WritePinOutput(GPIOC, 2U, 0);
	//   Delayms(3*500);
	//   GPIO_WritePinOutput(GPIOC, 2U, 1);

	PRINTF("Board Initialized.\n\r\r");
	float x = 3.14f;
	PRINTF("X = %f\r\n", x);
	bms.Interface = pcconf.IntType;
	bms.EVB = pcconf.EvbType;
	bms.NoClusters = pcconf.NoCluster;
	bms.CIDcurrent = pcconf.CIDcurrent;
	bms.Status = BMS_Init;

	// PIT set lower prio by setting higher value
	NVICSetIrqPrio(SPI0_IRQ, IP_PRIO_1);
	NVICSetIrqPrio(SPI1_IRQ, IP_PRIO_1);
	NVICSetIrqPrio(PIT_IRQ, IP_PRIO_1);

	// setup pit for 1ms timing
	PIT_LDVAL0 = BUSFREQ / 1000;						   // timer ch0 every 0.001 second
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // enable IRQ and timer ch0
	NVICEnIrq(PIT_IRQ);

	LED_RED_On();
	SOC = initialSOC(30); // Abdullah

	// DataMonitor_lcd(50, 50, 2, 25, 1, 0); // Abdullah

	slaveIF_initDriver(&(bms.Interface));
	slaveIF_initCluster(&(cluster[0])); // just tag id is needed for slaveIF_readReg()

	u4TagID = 1;
	gu4CIDSelected = 1;
	cidRoundRobin = 1;
	uint16_t yarab = 600;

	for(;;)
		{
			// PRINTF("Prev sample = %d\r\r\n", prev_samples);
			// PRINTF("prev_ccounter = %d\r\r\n", prev_ccounter);

			//		if(yarab >= 100)
			//		{
			//			yarab = 0;
			//			PRINTF("RESET FAN \r\r\n");
			//		}
			//
			//		TPM0_CH1_SetDutyCycle(yarab);
			//		PRINTF("RUNNING FAN WITH: %d\r\r\n",yarab );
			//		yarab = yarab+3;

			//		fanCtrl_fan1SetDuty(40);
			//		fanCtrl_fan2SetDuty(80);
			// PRINTF("RUNNING FAN 1 WITH: %d\r\r\n", 40);
			// PRINTF("RUNNING FAN 2 WITH: %d\r\r\n", 80);

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

				if (BMSInit(bms.NoClusters))
				{
					for (cid = 1; cid <= bms.NoClusters; cid++)
					{
						slaveIF_clearError();
						MC3377xGetSiliconRevision(cid, &(cluster[cid - 1])); // the sequence / order is required
						MC3377xGetSiliconType(cid, &(cluster[cid - 1]));	 // the sequence / order is required
					}
					bms.Status = BMS_Config;
				}
				break;

			case BMS_Config:
				slaveIF_clearError();
				LEDHandler(Off);
				for (cid = 1; cid <= bms.NoClusters; cid++)
				{
					MC3377xConfig(cid, CONF33771TPL);
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
				//! \todo how to start conversion? Global versus local? (global might overwrite ADC_CFG content)
				// chosen to start ADCs individual, to not change ADC_CFG register
				// in a normal system a global write to all MC33771B would be used
				timeStamp = msTick;

				slaveIF_clearError();

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
						// PRINTF("MC3377xGetRawMeasurements failed for CID %d\n", cid);
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

				if (slaveIF_faultPinStatus())
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
					//						slaveIF_SPICS(1);
					//					}
					//					if(bms->Interface==IntTPL)  {
					//						// SPI-Ard only
					//						slaveIF_SPICS(0);
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
					// DebugPrintMeasurements(cid, cluster[cid-1].NoCTs, &(rawResults[cid-1]));						// output values to host PC
					//  DebugPrintMeasurements(1, cluster[0].NoCTs, &(rawResults[0])); // output values to host PC

					//					uint16_t TeamStackVoltage = (rawResults[0].u16StackVoltage) * 2.44141 * 0.001;
					//					dataMonitor_clearScreen();
					//					ScreenIF_SetCursor(0, 1);
					//					dataMonitor_socDisp(TeamStackVoltage);
					//					ScreenIF_SetCursor(0, 0);
					//					uint8_t SOCPack = calculateSOC(rawResults[0], TeamStackVoltage, msTick);
					//					dataMonitor_socDisp(SOCPack);
					//					uint8_t soc = initialSOC(TeamStackVoltage);
					//				     ScreenIF_SetCursor(0, 2);
					//					dataMonitor_socDisp(soc);

					 //SlaveIF_enableCellBalancing(7, true, 0.5, cid);
					// dataMonitor_modeDisp(SleepMode);
					if (1)
					{
						DataMonitor_lcd(99, 90, 55.5, 52, NormalMode, FaultStatusNone);
						yarab = 0;
					}
					yarab++;
					// uint16_t temp1 = rawResults[0].u16ANVoltage[1];
					// dataMonitor_clearScreen();
					// ScreenIF_SetCursor(0, 0);
					// DataMonitor_lcd((tempSensorIf_Raw2Celsius(temp1)), 90, 55, 52, NormalMode, FaultStatusNone);
					//					dataMonitor_clearScreen();
					//					ScreenIF_SetCursor(0, 0);
					//					dataMonitor_tempDisp(tempSensorIf_Raw2Celsius(rawResults[0].u16ANVoltage[0]));
					//					ScreenIF_SetCursor(0, 1);
					//					dataMonitor_tempDisp(tempSensorIf_Raw2Celsius(rawResults[0].u16ANVoltage[1]));
					//					ScreenIF_SetCursor(0, 2);
					//					dataMonitor_tempDisp(tempSensorIf_Raw2Celsius(rawResults[0].u16ANVoltage[2]));
					//					ScreenIF_SetCursor(10, 3);
					//					dataMonitor_tempDisp(tempSensorIf_Raw2Celsius(rawResults[0].u16ANVoltage[3]));
					//					ScreenIF_SetCursor(10, 0);
					//					dataMonitor_tempDisp(tempSensorIf_Raw2Celsius(rawResults[0].u16ANVoltage[4]));
					//					ScreenIF_SetCursor(10, 1);
					//					dataMonitor_tempDisp(tempSensorIf_Raw2Celsius(rawResults[0].u16ANVoltage[5]));
					//					ScreenIF_SetCursor(10, 2);
					//					dataMonitor_tempDisp(tempSensorIf_Raw2Celsius(rawResults[0].u16ANVoltage[6]));

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

float initialSOC(uint16_t voltage)
{
	if (voltage > 10)
	{
		if (voltage >= PACK_OCV_FULL)
			return 100.0;
		else if (voltage <= PACK_OCV_EMPTY)
			return 0.0;
		else
			return ((voltage - PACK_OCV_EMPTY) / (PACK_OCV_FULL - PACK_OCV_EMPTY)) * 100.0;
	}
	else
	{
		if (voltage >= CELL_OCV_FULL)
			return 100.0;
		else if (voltage <= CELL_OCV_EMPTY)
			return 0.0;
		else
			return ((voltage - CELL_OCV_EMPTY) / (CELL_OCV_FULL - CELL_OCV_EMPTY)) * 100.0;
	}
}

/*
 * Function: calculateSOC
 * Description: Calculates SOC using a hybrid method (OCV initialization + Coulomb counting)
 * Parameters:
 *   - rawResults: Raw measurement data including Coulomb counter and current
 *   - voltage: Stack voltage reading
 *   - current_time: Current timestamp in milliseconds
 * Returns: Updated SOC percentage
 */
uint8_t calculateSOC(TYPE_MEAS_RESULTS_RAW rawResults, uint16_t voltage, uint32_t current_time)
{
	float capacityAh;
	uint64_t delta_ccounter = 0;
	uint64_t delta_samples = 0;
	if (voltage > 10)
	{
		capacityAh = PACK_BATTERY_CAPACITY_AH; // 21.0 Ah for pack
	}
	else
	{
		capacityAh = CELL_BATTERY_CAPACITY_AH; // 1.5 Ah for cell
	}

	// Initialize prev_time and SOC on first call
	//	if (prev_time == 0)
	//	{
	//		SOC = initialSOC(voltage); // Initialize with OCV
	//		prev_time = current_time;
	//		prev_ccounter = rawResults.s32CCCounter;
	//		prev_samples = rawResults.u16CCSamples;
	//		return SOC;
	//	}

	//	// Periodic recalibration with OCV (e.g., every 24 hours = 86,400,000 ms)
	//	if ((current_time - prev_time) >= 86400000)
	//	{
	//		SOC = initialSOC(voltage); // Recalibrate with OCV
	//		prev_time = current_time;
	//		prev_ccounter = rawResults.s32CCCounter;
	//		prev_samples = rawResults.u16CCSamples;
	//	}

	// Calculate time difference in seconds
	float delta_time_seconds = (current_time - prev_time) / 1000.0;

	// Check if samples are available to avoid division by zero
	if (rawResults.u16CCSamples > 0 && prev_samples > 0)
	{
		// Calculate change in Coulomb counter and samples
		delta_ccounter = rawResults.s32CCCounter - prev_ccounter;
		delta_samples = rawResults.u16CCSamples - prev_samples;

		// Calculate average current in Amps (V2RES = 0.0000006 V/LSB)
		float average_current = (delta_ccounter * 0.0000006) / delta_samples;
		float charge_ah = (average_current * delta_time_seconds) / 3600.0; // Convert to Ah

		// Calculate delta SOC
		float deltaSOC = (charge_ah / capacityAh) * 100.0;

		// Update SOC (adjust sign based on charge/discharge)
		SOC -= deltaSOC; // Subtract for discharge, adjust if needed
	}

	// Update previous values
	prev_ccounter = rawResults.s32CCCounter;
	prev_samples = rawResults.u16CCSamples;
	prev_time = current_time;

	//    // Clamp SOC between 0 and 100
	//    if (SOC > 100.0)
	//        SOC = 100.0;
	//    else if (SOC < 0.0)
	//        SOC = 0.0;

	// Print debug information
	PRINTF("CCounter: %d, Samples: %d, Delta CC: %d, Delta Samples: %d, Delta Time: f s, SOC: %d \r\r\n",
		   rawResults.s32CCCounter, rawResults.u16CCSamples, delta_ccounter, delta_samples, delta_time_seconds, SOC);

	return SOC;
}

// float calculateSOC(TYPE_MEAS_RESULTS_RAW rawResults, float voltage, uint32_t current_time)
// {
//     float capacityAh;

//     if (voltage > 10)
//     {
//         capacityAh = PACK_BATTERY_CAPACITY_AH;
//     }
//     else
//     {
//         capacityAh = CELL_BATTERY_CAPACITY_AH;
//     }

//     // Calculate charge difference using Coulomb counter
//     float delta_charge_coulombs = (float)(rawResults.s32CCCounter) * 0.0001;  // Assuming V2RES = 0.0001V, adjust if different
//     float delta_time_seconds = (current_time - prev_time) / 1000.0;           // Convert ms to seconds
//     float deltaSOC = (delta_charge_coulombs / (capacityAh * 3600.0)) * 100.0; // Convert to Ah and percentage

//     SOC += deltaSOC;
//     prev_time = current_time;

//     if (SOC > 100.0)
//         SOC = 100.0;
//     else if (SOC < 0.0)
//         SOC = 0.0;

//     return SOC;
// }

/*
 * Function: calculate_soh
 * Description: Calculates the State of Health (SOH) of the battery based on Coulomb counter data
 * Parameters:
 *   - rawResults: Raw measurement data including Coulomb counter and samples
 *   - voltage: Stack voltage reading to determine pack or cell context
 *   - nominal_capacity_mAh: Nominal capacity of the battery in milliamp-hours (mAh)
 *   - v2res: Reference voltage resolution (V2RES) in volts per LSB
 *   - current_time: Current timestamp in milliseconds
 *   - prev_time: Previous timestamp in milliseconds for time difference
 * Returns: SOH percentage (0.0 to 100.0)
 */
float calculate_soh(TYPE_MEAS_RESULTS_RAW rawResults, float voltage, float nominal_capacity_mAh,
					float v2res, uint32_t current_time, uint32_t prev_time)
{
	float capacityAh;

	// Determine nominal capacity based on voltage (pack or cell)
	if (voltage > 10)
	{
		capacityAh = nominal_capacity_mAh / 1000.0; // Convert mAh to Ah for pack (21 Ah for 14 cells)
	}
	else
	{
		capacityAh = nominal_capacity_mAh / (1000.0 * 14.0); // Convert mAh to Ah per cell (1.5 Ah)
	}

	// Initialize prev_time and prev values on first call
	if (prev_time == 0)
	{
		prev_time = current_time;
		prev_ccounter = rawResults.s32CCCounter;
		prev_samples = rawResults.u16CCSamples;
		return 100.0; // Initial SOH assumed 100% until measured
	}

	// Calculate time difference in seconds
	float delta_time_seconds = (current_time - prev_time) / 1000.0;

	// Check if samples are available to avoid division by zero
	if (rawResults.u16CCSamples > 0 && prev_samples > 0)
	{
		// Calculate change in Coulomb counter and samples
		long delta_ccounter = rawResults.s32CCCounter - prev_ccounter;
		uint16_t delta_samples = rawResults.u16CCSamples - prev_samples;

		// Calculate average current in Amps using V2RES
		float average_current = (delta_ccounter * v2res) / delta_samples;
		float charge_ah = (average_current * delta_time_seconds) / 3600.0; // Convert to Ah

		// Calculate current capacity based on accumulated charge
		float current_capacity_ah = capacityAh - (charge_ah / 1000.0); // Adjust based on discharge

		// Calculate SOH as a percentage of nominal capacity
		float soh = (current_capacity_ah / capacityAh) * 100.0;

		// Ensure SOH is within valid range
		if (soh > 100.0)
			soh = 100.0;
		else if (soh < 0.0)
			soh = 0.0;

		// Update previous values
		prev_ccounter = rawResults.s32CCCounter;
		prev_samples = rawResults.u16CCSamples;
		prev_time = current_time;

		// Return SOH
		return soh;
	}
	else
	{
		// Return previous SOH if no valid samples
		return 100.0; // Default to 100% if no change detected
	}
}
