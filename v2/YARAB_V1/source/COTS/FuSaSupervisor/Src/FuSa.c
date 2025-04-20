/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  FuSa driver
 *	File: 		FuSa.c
 */


#include <COTS/FuSaSupervisor/Inc/FuSa.h>  // Include the header file


volatile uint32_t systick_count = 0;
volatile bool fault_interrupt = false;
volatile bool data_interrupt  = false;

// Private variables to store the data
static FaultData currentFaultData;
static FaultData previousFaultData;
/* ============================= Data Update Function ============================= */
void FuSa_updateFaultData()
{
	// Store Previous value
	FuSa_getCurrentFaultData(&previousFaultData);

	// Read fault data from the Slave IF
	currentFaultData.overVoltageFlags = SlaveIF_readCellOverVoltageStatus();
	currentFaultData.underVoltageFlags = SlaveIF_readCellUnderVoltageStatus();

	GPIO_AN_Flags GpioAnFaults = SlaveIF_readGpioAnStatus();
	currentFaultData.gpioShortFlags = GpioAnFaults.GPIO_SH_Flags;
	currentFaultData.anOpenLoadFlags = GpioAnFaults.AN_OL_Flags;

	Temperature_Flags TempFault = SlaveIF_readOtUtStatus();
	currentFaultData.overtemperatureFlags = TempFault.Over_Temp_Flags;
	currentFaultData.undertemperatureFlags = TempFault.Under_Temp_Flags;

	currentFaultData.coulombSamplesSufficient = SlaveIF_readNumberCoulombSamples();
}


/* ============================= Data Get Functions ============================= */
bool FuSa_getCurrentFaultData(FaultData *data) {
	if (data == NULL) return false; // Check for invalid pointer

	// Copy the fault data to the provided pointer
	memcpy(data, &currentFaultData, sizeof(FaultData));

	return true;
}



bool FuSa_getPreviousFaultData(FaultData *data) {
	if (data == NULL) return false; // Check for invalid pointer

	// Copy the previous fault data to the provided pointer
	memcpy(data, &previousFaultData, sizeof(FaultData));

	return true;
}





void SysTick_Handler(void)
{
	systick_count++;
	if (systick_count % 10 == 0) // 1 sec
	{
		fault_interrupt = true;
#ifdef DebugInfoManager
		DebugInfo;
#endif
	}

	if (systick_count % 50 == 0) // 5 sec
	{
		data_interrupt = true;
#ifdef DebugInfoManager
		DebugInfo;
#endif
	}
}


void configure_systick(void)
{
	SysTick_Config(SystemCoreClock / 10); // smallest step is 100 mSec
}
