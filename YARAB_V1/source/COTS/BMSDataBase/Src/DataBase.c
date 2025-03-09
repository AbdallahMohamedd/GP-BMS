/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  BMSDataBase driver
 *	File: 		DataBase.c
 */

#include <COTS/BMSDataBase/Inc/DataBase.h>  // Include the header file

//#include "fsl_device_registers.h"

volatile uint32_t systick_count = 0;
volatile bool data_interrupt = false;
volatile bool fault_interrupt = false;

// Private variables to store the data
static MeasurementData currentMeasurementData;
static MeasurementData previousMeasurementData;
static FaultData currentFaultData;
static FaultData previousFaultData;
/* ============================= Data Update Function ============================= */
void DataBase_UpdateMeasurementData()
{
	// Store the previous measurement data
	DataBase_GetCurrentMeasurementData(&previousMeasurementData);

	// Read current data
	// Read all the measurement data from the Slave IF

	// Read Stack Voltage
	currentMeasurementData.stackVoltage = SlaveIF_ReadStackVoltage();

	// Read Cell Voltages
	for (int i = 0; i < 14; i++)
		currentMeasurementData.cellVoltages[i] = SlaveIF_ReadCellVoltage(i + 1); // Cell number 1 to 14

	//Read the current value by calling external func
	//currentMeasurementData.Current = measureCurrent();

	// Read temp values (AN0-AN6)
	// for (int i = 0; i < 7; i++)
	// currentMeasurementData.temperatures[i] = SlaveIF_getTempFromAn(i);  // Call the GPIO function
}


void DataBase_UpdateFaultData() {
	// Store Previous value
	DataBase_GetCurrentFaultData(&previousFaultData);

	// Read fault data from the Slave IF
	currentFaultData.overVoltageFlags = SlaveIF_ReadCellOverVoltageStatus();
	currentFaultData.underVoltageFlags = SlaveIF_ReadCellUnderVoltageStatus();

	GPIO_AN_Flags GpioAnFaults = SlaveIF_ReadGpioAnStatus();
	currentFaultData.gpioShortFlags = GpioAnFaults.GPIO_SH_Flag;
	currentFaultData.anOpenLoadFlags = GpioAnFaults.AN_OL_Flags;

	Temperature_Flags TempFault = SlaveIF_ReadOtUttatus();
	currentFaultData.overtemperatureFlags = TempFault.Over_Temp_Flags;
	currentFaultData.undertemperatureFlags = TempFault.Under_Temp_Flags;

	currentFaultData.coulombSamplesSufficient = SlaveIF_ReadNumberCoulombSamples();
}


/* ============================= Data Get Functions ============================= */
bool DataBase_GetCurrentMeasurementData(MeasurementData *data) {
	if (data == NULL) return false; // Check for invalid pointer

	// Copy the current measurement data to the provided pointer
	memcpy(data, &currentMeasurementData, sizeof(MeasurementData));

	return true;
}



bool DataBase_GetPreviousMeasurementData(MeasurementData *data) {
	if (data == NULL) return false; // Check for invalid pointer

	// Copy the previous measurement data to the provided pointer
	memcpy(data, &previousMeasurementData, sizeof(MeasurementData));

	return true;
}



bool DataBase_GetCurrentFaultData(FaultData *data) {
	if (data == NULL) return false; // Check for invalid pointer

	// Copy the fault data to the provided pointer
	memcpy(data, &currentFaultData, sizeof(FaultData));

	return true;
}



bool DataBase_GetPreviousFaultData(FaultData *data) {
	if (data == NULL) return false; // Check for invalid pointer

	// Copy the previous fault data to the provided pointer
	memcpy(data, &previousFaultData, sizeof(FaultData));

	return true;
}



void SysTick_Handler(void)
{
	systick_count++;
	if (systick_count % 1000 == 0) // 1 sec
	{
		fault_interrupt = true;
#ifdef DebugInfoManager
		DebugInfo;
#endif
	}

	if (systick_count % 10000 == 0) // 10 sec
	{
		data_interrupt = true;
#ifdef DebugInfoManager
		DebugInfo;
#endif
	}
}


void configure_systick(void)
{
	SysTick_Config(SystemCoreClock / 1000); // Interrupt 1 msec
}


