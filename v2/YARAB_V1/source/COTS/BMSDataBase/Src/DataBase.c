/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  BMSDataBase driver
 *	File: 		DataBase.c
 */

#include <COTS/BMSDataBase/Inc/DataBase.h>  // Include the header file



// Private variables to store the data
static MeasurementData currentMeasurementData;
static MeasurementData previousMeasurementData;

/* ============================= Data Update Function ============================= */
void DataBase_UpdateMeasurementData()
{
	// Store the previous measurement data
	DataBase_GetCurrentMeasurementData(&previousMeasurementData);

	// Read current data
	// Read all the measurement data from the Slave IF

	// Read Stack Voltage
	currentMeasurementData.stackVoltage = SlaveIF_readPackVoltage();

	// Read Cell Voltages
	for (int i = 0; i < 14; i++)
		currentMeasurementData.cellVoltages[i] = SlaveIF_readCellVoltage(i + 1); // Cell number 1 to 14

	//Read the current value by calling external func
	currentMeasurementData.Current = SlaveIF_readCurrent();

	// Read temp values (AN0-AN6)
	 for (int i = 0; i < 7; i++)
	 currentMeasurementData.temperatures[i] = SlaveIF_readTemperature(i);  // Call the GPIO function
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









