/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  BMSDataBase driver
 *	File: 		DataBase.h
 */
#include <COTS/SlaveControlIF/Inc/SlaveIF.h>
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>

#ifndef DATABASE_H_
#define DATABASE_H_

/*========================================================================================*/
/**************************************** Structure ***************************************/
/*========================================================================================*/

// Structure to hold battery measurement data
typedef struct {
    float cellVoltages[14];   // Array of Cell Voltage
    float stackVoltage;       // Stack Voltage
    float temperatures[7];    // Temperatures from AN0 to AN6
    float Current;            // Current
} MeasurementData;

/*========================================================================================*/
/****************************************** APIs ******************************************/
/*========================================================================================*/
/**
 * @brief Updates the battery measurement data by reading from the Slave IF.
 */
void DataBase_UpdateMeasurementData();

/**
 * @brief Gets the latest battery measurement data.
 *
 * @param data A pointer to the MeasurementData struct to store the retrieved data.
 * @return true if data retrieval was successful, false otherwise.
 */
bool DataBase_GetCurrentMeasurementData(MeasurementData *data);

/**
 * @brief Gets the previous battery measurement data.
 *
 * @param data A pointer to the MeasurementData struct to store the retrieved data.
 * @return true if data retrieval was successful, false otherwise.
 */
bool DataBase_GetPreviousMeasurementData(MeasurementData *data);

#endif /* DATABASE_H_ */

