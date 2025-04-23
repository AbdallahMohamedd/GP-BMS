/**
 * @file DataBase.c
 * @brief Implementation of the BMS Measurement Database module.
 *
 * @details This module provides a simple storage mechanism for the latest
 *          and previous sets of measurements obtained from the BMS slave device(s).
 *          It facilitates access to current and historical data for algorithms
 *          and diagnostics. It relies on the SlaveIF module to retrieve raw data.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: BMSDataBase driver
 */
//=============================================================================
// Includes
//=============================================================================
#include <COTS/BMSDataBase/Inc/DataBase.h>	 // Public header for this module

//=============================================================================
// Static Variables
//=============================================================================
/**
 * @brief Stores the most recently acquired measurement data set.
 * @details This structure holds the latest values read from the slave interface.
 *          Access is provided via DataBase_GetCurrentMeasurementData().
 */
static MeasurementData currentMeasurementData;

/**
 * @brief Stores the measurement data set from the *previous* update cycle.
 * @details Before updating `currentMeasurementData`, its contents are copied here.
 *          This allows comparing current data with the immediately preceding data.
 *          Access is provided via DataBase_GetPreviousMeasurementData().
 */
static MeasurementData previousMeasurementData;

//=============================================================================
// Public Function Implementations
//=============================================================================
/**
 * @brief Initializes the Measurement Database.
 * @details Sets both the current and previous measurement data structures to zero.
 *          This ensures a known initial state before the first update.
 *          Should be called once at system startup.
 */
void DataBase_Init(void)
{
	// Clear both data structures to ensure a known starting state (all zeros)
	memset(&currentMeasurementData, 0, sizeof(MeasurementData));
	memset(&previousMeasurementData, 0, sizeof(MeasurementData));
}

/**
 * @brief Updates the measurement database with fresh data from the slave interface.
 * @details 1. Copies the current `currentMeasurementData` into `previousMeasurementData`.
 *          2. Calls the appropriate `SlaveIF_read...` functions to get new values.
 *          3. Stores the newly read values into `currentMeasurementData`.
 * @note This function should be called periodically at the desired measurement rate.
 * @warning If `SlaveIF_read...` functions return error codes (e.g., -1.0f, -999.0f),
 *          these error codes will be stored directly in the `currentMeasurementData`
 *          structure. Subsequent consumers of this data must be prepared to handle
 *          these potential error indicators.
 */
void DataBase_UpdateMeasurementData()
{
	// Store the previous measurement data
	DataBase_GetCurrentMeasurementData(&previousMeasurementData);

	// Read current data
	// Read all the measurement data from the Slave IF

	// Read Stack Voltage
	currentMeasurementData.stackVoltage = SlaveIF_readPackVoltage();

	// Read Individual Cell Voltages
	for (uint8_t i = 0; i < MAX_CELLS_PER_SLAVE; i++)
	{
		// Cell numbers are typically 1-based in SlaveIF functions
		currentMeasurementData.cellVoltages[i] = SlaveIF_readCellVoltage(i + 1);
	}

	// Read Pack Current
	currentMeasurementData.Current = SlaveIF_readCurrent();

	// Read Temperatures (AN0-AN6)
	for (uint8_t i = 0; i < MAX_TEMP_SENSORS; i++)
	{
		currentMeasurementData.temperatures[i] = SlaveIF_readTemperature(i);
	}
}

/**
 * @brief Retrieves the most recently updated measurement data set.
 * @param[out] data Pointer to a `MeasurementData` structure where the current data will be copied.
 * @return bool True if the data was successfully copied (pointer was valid), false otherwise.
 */
bool DataBase_GetCurrentMeasurementData(MeasurementData *data)
{
	// Validate input pointer
	if (data == NULL)
	{
		return false;
	}

	// Copy the current measurement data to the provided structure
	memcpy(data, &currentMeasurementData, sizeof(MeasurementData));

	return true;
}

/**
 * @brief Retrieves the measurement data set from the previous update cycle.
 * @param[out] data Pointer to a `MeasurementData` structure where the previous data will be copied.
 * @return bool True if the data was successfully copied (pointer was valid), false otherwise.
 */
bool DataBase_GetPreviousMeasurementData(MeasurementData *data)
{
	// Validate input pointer
	if (data == NULL)
	{
		return false;
	}

	// Copy the previous measurement data to the provided structure
	memcpy(data, &previousMeasurementData, sizeof(MeasurementData));

	return true;
}

//=============================================================================
// End of File
//=============================================================================
