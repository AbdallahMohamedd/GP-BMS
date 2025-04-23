/**
 * @file DataBase.h
 * @brief Public interface header for the BMS Measurement Database module.
 *
 * @details This header file defines the public interface for the DataBase module,
 *          including the data structure used to hold measurements (`MeasurementData`)
 *          and the function prototypes for initializing, updating, and retrieving
 *          measurement data.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: BMSDataBase driver
 */

#ifndef DATABASE_H_
#define DATABASE_H_

//=============================================================================
// Includes
//=============================================================================
#include <COTS/SlaveControlIF/Inc/SlaveIF.h>
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>

//=============================================================================
// Defines
//=============================================================================
/**
 * @brief Maximum number of battery cells monitored by a single slave module.
 * @details Used to size arrays within the MeasurementData structure.
 */
#define MAX_CELLS_PER_SLAVE (14U)

/**
 * @brief Maximum number of temperature sensors monitored (e.g., connected to AN0-AN6).
 * @details Used to size arrays within the MeasurementData structure.
 */
#define MAX_TEMP_SENSORS    (7U)

//=============================================================================
// Typedefs
//=============================================================================

/**
 * @brief Structure to hold a complete set of BMS measurement data.
 * @details Contains voltages, temperatures, and current read from the slave device(s).
 */
typedef struct
{
	/** @brief Array storing individual cell voltages in Volts. Index 0 = Cell 1, ..., Index 13 = Cell 14. */
	float cellVoltages[MAX_CELLS_PER_SLAVE];

	/** @brief Total measured voltage of the battery stack/pack in Volts. */
	float stackVoltage;

	/** @brief Array storing temperatures in degrees Celsius. Index 0 = Sensor 0 (AN0), ..., Index 6 = Sensor 6 (AN6). */
	float temperatures[MAX_TEMP_SENSORS];

	/** @brief Measured battery pack current in Amperes. Positive might indicate discharge, negative charge (verify implementation). */
	float Current;

	// Optional: Add a timestamp field if needed
	// uint32_t timestamp; ///< System timestamp when the data was acquired.

} MeasurementData;

//=============================================================================
// Public Function Prototypes
//=============================================================================

/**
 * @brief Initializes the Measurement Database module.
 * @details Clears the internal data storage (current and previous measurements)
 *          to a known state (usually zeros). Must be called once at system startup
 *          before any other DataBase functions are used.
 */
void DataBase_Init(void);

/**
 * @brief Updates the internal measurement database by reading fresh data from the SlaveIF module.
 * @details This function fetches the latest stack voltage, cell voltages, temperatures,
 *          and current from the Slave Interface and stores them. The previously stored
 *          'current' data is moved to the 'previous' data buffer before the update.
 * @note Should be called periodically after triggering a measurement cycle on the slave device.
 */
void DataBase_UpdateMeasurementData(void);

/**
 * @brief Retrieves the most recently updated measurement data set.
 * @param[out] data Pointer to a `MeasurementData` structure where the current measurement
 *                  data will be safely copied.
 * @return bool Returns `true` if the `data` pointer is valid and the data was copied successfully.
 *              Returns `false` if the `data` pointer is NULL.
 */
bool DataBase_GetCurrentMeasurementData(MeasurementData *data);

/**
 * @brief Retrieves the measurement data set from the *previous* update cycle.
 * @details This provides access to the data as it was *before* the last call to
 *          `DataBase_UpdateMeasurementData`. Useful for calculating changes or rates.
 * @param[out] data Pointer to a `MeasurementData` structure where the previous measurement
 *                  data will be safely copied.
 * @return bool Returns `true` if the `data` pointer is valid and the data was copied successfully.
 *              Returns `false` if the `data` pointer is NULL.
 */
bool DataBase_GetPreviousMeasurementData(MeasurementData *data);

#endif /* DATABASE_H_ */
//=============================================================================
// End of File
//=============================================================================
