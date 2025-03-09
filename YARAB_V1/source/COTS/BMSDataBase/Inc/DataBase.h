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

// Structure to hold battery fault data
typedef struct {
    // Temperature flags
    uint8_t overtemperatureFlags;   // Overtemperature
    uint8_t undertemperatureFlags;  // Undertemperature

    // CB Faults Flags
    uint16_t CBShorted;
    uint16_t CBOpenLoad;

    // GPIO Flags
    uint8_t gpioShortFlags;
    uint8_t anOpenLoadFlags;

    // Volt Flags
    uint16_t  overVoltageFlags;
    uint16_t  underVoltageFlags;

    // Number of Coulomb samples Sufficient
    bool coulombSamplesSufficient;
} FaultData;


/*========================================================================================*/
/****************************************** APIs ******************************************/
/*========================================================================================*/
/**
 * @brief Updates the battery measurement data by reading from the Slave IF.
 */
void DataBase_UpdateMeasurementData();

/**
 * @brief Updates the battery fault data by reading from the Slave IF.
 */
void DataBase_UpdateFaultData();

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

/**
 * @brief Gets the latest battery fault data.
 *
 * @param data A pointer to the FaultData struct to store the retrieved data.
 * @return true if data retrieval was successful, false otherwise.
 */
bool DataBase_GetCurrentFaultData(FaultData *data);

/**
 * @brief Gets the previous battery fault data.
 *
 * @param data A pointer to the FaultData struct to store the retrieved data.
 * @return true if data retrieval was successful, false otherwise.
 */
bool DataBase_GetPreviousFaultData(FaultData *data);









void SysTick_Handler(void);
void configure_systick(void);









#endif /* DATABASE_H_ */

