/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  FuSa driver
 *	File: 		FuSa.h
 */
#include <COTS/SlaveControlIF/Inc/SlaveIF.h>
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>


#ifndef COTS_FUSASUPERVISOR_INC_FUSA_H_
#define COTS_FUSASUPERVISOR_INC_FUSA_H_


/*========================================================================================*/
/**************************************** Structure ***************************************/
/*========================================================================================*/
// Structure to hold battery fault data
typedef struct {
    // Temperature flags
    uint8_t overtemperatureFlags;   // Over temperature
    uint8_t undertemperatureFlags;  // Under temperature

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
 * @brief Updates the battery fault data by reading from the Slave IF.
 */
void FuSa_updateFaultData();

/**
 * @brief Gets the latest battery fault data.
 *
 * @param data A pointer to the FaultData struct to store the retrieved data.
 * @return true if data retrieval was successful, false otherwise.
 */
bool FuSa_getCurrentFaultData(FaultData *data);

/**
 * @brief Gets the previous battery fault data.
 *
 * @param data A pointer to the FaultData struct to store the retrieved data.
 * @return true if data retrieval was successful, false otherwise.
 */
bool FuSa_getPreviousFaultData(FaultData *data);



void SysTick_Handler(void);
void configure_systick(void);


#endif /* FuSaASUPERVISOR_INC_FUSA_H_ */
