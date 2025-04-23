/**
 * @file    DebugInfo.c
 * @brief   Debug information manager for the Battery Management System (BMS).
 *
 * @details This module provides functions to print detailed debug messages,
 *          particularly for fault conditions, to assist in diagnostics and
 *          troubleshooting. It interfaces with the FuSa module to interpret
 *          fault data and outputs messages via the serial console.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Dependencies: Relies on FuSa module for fault data.
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>

//=============================================================================
// Public Function Declarations
//=============================================================================
/**
 * @brief Prints detailed fault reasons to the serial console.
 * @param[in] faultData Pointer to the FaultData structure containing fault status.
 * @details This function checks all fault flags in the provided FaultData and
 *          prints specific messages for each detected fault. It covers faults
 *          from FAULT1, FAULT2, FAULT3 registers and their detailed status.
 *          Messages are prefixed with a timestamp (systick_count) for tracking.
 */
void DebugInfo_PrintFaultReason(const FaultData *faultData)
{
	if (faultData == NULL)
	{
		PRINTF("[ERROR] DebugInfo: Invalid fault data pointer!\n\r\r");
		return;
	}

	extern volatile uint32_t systick_count; // From main.c
	PRINTF("\n[FAULT DETECTED] Time: %lu ms - Circuit Disconnected (SSR Opened)\n", systick_count * 100);
	PRINTF("------------------------------------------------------------\n");

	// Check FAULT1-related faults
	if (faultData->overVoltageFlags)
	{
		PRINTF("FAULT: Cell Over-Voltage detected. Affected cells (bitmask): 0x%04X\n",
				faultData->overVoltageFlags);
	}
	if (faultData->underVoltageFlags)
	{
		PRINTF("FAULT: Cell Under-Voltage detected. Affected cells (bitmask): 0x%04X\n",
				faultData->underVoltageFlags);
	}
	if (faultData->overtemperatureFlags)
	{
		PRINTF("FAULT: Over-Temperature detected. Affected sensors (bitmask): 0x%02X\n",
				faultData->overtemperatureFlags);
	}
	if (faultData->undertemperatureFlags)
	{
		PRINTF("FAULT: Under-Temperature detected. Affected sensors (bitmask): 0x%02X\n",
				faultData->undertemperatureFlags);
	}

	// Check FAULT2-related faults
	if (faultData->gpioShortFlags)
	{
		PRINTF("FAULT: GPIO Short Circuit detected. Affected GPIOs (bitmask): 0x%02X\n",
				faultData->gpioShortFlags);
	}
	if (faultData->anOpenLoadFlags)
	{
		PRINTF("FAULT: Analog Open Load detected. Affected channels (bitmask): 0x%02X\n",
				faultData->anOpenLoadFlags);
	}
	if (faultData->cbShortFlags)
	{
		PRINTF("FAULT: Cell Balancing Short Circuit detected. Affected channels (bitmask): 0x%04X\n",
				faultData->cbShortFlags);
	}
	if (faultData->cbOpenFlags)
	{
		PRINTF("FAULT: Cell Balancing Open Load detected. Affected channels (bitmask): 0x%04X\n",
				faultData->cbOpenFlags);
	}

	// Check raw fault registers for other conditions
	if (faultData->rawFault1Status && !faultData->overVoltageFlags &&
			!faultData->underVoltageFlags && !faultData->overtemperatureFlags &&
			!faultData->undertemperatureFlags)
	{
		PRINTF("FAULT: Other FAULT1 conditions detected (raw): 0x%04X\n",
				faultData->rawFault1Status);
	}
	if (faultData->rawFault2Status && !faultData->gpioShortFlags &&
			!faultData->anOpenLoadFlags && !faultData->cbShortFlags &&
			!faultData->cbOpenFlags)
	{
		PRINTF("FAULT: Other FAULT2 conditions detected (raw): 0x%04X\n",
				faultData->rawFault2Status);
	}
	if (faultData->rawFault3Status)
	{
		PRINTF("FAULT: FAULT3 conditions detected (raw): 0x%04X\n",
				faultData->rawFault3Status);
	}

	PRINTF("------------------------------------------------------------\n");
	PRINTF("ACTION: Battery disconnected to ensure safety.\n\n");
}

//=============================================================================
// End of File
//=============================================================================
