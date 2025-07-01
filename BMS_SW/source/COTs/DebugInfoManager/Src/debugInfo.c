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
#include <source/COTs/BMSDataBase/Inc/database.h>
#define READ_BIT(address, bit)		     ((address & (1 << bit)) >> bit)
extern uint16_t faultFlag;
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
void DebugInfo_PrintFaultReason(TYPE_STATUS *faultData)
{
	dataMonitor_faultDisp(1);
	if (faultData == NULL)
	{
		PRINTF("[ERROR] DebugInfo: Invalid fault data pointer!\r\r\n");
		return;
	}

	//extern volatile uint32_t pit_count; // From main.c
	//PRINTF("\r\r\n[FAULT DETECTED] Time: %lu ms - Circuit Disconnected (SSR Opened)\r\r\n", pit_count * 100);
	PRINTF("------------------------------------------------------------\r\r\n");

	// Check FAULT1-related faults
	if (faultData->u16CellOV)
	{
		for (int i = 0; i < 14; i++)
		{
			if(READ_BIT(faultData->u16CellOV,i))
				PRINTF("FAULT: Cell Over-Voltage detected. Cell %d\r\r\n", i+1);
		}
		faultFlag++;
	}
	if (faultData->u16CellUV)
	{
		for (int i = 0; i < 14; i++)
		{
			if(READ_BIT(faultData->u16CellUV,i))
				PRINTF("FAULT: Cell Under-Voltage detected. Cell %d\r\r\n", i+1);
		}
		faultFlag++;

	}
	if (falg_temp)
	{
		PRINTF("FAULT: Over-Temperature detected, in sensor no. 7\r\r\n");
	}

	// Check FAULT2-related faults
	if (faultData->u16GPIOOpen)
	{
		PRINTF("FAULT: GPIO Short Circuit detected. Affected GPIOs (bitmask): 0x%02X\r\r\n",
				faultData->u16GPIOOpen);
	}

	if (faultData->u16CBShort)
	{
		PRINTF("FAULT: Cell Balancing Short Circuit detected. Affected channels (bitmask): 0x%04X\r\r\n",
				faultData->u16CBShort);
	}
	if (faultData->u16CBOpen)
	{
		PRINTF("FAULT: Cell Balancing Open Load detected. Affected channels (bitmask): 0x%04X\r\r\n",
				faultData->u16CBOpen);
	}

	// Check raw fault registers for other conditions
	if (faultData->u16Fault1 && !faultData->u16CellOV &&
			!faultData->u16CellUV && !faultData->u16ANOtUt)
	{
		PRINTF("FAULT: Other FAULT1 conditions detected (raw): 0x%04X\r\r\n",
				faultData->u16Fault1);
	}
	if (faultData->u16Fault2 && !faultData->u16GPIOOpen &&
			!faultData->u16CBShort && !faultData->u16GPIOOpen &&
			!faultData->u16CBOpen)
	{
		PRINTF("FAULT: Other FAULT2 conditions detected (raw): 0x%04X\r\r\n",
				faultData->u16Fault2);
	}
	if (faultData->u16Fault3)
	{
		PRINTF("FAULT: FAULT3 conditions detected (raw): 0x%04X\r\r\n",
				faultData->u16Fault3);
	}

	PRINTF("------------------------------------------------------------\r\r\n");
	PRINTF("ACTION: Battery disconnected to ensure safety.\r\r\n");
}


//=============================================================================
// End of File
//=============================================================================
