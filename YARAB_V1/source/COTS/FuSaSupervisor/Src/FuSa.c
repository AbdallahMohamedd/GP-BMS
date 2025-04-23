/**
 * @file FuSa.c
 * @brief Implementation of the Functional Safety (FuSa) Supervisor module.
 *
 * @details This module periodically checks summary fault registers from the BMS slave(s).
 *          If a summary fault is detected, it reads the detailed status registers
 *          related to that fault category. It uses the SysTick timer for timing.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: FuSa Supervisor driver (Functional Safety)
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTS/FuSaSupervisor/Inc/FuSa.h> // Public header for this module

//=============================================================================
// Module-Scope Variables (Static & Volatile)
//=============================================================================
/**
 * @brief Counter incremented by the SysTick handler.
 * @details Used to generate periodic events based on SysTick intervals.
 *          Declared volatile as it's modified in an ISR and read elsewhere.
 */
volatile uint32_t systick_count = 0;

/**
 * @brief Flag set by SysTick handler to signal a fault check is due.
 * @details Typically checked in the main loop or a dedicated task.
 *          Declared volatile as it's modified in an ISR and read elsewhere.
 */
volatile bool fault_interrupt = false;

/**
 * @brief Flag set by SysTick handler to signal a data update is due.
 * @details Typically checked in the main loop or a dedicated task.
 *          Declared volatile as it's modified in an ISR and read elsewhere.
 */
volatile bool data_interrupt = false;

/**
 * @brief Stores the most recently acquired fault data set.
 * @details Holds the latest fault status read from the slave interface.
 */
static FaultData currentFaultData;

/**
 * @brief Stores the fault data set from the *previous* update cycle.
 * @details Used for comparison or tracking changes in fault status.
 */
static FaultData previousFaultData;

//=============================================================================
// Public Function Implementations
//=============================================================================
/**
 * @brief Initializes the FuSa module.
 */
void FuSa_Init(void)
{
#ifdef FUSA_DEBUG_UPDATE
	PRINTF("FuSa: Initializing...\n\r\r");
#endif
	memset(&currentFaultData, 0, sizeof(FaultData));
	memset(&previousFaultData, 0, sizeof(FaultData));
	systick_count = 0;
	fault_interrupt = false;
	data_interrupt = false;
}

/**
 * @brief Updates the fault database based on fault register checks.
 */
void FuSa_updateFaultData(void)
{
	uint16_t fault1, fault2, fault3;
	bool primary_read_success = true; // Assume success initially for this cycle

#ifdef FUSA_DEBUG_UPDATE
	PRINTF("FuSa: Starting Fault Data Update Cycle.\n\r\r");
#endif
	// Step 1: Store previous data
	memcpy(&previousFaultData, &currentFaultData, sizeof(FaultData));

	// Step 2: Read the main fault status registers
#ifdef FUSA_DEBUG_VERBOSE
	PRINTF("FuSa: Reading FAULT1/2/3 registers...\n\r\r");
#endif
	fault1 = SlaveIF_getFault1Status();
	fault2 = SlaveIF_getFault2Status();
	fault3 = SlaveIF_getFault3Status();

	// Check if primary reads were successful
	if (fault1 == READ_ERROR_VALUE || fault2 == READ_ERROR_VALUE || fault3 == READ_ERROR_VALUE)
	{
		PRINTF("ERROR [FuSa]: Failed to read one or more FAULT status registers!\n\r\r");
		primary_read_success = false;
		// Do not update currentFaultData further, retain previous data
		// The readSuccess flag remains false for this cycle.
		return 0; // Exit the update function
	}

	// Store raw values regardless of success for potential analysis
	currentFaultData.rawFault1Status = fault1;
	currentFaultData.rawFault2Status = fault2;
	currentFaultData.rawFault3Status = fault3;
#ifdef FUSA_DEBUG_UPDATE
	PRINTF("FuSa: FAULT1=0x%04X, FAULT2=0x%04X, FAULT3=0x%04X\n\r", fault1, fault2, fault3);
#endif

	// Step 3: Conditionally read detailed fault information
	// --- Check FAULT1 Summary Bits ---
	if (READ_BIT(fault1, FAULT1_CT_OV_FLT_POS))
	{
#ifdef FUSA_DEBUG_VERBOSE
		PRINTF("FuSa: CT_OV fault summary set. Reading CELL_OV_FLT...\n\r\r");
#endif
		currentFaultData.overVoltageFlags = SlaveIF_readCellOverVoltageStatus();
		if (currentFaultData.overVoltageFlags == READ_ERROR_VALUE)
		{
			PRINTF("ERROR [FuSa]: Failed to read CELL_OV_FLT details!\n\r\r");
			currentFaultData.overVoltageFlags = 0xFFFF; // Indicate specific detail read error
		}
	}
	else
	{
		currentFaultData.overVoltageFlags = 0; // Clear if no summary fault
	}

	if (READ_BIT(fault1, FAULT1_CT_UV_FLT_POS))
	{
#ifdef FUSA_DEBUG_VERBOSE
		PRINTF("FuSa: CT_UV fault summary set. Reading CELL_UV_FLT...\n\r\r");
#endif
		currentFaultData.underVoltageFlags = SlaveIF_readCellUnderVoltageStatus();
		if (currentFaultData.underVoltageFlags == READ_ERROR_VALUE)
		{
			PRINTF("ERROR [FuSa]: Failed to read CELL_UV_FLT details!\n\r\r");
			currentFaultData.underVoltageFlags = 0xFFFF; // Indicate specific detail read error
		}
	}
	else
	{
		currentFaultData.underVoltageFlags = 0;
	}

	if (READ_BIT(fault1, FAULT1_AN_OT_FLT_POS) || READ_BIT(fault1, FAULT1_AN_UT_FLT_POS))
	{
#ifdef FUSA_DEBUG_VERBOSE
		PRINTF("FuSa: AN_OT/UT fault summary set. Reading AN_OT_UT_FLT_STS...\n\r\r");
#endif
		Temperature_Flags tempFlags = SlaveIF_readOtUtStatus();
		if (tempFlags.Over_Temp_Flags == 0xFF && tempFlags.Under_Temp_Flags == 0xFF)
		{
			PRINTF("ERROR [FuSa]: Failed to read AN_OT_UT_FLT_STS details!\n\r\r");
			currentFaultData.overtemperatureFlags = 0xFF; // Indicate error
			currentFaultData.undertemperatureFlags = 0xFF;
		}
		else
		{
			currentFaultData.overtemperatureFlags = tempFlags.Over_Temp_Flags;
			currentFaultData.undertemperatureFlags = tempFlags.Under_Temp_Flags;
		}
	}
	else
	{
		currentFaultData.overtemperatureFlags = 0;
		currentFaultData.undertemperatureFlags = 0;
	}

	// --- Check FAULT2 Summary Bits ---
	if (READ_BIT(fault2, FAULT2_GPIO_SHORT_FLT_POS) || READ_BIT(fault2, FAULT2_AN_OPEN_FLT_POS))
	{
#ifdef FUSA_DEBUG_VERBOSE
		PRINTF("FuSa: GPIO_SHORT/AN_OPEN fault summary set. Reading GPIO_SH_AN_OL_STS...\n\r\r");
#endif
		GPIO_AN_Flags gpioFlags = SlaveIF_readGpioAnStatus();
		if (gpioFlags.GPIO_SH_Flags == 0xFF && gpioFlags.AN_OL_Flags == 0xFF)
		{
			PRINTF("ERROR [FuSa]: Failed to read GPIO_SH_AN_OL_STS details!\n\r\r");
			currentFaultData.gpioShortFlags = 0xFF;
			currentFaultData.anOpenLoadFlags = 0xFF;
		}
		else
		{
			currentFaultData.gpioShortFlags = gpioFlags.GPIO_SH_Flags;
			currentFaultData.anOpenLoadFlags = gpioFlags.AN_OL_Flags;
		}
	}
	else
	{
		currentFaultData.gpioShortFlags = 0;
		currentFaultData.anOpenLoadFlags = 0;
	}

	if (READ_BIT(fault2, FAULT2_CB_SHORT_FLT_POS))
	{
#ifdef FUSA_DEBUG_VERBOSE
		PRINTF("FuSa: CB_SHORT fault summary set. Reading CB_SHORT_FLT...\n\r\r");
#endif
		currentFaultData.cbShortFlags = SlaveIF_readCellBalancingShortedStatus();
		if (currentFaultData.cbShortFlags == READ_ERROR_VALUE)
		{
			PRINTF("ERROR [FuSa]: Failed to read CB_SHORT_FLT details!\n\r\r");
			currentFaultData.cbShortFlags = 0xFFFF;
		}
	}
	else
	{
		currentFaultData.cbShortFlags = 0;
	}

	if (READ_BIT(fault2, FAULT2_CB_OPEN_FLT_POS))
	{
#ifdef FUSA_DEBUG_VERBOSE
		PRINTF("FuSa: CB_OPEN fault summary set. Reading CB_OPEN_FLT...\n\r\r");
#endif
		currentFaultData.cbOpenFlags = SlaveIF_readCellBalancingOpenLoadStatus();
		if (currentFaultData.cbOpenFlags == READ_ERROR_VALUE)
		{
			PRINTF("ERROR [FuSa]: Failed to read CB_OPEN_FLT details!\n\r\r");
			currentFaultData.cbOpenFlags = 0xFFFF;
		}
	}
	else
	{
		currentFaultData.cbOpenFlags = 0;
	}

	// Add checks for other relevant FAULT bits and read corresponding details...

#ifdef FUSA_DEBUG_UPDATE
	PRINTF("FuSa: Fault Data Update Cycle Finished.\n\r\r");
#endif
}

/**
 * @brief Retrieves the most recently updated fault data set.
 */
bool FuSa_getCurrentFaultData(FaultData *data)
{
	if (data == NULL)
		return false;
	memcpy(data, &currentFaultData, sizeof(FaultData));
	return true;
}

/**
 * @brief Retrieves the fault data set from the previous update cycle.
 */
bool FuSa_getPreviousFaultData(FaultData *data)
{
	if (data == NULL)
		return false;
	memcpy(data, &previousFaultData, sizeof(FaultData));
	return true;
}

/**
 * @brief Configures the SysTick timer and enables its interrupt.
 */
void FuSa_configure_systick(void)
{
#ifdef FUSA_DEBUG_UPDATE // Or FUSA_DEBUG_INIT
	PRINTF("FuSa: Configuring SysTick Timer...\n\r\r");
#endif
	// Configure for 100 ms interval (10 Hz)
	SysTick_Config(SystemCoreClock / 10);
}

//=============================================================================
// Interrupt Service Routine (ISR) Implementations
//=============================================================================
/**
 * @brief SysTick Interrupt Service Routine.
 */
void SysTick_Handler(void)
{
	// Increment the global SysTick counter
	systick_count++;

#ifdef FUSA_DEBUG_ISR
	// Print only occasionally to avoid flooding console
	if (systick_count % 10 == 0)
	{ // Example: Print every second
		PRINTF("SysTick: Count = %lu\n\r", systick_count);
	}
#endif

	// Trigger fault check (e.g., every 10 ticks = 1 second)
	if (systick_count % 10 == 0)
	{
#ifdef FUSA_DEBUG_ISR
		PRINTF("SysTick: Setting fault_interrupt flag.\n\r\r");
#endif
		fault_interrupt = true;
	}

	// Trigger less frequent data update (e.g., every 50 ticks = 5 seconds)
	if (systick_count % 50 == 0)
	{
#ifdef FUSA_DEBUG_ISR
		PRINTF("SysTick: Setting data_interrupt flag.\n\r\r");
#endif
		data_interrupt = true;
	}

	if (systick_count == UINT32_MAX)
	{
#ifdef FUSA_DEBUG_ISR
		PRINTF("SysTick: Counter rollover occurred.\n\r\r");
#endif
		systick_count = 0;
	}
}

//=============================================================================
// End of File
//=============================================================================