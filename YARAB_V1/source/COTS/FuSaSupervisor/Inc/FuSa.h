/**
 * @file fusa.h
 * @brief Public interface for the Functional Safety (FuSa) Supervisor module.
 *
 * @details Declares functions and types for fault monitoring and data retrieval.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: FuSa Supervisor driver (Functional Safety)
 */

#ifndef FuSaA_SUPERVISOR
#define FuSaA_SUPERVISOR

//=============================================================================
// Includes
//=============================================================================
#include <COTS/SlaveControlIF/Inc/SlaveIF.h>     // Needed for reading fault/status registers
#include <COTS/DebugInfoManager/Inc/DebugInfo.h> // For the DebugInfo macro

//=============================================================================
// Defines - Fault Register Bit Positions (Based on Datasheet Tables 58, 59, 60)
//=============================================================================
// --- FAULT1_STATUS ($24) Bits ---
#define FAULT1_CT_UV_FLT_POS (0U)        ///< Summary: Cell Terminal Undervoltage Fault
#define FAULT1_CT_OV_FLT_POS (1U)        ///< Summary: Cell Terminal Overvoltage Fault
#define FAULT1_AN_UT_FLT_POS (2U)        ///< Summary: Analog Input Undertemperature Fault
#define FAULT1_AN_OT_FLT_POS (3U)        ///< Summary: Analog Input Overtemperature Fault
#define FAULT1_IS_OC_FLT_POS (4U)        ///< ISENSE Overcurrent Fault (Sleep Mode Only)
#define FAULT1_IS_OL_FLT_POS (5U)        ///< ISENSE Open Load Fault
#define FAULT1_I2C_ERR_FLT_POS (6U)      ///< I2C Communication Error Fault
#define FAULT1_GPIO0_WUP_FLT_POS (7U)    ///< GPIO0 Wake-up Fault/Event
#define FAULT1_CSB_WUP_FLT_POS (8U)      ///< CSB Wake-up Fault/Event
#define FAULT1_COM_ERR_FLT_POS (9U)      ///< Communication Error Fault (CRC, Framing, etc.)
#define FAULT1_COM_LOSS_FLT_POS (10U)    ///< Communication Loss Timeout Fault
#define FAULT1_VPWR_LV_FLT_POS (11U)     ///< VPWR Low-Voltage Fault
#define FAULT1_VPWR_OV_FLT_POS (12U)     ///< VPWR Overvoltage Fault
#define FAULT1_COM_ERR_OVR_FLT_POS (13U) ///< COM_STATUS Counter Overflow Fault
#define FAULT1_RESET_FLT_POS (14U)       ///< Reset Indication (HW, SW, COM Loss, OSC)
#define FAULT1_POR_FLT_POS (15U)         ///< Power On Reset Indication

// --- FAULT2_STATUS ($25) Bits ---
#define FAULT2_FUSE_ERR_FLT_POS (0U)   ///< Fuse Array Error (ECC Double Error)
#define FAULT2_DED_ERR_FLT_POS (1U)    ///< Fuse Array ECC Corrected Error
#define FAULT2_OSC_ERR_FLT_POS (2U)    ///< Oscillator Fault
#define FAULT2_CB_OPEN_FLT_POS (3U)    ///< Summary: Cell Balance Open Load Fault
#define FAULT2_CB_SHORT_FLT_POS (4U)   ///< Summary: Cell Balance Shorted Load Fault
#define FAULT2_GPIO_SHORT_FLT_POS (5U) ///< Summary: GPIO Short Fault
#define FAULT2_AN_OPEN_FLT_POS (6U)    ///< Summary: Analog Input Open Load Fault
#define FAULT2_IDLE_MODE_FLT_POS (7U)  ///< Idle Mode Entry Indication
#define FAULT2_IC_TSD_FLT_POS (8U)     ///< Internal IC Thermal Shutdown Fault
#define FAULT2_GND_LOSS_FLT_POS (9U)   ///< Ground Loss Fault
#define FAULT2_ADC1_A_FLT_POS (10U)    ///< ADC1-A Fault
#define FAULT2_ADC1_B_FLT_POS (11U)    ///< ADC1-B Fault
#define FAULT2_VANA_UV_FLT_POS (12U)   ///< VANA Undervoltage Fault
#define FAULT2_VANA_OV_FLT_POS (13U)   ///< VANA Overvoltage Fault
#define FAULT2_VCOM_UV_FLT_POS (14U)   ///< VCOM Undervoltage Fault
#define FAULT2_VCOM_OV_FLT_POS (15U)   ///< VCOM Overvoltage Fault

// --- FAULT3_STATUS ($26) Bits ---
#define FAULT3_EOT_CB1_FLT_POS  (0U)  ///< End of Timer Cell Balance 1
#define FAULT3_EOT_CB2_FLT_POS  (1U)  ///< End of Timer Cell Balance 2
#define FAULT3_EOT_CB3_FLT_POS  (2U)  ///< End of Timer Cell Balance 3
#define FAULT3_EOT_CB4_FLT_POS  (3U)  ///< End of Timer Cell Balance 4
#define FAULT3_EOT_CB5_FLT_POS  (4U)  ///< End of Timer Cell Balance 5
#define FAULT3_EOT_CB6_FLT_POS  (5U)  ///< End of Timer Cell Balance 6
#define FAULT3_EOT_CB7_FLT_POS  (6U)  ///< End of Timer Cell Balance 7
#define FAULT3_EOT_CB8_FLT_POS  (7U)  ///< End of Timer Cell Balance 8
#define FAULT3_EOT_CB9_FLT_POS  (8U)  ///< End of Timer Cell Balance 9
#define FAULT3_EOT_CB10_FLT_POS (9U)  ///< End of Timer Cell Balance 10
#define FAULT3_EOT_CB11_FLT_POS (10U) ///< End of Timer Cell Balance 11
#define FAULT3_EOT_CB12_FLT_POS (11U) ///< End of Timer Cell Balance 12
#define FAULT3_EOT_CB13_FLT_POS (12U) ///< End of Timer Cell Balance 13
#define FAULT3_EOT_CB14_FLT_POS (13U) ///< End of Timer Cell Balance 14
#define FAULT3_DIAG_TO_FLT_POS  (14U) ///< Diagnostic Timeout Fault
#define FAULT3_CC_OVR_FLT_POS   (15U) ///< Coulomb Counter Overflow Fault (Value or Samples)

/*========================================================================================*/
// Typedefs
/*========================================================================================*/
/**
 * @brief Structure to hold the fault status data collected from the slave.
 */
typedef struct
{
	// Raw fault register values
	uint16_t rawFault1Status; ///< Raw value from FAULT1_STATUS register ($24)
	uint16_t rawFault2Status; ///< Raw value from FAULT2_STATUS register ($25)
	uint16_t rawFault3Status; ///< Raw value from FAULT3_STATUS register ($26)

	// Detailed fault flags (updated conditionally based on raw status)
	uint16_t overVoltageFlags;     ///< Detailed cell OV flags (from CELL_OV_FLT, read if FAULT1[CT_OV_FLT])
	uint16_t underVoltageFlags;    ///< Detailed cell UV flags (from CELL_UV_FLT, read if FAULT1[CT_UV_FLT])
	uint8_t gpioShortFlags;        ///< Detailed GPIO short flags (from GPIO_SH_AN_OL_STS, read if FAULT2[GPIO_SHORT_FLT])
	uint8_t anOpenLoadFlags;       ///< Detailed AN open load flags (from GPIO_SH_AN_OL_STS, read if FAULT2[AN_OPEN_FLT])
	uint8_t overtemperatureFlags;  ///< Detailed AN OT flags (from AN_OT_UT_FLT_STS, read if FAULT1[AN_OT_FLT])
	uint8_t undertemperatureFlags; ///< Detailed AN UT flags (from AN_OT_UT_FLT_STS, read if FAULT1[AN_UT_FLT])

	// Optional: Add other status fields read conditionally if needed
	uint16_t cbShortFlags; ///< Detailed CB short flags (from CB_SHORT_FLT, read if FAULT2[CB_SHORT_FLT])
	uint16_t cbOpenFlags;  ///< Detailed CB open flags (from CB_OPEN_FLT, read if FAULT2[CB_OPEN_FLT])

} FaultData;

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief Initializes the FuSa module.
 * @details Resets fault data and interrupt flags.
 */
void FuSa_Init(void);

/**
 * @brief Updates the fault database based on fault register checks.
 * @details Reads fault summary registers and, if faults are detected, reads detailed
 *          status registers to update the fault database.
 */
void FuSa_updateFaultData(void);

/**
 * @brief Retrieves the most recently updated fault data set.
 * @param data Pointer to a FaultData structure to store the current fault data.
 * @return true if data was successfully retrieved, false if the input pointer is NULL.
 */
bool FuSa_getCurrentFaultData(FaultData *data);

/**
 * @brief Retrieves the fault data set from the previous update cycle.
 * @param data Pointer to a FaultData structure to store the previous fault data.
 * @return true if data was successfully retrieved, false if the input pointer is NULL.
 */
bool FuSa_getPreviousFaultData(FaultData *data);


#endif /* FuSaA_SUPERVISOR */


//=============================================================================
// End of File
//=============================================================================
