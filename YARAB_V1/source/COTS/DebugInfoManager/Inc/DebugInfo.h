/**
 * @file DebugInfo.h
 * @brief Provides simple debug message capabilities for the BMS project.
 *
 * @details This header defines a macro (`DebugInfo`) to print standard debug
 *          information (function name, file name, line number) to the console.
 *          The printing is conditionally compiled based on the `DebugInfoManager` macro.
 *
 * @note To enable debug prints globally, uncomment the line `#define DebugInfoManager`.
 * @note To disable debug prints globally, ensure `#define DebugInfoManager` is commented out or undefined.
 * @note Usage: Place the following block where a debug print is needed:
 *       #ifdef DebugInfoManager
 *           DebugInfo;
 *       #endif
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: DebugInfoManager
 */

#ifndef DEBUG_INFO_MANAGER
#define DEBUG_INFO_MANAGER

//=============================================================================
// Includes
//=============================================================================
#include <COTS/Public/Inc/helpful.h>
#include <COTS/FuSaSupervisor/Inc/FuSa.h>

//=============================================================================
// Debug Control Macro Definition (General)
//=============================================================================
/**
 * @brief Master switch to enable/disable all `DebugInfo` prints at compile time.
 * @details Uncomment the following line to globally enable the debug messages
 *          defined by the `DebugInfo` macro throughout the project.
 *          Leave it commented out to disable them (reducing code size and execution time).
 */
// #define DebugInfoManager

/**
 * @brief Prints standard debug context information (function, file, line).
 * @details This macro, when used within the `#ifdef DebugInfoManager` block,
 *          will print the current function name, the source file name, and the
 *          line number where the macro is called. Useful for tracing execution flow.
 */
#define DebugInfo PRINTF("DebugInfo: @ Func: %s, File: %s, Line: %d \n\r\r", __func__, __FILE__, __LINE__)

//=============================================================================
//  Module-Specific Debug Switches
//=============================================================================
//  --- SlaveIF Module Debug Switches ---
/**
 * @brief Define these macros to enable specific debug print categories within SlaveIF.c.
 * @details Uncomment the desired lines to activate the corresponding PRINTF statements
 *          in the SlaveIF driver implementation. Keep them commented out to disable.
 */
// #define SLAVEIF_DEBUG_INIT      ///< Enable SlaveIF Initialization step prints
// #define SLAVEIF_DEBUG_CONFIG    ///< Enable SlaveIF Configuration step prints
// #define SLAVEIF_DEBUG_COMM      ///< Enable SlaveIF SPI frame construction/reception prints
// #define SLAVEIF_DEBUG_ISR       ///< Enable SlaveIF ISR entry/workaround prints
// #define SLAVEIF_DEBUG_DMA       ///< Enable SlaveIF DMA Callback prints (Note: May not trigger with ISR workaround)
// #define SLAVEIF_DEBUG_MEAS      ///< Enable SlaveIF Measurement trigger/status prints
// #define SLAVEIF_DEBUG_FAULT     ///< Enable SlaveIF Fault status reading prints
// #define SLAVEIF_DEBUG_BALANCING ///< Enable SlaveIF Cell Balancing operation prints
// #define SLAVEIF_DEBUG_TEMP      ///< Enable SlaveIF Temperature reading/conversion prints
// #define SLAVEIF_DEBUG_VOLTAGE   ///< Enable SlaveIF Voltage reading prints
// #define SLAVEIF_DEBUG_CURRENT   ///< Enable SlaveIF Current reading prints
// #define SLAVEIF_DEBUG_GPIO      ///< Enable SlaveIF GPIO status prints
// #define SLAVEIF_DEBUG
// --- FuSa Module Debug Switches ---
/**
 * @brief Define these macros to enable specific debug print categories within FuSa.c.
 * @details Uncomment the desired lines to activate the corresponding PRINTF statements.
 */
// #define FUSA_DEBUG_UPDATE   ///< Enable prints during FuSa_updateFaultData execution
// #define FUSA_DEBUG_VERBOSE  ///< Enable even more detailed prints within update function

// --- ChargingManager Module Debug Switches ---
/**
 * @brief Define this macro to enable debug prints for SSR operations in ChargingManager.c.
 * @details Uncomment to activate PRINTF statements for SSR open/close and initialization.
 */
// #define CHARGING_DEBUG_SSR ///< Enable ChargingManager SSR operation prints

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
// void DebugInfo_PrintFaultReason(const FaultData *faultData);


#endif /* DEBUG_INFO_MANAGER */


//=============================================================================
// End of File
//=============================================================================
