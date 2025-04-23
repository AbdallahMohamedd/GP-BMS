/**
 * @file SlaveIF.h
 * @brief Public interface header for the Slave Interface (SlaveIF) driver.
 *
 * @details This header file defines the public functions, structures, and constants
 *          for interacting with the MC33771B battery monitoring IC via the
 *          MC33664 TPL transceiver. It provides APIs for initialization,
 *          configuration, measurement reading, fault monitoring, and cell balancing.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Slave_Control_IF driver
 */

#ifndef SLAVE_CENTER_IF_H
#define SLAVE_CENTER_IF_H

//=============================================================================
// Includes
//=============================================================================
#include <stdint.h>	 // For standard integer types (uint8_t, uint16_t, etc.)
#include <stdbool.h> // For boolean type (true, false)
#include "fsl_spi.h"
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>
#define READ_ERROR_VALUE 0
//=============================================================================
// Defines and Macros
//=============================================================================
// --- Symbolic names for Slave Register Addresses ---
// These are defined here for documentation and potential use in application code,
// although direct register access should generally be done via the provided APIs.

// --- Initialization and System Configuration ---
#define INIT_REGISTER (0x00U)		///< Device initialization register Address ($00) - Note: Corrected address from original
#define SYS_CFG_GLOBAL_ADDR (0x01U) ///< Global system configuration register Address ($01) - Note: Corrected address
#define SYS_CFG1_ADDR (0x02U)		///< System configuration register 1 Address ($02) - Note: Corrected address
#define SYS_CFG2_ADDR (0x03U)		///< System configuration register 2 Address ($03) - Note: Corrected address
#define SYS_DIAG_ADDR (0x04U)		///< System diagnostic register Address ($04) - Note: Corrected address

// --- ADC Configuration ---
#define ADC_CFG_ADDR (0x05U)		  ///< ADC configuration register Address ($05) - Note: Corrected address
#define ADC2_OFFSET_COMP_ADDR (0x06U) ///< ADC2 offset compensation register Address ($06) - Note: Corrected address

// --- Voltage Fault Monitoring ---
#define OV_UV_EN_ADDR (0x07U)	 ///< Over/under voltage Enable register Address ($07) - Note: Corrected address
#define CELL_OV_FLT_ADDR (0x08U) ///< Over voltage Fault status register Address ($08) - Note: Corrected address
#define CELL_UV_FLT_ADDR (0x09U) ///< Under voltage Fault status register Address ($09) - Note: Corrected address

// --- Cell Balancing Configuration ---
#define CB1_CFG_ADDR (0x0AU)  ///< Cell 1 balance configuration register Address ($0A) - Note: Corrected address
#define CB2_CFG_ADDR (0x0BU)  ///< Cell 2 balance configuration register Address ($0B) - Note: Corrected address
#define CB3_CFG_ADDR (0x0CU)  ///< Cell 3 balance configuration register Address ($0C) - Note: Corrected address
#define CB4_CFG_ADDR (0x0DU)  ///< Cell 4 balance configuration register Address ($0D) - Note: Corrected address
#define CB5_CFG_ADDR (0x0EU)  ///< Cell 5 balance configuration register Address ($0E) - Note: Corrected address
#define CB6_CFG_ADDR (0x0FU)  ///< Cell 6 balance configuration register Address ($0F) - Note: Corrected address
#define CB7_CFG_ADDR (0x10U)  ///< Cell 7 balance configuration register Address ($10)
#define CB8_CFG_ADDR (0x11U)  ///< Cell 8 balance configuration register Address ($11)
#define CB9_CFG_ADDR (0x12U)  ///< Cell 9 balance configuration register Address ($12)
#define CB10_CFG_ADDR (0x13U) ///< Cell 10 balance configuration register Address ($13)
#define CB11_CFG_ADDR (0x14U) ///< Cell 11 balance configuration register Address ($14)
#define CB12_CFG_ADDR (0x15U) ///< Cell 12 balance configuration register Address ($15)
#define CB13_CFG_ADDR (0x16U) ///< Cell 13 balance configuration register Address ($16)
#define CB14_CFG_ADDR (0x17U) ///< Cell 14 balance configuration register Address ($17)

// --- Cell Balancing Status/Fault ---
#define CB_OPEN_FLT_ADDR (0x18U)  ///< Open circuit fault detection for cell balancing Address ($18) - Note: Corrected address
#define CB_SHORT_FLT_ADDR (0x19U) ///< Short circuit fault detection for cell balancing Address ($19) - Note: Corrected address
#define CB_DRV_STS_ADDR (0x1AU)	  ///< Cell balancing driver status register Address ($1A) - Note: Corrected address

// --- GPIO Configuration ---
#define GPIO_CFG1_ADDR (0x1BU) ///< GPIO configuration register 1 Address ($1B) - Note: Corrected address

// --- Temperature and GPIO/AN Fault Status ---
#define AN_OT_UT_FLT_STS_ADDR (0x1EU)  ///< OT/UT Fault status register Address ($1E) - Note: Corrected address
#define GPIO_SH_AN_OL_STS_ADDR (0x1FU) ///< GPIO short/AN open diagnostic status register Address ($1F) - Note: Corrected address

// --- Misc Status ---
#define I_STATUS (0x20U)   ///< PGA DAC value / ADC2 status Address ($20) - Note: Corrected address
#define COM_STATUS (0x21U) ///< Communication status (CRC error count) Address ($21) - Note: Corrected address

// --- Fault Status Summary ---
#define FAULT1_STATUS_ADDR (0x22U) ///< Fault status 1 register Address ($22) - Note: Corrected address
#define FAULT2_STATUS_ADDR (0x23U) ///< Fault status 2 register Address ($23) - Note: Corrected address
#define FAULT3_STATUS_ADDR (0x24U) ///< Fault status 3 register Address ($24) - Note: Corrected address

// --- Coulomb Counter ---
#define CC_NB_SAMPLES_ADDR (0x2BU) ///< Number of samples in coulomb counter Address ($2B) - Note: Corrected address
#define COULOMB_CNT1 (0x2CU)	   ///< Coulomb counting accumulator MSB Address ($2C) - Note: Corrected address
#define COULOMB_CNT2 (0x2DU)	   ///< Coulomb counting accumulator LSB Address ($2D) - Note: Corrected address

// --- Measurements ---
#define MEAS_ISENSE1_ADDR (0x2EU)	///< Current Measurement MSB / ADC2 MSB Address ($2E) - Note: Corrected address
#define MEAS_ISENSE2_ADDR (0x2FU)	///< Current Measurement LSB / ADC2 LSB Address ($2F) - Note: Corrected address
#define MEAS_STACK_ADDR (0x30U)		///< Stack voltage measurement Address ($30) - Note: Corrected address
#define MEAS_CELL1_ADDR (0x31U)		///< Cell 1 voltage measurement Address ($31) - Note: Corrected address
#define MEAS_CELL2_ADDR (0x32U)		///< Cell 2 voltage measurement Address ($32) - Note: Corrected address
#define MEAS_CELL3_ADDR (0x33U)		///< Cell 3 voltage measurement Address ($33) - Note: Corrected address
#define MEAS_CELL4_ADDR (0x34U)		///< Cell 4 voltage measurement Address ($34) - Note: Corrected address
#define MEAS_CELL5_ADDR (0x35U)		///< Cell 5 voltage measurement Address ($35) - Note: Corrected address
#define MEAS_CELL6_ADDR (0x36U)		///< Cell 6 voltage measurement Address ($36) - Note: Corrected address
#define MEAS_CELL7_ADDR (0x37U)		///< Cell 7 voltage measurement Address ($37) - Note: Corrected address
#define MEAS_CELL8_ADDR (0x38U)		///< Cell 8 voltage measurement Address ($38) - Note: Corrected address
#define MEAS_CELL9_ADDR (0x39U)		///< Cell 9 voltage measurement Address ($39) - Note: Corrected address
#define MEAS_CELL10_ADDR (0x3AU)	///< Cell 10 voltage measurement Address ($3A) - Note: Corrected address
#define MEAS_CELL11_ADDR (0x3BU)	///< Cell 11 voltage measurement Address ($3B) - Note: Corrected address
#define MEAS_CELL12_ADDR (0x3CU)	///< Cell 12 voltage measurement Address ($3C) - Note: Corrected address
#define MEAS_CELL13_ADDR (0x3DU)	///< Cell 13 voltage measurement Address ($3D) - Note: Corrected address
#define MEAS_CELL14_ADDR (0x3EU)	///< Cell 14 voltage measurement Address ($3E) - Note: Corrected address
#define MEAS_AN0_ADDR (0x3FU)		///< AN0 voltage measurement Address ($3F) - Note: Corrected address
#define MEAS_AN1_ADDR (0x40U)		///< AN1 voltage measurement Address ($40) - Note: Corrected address
#define MEAS_AN2_ADDR (0x41U)		///< AN2 voltage measurement Address ($41) - Note: Corrected address
#define MEAS_AN3_ADDR (0x42U)		///< AN3 voltage measurement Address ($42) - Note: Corrected address
#define MEAS_AN4_ADDR (0x43U)		///< AN4 voltage measurement Address ($43) - Note: Corrected address
#define MEAS_AN5_ADDR (0x44U)		///< AN5 voltage measurement Address ($44) - Note: Corrected address
#define MEAS_AN6_ADDR (0x45U)		///< AN6 voltage measurement Address ($45) - Note: Corrected address
#define MEAS_IC_TEMP (0x46U)		///< IC temperature measurement Address ($46) - Note: Corrected address
#define MEAS_VBG_DIAG_ADC1A (0x47U) ///< ADC1A band gap reference measurement Address ($47) - Note: Corrected address
#define MEAS_VBG_DIAG_ADC1B (0x48U) ///< ADC1B band gap reference measurement Address ($48) - Note: Corrected address

// --- Thresholds ---
#define TH_ALL_CT_ADDR (0x49U) ///< CTx global over/under voltage threshold Address ($49) - Note: Corrected address
#define TH_CT1_ADDR (0x4AU)	   ///< CT1 over/under voltage threshold Address ($4A) - Note: Corrected address
#define TH_CT2_ADDR (0x4BU)	   ///< CT2 over/under voltage threshold Address ($4B) - Note: Corrected address
// ... (Addresses TH_CT3 to TH_CT14 increment sequentially down to $57)
#define TH_CT14_ADDR (0x57U) ///< CT14 over/under voltage threshold Address ($57) - Note: Corrected address

#define TH_ANO_OT_ADDR (0x58U) ///< AN0 overtemperature threshold Address ($58) - Note: Corrected address
#define TH_AN1_OT_ADDR (0x59U) ///< AN1 overtemperature threshold Address ($59) - Note: Corrected address
// ... (Addresses TH_AN2_OT to TH_AN6_OT increment sequentially down to $5E)
#define TH_AN6_OT_ADDR (0x5EU) ///< AN6 overtemperature threshold Address ($5E) - Note: Corrected address

#define TH_AN0_UT_ADDR (0x5FU) ///< AN0 undertemperature threshold Address ($5F) - Note: Corrected address
#define TH_AN1_UT_ADDR (0x60U) ///< AN1 undertemperature threshold Address ($60) - Note: Corrected address
// ... (Addresses TH_AN2_UT to TH_AN6_UT increment sequentially down to $65)
#define TH_AN6_UT_ADDR (0x65U) ///< AN6 undertemperature threshold Address ($65) - Note: Corrected address

#define TH_ISENSE_OC_ADDR (0x66U)  ///< ISENSE overcurrent threshold Address ($66) - Note: Corrected address
#define TH_COULOMB_CNT_MSB (0x67U) ///< Coulomb counter threshold (MSB) Address ($67) - Note: Corrected address
#define TH_COULOMB_CNT_LSB (0x68U) ///< Coulomb counter threshold (LSB) Address ($68) - Note: Corrected address

// --- Device Info ---
#define SILICON_REV (0x69U) ///< Silicon revision register Address ($69) - Note: Corrected address

// --- Bit Definitions for specific registers ---
// CBx_CFG (Cell Balance Config) - Example: $0A to $17
#define CB_CFG_DURATION_MASK (0x1FFU) ///< Mask for balancing duration (bits 8:0)
#define CB_CFG_ENABLE_BIT (9U)		  ///< Bit position for Cell Balance Enable (CB_EN)

// ADC_CFG ($05) - Example Bit Positions
#define ADC_CFG_RES1A_SHIFT (0U)
#define ADC_CFG_RES1B_SHIFT (2U)
#define ADC_CFG_RES2_SHIFT (4U)
#define ADC_CFG_PGA_GAIN_SHIFT (8U)
#define ADC_CFG_PGA_GAIN_S_SHIFT (9U) // Corrected: PGA_GAIN_S is bits 9:8
#define ADC_CFG_CC_RST_BIT (10U)	  // Corrected: CC_RST is bit 10
#define ADC_CFG_SOC_BIT (11U)		  // Corrected: SOC is bit 11
#define ADC_CFG_TAGID_SHIFT (12U)	  // Corrected: TAGID is bits 15:12

// ADC2_OFFSET_COMP ($06) - Example Bit Positions
#define ADC2_OFFSET_SHIFT (0U)
#define ADC2_OFFSET_MASK (0x00FFU) // Mask for offset bits 7:0
#define CC_RST_CFG_BIT_POS (14U)
#define FREE_CNT_BIT_POS (15U)

//=============================================================================
// Typedefs
//=============================================================================

/**
 * @brief Structure to hold Over Temperature (OT) and Under Temperature (UT) flags.
 * @details Each field holds 7 flags (bit 0 for AN0 to bit 6 for AN6).
 */
typedef struct
{
	uint8_t Over_Temp_Flags;  ///< OT flags (Bit 0: AN0 .. Bit 6: AN6)
	uint8_t Under_Temp_Flags; ///< UT flags (Bit 0: AN0 .. Bit 6: AN6)
} Temperature_Flags;

/**
 * @brief Structure to hold GPIO Short-to-Ground and Analog Open-Load flags.
 * @details Each field holds 7 flags (bit 0 for GPIO0/AN0 to bit 6 for GPIO6/AN6).
 */
typedef struct
{
	uint8_t GPIO_SH_Flags; ///< GPIO Short flags (Bit 0: GPIO0 .. Bit 6: GPIO6)
	uint8_t AN_OL_Flags;   ///< Analog Open Load flags (Bit 0: AN0 .. Bit 6: AN6)
} GPIO_AN_Flags;

//=============================================================================
// Public Function Prototypes
//=============================================================================

//=============================================================================
// --- Initialization and Control ---
//=============================================================================
/**
 * @brief Initializes the SPI and DMA peripherals for communication.
 * @details Must be called once before any other SlaveIF functions.
 */
void SlaveIF_initTransfer(void);

/**
 * @brief Enables the MC33664 transceiver for TPL communication.
 * @details Sets the EN pin high.
 * @return bool True if the operation was successful (pin set).
 */
bool SlaveIF_tplEnable(void);

/**
 * @brief Sends a wake-up sequence to the MC33771B via the TPL line.
 * @details Uses specific timings on the SPI CS pin (temporarily as GPIO).
 */
void SlaveIF_wakeUp(void);

/**
 * @brief Sets the MC33771B device to enter or exit sleep mode request state.
 * @param enterSleepMode True to request sleep mode, false to clear sleep request.
 * @return bool True if the register write was successful, false otherwise.
 */
bool SlaveIF_setSleepMode(bool enterSleepMode);

//=============================================================================
// --- ADC Configuration & Measurement Trigger ---
//=============================================================================
/**
 * @brief Configures the ADC Configuration Register ($06).
 * @details Sets ADC resolutions and current sense (ADC2) PGA settings.
 *          - Sets ADC1A, ADC1B, ADC2 resolutions (e.g., 16-bit).
 *          - Configures ADC2 PGA (e.g., Auto Gain starting at 4x).
 *          - Ensures SOC (Start of Conversion) bit remains 0.
 *          Uses predefined values from `SlaveIF_Cfg.h`. Refer to Datasheet Table 50.
 * @return bool True if the register write was successful, false otherwise.
 */
bool SlaveIF_configAdc(void);

/**
 * @brief Configures the ADC2 Offset Compensation Register ($07).
 * @details Writes to the ADC2_OFFSET_COMP register. Sets the PCB offset compensation
 *          value and configures Coulomb Counter (CC) behavior (free-running, reset source).
 * @param pcbOffset Signed 8-bit value (-128 to +127) for PCB offset compensation.
 *                  Use 0 if no specific offset calibration is performed. Stored in bits 7:0.
 * @return bool True if the register write was successful, false otherwise.
 * @note Refer to Datasheet Rev 8.0, Table 51.
 */
bool SlaveIF_configAdcOffset(int8_t pcbOffset);

//=============================================================================
// --- Init Slave Setting & Measurement Trigger ---
//=============================================================================
/**
 * @brief Performs basic initialization sequence for the MC33771B slave device.
 * @details Calls essential configuration functions (system, ADC, OV/UV enable, GPIO).
 *          Does NOT configure protection thresholds. Call SlaveIF_ConfigureProtectionThresholds()
 *          separately after this function if needed.
 * @return bool True if all basic initialization steps succeed, false otherwise.
 */
bool SlaveIF_init(void);

/**
 * @brief Configures all protection thresholds (OV, UV, OT, UT, OC) on the MC33771B.
 * @details Sets global voltage limits, temperature limits for AN0-AN6, and overcurrent limit
 *          using internally defined placeholder values (modify inside .c file or change to parameters).
 * @return bool True if ALL threshold configurations were written successfully, false otherwise.
 */
bool SlaveIF_ConfigureProtectionThresholds(void);

/**
 * @brief Triggers a single on-demand ADC measurement cycle with an incremented Tag ID.
 * @details Increments the internal Tag ID (0-15), then sets the SOC bit in ADC_CFG
 *          register along with the new Tag ID to start a conversion cycle.
 * @return bool True if the SOC command was successfully sent and optionally confirmed, false otherwise.
 */
bool SlaveIF_startMeasurementCycle(void);

//=============================================================================
// --- Fault Status Reading ---
//=============================================================================
/**
 * @brief Reads the raw value of the Fault Status Register 1 (FAULT1_STATUS).
 * @return uint16_t Raw register value, or READ_ERROR_VALUE on failure.
 */
uint16_t SlaveIF_getFault1Status(void);

/**
 * @brief Reads the raw value of the Fault Status Register 2 (FAULT2_STATUS).
 * @return uint16_t Raw register value, or READ_ERROR_VALUE on failure.
 */
uint16_t SlaveIF_getFault2Status(void);

/**
 * @brief Reads the raw value of the Fault Status Register 3 (FAULT3_STATUS).
 * @return uint16_t Raw register value, or READ_ERROR_VALUE on failure.
 */
uint16_t SlaveIF_getFault3Status(void);

//=============================================================================
// --- Voltage Fault Monitoring ---
//=============================================================================
/**
 * @brief Reads the Over-Voltage (OV) fault status for all cells.
 * @return uint16_t A bitmask where bit N corresponds to cell N+1 (0=OK, 1=Fault). Bits 0-13 used. Returns 0xFFFF on error.
 */
uint16_t SlaveIF_readCellOverVoltageStatus(void);

/**
 * @brief Reads the Under-Voltage (UV) fault status for all cells.
 * @return uint16_t A bitmask where bit N corresponds to cell N+1 (0=OK, 1=Fault). Bits 0-13 used. Returns 0xFFFF on error.
 */
uint16_t SlaveIF_readCellUnderVoltageStatus(void);

//=============================================================================
// --- Cell Balancing ---
//=============================================================================
/**
 * @brief Enables or disables cell balancing for a specific cell with a timer.
 * @param cellNumber Cell number to configure (1-14).
 * @param enable True to enable balancing, false to disable.
 * @param timerValueInMinutes Balancing duration in minutes (0 = indefinite). Max ~255.5 mins.
 * @return bool True if the configuration write was successful, false otherwise.
 */
bool SlaveIF_enableCellBalancing(uint8_t cellNumber, bool enable, float timerValueInMinutes);

/**
 * @brief Reads the status of the cell balancing driver FETs (ON/OFF).
 * @return uint16_t A bitmask where bit N corresponds to cell N+1 driver status (0=OFF, 1=ON). Bits 0-13 used. Returns 0xFFFF on error.
 */
uint16_t SlaveIF_readCellBalancingDriverStatus(void);

/**
 * @brief Reads the short-circuit fault status of the cell balancing FETs.
 * @return uint16_t A bitmask where bit N corresponds to cell N+1 short fault (0=OK, 1=Fault). Bits 0-13 used. Returns 0xFFFF on error.
 */
uint16_t SlaveIF_readCellBalancingShortedStatus(void);

/**
 * @brief Reads the open-load fault status of the cell balancing paths.
 * @return uint16_t A bitmask where bit N corresponds to cell N+1 open fault (0=OK, 1=Fault). Bits 0-13 used. Returns 0xFFFF on error.
 */
uint16_t SlaveIF_readCellBalancingOpenLoadStatus(void);

//=============================================================================
// --- GPIO and Temperature Status/Faults ---
//=============================================================================
/**
 * @brief Reads the GPIO analog input fault status (Short and Open Load).
 * @return GPIO_AN_Flags Structure containing short and open load flags for AN0-AN6. Returns {0xFF, 0xFF} on error.
 */
GPIO_AN_Flags SlaveIF_readGpioAnStatus(void);

/**
 * @brief Reads Over-Temperature (OT) and Under-Temperature (UT) fault status for AN0-AN6.
 * @return Temperature_Flags Structure containing OT and UT flags for AN0-AN6. Returns {0xFF, 0xFF} on error.
 */
Temperature_Flags SlaveIF_readOtUtStatus(void);

//=============================================================================
// --- Measurements ---
//=============================================================================
/**
 * @brief Reads the temperature from a specified ANx/GPIO pin.
 * @details Converts the raw ADC voltage to temperature using an external conversion function
 *          (e.g., `Thermistor_ConvertToCelsius`, which must be provided elsewhere).
 * @param sensorNumber GPIO/ANx input number (0-6).
 * @return float Temperature in degrees Celsius. Returns `TEMP_READ_ERROR` on failure or invalid sensor.
 */
float SlaveIF_readTemperature(uint8_t sensorNumber);

/**
 * @brief Reads the voltage of a specified cell.
 * @param cellNumber Cell number to read (1-14).
 * @return float Cell voltage in Volts. Returns `VOLTAGE_READ_ERROR` on failure.
 */
float SlaveIF_readCellVoltage(uint8_t cellNumber);

/**
 * @brief Reads the total stack voltage (V_PWR supply voltage).
 * @return float Total stack voltage in Volts. Returns `VOLTAGE_READ_ERROR` on failure.
 */
float SlaveIF_readPackVoltage(void);

/**
 * @brief Reads the current measurement from the ISENSE inputs.
 * @details Calculates current based on ADC readings, settled PGA gain, and shunt resistance.
 *          Uses the standard interpretation for ADC2 result (18-bit signed).
 * @warning Ensure `SHUNT_RESISTANCE_OHMS` is correctly defined in `SlaveIF_Cfg.h`.
 * @return float Current in Amperes (A). Returns `CURRENT_READ_ERROR` on failure.
 */
float SlaveIF_readCurrent(void);

//=============================================================================
// --- Coulomb Counter ---
//=============================================================================
/**
 * @brief Reads the number of accumulated Coulomb Counter samples.
 * @return uint16_t Number of samples (0-65535). Returns 0xFFFF on error.
 */
uint16_t SlaveIF_readNumberCoulombSamples(void);

/**
 * @brief Reads the integrated Coulomb Counter value (raw accumulated ADC counts).
 * @return uint16_t Raw accumulated value. Returns 0xFFFF on error. Needs scaling for charge.
 */
uint16_t SlaveIF_readCoulombCountRaw(void);

/**
 * @brief Checks if there are sufficient Coulomb Counter samples accumulated (>= 1).
 * @return bool True if samples >= 1 and read was successful, false otherwise.
 */
bool SlaveIF_isCoulombSamplesSufficient(void);

#endif /* SLAVE_CENTER_IF_H */
//=============================================================================
// End of File
//=============================================================================
