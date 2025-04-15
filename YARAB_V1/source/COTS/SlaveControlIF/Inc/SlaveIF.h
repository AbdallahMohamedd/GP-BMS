/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:  Slave_Control_IF driver
 *	File: 		SlaveIF.h
 */



#include "fsl_spi.h"
#include "MKL25Z4.h"
#include "stdint.h"
#include "fsl_debug_console.h"
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>


#ifndef SLAVE_CENTER_IF_H
#define SLAVE_CENTER_IF_H







// --- Resolution Values (Table 9) ---
#define VCT_ANX_RES_V            152.58789f // Cell voltage resolution uV/LSB (from Table 9)
#define VVPWR_RES                2441.41f   // Pack voltage resolution mV/LSB (typo in datasheet? units uV/LSB?) Assuming uV/LSB based on scale.
#define V2RES_UV_PER_LSB         0.6f       // Current sense user register resolution uV/LSB (Table 9)
#define MEAS_DATA_READY_BIT      15	        // 0b1000 0000 0000 0000
#define ADC2_SAT_BIT             15u        // Bit 15 in MEAS_ISENSE2   ($31)
#define PGA_GCHANGE_BIT          14u        // Bit 14 in MEAS_ISENSE2   ($31)
#define MEAS_ISENSE2_LSB_MASK    0x000Fu    // Bits 0-3 in MEAS_ISENSE2 ($31)
#define MEAS_ISENSE2_MSB_MASK    0x7FFFu

// --- Shunt Resistor Value (!! Must be adapted to actual hardware !!) ---
#define SHUNT_RESISTANCE_OHMS    0.0001f // Example: 100 micro-Ohms


// Structure to hold the temperature flags
typedef struct {
	uint8_t Over_Temp_Flags;
	uint8_t Under_Temp_Flags;
} Temperature_Flags;


// Structure to hold the GPIO_AN flags
typedef struct {
	uint8_t GPIO_SH_Flags;
	uint8_t AN_OL_Flags;
} GPIO_AN_Flags;


/*=====================================================================================*/
/*============================== Initialization Register ==============================*/
/*=====================================================================================*/
/*
    The initialization register is used to configure the initial settings of the device.
    This includes enabling or disabling specific features during startup.
 */
#define INIT_REGISTER                       0x01          // Device initialization register
/*
    The system configuration registers control various global settings of the device,
    including power management and communication settings.
 */
#define SYS_CFG_GLOBAL_ADDR                 0x02          // Global system configuration register
#define SYS_CFG1_ADDR                       0x03          // System configuration register 1
#define SYS_CFG2_ADDR                       0x04          // System configuration register 2
#define SYS_DIAG_ADDR                       0x05          // System diagnostic register
/*
    The ADC configuration registers control the settings for the analog-to-digital converter (ADC),
    which is responsible for measuring voltages and currents within the system.
 */
#define ADC_CFG_ADDR                        0x06          // ADC configuration register
#define ADC2_OFFSET_COMP_ADDR               0x07          // ADC2 offset compensation register

// ADC_CFG ($06 - Table 50)
#define ADC_CFG_RESOLUTION_16BIT 			0x03u // Binary '11'
#define ADC_CFG_PGA_GAIN_SHIFT   			8u
#define ADC_CFG_PGA_GAIN_4X      			0x00u // '0000' - Gain = 4
#define ADC_CFG_PGA_GAIN_16X     			0x01u // '0001' - Gain = 16
#define ADC_CFG_PGA_GAIN_64X     			0x02u // '0010' - Gain = 64
#define ADC_CFG_PGA_GAIN_256X    			0x03u // '0011' - Gain = 256
#define ADC_CFG_PGA_GAIN_AUTO_4X 			0x04u // '0100' - Auto Gain, starts at 4x
#define ADC_CFG_TAGID_SHIFT                 8     // Starting bit position for the 4-bit Tag ID field
#define ADC_CFG_TAGID_MASK                  0x0F  // Mask to isolate the 4 Tag ID bits (binary 1111)


#define ADC_CFG_SOC_BIT          			15u

// ADC2_OFFSET_COMP ($07 - Table 51)
#define ADC2_OFFSET_SHIFT        			0u
#define FREE_CNT_BIT_POS         			15u // Bit position for FREE_CNT
#define CC_RST_CFG_BIT_POS      			14u // Bit position for CC_RST_CFG


/*====================================================================================================*/
/*============================== Voltage and Fault Monitoring Registers ==============================*/
/*====================================================================================================*/
/*
    These registers manage fault detection related to voltage levels,
    such as overvoltage (OV) and under voltage (UV) conditions.
 */
#define OV_UV_EN_ADDR                       0x08          // Overvoltage/under voltage enable register
#define CELL_OV_FLT_ADDR                    0x09          // Overvoltage fault register
#define CELL_UV_FLT_ADDR                    0x0A          // under voltage fault register

/*====================================================================================================*/
/*============================== Cell Balancing Configuration Registers ==============================*/
/*====================================================================================================*/
/*
    These registers configure the cell balancing feature, allowing control over individual cell balancing drivers.
 */
#define CB1_CFG_ADDR                        0x0C          // Cell 1 balance configuration register
#define CB2_CFG_ADDR                        0x0D          // Cell 2 balance configuration register
#define CB3_CFG_ADDR                        0x0E          // Cell 3 balance configuration register
#define CB4_CFG_ADDR                        0x0F          // Cell 4 balance configuration register
#define CB5_CFG_ADDR                        0x10          // Cell 5 balance configuration register
#define CB6_CFG_ADDR                        0x11          // Cell 6 balance configuration register
#define CB7_CFG_ADDR                        0x12          // Cell 7 balance configuration register
#define CB8_CFG_ADDR                        0x13          // Cell 8 balance configuration register
#define CB9_CFG_ADDR                        0x14          // Cell 9 balance configuration register
#define CB10_CFG_ADDR                       0x15          // Cell 10 balance configuration register
#define CB11_CFG_ADDR                       0x16          // Cell 11 balance configuration register
#define CB12_CFG_ADDR                       0x17          // Cell 12 balance configuration register
#define CB13_CFG_ADDR                       0x18          // Cell 13 balance configuration register
#define CB14_CFG_ADDR                       0x19          // Cell 14 balance configuration register
// Bit Definitions for CBx_CFG
#define CB_CFG_DURATION_MASK                0x1FF         // Mask for balancing duration (9 bits)
/*
    These registers store fault conditions related to cell balancing,
    including open and short circuit faults.
 */
#define CB_OPEN_FLT_ADDR                    0x1A          // Open circuit fault detection for cell balancing
#define CB_SHORT_FLT_ADDR                   0x1B          // Short circuit fault detection for cell balancing
#define CB_DRV_STS_ADDR                     0x1C          // Cell balancing driver status register

/*==========================================================================================*/
/*============================== GPIO Configuration Registers ==============================*/
/*==========================================================================================*/
/*
    These registers configure the GPIO (General Purpose Input/Output) ports,
    allowing control over the functionality and behavior of the GPIO pins.
 */
#define GPIO_CFG1_ADDR                      0x1D          // GPIO configuration register 1

/*=======================================================================================*/
/*================================ Temp. fault Registers ================================*/
/*=======================================================================================*/
#define AN_OT_UT_FLT_STS_ADDR					0x20	      // OT UT FLT register

/*=======================================================================================*/
/*============================== Short/Open Diag Registers ==============================*/
/*=======================================================================================*/
/*
 These registers check for short and open diagnostics
 */
#define GPIO_SH_AN_OL_STS_ADDR              0x21          // GPIO short/ open diagnostic status
#define I_STATUS                            0x22          // PGA DAC value

/*============================================================================================*/
/*============================== Communication status Registers ==============================*/
/*============================================================================================*/
/*
 These register has communication number of crc error count
 */
#define COM_STATUS                          0x23          // Number of CRC error counted

/*====================================================================================*/
/*============================== Fault status Registers ==============================*/
/*====================================================================================*/
/*
  These Registers shows the fault statues for different components.
 */
#define FAULT1_STATUS_ADDR                  0x24          // Fault status 1
#define FAULT2_STATUS_ADDR                  0x25          // Fault status 2
#define FAULT3_STATUS_ADDR                  0x26          // Fault status 3


/*=======================================================================================================*/
/*============================== Coulomb count number of samples Registers ==============================*/
/*=======================================================================================================*/
/*
 This register used to count coulomb count number of samples
 */
#define CC_NB_SAMPLES_ADDR                  0x2D          // Number of samples in coulomb counter

/*====================================================================================================*/
/*============================== Coulomb Counting Accumulator Registers ==============================*/
/*====================================================================================================*/
/*
 These register used to Accumulated Samples
 */
#define COULOMB_CNT1                        0x2E          // Coulomb counting accumulator MSB
#define COULOMB_CNT2                        0x2F          // Coulomb counting accumulator LSB

/*===========================================================================================*/
/*============================== Current Measurement Registers ==============================*/
/*===========================================================================================*/
/*
 These register used to to measure the the current .
 */
#define MEAS_ISENSE1_ADDR                   0x30          // Current Measurement MSB
#define MEAS_ISENSE2_ADDR                   0x31          // Current Measurement LSB

/*================================================================================================*/
/*============================== Cell Voltage Measurement Registers ==============================*/
/*================================================================================================*/
/*    This register Stack voltage measurement */
#define MEAS_STACK_ADDR                     0x32          // Stack voltage measurement
/*
 * These register used to measure cell voltage.
 */
#define MEAS_CELL14_ADDR                    0x33          // Cell 14 voltage measurement
#define MEAS_CELL13_ADDR                    0x34          // Cell 13 voltage measurement
#define MEAS_CELL12_ADDR                    0x35          // Cell 12 voltage measurement
#define MEAS_CELL11_ADDR                    0x36          // Cell 11 voltage measurement
#define MEAS_CELL10_ADDR                    0x37          // Cell 10 voltage measurement
#define MEAS_CELL9_ADDR                     0x38          // Cell 9 voltage measurement
#define MEAS_CELL8_ADDR                     0x39          // Cell 8 voltage measurement
#define MEAS_CELL7_ADDR                     0x3A          // Cell 7 voltage measurement
#define MEAS_CELL6_ADDR                     0x3B          // Cell 6 voltage measurement
#define MEAS_CELL5_ADDR                     0x3C          // Cell 5 voltage measurement
#define MEAS_CELL4_ADDR                     0x3D          // Cell 4 voltage measurement
#define MEAS_CELL3_ADDR                     0x3E          // Cell 3 voltage measurement
#define MEAS_CELL2_ADDR                     0x3F          // Cell 2 voltage measurement
#define MEAS_CELL1_ADDR                     0x40          // Cell 1 voltage measurement
/*
 * ANX is analog input
 */
#define MEAS_AN6_ADDR                       0x41          // AN6 voltage measurement
#define MEAS_AN5_ADDR                       0x42          // AN5 voltage measurement
#define MEAS_AN4_ADDR                       0x43          // AN4 voltage measurement
#define MEAS_AN3_ADDR                       0x44          // AN3 voltage measurement
#define MEAS_AN2_ADDR                       0x45          // AN2 voltage measurement
#define MEAS_AN1_ADDR                       0x46          // AN1 voltage measurement
#define MEAS_AN0_ADDR                       0x47          // ANO voltage measurement
/*
 * These register IC temperature measurement
 */
#define MEAS_IC_TEMP                        0x48          // IC temperature measurement

/*============================================================================================================*/
/*============================== ADCIA band gap reference measurement Registers ==============================*/
/*============================================================================================================*/
/*
    These register isADCIA band gap reference measurement
 */
#define MEAS_VBG_DIAG_ADC1A                 0x49          // ADCIA band gap reference measurement

/*============================================================================================================*/
/*============================== ADCIB band gap reference measurement Registers ==============================*/
/*============================================================================================================*/
/*
    These register isADCIB band gap reference measurement
 */
#define MEAS_VBG_DIAG_ADC1B                 0x4A          // ADCIB band gap reference measurement

/*==================================================================================*/
/*==============================  Threshold Registers ==============================*/
/*==================================================================================*/
/*
    These register is CTx over and under voltage threshold Registers
 */
#define TH_ALL_CT_ADDR                      0x4B          // CTx  over and under voltage threshold
#define TH_CT14                             0x4C          // CT14 over and under voltage threshold
#define TH_CT13                             0x4D          // CT13 over and under voltage threshold
#define TH_CT12                             0x4E          // CT12 over and under voltage threshold
#define TH_CT11                             0x4F          // CT11 over and under voltage threshold
#define TH_CT10                             0x50          // CT10 over and under voltage threshold
#define TH_CT9                              0x51          // CT9  over and under voltage threshold
#define TH_CT8                              0x52          // CT8  over and under voltage threshold
#define TH_CT7                              0x53          // CT7  over and under voltage threshold
#define TH_CT6                              0x54          // CT6  over and under voltage threshold
#define TH_CT5                              0x55          // CT5  over and under voltage threshold
#define TH_CT4                              0x56          // CT4  over and under voltage threshold
#define TH_CT3                              0x57          // CT3  over and under voltage threshold
#define TH_CT2                              0x58          // CT2  over and under voltage threshold
#define TH_CT1_ADDR                              0x59          // CT1  over and under voltage threshold
/*
    These register is TH_ANx_OT overtemperature threshold Registers
 */
#define TH_AN6_OT_ADDR                      0x5A          // AN6 overtemperature threshold
#define TH_AN5_OT_ADDR                      0x5B          // AN5 overtemperature threshold
#define TH_AN4_OT_ADDR                      0x5C          // AN4 overtemperature threshold
#define TH_AN3_OT_ADDR                      0x5D          // AN3 overtemperature threshold
#define TH_AN2_OT_ADDR                      0x5E          // AN2 overtemperature threshold
#define TH_AN1_OT_ADDR                      0x5F          // AN1 overtemperature threshold
#define TH_ANO_OT_ADDR                      0x60          // ANO overtemperature threshold
/*
   These register is ANX undertemperature threshold  Registers
 */
#define TH_AN6_UT_ADDR                      0x61          // AN6 undertemperature threshold
#define TH_AN5_UT_ADDR                      0x62          // AN5 undertemperature threshold
#define TH_AN4_UT_ADDR                      0x63          // AN4 undertemperature threshold
#define TH_AN3_UT_ADDR                      0x64          // AN3 undertemperature threshold
#define TH_AN2_UT_ADDR                      0x65          // AN2 undertemperature threshold
#define TH_AN1_UT_ADDR                      0x66          // AN1 undertemperature threshold
#define TH_AN0_UT_ADDR                      0x67          // ANO undertemperature threshold
/*
 This register ISENSE overcurrent threshold Register
 */
#define TH_ISENSE_OC_ADDR                   0x68          // ISENSE overcurrent threshold
/*
  This register over Coulomb counter threshold
 */
#define TH_COULOMB_CNT_MSB                  0x69          // Coulomb counter threshold (MSB)
#define TH_COULOMB_CNT_LSB                  0x6A          // Coulomb counter threshold (LSB)

/*========================================================================================*/
/*============================== Silicon revision Registers ==============================*/
/*========================================================================================*/
/*
   This register Silicon revision
 */
#define SILICON_REV                         0x6B          // Silicon revision
// Bit Definitions for SILICON_REV
#define SILICON_REV_MASK                    0xFFFF        // Silicon revision mask
#define SILICON_REV_SHIFT                   0             // no shift




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*========================================================================================*/
/****************************************** APIs ******************************************/
/*========================================================================================*/

bool SlaveIF_wakeUp(void);


/**
 * @brief Initializes the SPI transfer module for communication with the MC33771B via the MC33664 transceiver.
 */
void SlaveIF_initTransfer(void);

/* ============================= System Configuration APIs ============================= */
/**
 * @brief Configures the initialization of the MC33771B with Cluster ID and bus switch settings.
 * @param channelSwitch Boolean value: true to enable bus switch, false to disable it.
 */
void SlaveIF_setupSystem(bool channelSwitch);

/**
 * @brief Sets the MC33771B to sleep mode to save power or wakes it up.
 * @param sleepMode Boolean value: true to enter sleep mode, false to wake up.
 */
void SlaveIF_setSleepMode(bool sleepMode);

/**
 * @brief Configures System Configuration Register 1 with predefined settings.
 */
void SlaveIF_configSystem1(void);

/**
 * @brief Configures System Configuration Register 2 with predefined settings.
 */
void SlaveIF_configSystem2(void);

/* ============================= ADC Configuration APIs ============================= */
/**
 * @brief Configures the ADC resolutions and the current sense PGA settings.
 * @details Writes to the ADC_CFG register ($06). Sets all ADCs (ADC1A, ADC1B, ADC2)
 *          to 16-bit resolution and configures the ADC2 PGA for automatic gain
 *          control, starting at 4x. Ensures SOC bit remains 0.
 * @note Assumes ADC_CFG_ADDR, ADC_CFG_RESOLUTION_16BIT, ADC_CFG_PGA_GAIN_AUTO_4X,
 *       ADC_CFG_PGA_GAIN_SHIFT macros/constants are defined. Refer to Table 50.
 */
void SlaveIF_configAdc(void);

/**
 * @brief Configures the ADC2 offset compensation register.
 * @details Writes to the ADC2_OFFSET_COMP ($07) register. Sets the PCB offset to 0
 *          and configures Coulomb Counter behavior (free-running, rollover, reset on explicit command).
 * @param pcbOffset Signed 8-bit value (-128 to 127) for PCB offset compensation.
 *                  This value usually requires system calibration. Default 0.
 * (Based on Datasheet Rev 8.0, Table 51)
 */
void SlaveIF_configAdcOffset(int8_t pcbOffset);

/**
 * @brief Triggers a single on-demand measurement cycle using a fixed Tag ID of 0.
 * @details This function initiates a new measurement sequence by the MC33771A.
 *          It reads the current configuration from the ADC_CFG register ($06),
 *          preserves existing settings (like resolution and PGA gain), sets the
 *          Start of Conversion (SOC) bit (bit 12) to 1, ensures the Tag ID
 *          (bits 11-8) is 0, and then writes the modified configuration back to
 *          the ADC_CFG register.
 * @return true If both the initial read of ADC_CFG and the subsequent write
 *              to trigger the conversion were successful via SPI.
 * @return false If the initial read from ADC_CFG failed, or if the final write
 *               to ADC_CFG failed (indicating the SOC command was not sent).
 * @note Setting the SOC bit prompts the slave device to perform a full measurement
 *       cycle according to the current ADC configuration. This function uses a
 *       fixed Tag ID of 0 for simplicity, as tag correlation might be limited based
 *       on the device's response behavior for individual measurement registers.
 */
bool SlaveIF_startMeasurementCycle(void);
/* ============================= Voltage Fault Monitoring APIs ============================= */
/**
 * @brief Enables overvoltage and undervoltage detection for all cells.
 */
void SlaveIF_enableOvUv(void);

/**
 * @brief Reads the CELL_OV_FLT register to retrieve the overvoltage status of each cell.
 * @return uint16_t A 16-bit value where bits 0-13 represent the overvoltage status of cells CT1-CT14 (1 = fault, 0 = no fault).
 *         Bits 14-15 are reserved and always 0.
 */
uint16_t SlaveIF_readCellOverVoltageStatus(void);

/**
 * @brief Reads the CELL_UV_FLT register to retrieve the undervoltage status of each cell.
 * @return uint16_t A 16-bit value where bits 0-13 represent the undervoltage status of cells CT1-CT14 (1 = fault, 0 = no fault).
 *         Bits 14-15 are reserved and always 0.
 */
uint16_t SlaveIF_readCellUnderVoltageStatus(void);

/* ============================= Cell Balancing APIs ============================= */
/**
 * @brief Configures cell balancing for a specific cell with a timer.
 * @param cellNumber The cell number to configure (1 to 14).
 * @param enable Boolean value: true to enable cell balancing, false to disable it.
 * @param timerValueInMinutes The duration of cell balancing in minutes (e.g., 1.5 for 1.5 minutes).
 */
void SlaveIF_enableCellBalancing(uint8_t cellNumber, bool enable, float timerValueInMinutes);

/**
 * @brief Reads the CB_DRV_STS register and prints the status of each cell balancing driver.
 */
void SlaveIF_readCellBalancingDriverStatus(void);

/**
 * @brief Reads the CB_SHORT_FLT register to retrieve the shorted load status of cell balancing circuits.
 * @return uint16_t A 16-bit value where each bit represents the shorted status of a cell’s balancing circuit.
 */
uint16_t SlaveIF_readCellBalancingShortedStatus(void);

/**
 * @brief Reads the CB_OPEN_FLT register to retrieve the open load status of cell balancing circuits.
 * @return uint16_t A 16-bit value where each bit represents the open load status of a cell’s balancing circuit.
 */
uint16_t SlaveIF_readCellBalancingOpenLoadStatus(void);

/* ============================= GPIO Configuration APIs ============================= */
/**
 * @brief Configures all GPIO pins (0-6) as analog inputs for ratiometric temperature measurement.
 */
void SlaveIF_configAllGpiosForTempSensors(void);

/* ============================= Temperature APIs ============================= */
/**
 * @brief Reads the AN_OT_UT_FLT_STS register to retrieve overtemperature and undertemperature flags.
 * @return Temperature_Flags A structure containing two 8-bit fields:
 *         - Over_Temp_Flags: Bits 0-6 indicate overtemperature faults for AN0-AN6.
 *         - Under_Temp_Flags: Bits 0-6 indicate undertemperature faults for AN0-AN6.
 */
Temperature_Flags SlaveIF_readOtUtStatus(void);

/**
 * @brief Reads the temperature from a specified GPIO sensor (AN0-AN6).
 * @param sensorNumber The GPIO sensor number (0 to 6).
 * @return float The temperature in degrees Celsius. Returns -999.0 if the sensor number is invalid or data is not ready.
 */
float SlaveIF_readTemperature(uint8_t sensorNumber);

/* ============================= GPIO Flag APIs ============================= */
/**
 * @brief Reads the GPIO_SH_AN_OL_STS register to retrieve GPIO short circuit and analog open load flags.
 * @return GPIO_AN_Flags A structure containing two 8-bit fields:
 *         - GPIO_SH_Flag: Bits 0-6 indicate short circuit faults for GPIO0-GPIO6.
 *         - AN_OL_Flags: Bits 0-6 indicate open load faults for AN0-AN6.
 */
GPIO_AN_Flags SlaveIF_readGpioAnStatus(void);

/* ============================= Coulomb Counter APIs ============================= */
/**
 * @brief Reads the CC_NB_SAMPLES register to retrieve the number of accumulated coulomb counter samples.
 * @return uint16_t The number of samples accumulated for coulomb counting.
 */
uint16_t SlaveIF_readNumberCoulombSamples(void);

/**
 * @brief Checks if the number of coulomb counter samples is sufficient for a valid reading.
 * @return bool True if the number of samples is >= 1, false otherwise.
 */
bool SlaveIF_isCoulombSamplesSufficient(void);

/* ============================= Voltage, Current & Temperature Measurement APIs ============================= */
/**
 * @brief Reads the voltage of a specified cell from the MEAS_CELLx register.
 * @param cellNumber The cell number to read (1 to 14).
 * @return float The cell voltage in volts. Returns 0.0 if data is invalid or not ready.
 */
float SlaveIF_readCellVoltage(uint8_t cellNumber);

/**
 * @brief Reads the total stack voltage from the MEAS_STACK register.
 * @return float The stack voltage in volts. Returns -1.0 if data is not ready.
 */
float SlaveIF_readPackVoltage(void);

/**
 * @brief Reads current combining 14 MSB (+$30 bit 14 as sign) + 4 LSB ($31) -> 18-bit signed value (USER INTERPRETATION).
 * @details This function implements a user-specified interpretation of current reading:
 *          - Reads $30 (MEAS_ISENSE1) & $31 (MEAS_ISENSE2).
 *          - Checks Data Ready ($30[15]).
 *          - Uses $30[14] as the SIGN BIT for the combined value.
 *          - Uses $30[13:0] as the 14 MSBs.
 *          - Uses $31[3:0] as the 4 LSBs.
 *          - Combines into an 18-bit value and sign-extends to 32 bits.
 *          - Determines PGA gain from $06 (ADC_CFG).
 *          - Scales the 18-bit signed value relative to ±150mV input range *and* PGA gain to find VIND.
 *          - Calculates current using Ohm's law.
 *          !!! WARNING: This method STRONGLY DEVIATES from the standard interpretation
 *              of datasheet Rev 8.0 (Tables 80, 81, 9). Use with extreme caution and validation. !!!
 * @return Current in amperes (A), or a distinct negative value signifying an error.
 */
float SlaveIF_readCurrent(void);

/* ========================= Public Threshold Configuration Functions ========================= */

/**
 * @brief Sets the global Overvoltage (OV) and Undervoltage (UV) thresholds for all cells.
 *
 * This function calculates the appropriate 16-bit value based on the desired
 * voltage thresholds and writes it to the TH_ALL_CT register (0x4B) via SPI.
 * The OV threshold occupies the high byte (bits 15-8) and the UV threshold
 * occupies the low byte (bits 7-0).
 *
 * @param ov_volts Desired global Overvoltage threshold in Volts.
 * @param uv_volts Desired global Undervoltage threshold in Volts.
 * @return true If the SPI write operation to the slave device was successful.
 * @return false If the SPI write operation failed (e.g., timeout, CRC error).
 *
 * @note Refer to MC33771A Datasheet (Rev 8.0) Table 82 for TH_ALL_CT register details
 *       and Table 43 for the register address. Resolution for both OV/UV is
 *       19.53125 mV/LSB.
 */
bool SlaveIf_setGlobalOvUvThreshold(float ov_volts, float uv_volts);

/**
 * @brief Sets the individual Overvoltage (OV) and Undervoltage (UV) thresholds for a specific cell.
 *
 * This function calculates the appropriate 16-bit value for the specified cell
 * index and writes it to the corresponding TH_CTx register (0x4C to 0x59) via SPI.
 * The OV threshold occupies the high byte (bits 15-8) and the UV threshold
 * occupies the low byte (bits 7-0).
 *
 * @param cell_index The target cell index (1 to 14). Index 1 corresponds to TH_CT1 (0x59),
 *                   index 14 corresponds to TH_CT14 (0x4C).
 * @param ov_volts Desired Overvoltage threshold for the specified cell in Volts.
 * @param uv_volts Desired Undervoltage threshold for the specified cell in Volts.
 * @return true If the cell_index is valid and the SPI write operation was successful.
 * @return false If the cell_index is invalid or the SPI write operation failed.
 *
 * @note Refer to MC33771A Datasheet (Rev 8.0) Table 83 for TH_CTx register details
 *       and Table 43 for register addresses. Resolution is 19.53125 mV/LSB.
 */
bool SlaveIf_setCellOvUvThreshold(uint8_t cell_index, float ov_volts, float uv_volts);

/**
 * @brief Sets the Overtemperature (OT) threshold for a specific analog input (GPIOx used as ANx).
 *
 * This function calculates the appropriate 10-bit value based on the desired
 * threshold voltage (typically from an NTC) and writes it to the corresponding
 * TH_ANx_OT register (0x5A to 0x60) via SPI.
 *
 * @param anx_index The target analog input index (0 to 6). Index 0 corresponds to TH_AN0_OT (0x60),
 *                  index 6 corresponds to TH_AN6_OT (0x5A).
 * @param overTemp_inVolt Desired Overtemperature threshold expressed as a voltage in Volts.
 * @return true If the anx_index is valid and the SPI write operation was successful.
 * @return false If the anx_index is invalid or the SPI write operation failed.
 *
 * @note Refer to MC33771A Datasheet (Rev 8.0) Table 84 for TH_ANx_OT register details
 *       and Table 43 for register addresses. Resolution is 4.8828125 mV/LSB.
 *       The calculated value occupies bits 9-0 of the register.
 */
bool SlaveIf_setOverTempThreshold(uint8_t anx_index, float overTemp_inVolt);

/**
 * @brief Sets the Undertemperature (UT) threshold for a specific analog input (GPIOx used as ANx).
 *
 * This function calculates the appropriate 10-bit value based on the desired
 * threshold voltage (typically from an NTC) and writes it to the corresponding
 * TH_ANx_UT register (0x61 to 0x67) via SPI.
 *
 * @param anx_index The target analog input index (0 to 6). Index 0 corresponds to TH_AN0_UT (0x67),
 *                  index 6 corresponds to TH_AN6_UT (0x61).
 * @param underTemp_inVolt Desired Undertemperature threshold expressed as a voltage in Volts.
 * @return true If the anx_index is valid and the SPI write operation was successful.
 * @return false If the anx_index is invalid or the SPI write operation failed.
 *
 * @note Refer to MC33771A Datasheet (Rev 8.0) Table 85 for TH_ANx_UT register details
 *       and Table 43 for register addresses. Resolution is 4.8828125 mV/LSB.
 *       The calculated value occupies bits 9-0 of the register.
 */
bool SlaveIf_setUnderTempThreshold(uint8_t anx_index, float underTemp_inVolt);

/**
 * @brief Sets the Overcurrent (OC) threshold during sleep mode.
 *
 * This function calculates the appropriate 12-bit value based on the desired
 * overcurrent limit and the shunt resistor value, then writes it to the
 * TH_ISENSE_OC register (0x68) via SPI.
 *
 * @param overCurrent_amps Desired Overcurrent threshold in Amperes.
 * @param shunt_resistance_micro_ohms The resistance of the external current shunt
 *                                    resistor in micro-Ohms (e.g., 100 for 100uOhm).
 * @return true If the SPI write operation to the slave device was successful.
 * @return false If the SPI write operation failed.
 *
 * @note Refer to MC33771A Datasheet (Rev 8.0) Table 86 for TH_ISENSE_OC register details
 *       and Table 43 for the register address. Resolution is 1.2 uV/LSB.
 *       The calculated value occupies bits 11-0 of the register.
 */
bool SlaveIf_setOverCurrentThreshold(float overCurrent_amps, float shunt_resistance_micro_ohms);

#endif
