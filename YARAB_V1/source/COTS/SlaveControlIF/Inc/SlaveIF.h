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

#define VCT_ANX_RES_V 152.58789    // µV/LSB
#define VVPWR_RES 2441.41 		   // µV/LSB
#define MEAS_DATA_READY 0x8000	   // 0b1000 0000 0000 0000



// Structure to hold the temperature flags
typedef struct {
	uint8_t Over_Temp_Flags;
	uint8_t Under_Temp_Flags;
} Temperature_Flags;




// Structure to hold the GPIO_AN flags
typedef struct {
	uint8_t GPIO_SH_Flag;
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
#define SYS_CFG_GLOBAL                      0x02          // Global system configuration register
#define SYS_CFG1                            0x03          // System configuration register 1
#define SYS_CFG2                            0x04          // System configuration register 2
#define SYS_DIAG                            0x05          // System diagnostic register
/*
    The ADC configuration registers control the settings for the analog-to-digital converter (ADC),
    which is responsible for measuring voltages and currents within the system.
 */
#define ADC_CFG                             0x06          // ADC configuration register
#define ADC2_OFFSET_COMP                    0x07          // ADC2 offset compensation register



/*====================================================================================================*/
/*============================== Voltage and Fault Monitoring Registers ==============================*/
/*====================================================================================================*/
/*
    These registers manage fault detection related to voltage levels,
    such as overvoltage (OV) and under voltage (UV) conditions.
 */
#define OV_UV_EN_ADDR                       0x08          // Overvoltage/under voltage enable register
#define CELL_OV_FLT                         0x09          // Overvoltage fault register
#define CELL_UV_FLT                         0x0A          // under voltage fault register

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
#define GPIO_CFG2                           0x1E          // GPIO configuration register 2
#define GPIO_STS                            0x1F          // GPIO status register

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
#define MEAS_ISENSE1                        0x30          // ISENSE measurement
#define MEAS_ISENSE2                        0x31          // ISENSE measurement

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
#define MEAS_AN6                            0x41          // AN6 voltage measurement
#define MEAS_AN5                            0x42          // AN5 voltage measurement
#define MEAS_AN4                            0x43          // AN4 voltage measurement
#define MEAS_AN3                            0x44          // AN3 voltage measurement
#define MEAS_AN2                            0x45          // AN2 voltage measurement
#define MEAS_AN1                            0x46          // AN1 voltage measurement
#define MEAS_ANO                            0x47          // ANO voltage measurement
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
#define TH_ALL_CT                           0x4B          // CTx  over and under voltage threshold
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
#define TH_CT1                              0x59          // CT1  over and under voltage threshold
/*
    These register is TH_ANx_OT overtemperature threshold Registers
 */
#define TH_AN6_OT                           0x5A          // AN6 overtemperature threshold
#define TH_AN5_OT                           0x5B          // AN5 overtemperature threshold
#define TH_AN4_OT                           0x5C          // AN4 overtemperature threshold
#define TH_AN3_OT                           0x5D          // AN3 overtemperature threshold
#define TH_AN2_OT                           0x5E          // AN2 overtemperature threshold
#define TH_AN1_OT                           0x5F          // AN1 overtemperature threshold
#define TH_ANO_OT                           0x60          // ANO overtemperature threshold
/*
   These register is ANX undertemperature threshold  Registers
 */
#define TH_AN6_UT                           0x61          // AN6 undertemperature threshold
#define TH_AN5_UT                           0x62          // AN5 undertemperature threshold
#define TH_AN4_UT                           0x63          // AN4 undertemperature threshold
#define TH_AN3_UT                           0x64          // AN3 undertemperature threshold
#define TH_AN2_UT                           0x65          // AN2 undertemperature threshold
#define TH_AN1_UT                           0x66          // AN1 undertemperature threshold
#define TH_ANO_UT                           0x67          // ANO undertemperature threshold
/*
 This register ISENSE overcurrent threshold Register
 */
#define TH_ISENSE_OC                        0x68          // ISENSE overcurrent threshold
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

/**
 * @brief Initializes the SPI transfer module.
 */
void SlaveIF_TransferInit(void);

/* ============================= System Configuration APIs ============================= */
/**
 * @brief Enables or disables initialization of the MC33771B.
 * @param channel_switch A boolean value (1 or 0) to enable or disable MC33771B initialization.
 */
void SlaveIF_SystemSetup(bool channel_switch);

/**
 * @brief Enables or disables sleep mode to save power.
 * @param Sleep_Mode A boolean value (1 or 0) to enable or disable sleep mode in MC33771B.
 */
void SlaveIF_Go2SleepMode(bool Sleep_Mode);

/**
 * @brief Configures System Configuration Register 1.
 */
void SlaveIF_SystemConfig1(void);

/**
 * @brief Configures System Configuration Register 2.
 */
void SlaveIF_SystemConfig2(void);

/* ============================= ADC Configuration APIs ============================= */
/**
 * @brief Configures ADC settings.
 */
void SlaveIF_CfgADC(void);

/**
 * @brief Sets ADC2 offset compensation.
 */
void SlaveIF_CfgADCOffset(void);

/* ============================= Voltage Fault Monitoring APIs ============================= */
/**
 * @brief Enables or disables overvoltage/under voltage detection.
 */
void SlaveIF_EnableOVUV(void);

/**
 * @brief Reads the CELL_OV_FLT register and returns the overvoltage status of each cell.
 * @return uint16_t A bitfield where each bit represents the overvoltage status of a cell.
 *         Bit 0: CT1_OV_FLT, Bit 1: CT2_OV_FLT, ..., Bit 13: CT14_OV_FLT
 *         Bit 14 and Bit 15 are reserved and will always be 0.
 */
uint16_t SlaveIF_ReadCellOverVoltageStatus(void);

/**
 * @brief Reads the CELL_UV_FLT register and returns the under voltage status of each cell.
 * @return uint16_t A bitfield where each bit represents the under voltage status of a cell.
 *         Bit 0: CT1_UV_FLT, Bit 1: CT2_UV_FLT, ..., Bit 13: CT14_UV_FLT
 *         Bit 14 and Bit 15 are reserved and will always be 0.
 */
uint16_t SlaveIF_ReadCellUnderVoltageStatus(void);

/* ============================= Cell Balancing APIs ============================= */
/**
 * @brief Configures the cell balancing for a specific cell.
 * @param cellNumber The cell number to configure (1 to 14).
 * @param enable     Enable or disable cell balancing (true or false).
 * @param Timer_Value_In_Minutes The cell balance timer value in minutes (e.g., 1.5 for one and a half minutes).
 */
void SlaveIF_EnableCellBalancing(uint8_t cellNumber, bool enable, float Timer_Value_In_Minutes);

/**
 * @brief Reads the CB_DRV_STS register and prints the status of each cell balance driver.
 */
void SlaveIF_ReadCellBalancingDriverStatus(void);

/**
 * @brief Reads the CB_SHORT_FLT register and returns the shorted load status for cell balancing circuits.
 * @return uint16_t A bitfield representing the shorted load status of each cell.
 */
uint16_t SlaveIF_ReadCellBalancingShortedStatus(void);

/**
 * @brief Reads the CB_OPEN_FLT register and returns the open load status for cell balancing circuits.
 * @return uint16_t A bitfield representing the open load status of each cell.
 */
uint16_t SlaveIF_ReadCellBalancingOpenLoadStatus(void);

/* ============================= GPIO Configuration APIs ============================= */
/**
 * @brief Configures all GPIO pins as analog inputs for ratiometric temperature measurement.
 */
void SlaveIF_CfgAllGpiosForTempSensors(void);

/* ============================= Temp APIs ============================= */
/**
 * @brief Reads the AN_OT_UT_FLT register and returns both overtemperature and undertemperature flags.
 * @return Temperature_Flags A structure containing the overtemperature and undertemperature flags.
 */
Temperature_Flags SlaveIF_ReadOtUttatus(void);

/* ============================= GPIO flag APIs ============================= */
/**
 * @brief Reads the GPIO_SHORT_ANx_OPEN_STS register and returns the GPIO and AN open load flags.
 * @return GPIO_AN_Flags A structure containing the GPIO short circuit and AN open load flags.
 */
GPIO_AN_Flags SlaveIF_ReadGpioAnStatus(void);

/* ============================= Coulomb Counter & Threshold Configuration ============================= */
/**
 * @brief Reads the CC_NB_SAMPLES register and returns the number of samples.
 * @return uint16_t The number of samples accumulated for the coulomb count value.
 */
uint16_t SlaveIF_ReadNumberCoulombSamples(void);

/**
 * @brief Checks if the number of coulomb counter samples is sufficient for a valid reading.
 * @return true if number is sufficient and false otherwise.
 */
bool SlaveIF_IsCoulombSamplesSufficient(void);

/* ============================= Voltage, Current & Temperature Measurement APIs ============================= */
/**
 * @brief Reads the Cell_Voltage value from MEAS_CELLx register.
 * @param cellNumber The cell number to read (1 to 14).
 * @return float The Cell_Voltage value from the specified cell, or 0 if data is invalid.
 */
float SlaveIF_ReadCellVoltage(uint8_t cellNumber);

/**
 * @brief Reads the stack voltage measurement.
 * @return float The stack voltage in Volts (adjust scaling if needed). Returns -1.0 if data is not ready.
 */
float SlaveIF_ReadStackVoltage(void);



#endif
