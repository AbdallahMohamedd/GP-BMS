/**
 * @file SlaveIF_Cfg.h
 * @brief Configuration header for the Slave Interface (SlaveIF) driver.
 *
 * @details This file contains hardware-specific configurations for the SlaveIF module,
 *          including timing values, ADC settings, TagID lists, and configuration arrays
 *          for MC33771x/MC33772x devices. Adjust these values based on your specific
 *          hardware and application requirements.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Slave_Control_IF driver
 */

#ifndef SLAVE_IF_CFG_H
#define SLAVE_IF_CFG_H

//=============================================================================
// Includes
//=============================================================================
#include "stdint.h"                  // For standard integer types
#include "source/COTs/NTC/Inc/ntc.h" // For NTC temperature definitions

//=============================================================================
// Timing Constants
//=============================================================================
/**
 * @brief Time to wait after a reset.
 * @details Time in milliseconds required for the device to be ready after a reset.
 * @note Reference: MC33771B Datasheet, VPWR(READY) to Device Ready (5 ms).
 */
#define WAIT_AFTER_RESET (5U) //!< Time [ms] to wait after reset.

/**
 * @brief Time to wait after wakeup.
 * @details Time in milliseconds to transition from Sleep Mode to Normal Mode.
 * @note Reference: MC33771B Datasheet, tWAKE-UP (1 ms).
 */
#define WAIT_AFTER_WAKEUP (1U) //!< Time [ms] to wait after wakeup.

/**
 * @brief Settling time for RCs.
 * @details Time in microseconds to wait for RC components to stabilize.
 */
#define SETTLING_TIME (1000U) //!< Time [us] for RC settling.

//=============================================================================
// ADC and Threshold Constants
//=============================================================================
/**
 * @brief Voltage scaling factor for cell measurements.
 * @details Conversion factor for cell voltage in Volts.
 * @note Unit: Volts.
 */
#define SCALE_VCELL (152.5925E-6) //!< Voltage scaling factor [V].

/**
 * @brief Open circuit threshold value.
 * @details Threshold for detecting open circuit, calculated as 150mV / SCALE_VCELL.
 */
#define OPEN_THRESHOLD ((uint16_t)(0.100 / SCALE_VCELL)) //!< Threshold for 150mV.

/**
 * @brief Default ADC configuration.
 * @details Combines PGA gain and ADC resolution settings for reset state.
 */
#define ADC_CFG_DEFAULT (PGA_GAIN_AUTO | ADC1_A_14bit | ADC1_B_14bit | ADC2_16bit) //!< Default ADC settings.

/**
 * @brief Custom ADC configuration.
 * @details Sets PGA gain to auto and all ADCs to 16-bit resolution.
 */
#define ADC_CFG_SETTING (PGA_GAIN_AUTO | ADC1_A_16bit | ADC1_B_16bit | ADC2_16bit) //!< Custom ADC settings.

/**
 * @brief Resolution factor for thresholds.
 * @details Conversion factor for over/under voltage thresholds in Volts/LSB.
 */
#define FACTOR_THRESHOLD (256.0 / 5.0) //!< Resolution [V/LSB] for thresholds.

/**
 * @brief Macro to calculate OV/UV threshold values.
 * @details Combines overvoltage (ov) and undervoltage (uv) thresholds into a 16-bit value.
 * @param ov Overvoltage threshold in Volts.
 * @param uv Undervoltage threshold in Volts.
 * @return uint16_t Combined threshold value.
 */
#define TH_OVUV_VALUE(ov, uv) ((uint16_t)(((uint8_t)(ov * FACTOR_THRESHOLD) << 8) | ((uint8_t)(uv * FACTOR_THRESHOLD) << 0)))

//=============================================================================
// Static Variables
//=============================================================================
/**
 * @brief CRC table for polynomial 0x2F.
 */
static const uint8_t crc_table[256] = {
    0x00, 0x2f, 0x5e, 0x71, 0xbc, 0x93, 0xe2, 0xcd, 0x57, 0x78, 0x09, 0x26, 0xeb, 0xc4, 0xb5, 0x9a,
    0xae, 0x81, 0xf0, 0xdf, 0x12, 0x3d, 0x4c, 0x63, 0xf9, 0xd6, 0xa7, 0x88, 0x45, 0x6a, 0x1b, 0x34,
    0x73, 0x5c, 0x2d, 0x02, 0xcf, 0xe0, 0x91, 0xbe, 0x24, 0x0b, 0x7a, 0x55, 0x98, 0xb7, 0xc6, 0xe9,
    0xdd, 0xf2, 0x83, 0xac, 0x61, 0x4e, 0x3f, 0x10, 0x8a, 0xa5, 0xd4, 0xfb, 0x36, 0x19, 0x68, 0x47,
    0xe6, 0xc9, 0xb8, 0x97, 0x5a, 0x75, 0x04, 0x2b, 0xb1, 0x9e, 0xef, 0xc0, 0x0d, 0x22, 0x53, 0x7c,
    0x48, 0x67, 0x16, 0x39, 0xf4, 0xdb, 0xaa, 0x85, 0x1f, 0x30, 0x41, 0x6e, 0xa3, 0x8c, 0xfd, 0xd2,
    0x95, 0xba, 0xcb, 0xe4, 0x29, 0x06, 0x77, 0x58, 0xc2, 0xed, 0x9c, 0xb3, 0x7e, 0x51, 0x20, 0x0f,
    0x3b, 0x14, 0x65, 0x4a, 0x87, 0xa8, 0xd9, 0xf6, 0x6c, 0x43, 0x32, 0x1d, 0xd0, 0xff, 0x8e, 0xa1,
    0xe3, 0xcc, 0xbd, 0x92, 0x5f, 0x70, 0x01, 0x2e, 0xb4, 0x9b, 0xea, 0xc5, 0x08, 0x27, 0x56, 0x79,
    0x4d, 0x62, 0x13, 0x3c, 0xf1, 0xde, 0xaf, 0x80, 0x1a, 0x35, 0x44, 0x6b, 0xa6, 0x89, 0xf8, 0xd7,
    0x90, 0xbf, 0xce, 0xe1, 0x2c, 0x03, 0x72, 0x5d, 0xc7, 0xe8, 0x99, 0xb6, 0x7b, 0x54, 0x25, 0x0a,
    0x3e, 0x11, 0x60, 0x4f, 0x82, 0xad, 0xdc, 0xf3, 0x69, 0x46, 0x37, 0x18, 0xd5, 0xfa, 0x8b, 0xa4,
    0x05, 0x2a, 0x5b, 0x74, 0xb9, 0x96, 0xe7, 0xc8, 0x52, 0x7d, 0x0c, 0x23, 0xee, 0xc1, 0xb0, 0x9f,
    0xab, 0x84, 0xf5, 0xda, 0x17, 0x38, 0x49, 0x66, 0xfc, 0xd3, 0xa2, 0x8d, 0x40, 0x6f, 0x1e, 0x31,
    0x76, 0x59, 0x28, 0x07, 0xca, 0xe5, 0x94, 0xbb, 0x21, 0x0e, 0x7f, 0x50, 0x9d, 0xb2, 0xc3, 0xec,
    0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a, 0x15, 0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42};

#endif /* SLAVE_IF_CFG_H */
//=============================================================================
// End of File
//=============================================================================
