/**
 * @file SlaveIF_Cfg.h
 * @brief Configuration header for the Slave Interface (SlaveIF) driver.
 *
 * @details This file contains project-specific configurations for the SlaveIF module,
 *          including pin assignments, SPI/DMA settings, peripheral choices, timing values,
 *          and constants used within the SlaveIF driver.
 *          Review and adjust these values according to your specific hardware and
 *          application requirements.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Abdullah Mohamed
 * @note Component: Slave_Control_IF driver
 */

#ifndef COTS_SLAVECONTROLLERIF_INC_SLAVEIF_CFG_H_
#define COTS_SLAVECONTROLLERIF_INC_SLAVEIF_CFG_H_

//=============================================================================
// Includes
//=============================================================================
// Include necessary SDK headers for types used in this configuration file
#include "fsl_gpio.h"     // For gpio_pin_config_t
#include "fsl_dmamux.h"   // For DMAMUX types/functions if indirectly used
#include "fsl_dma.h"      // For DMA types/functions if indirectly used
#include "fsl_spi_dma.h"  // For SPI DMA types/functions if indirectly used
#include "fsl_port.h"     // For PORT types/macros if needed for muxing defines
#include "stdint.h"       // Standard integer types

// Include project-specific helpers if they define types/macros used *here*
#include <COTS/DebugInfoManager/Inc/DebugInfo.h>

//=============================================================================
// Transceiver Pin Configuration
//=============================================================================

/**
 * @brief GPIO configuration structure for the Transceiver EN pin (Output).
 * @details Configured as a digital output, initially set low (transceiver disabled).
 */
static const gpio_pin_config_t enPinConfig = {
		.pinDirection = kGPIO_DigitalOutput,
		.outputLogic = 0U // Default state is LOW
};

/**
 * @brief GPIO configuration structure for the Transceiver INTB pin (Input).
 * @details Configured as a digital input to read the interrupt status from the transceiver.
 */
static const gpio_pin_config_t intbPinConfig = {
		.pinDirection = kGPIO_DigitalInput,
		.outputLogic = 0U // Output logic is ignored for inputs
};

// --- Pin Definitions ---
#define EN_TRANSCEIVER_GPIO     GPIOE ///< GPIO peripheral instance for the EN pin (e.g., GPIOE).
#define EN_TRANSCEIVER_PIN      (0U)  ///< Pin number within the GPIO peripheral for the EN pin.
#define INTB_TRANSCEIVER_GPIO   GPIOA ///< GPIO peripheral instance for the INTB pin (e.g., GPIOA).
#define INTB_TRANSCEIVER_PIN    (12U) ///< Pin number within the GPIO peripheral for the INTB pin.

//=============================================================================
// SPI Pin Configuration
//=============================================================================
#define SPI_CS_PORT             PORTC ///< PORT peripheral instance for the SPI Chip Select (CS) pin.
#define SPI_CS_GPIO             GPIOC ///< GPIO peripheral instance for the SPI CS pin (used during wake-up sequence).
#define SPI_CS_PIN              (4U)  ///< Pin number within the GPIO peripheral for the SPI CS pin.
// Note: The MUX setting for the CS pin's SPI function (e.g., kPORT_MuxAlt2) is handled in SlaveIF.c

//=============================================================================
// Core Communication Configuration
//=============================================================================
#define SPI_TX                  SPI0       ///< SPI peripheral instance used for Master Transmit.
#define SPI_RX                  SPI1       ///< SPI peripheral instance used for Slave Receive.
#define SPI_BAUDRATE            (2000000U) ///< SPI communication speed in Baud/s (e.g., 2 MHz).
#define BUFFER_SIZE             (5U)       ///< Size of one SPI frame in bytes (40 bits for MC33xxx). Alias for SPI_FRAME_SIZE.
#define DEFAULT_CID             (0x0001U)  ///< Default Cluster ID used in SPI communication frames (0x00 - 0x0F).
#define SPI_TIMEOUT_US          (500000U)  ///< Timeout in microseconds for waiting SPI operations (adjust as needed).
#define SPI_RETRY_COUNT         (3U)       ///< Number of retries attempted on SPI write failure.
#define SPI_READ_ERROR_VALUE    (0x0000U)  ///< Default value returned by SlaveIF_readRegister on failure (can be customized).

//=============================================================================
// DMA Configuration
//=============================================================================
#define DMA_CHANNEL_SPI_TX      (0U) ///< DMA channel number assigned to SPI Transmit.
#define DMA_CHANNEL_SPI_RX      (1U) ///< DMA channel number assigned to SPI Receive.

//=============================================================================
// Device Specific Constants & Hardware Parameters
//=============================================================================

// --- Resolution Values (Refer to MC33771B Datasheet Table 9) ---
// Note: These are physical constants of the chip, often better placed in SlaveIF.c or SlaveIF.h if not configurable.
#define VCT_ANX_RES_V           (152.58789f) ///< Cell voltage & Analog input resolution (uV/LSB). Assumes 16-bit mode.
#define VVPWR_RES               (2441.41f)   ///< Pack voltage resolution (uV/LSB). Assumes 16-bit mode.
#define V2RES_UV_PER_LSB        (0.6f)       ///< Current sense user register resolution (uV/LSB). (Used for manual calculation if needed).

// --- Bit Definitions & Masks (Related to Measurement Registers) ---
// Note: These are fixed bit definitions, often better placed in SlaveIF.c or SlaveIF.h.
#define MEAS_DATA_READY_BIT     (15U)      ///< Bit position indicating data readiness in measurement registers.
#define ADC2_SAT_BIT            (15U)      ///< Saturation bit for ADC2 in MEAS_ISENSE2 ($31).
#define PGA_GCHANGE_BIT         (14U)      ///< PGA Gain Change bit in MEAS_ISENSE2 ($31).
#define MEAS_ISENSE2_LSB_MASK   (0x000FU)  ///< Mask for the 4 LSBs (user interpretation) in MEAS_ISENSE2 ($31).
#define MEAS_ISENSE2_MSB_MASK   (0x7FFFU)  ///< Mask for the 15 MSB bits (standard interpretation) in MEAS_ISENSE1/2.

// --- Shunt Resistor Value ---
/**
 * @brief Resistance of the shunt resistor used for current measurement.
 * @warning !! THIS VALUE MUST MATCH YOUR ACTUAL HARDWARE !!
 *          An incorrect value will lead to inaccurate current readings.
 * @note Unit: Ohms
 */
#define SHUNT_RESISTANCE_OHMS   (0.0001f) // Example: 100 micro-Ohms = 0.0001 Ohms

//=============================================================================
// CRC Lookup Table (CRC-8/MAXIM)
//=============================================================================
/**
 * @brief Lookup table for fast CRC-8/MAXIM calculation (Polynomial 0x31, Init 0xFF).
 * @details Used for ensuring data integrity in SPI communication frames with the MC33664/MC33771B.
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
		0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a, 0x15, 0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42
};

#endif /* COTS_SLAVECONTROLLERIF_INC_SLAVEIF_CFG_H_ */
//=============================================================================
// End of File
//=============================================================================
