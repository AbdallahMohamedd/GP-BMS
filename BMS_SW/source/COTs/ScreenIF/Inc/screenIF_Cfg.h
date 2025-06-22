/**
 * @file       ScreenIF_Cfg.h
 * @brief      Configuration header for the Screen Interface (ScreenIF) driver.
 *
 * @details    This header file contains configuration definitions and function
 *             prototypes for the I2C-based ScreenIF driver, including hardware
 *             mappings and initialization routines.
 *
 * @note       Project: Graduation Project - Battery Management System
 * @note       Engineer: Amr Ahmed
 * @note       Component: Screen Interface driver
 */

#ifndef SCREENIF_CFG_H_
#define SCREENIF_CFG_H_

//=============================================================================
// Includes
//=============================================================================
#include "fsl_i2c_cmsis.h"
#include "fsl_i2c.h"
#include "Driver_I2C.h"
#include "fsl_i2c_dma.h"

//=============================================================================
// Command Definitions
//=============================================================================
#define I2C_MASTER           Driver_I2C0
#define I2C_DMAMUX_BASEADDR  DMAMUX0
#define I2C_DMA_BASEADDR     DMA0
#define LCD_ADDR             0x27

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief      Retrieves the frequency of the I2C0 clock source.
 * @details    Returns the clock frequency for the I2C0 peripheral, used for
 *             timing I2C operations.
 * @return     Clock frequency in Hz.
 */
uint32_t I2C0_GetFreq(void);

/**
 * @brief      I2C master signal event handler.
 * @details    Handles events from the I2C master, setting the completion flag
 *             when a transfer is done.
 * @param      event I2C event code (e.g., ARM_I2C_EVENT_TRANSFER_DONE).
 */
void I2C_MasterSignalEvent_t(uint32_t event);

/**
 * @brief      Initializes the I2C interface with DMA support.
 * @details    Configures the DMAMUX, DMA, and I2C peripherals for master mode
 *             operation with standard bus speed.
 */
void I2C_init(void);

/**
 * @brief      Initializes the I2C0 pins.
 * @details    Configures the GPIO pins used by the I2C0 peripheral.
 */
void I2C0_InitPins(void);

/**
 * @brief      Deinitializes the I2C0 pins.
 * @details    Resets the GPIO pins used by the I2C0 peripheral to their default state.
 */
void I2C0_DeinitPins(void);


#endif /* SCREENIF_CFG_H_ */
