/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Amr Ahmed
 *	Component:  Screen_IF driver
 *	File: 		ScreenIF_Cfg.h
 */

#ifndef COTS_SCREENIF_INC_SCREENIF_CFG_H_
#define COTS_SCREENIF_INC_SCREENIF_CFG_H_


#include "fsl_i2c_cmsis.h"
#include "fsl_i2c.h"
#include "Driver_I2C.h"
#include "fsl_i2c_dma.h"
#define I2C_MASTER Driver_I2C0
#define I2C_DMAMUX_BASEADDR DMAMUX0
#define I2C_DMA_BASEADDR DMA0
#define LCD_ADDR 0x27
uint32_t I2C0_GetFreq(void);
void I2C_MasterSignalEvent_t(uint32_t event);
void I2C_init(void);
void I2C0_InitPins(void);
void I2C0_DeinitPins(void);


#endif /* COTS_SCREENIF_INC_SCREENIF_CFG_H_ */
