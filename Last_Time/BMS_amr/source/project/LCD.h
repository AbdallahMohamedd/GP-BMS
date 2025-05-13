/*
 * LCD.h
 *
 *  Created on: May 13, 2025
 *      Author: amrahmed
 */

#ifndef PROJECT_LCD_H_
#define PROJECT_LCD_H_
#include "fsl_i2c_cmsis.h"
#include "fsl_i2c.h"
#include "Driver_I2C.h"
#include "fsl_i2c_dma.h"
#include "drivers/tpm1.h"
#define I2C_MASTER Driver_I2C0
#define I2C_DMAMUX_BASEADDR DMAMUX0
#define I2C_DMA_BASEADDR DMA0
#define LCD_ADDR 0x27
uint32_t I2C0_GetFreq(void);
void I2C_MasterSignalEvent_t(uint32_t event);
void I2C_init(void);
void I2C0_InitPins(void);
void I2C0_DeinitPins(void);
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80
/* Entry Mode */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* Display On/Off */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/* Cursor Shift */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/* Function Set */
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Backlight */
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

/* Enable Bit */
#define ENABLE 0x04

/* Read Write Bit */
#define RW 0x0

/* Register Select Bit */
#define RS 0x01

static void ScreenIF_SendCommand(uint8_t);
static void ScreenIF_SendChar(uint8_t);
static void ScreenIF_Send(uint8_t value, uint8_t mode);
static void ScreenIF_Write4Bits(uint8_t);
static void ScreenIF_ExpanderWrite(uint8_t);
static void ScreenIF_PulseEnable(uint8_t);
//static void DelayUS(uint32_t);
void ScreenIF_Init(uint8_t rows);
void ScreenIF_Clear();
void ScreenIF_Home();
void ScreenIF_CreateSpecialChar(uint8_t, uint8_t[]);
void ScreenIF_SetCursor(uint8_t, uint8_t);
void ScreenIF_PrintStr(const char[]);

/* ============== Display SOC APIS =============== */
/**
 * @brief Receives SOC and Displays it on the LCD
 * @param SOC of the battery Pack
 */
void DataMonitor_soc_disp(uint8_t soc);
/* ============== Display SOH APIS =============== */
/**
 * @brief Receives SOH and Displays it on the LCD
 * @param SOH of the battery Pack
 */
void DataMonitor_soh_disp(uint8_t soh);
/* ============== Display Current APIS =============== */
/**
 * @brief Receives Current and Displays it on the LCD
 * @param Current of the battery Pack
 */
void DataMonitor_current_disp(float current);
/* ============== Display Fault Status APIS =============== */
/**
 * @brief Receives Fault Status and Displays it on the LCD
 * @param Fault Status of the battery Pack
 */
void DataMonitor_Fault_disp(uint8_t fault);
/* ============== Display Temp APIS =============== */
/**
 * @brief Receives Temp and Displays it on the LCD
 * @param Temp of the battery Pack
 */
void DataMonitor_Temp_disp(float temp);
/* ============== Display =============== */
/**
 * @brief Uses the functions above to display all the values required
 * @param SOC of the battery Pack
 * @param SOH of the battery Pack
 * @param Current of the battery Pack
 * @param temp of the battery pack
 * @param fault status
 */
void DataMonitor_Mode_disp(uint8_t mode);
void DataMonitor_lcd(uint8_t soc,uint8_t soh,float current,float temp,uint8_t mode,uint8_t fault);
void floatToString(float num, char *str, int precision) ;

#endif /* PROJECT_LCD_H_ */
