/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Amr Ahmed
 *	Component:  Screen_IF driver
 *	File: 		ScreenIF.h
 */

#ifndef COTS_SCREENIF_INC_SCREENIF_H_
#define COTS_SCREENIF_INC_SCREENIF_H_


/* Command */
#include "source/drivers/tpm1.h"
#include "COTS/ScreenIF/Inc/ScreenIF_Cfg.h"
//#include <COTS/DebugInfoManager/Inc/DebugInfo.h>

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


//void DelayUS(uint32_t);
void ScreenIF_Init(uint8_t rows);
void ScreenIF_Clear();
void ScreenIF_Home();
void ScreenIF_CreateSpecialChar(uint8_t, uint8_t[]);
void ScreenIF_SetCursor(uint8_t, uint8_t);
void ScreenIF_PrintStr(const char[]);
/*................................AMR,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,*/




#endif /* COTS_SCREENIF_INC_SCREENIF_H_ */
