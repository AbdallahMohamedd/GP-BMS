/**
 * @file       ScreenIF.h
 * @brief      Public interface header for the Screen Interface (ScreenIF) driver.
 *
 * @details    This header defines the public APIs and constants for the ScreenIF
 *             module, which controls an LCD via I2C. It includes command codes,
 *             mode settings, and function prototypes for LCD operations.
 *
 * @note       Project: Graduation Project - Battery Management System
 * @note       Engineer: Amr Ahmed
 * @note       Component: Screen Interface driver
 */

#ifndef SCREENIF_H_
#define SCREENIF_H_

//=============================================================================
// Includes
//=============================================================================
#include <COTs/ScreenIF/Inc/screenIF_Cfg.h>
#include "COTs/DebugInfoManager/Inc/debugInfo.h"

//=============================================================================
// Command Definitions
//=============================================================================
#define LCD_CLEARDISPLAY           0x01U
#define LCD_RETURNHOME             0x02U
#define LCD_ENTRYMODESET           0x04U
#define LCD_DISPLAYCONTROL         0x08U
#define LCD_CURSORSHIFT            0x10U
#define LCD_FUNCTIONSET            0x20U
#define LCD_SETCGRAMADDR           0x40U
#define LCD_SETDDRAMADDR           0x80U

//=============================================================================
// Entry Mode Definitions
//=============================================================================
#define LCD_ENTRYRIGHT             0x00U
#define LCD_ENTRYLEFT              0x02U
#define LCD_ENTRYSHIFTINCREMENT    0x01U
#define LCD_ENTRYSHIFTDECREMENT    0x00U

//=============================================================================
// Display On/Off Definitions
//=============================================================================
#define LCD_DISPLAYON              0x04U
#define LCD_DISPLAYOFF             0x00U
#define LCD_CURSORON               0x02U
#define LCD_CURSOROFF              0x00U
#define LCD_BLINKON                0x01U
#define LCD_BLINKOFF               0x00U

//=============================================================================
// Cursor Shift Definitions
//=============================================================================
#define LCD_DISPLAYMOVE            0x08U
#define LCD_CURSORMOVE             0x00U
#define LCD_MOVERIGHT              0x04U
#define LCD_MOVELEFT               0x00U

//=============================================================================
// Function Set Definitions
//=============================================================================
#define LCD_8BITMODE               0x10U
#define LCD_4BITMODE               0x00U
#define LCD_2LINE                  0x08U
#define LCD_1LINE                  0x00U
#define LCD_5x10DOTS               0x04U
#define LCD_5x8DOTS                0x00U

//=============================================================================
// Backlight Definitions
//=============================================================================
#define LCD_BACKLIGHT              0x08U
#define LCD_NOBACKLIGHT            0x00U

//=============================================================================
// Control Bit Definitions
//=============================================================================
#define ENABLE                     0x04U
#define RW                         0x00U
#define RS                         0x01U

//=============================================================================
// Public Function Prototypes
//=============================================================================
/**
 * @brief      Initializes the LCD with the specified number of rows.
 * @details    Configures the LCD for 4-bit mode and sets up display parameters
 *             based on the number of rows.
 */
void ScreenIF_Init();

/**
 * @brief      Clears the LCD display.
 * @details    Sends the clear display command and waits for completion.
 */
void ScreenIF_Clear();

/**
 * @brief      Returns the cursor to the home position.
 * @details    Moves the cursor to the top-left position of the LCD.
 */
void ScreenIF_Home();

/**
 * @brief      Creates a custom character at the specified location.
 * @details    Defines a custom 5x8 character pattern in the CGRAM.
 * @param      location Memory location (0-7) for the custom character.
 * @param      charmap Array of 8 bytes defining the character pattern.
 */
void ScreenIF_CreateSpecialChar(uint8_t location, uint8_t charmap[]);

/**
 * @brief      Sets the cursor position on the LCD.
 * @details    Moves the cursor to the specified column and row.
 * @param      col Column number (0-based).
 * @param      row Row number (0-based, up to dpRows-1).
 */
void ScreenIF_SetCursor(uint8_t col, uint8_t row);

/**
 * @brief      Prints a string on the LCD.
 * @details    Displays the input string starting at the current cursor position.
 * @param      c Pointer to the null-terminated string to print.
 */
void ScreenIF_PrintStr(const char c[]);

#endif /* SCREENIF_H_ */
