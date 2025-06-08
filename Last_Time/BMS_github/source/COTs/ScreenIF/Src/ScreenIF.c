/**
 * @file       ScreenIF.c
 * @brief      Implementation of the Screen Interface (ScreenIF) driver for LCD control.
 *
 * @details    This file provides the implementation for interfacing with an LCD
 *             using I2C communication, including initialization, command sending,
 *             character display, and cursor management. It supports a 4-bit mode
 *             LCD with backlight control and custom character creation.
 *
 * @note       Project: Graduation Project - Battery Management System
 * @note       Engineer: Amr Ahmed
 * @note       Component: Screen Interface driver
 */

//=============================================================================
// Includes
//=============================================================================
#include "COTS/ScreenIF/Inc/ScreenIF.h"

//=============================================================================
// Static Function Prototypes
//=============================================================================
static void ScreenIF_SendCommand   (uint8_t);
static void ScreenIF_SendChar      (uint8_t);
static void ScreenIF_Send          (uint8_t, uint8_t);
static void ScreenIF_Write4Bits    (uint8_t);
static void ScreenIF_ExpanderWrite (uint8_t);
static void ScreenIF_PulseEnable   (uint8_t);

//=============================================================================
// Global Variables
//=============================================================================
volatile bool g_MasterCompletionFlag;

uint8_t dpFunction;
uint8_t dpControl;
uint8_t dpMode;
uint8_t dpRows;
uint8_t dpBacklight;

uint8_t special1[8] = {
    0b00000,
    0b11001,
    0b11011,
    0b00110,
    0b01100,
    0b11011,
    0b10011,
    0b00000
};

uint8_t special2[8] = {
    0b11000,
    0b11000,
    0b00110,
    0b01001,
    0b01000,
    0b01001,
    0b00110,
    0b00000
};

//=============================================================================
// Static Function Definitions
//=============================================================================
/**
 * @brief      Writes data to the I2C expander.
 * @details    Sends a byte to the I2C expander, combining it with the backlight
 *             control bit.
 * @param      data Byte to write to the expander.
 */
static void ScreenIF_ExpanderWrite(uint8_t data)
{
    uint8_t output = data | dpBacklight; // Keep backlight ON
    g_MasterCompletionFlag = false;
    I2C_MASTER.MasterTransmit(LCD_ADDR, &output, 1, false);

    //while (!g_MasterCompletionFlag) {} // Wait for I2C transfer

#ifdef SCREENIF_DEBUG_I2C
    PRINTF("ScreenIF: Expander write data: 0x%02x\n", output);
#endif
}

/**
 * @brief      Generates an enable pulse for the LCD.
 * @details    Sends an enable pulse by setting and clearing the ENABLE bit
 *             with a delay between transitions.
 * @param      data Data byte to pulse with the enable signal.
 */
static void ScreenIF_PulseEnable(uint8_t data)
{
    ScreenIF_ExpanderWrite(data | ENABLE | dpBacklight);
    Delay(3 * 20);
    ScreenIF_ExpanderWrite((data & ~ENABLE) | dpBacklight);
    Delay(3 * 20);

#ifdef SCREENIF_DEBUG_CMD
    PRINTF("ScreenIF: Enable pulse sent for data: 0x%02x\n", data);
#endif
}

/**
 * @brief      Sends a command to the LCD.
 * @details    Transmits a command byte to the LCD by splitting it into high
 *             and low nibbles and pulsing the enable signal.
 * @param      cmd Command byte to send to the LCD.
 */
static void ScreenIF_SendCommand(uint8_t cmd)
{
    uint8_t high_nibble = (cmd & 0xF0) | dpBacklight; // Keep backlight ON
    uint8_t low_nibble  = ((cmd << 4) & 0xF0) | dpBacklight; // Keep backlight ON

    ScreenIF_ExpanderWrite(high_nibble);
    ScreenIF_PulseEnable(high_nibble);

    ScreenIF_ExpanderWrite(low_nibble);
    ScreenIF_PulseEnable(low_nibble);

#ifdef SCREENIF_DEBUG_CMD
    PRINTF("ScreenIF: Command sent: 0x%02x\n", cmd);
#endif
}

/**
 * @brief      Sends a character or data to the LCD.
 * @details    Transmits a data byte to the LCD, splitting it into high and
 *             low nibbles with the specified mode (e.g., RS for data).
 * @param      value Data byte to send.
 * @param      mode Mode bit (e.g., RS for data mode).
 */
static void ScreenIF_Send(uint8_t value, uint8_t mode)
{
    uint8_t highnib = value & 0xF0;
    uint8_t lownib  = (value << 4) & 0xF0;
    ScreenIF_Write4Bits((highnib) | mode);
    ScreenIF_Write4Bits((lownib) | mode);
}

/**
 * @brief      Writes 4 bits to the LCD.
 * @details    Sends a 4-bit value to the LCD via the I2C expander and
 *             generates an enable pulse.
 * @param      value 4-bit value to write.
 */
static void ScreenIF_Write4Bits(uint8_t value)
{
    ScreenIF_ExpanderWrite(value);
    ScreenIF_PulseEnable(value);
}

/**
 * @brief      Sends a character to the LCD.
 * @details    Transmits a single character to the LCD using the data mode
 *             (RS bit set).
 * @param      ch Character byte to send.
 */
static void ScreenIF_SendChar(uint8_t ch)
{
    ScreenIF_Send(ch, RS);

#ifdef SCREENIF_DEBUG_DATA
    PRINTF("ScreenIF: Character sent: %c (0x%02x)\n", ch, ch);
#endif
}

/**
 * @brief      Displays the LCD screen.
 * @details    Turns on the display by setting the LCD_DISPLAYON bit and
 *             sending the display control command.
 */
static void ScreenIF_Display()
{
    dpControl |= LCD_DISPLAYON;
    ScreenIF_SendCommand(LCD_DISPLAYCONTROL | dpControl);

#ifdef SCREENIF_DEBUG_CMD
    PRINTF("ScreenIF: Display turned on\n");
#endif
}
//=============================================================================
// Public Function Definitions
//=============================================================================
/**
 * @brief      Retrieves the frequency of the I2C0 clock source.
 * @details    Returns the clock frequency for the I2C0 peripheral, used for
 *             timing I2C operations.
 * @return     Clock frequency in Hz.
 */
uint32_t I2C0_GetFreq(void)
{
    return CLOCK_GetFreq(I2C0_CLK_SRC);
}

/**
 * @brief      I2C master signal event handler.
 * @details    Handles events from the I2C master, setting the completion flag
 *             when a transfer is done.
 * @param      event I2C event code (e.g., ARM_I2C_EVENT_TRANSFER_DONE).
 */
void I2C_MasterSignalEvent_t(uint32_t event)
{
    /* Transfer done */
    if (event == ARM_I2C_EVENT_TRANSFER_DONE)
    {
        g_MasterCompletionFlag = true;
#ifdef SCREENIF_DEBUG_I2C
        PRINTF("ScreenIF: I2C transfer completed\n");
#endif
    }
}

/**
 * @brief      Initializes the I2C interface with DMA support.
 * @details    Configures the DMAMUX, DMA, and I2C peripherals for master mode
 *             operation with standard bus speed.
 */
void I2C_init(void)
{
    DMAMUX_Init(I2C_DMAMUX_BASEADDR);
    DMA_Init(I2C_DMA_BASEADDR);

    /* Enable DMA0 IRQ and set the priority */
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_SetPriority(DMA0_IRQn, 1);

    /* Init I2C */
    I2C_MASTER.Initialize(I2C_MasterSignalEvent_t);
    I2C_MASTER.PowerControl(ARM_POWER_FULL);
    I2C_MASTER.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);

#ifdef SCREENIF_DEBUG_INIT
    PRINTF("ScreenIF: I2C initialization completed\n");
#endif
}

/**
 * @brief      Initializes the LCD with the specified number of rows.
 * @details    Configures the LCD for 4-bit mode and sets up display parameters
 *             based on the number of rows.
 * @param      rows Number of rows on the LCD (1 or 2).
 */
void ScreenIF_Init()
{
	I2C_init();
	uint8_t rows = 4;
    dpRows      = rows;
    dpBacklight = LCD_BACKLIGHT;
    dpFunction  = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

    if (dpRows > 1)
    {
        dpFunction |= LCD_2LINE;
    }
    else
    {
        dpFunction |= LCD_5x10DOTS;
    }

    /* Wait for initialization */
    Delay(3 * 500);

    ScreenIF_ExpanderWrite(dpBacklight);
    Delay(3 * 1000);

    /* 4-bit Mode Initialization */
    ScreenIF_Write4Bits(0x03U << 4);
    Delay(3 * 4500);
    ScreenIF_Write4Bits(0x03U << 4);
    Delay(3 * 4500);
    ScreenIF_Write4Bits(0x03U << 4);
    Delay(3 * 4500);
    ScreenIF_Write4Bits(0x02U << 4);
    Delay(3 * 100);

    /* Set Display Function */
    ScreenIF_SendCommand(LCD_FUNCTIONSET | dpFunction);
    dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    ScreenIF_Display();
    ScreenIF_Clear();

    /* Set Entry Mode */
    dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    ScreenIF_SendCommand(LCD_ENTRYMODESET | dpMode);
    Delay(3 * 4500);

    ScreenIF_Home();
    ScreenIF_Clear();
#ifdef SCREENIF_DEBUG_INIT
    PRINTF("ScreenIF: LCD initialization completed for %d rows\n", rows);
#endif
}

/**
 * @brief      Clears the LCD display.
 * @details    Sends the clear display command and waits for completion.
 */
void ScreenIF_Clear()
{
    ScreenIF_SendCommand(LCD_CLEARDISPLAY);
    Delay(3 * 2000);

#ifdef SCREENIF_DEBUG_CMD
    PRINTF("ScreenIF: LCD cleared\n");
#endif
}

/**
 * @brief      Returns the cursor to the home position.
 * @details    Moves the cursor to the top-left position of the LCD.
 */
void ScreenIF_Home()
{
    ScreenIF_SendCommand(LCD_RETURNHOME);
    Delay(3 * 2000);

#ifdef SCREENIF_DEBUG_CMD
    PRINTF("ScreenIF: LCD returned to home position\n");
#endif
}

/**
 * @brief      Creates a custom character at the specified location.
 * @details    Defines a custom 5x8 character pattern in the CGRAM.
 * @param      location Memory location (0-7) for the custom character.
 * @param      charmap Array of 8 bytes defining the character pattern.
 */
void ScreenIF_CreateSpecialChar(uint8_t location, uint8_t charmap[])
{
    location &= 0x7;
    ScreenIF_SendCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++)
    {
        ScreenIF_SendChar(charmap[i]);
    }

#ifdef SCREENIF_DEBUG_DATA
    PRINTF("ScreenIF: Special character created at location: %d\n", location);
#endif
}

/**
 * @brief      Sets the cursor position on the LCD.
 * @details    Moves the cursor to the specified column and row.
 * @param      col Column number (0-based).
 * @param      row Row number (0-based, up to dpRows-1).
 */
void ScreenIF_SetCursor(uint8_t col, uint8_t row)
{
    int row_offsets[] = { 0x00U, 0x40U, 0x14U, 0x54U };
    if (row >= dpRows)
    {
        row = dpRows - 1;
    }
    ScreenIF_SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));

#ifdef SCREENIF_DEBUG_CMD
    PRINTF("ScreenIF: Cursor set to col: %d, row: %d\n", col, row);
#endif
}

/**
 * @brief      Prints a string on the LCD.
 * @details    Displays the input string starting at the current cursor position.
 * @param      c Pointer to the null-terminated string to print.
 */
void ScreenIF_PrintStr(const char c[])
{
    while (*c) ScreenIF_SendChar(*c++);

#ifdef SCREENIF_DEBUG_DATA
    PRINTF("ScreenIF: String printed to LCD\n");
#endif
}

/**
 * @brief      Initializes the I2C0 pins.
 * @details    Configures the GPIO pins used by the I2C0 peripheral.
 */
void I2C0_InitPins(void)
{
}

/**
 * @brief      Deinitializes the I2C0 pins.
 * @details    Resets the GPIO pins used by the I2C0 peripheral to their default state.
 */
void I2C0_DeinitPins(void)
{
}
//=============================================================================
// End of File
//=============================================================================
