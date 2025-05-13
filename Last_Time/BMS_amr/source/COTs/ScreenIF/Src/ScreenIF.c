/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Amr Ahmed
 *	Component:  Screen_IF driver
 *	File: 		ScreenIF.c
 */
#include "COTS/ScreenIF/Inc/ScreenIF.h"



static void ScreenIF_SendCommand(uint8_t);
static void ScreenIF_SendChar(uint8_t);
static void ScreenIF_Send(uint8_t, uint8_t);
static void ScreenIF_Write4Bits(uint8_t);
static void ScreenIF_ExpanderWrite(uint8_t);
static void ScreenIF_PulseEnable(uint8_t);


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


uint32_t I2C0_GetFreq(void)
{
    return CLOCK_GetFreq(I2C0_CLK_SRC);
}

void I2C_MasterSignalEvent_t(uint32_t event)
{
    /*  Transfer done */
    if (event == ARM_I2C_EVENT_TRANSFER_DONE)
    {
        g_MasterCompletionFlag = true;
    }
}

void I2C_init(void){
    DMAMUX_Init(I2C_DMAMUX_BASEADDR);
    DMA_Init(I2C_DMA_BASEADDR);

    /*enable DMA0 IRQ and set the priority*/
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_SetPriority(DMA0_IRQn, 1);

    /*Init I2C*/
    I2C_MASTER.Initialize(I2C_MasterSignalEvent_t);
    I2C_MASTER.PowerControl(ARM_POWER_FULL);
    I2C_MASTER.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
}

static void ScreenIF_ExpanderWrite(uint8_t data)
{
    uint8_t output = data | dpBacklight;  // Keep backlight ON
    g_MasterCompletionFlag = false;
    I2C_MASTER.MasterTransmit(LCD_ADDR, &output, 1, false);

    //while (!g_MasterCompletionFlag) {}  // Wait for I2C transfer
}
static void ScreenIF_PulseEnable(uint8_t data)
{
	ScreenIF_ExpanderWrite(data | ENABLE | dpBacklight);
    Delay(3*20);
    ScreenIF_ExpanderWrite((data & ~ENABLE) | dpBacklight);
    Delay(3*20);
}


/* Send Command to LCD */
static void ScreenIF_SendCommand(uint8_t cmd)
{
    uint8_t high_nibble = (cmd & 0xF0) | dpBacklight;  //  Keep backlight ON
    uint8_t low_nibble = ((cmd << 4) & 0xF0) | dpBacklight;  //  Keep backlight ON

    ScreenIF_ExpanderWrite(high_nibble);
    ScreenIF_PulseEnable(high_nibble);

    ScreenIF_ExpanderWrite(low_nibble);
    ScreenIF_PulseEnable(low_nibble);
}


void ScreenIF_Home() {
	 ScreenIF_SendCommand(LCD_RETURNHOME);
     Delay(3*2000);
}
/* Send Data (Characters) to LCD */
static void ScreenIF_Send(uint8_t value, uint8_t mode)
{
  uint8_t highnib = value & 0xF0;
  uint8_t lownib = (value<<4) & 0xF0;
  ScreenIF_Write4Bits((highnib)|mode);
  ScreenIF_Write4Bits((lownib)|mode);
}
static void ScreenIF_Write4Bits(uint8_t value)
{
	ScreenIF_ExpanderWrite(value);
	ScreenIF_PulseEnable(value);
}
void ScreenIF_Clear() { ScreenIF_SendCommand(LCD_CLEARDISPLAY); Delay(3*2000); }
void ScreenIF_Display() { dpControl |= LCD_DISPLAYON; ScreenIF_SendCommand(LCD_DISPLAYCONTROL | dpControl); }

/* LCD Initialization */
void ScreenIF_Init(uint8_t rows)
{
    dpRows = rows;
    dpBacklight = LCD_BACKLIGHT;
    dpFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

    if (dpRows > 1)
    {
        dpFunction |= LCD_2LINE;
    }
    else
    {
        dpFunction |= LCD_5x10DOTS;
    }

    /* Wait for initialization */

    Delay(3*500);

    ScreenIF_ExpanderWrite(dpBacklight);
    Delay(3*1000);

    /* 4-bit Mode Initialization */
    ScreenIF_Write4Bits(0x03 << 4);
    Delay(3*4500);
    ScreenIF_Write4Bits(0x03 << 4);
    Delay(3*4500);
    ScreenIF_Write4Bits(0x03 << 4);
    Delay(3*4500);
    ScreenIF_Write4Bits(0x02 << 4);
    Delay(3*100);

    /* Set Display Function */
    ScreenIF_SendCommand(LCD_FUNCTIONSET | dpFunction);
    dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    ScreenIF_Display();
    ScreenIF_Clear();

    /* Set Entry Mode */
    dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    ScreenIF_SendCommand(LCD_ENTRYMODESET | dpMode);
    Delay(3*4500);

    ScreenIF_Home();
}
void ScreenIF_SetCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if (row >= dpRows)
  {
    row = dpRows-1;
  }
  ScreenIF_SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/* Print String on LCD */
void ScreenIF_PrintStr(const char c[])
{
  while(*c) ScreenIF_SendChar(*c++);
}
void ScreenIF_CreateSpecialChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7;
  ScreenIF_SendCommand(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++)
  {
	  ScreenIF_SendChar(charmap[i]);
  }
}
static void ScreenIF_SendChar(uint8_t ch)
{
	ScreenIF_Send(ch, RS);
}
void I2C0_InitPins(void)
{
}
void I2C0_DeinitPins(void)
{
}

