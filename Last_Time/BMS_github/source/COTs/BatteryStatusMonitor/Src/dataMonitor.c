/**
 * @file        dataMonitor.c
 * @brief       Implementation of the Data Monitor driver for LCD display.
 *
 * @details     This file contains the implementation for displaying battery management
 *              system parameters such as State of Charge (SOC), State of Health (SOH),
 *              current, temperature, mode, and fault status on an LCD. It interfaces
 *              with the ScreenIF module for LCD operations and provides utility functions
 *              for formatting data for display.
 *
 * @note        Project: Graduation Project - Battery Management System
 * @note        Engineer: Amr Ahmed
 * @note        Component: Data Monitor driver
 */

//=============================================================================
// Includes
//=============================================================================
#include "COTs/BatteryStatusMonitor/Inc/dataMonitor.h"

//=============================================================================
// Public Function Definitions
//=============================================================================
/**
 * @brief      Startup with Graduation project BMS
 */
void DataMonitor_startUp(){
	dataMonitor_clearScreen();
    ScreenIF_SetCursor(0,0);
    ScreenIF_PrintStr("Graduation");
    ScreenIF_SetCursor(11,0);
    ScreenIF_PrintStr("Project");
    ScreenIF_SetCursor(0,1);
    ScreenIF_PrintStr("Sponsored by:");
    ScreenIF_SetCursor(11,2);
    ScreenIF_PrintStr("Vehiclevo");
    ScreenIF_SetCursor(0,3);
    ScreenIF_PrintStr("Dr.:Mohamed Mostafa");
}

/**
 * @brief      Clears the LCD display.
 * @details    Sends the clear display command and waits for completion.
 */
void dataMonitor_clearScreen(void)
{
    ScreenIF_Clear();
}

/**
 * @brief       Converts a float to a string with specified precision.
 * @details     Converts a floating-point number to a string representation, handling
 *              negative numbers, integer and fractional parts, and rounding to the
 *              specified precision. The result is stored in the provided buffer.
 * @param       num Floating-point number to convert.
 * @param       str Pointer to the output string buffer.
 * @param       precision Number of decimal places to include.
 */
void DataMonitor_float2str(float num, char *str, int precision)
{
    int32_t int_part = (int32_t)num;
    float frac_part = num - (float)int_part;
    char *ptr = str;

    // Handle negative numbers
    if (num < 0.0f)
    {
        *ptr++ = '-';
        int_part = -int_part;
        frac_part = -frac_part;
    }

    // Convert integer part
    if (int_part == 0)
    {
        *ptr++ = '0';
    }
    else
    {
        char *start = ptr;
        while (int_part > 0)
        {
            *ptr++ = (int_part % 10) + '0';
            int_part /= 10;
        }
        // Reverse the integer part
        char *end = ptr - 1;
        while (start < end)
        {
            char tmp = *start;
            *start++ = *end;
            *end-- = tmp;
        }
    }

    // Add decimal point
    *ptr++ = '.';

    // Convert fractional part with rounding
    float epsilon = 0.5f;
    for (int i = 0; i < precision; i++)
    {
        epsilon *= 0.1f; // Create precision-specific epsilon
    }

    frac_part += epsilon; // Add rounding factor

    while (precision--)
    {
        frac_part *= 10.0f;
        int digit = (int)frac_part;
        *ptr++ = digit + '0';
        frac_part -= digit;
    }

    // Null-terminate the string
    *ptr = '\0';

#ifdef DATAMONITOR_DEBUG_CONVERSION
    PRINTF("DataMonitor: Converted float %f to string: %s\r\r\n", num, str);
#endif
}

/**
 * @brief       Displays the State of Charge (SOC) on the LCD.
 * @details     Formats the SOC value as a percentage and displays it on the LCD
 *              using the ScreenIF module.
 * @param       soc State of Charge of the battery pack (0 to 100).
 */
void dataMonitor_socDisp(float soc)
{
    char Buffer[16];
    ScreenIF_PrintStr("SOC:");
    DataMonitor_float2str(soc, Buffer, 2);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("%");
#ifdef DATAMONITOR_DEBUG_SOC
    PRINTF("DataMonitor: Displayed SOC: %d%%\r\r\n", soc);
#endif
}

/**
 * @brief       Displays the State of Health (SOH) on the LCD.
 * @details     Formats the SOH value as a percentage and displays it on the LCD
 *              using the ScreenIF module.
 * @param       soh State of Health of the battery pack (0 to 100).
 */
void dataMonitor_sohDisp(uint8_t soh)
{
    char Buffer[16];
    ScreenIF_PrintStr("SOH:");
    sprintf(Buffer, "%d", soh);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("%");

#ifdef DATAMONITOR_DEBUG_SOH
    PRINTF("DataMonitor: Displayed SOH: %d%%\r\r\n", soh);
#endif
}

/**
 * @brief       Displays the current on the LCD.
 * @details     Converts the current value to a string with two decimal places
 *              and displays it on the LCD with the unit "A".
 * @param       current Current of the battery pack in amperes.
 */
void dataMonitor_currentDisp(float current)
{
    char Buffer[16];
    ScreenIF_PrintStr("I=");
    DataMonitor_float2str(current, Buffer, 2);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("A");

#ifdef DATAMONITOR_DEBUG_CURRENT
    PRINTF("DataMonitor: Displayed current: %f A\r\r\n", current);
#endif
}

/**
 * @brief       Displays the fault status on the LCD.
 * @details     Displays a warning message for fault condition (FAULT_STATUS_ACTIVE) or a
 *              no-fault message (FAULT_STATUS_NONE) on the LCD using the ScreenIF module.
 * @param       fault Fault status of the battery pack.
 */
void dataMonitor_faultDisp(BMSFaultStatus_t fault)
{
    if (fault == FaultStatusActive)
    {
        ScreenIF_Clear();
        ScreenIF_SetCursor(0, 0);
        ScreenIF_PrintStr("Fault:");
        ScreenIF_SetCursor(5, 1);
        ScreenIF_PrintStr("warning!!!");
        ScreenIF_SetCursor(0, 2);
        ScreenIF_PrintStr("Check UART for INFO");

#ifdef DATAMONITOR_DEBUG_FAULT
        PRINTF("DataMonitor: Displayed fault status: Warning\r\r\n");
#endif
    }
    else if (fault == FaultStatusNone)
    {
        ScreenIF_SetCursor(0, 2);
        ScreenIF_PrintStr("Fault Status:");
        ScreenIF_SetCursor(0, 3);
        ScreenIF_PrintStr("No Fault");

#ifdef DATAMONITOR_DEBUG_FAULT
        PRINTF("DataMonitor: Displayed fault status: No Fault\r\r\n");
#endif
    }
}
/**
 * @brief       Displays the temperature on the LCD.
 * @details     Converts the temperature value to a string with two decimal places
 *              and displays it on the LCD with the unit "C".
 * @param       temp Temperature of the battery pack in degrees Celsius.
 */
void dataMonitor_tempDisp(float temp)
{
    char Buffer[16];
    ScreenIF_PrintStr("Temp:");
    DataMonitor_float2str(temp, Buffer, 1);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("C");

#ifdef DATAMONITOR_TEMP_DEBUG
    PRINTF("DataMonitor: Displayed temperature: %f C\r\r\n", temp);
#endif
}

/**
 * @brief       Displays the operating mode on the LCD.
 * @details     Displays the operating mode (Normal, Sleep, or Diagnostics) based
 *              on the input mode value.
 * @param       mode Operating mode of the battery pack
 */
void dataMonitor_modeDisp(BMSMode_t mode)
{
    if (mode == NormalMode)
    {
        ScreenIF_PrintStr("Normal");
#ifdef DATAMONITOR_MODE_DEBUG
        PRINTF("DataMonitor: Displayed mode: Normal\r\r\n");
#endif
    }
    else if (mode == SleepMode)
    {
        ScreenIF_PrintStr("Sleep");
#ifdef DATAMONITOR_MODE_DEBUG
        PRINTF("DataMonitor: Displayed mode: Sleep\r\r\n");
#endif
    }
    else if (mode == DiagnosticMode)
    {
        ScreenIF_PrintStr("Diagnostics");
#ifdef DATAMONITOR_MODE_DEBUG
        PRINTF("DataMonitor: Displayed mode: Diagnostics\r\r\n");
#endif
    }
}

/**
 * @brief       Displays all battery parameters on the LCD.
 * @details     Initializes the LCD with 4 lines, clears the display, and calls
 *              individual display functions to show SOC, SOH, current, temperature,
 *              mode, and fault status.
 * @param       soc State of Charge of the battery pack (0 to 100).
 * @param       soh State of Health of the battery pack (0 to 100).
 * @param       current Current of the battery pack in amperes.
 * @param       temp Temperature of the battery pack in degrees Celsius.
 * @param       mode Operating mode of the battery pack (0: Normal, 1: Sleep, 2: Diagnostics).
 * @param       fault Fault status of the battery pack (0 for no fault, 1 for fault).
 */
void dataMonitor_packInfo(float soc, uint8_t soh, float current, float temp, BMSMode_t mode, BMSFaultStatus_t fault)
{
    ScreenIF_Clear();
    ScreenIF_SetCursor(6, 0);
    ScreenIF_PrintStr("Pack Info");
    ScreenIF_SetCursor(0, 1);
    dataMonitor_socDisp(soc);
    ScreenIF_SetCursor(0, 2);
    dataMonitor_tempDisp(temp);
    ScreenIF_SetCursor(0, 3);
    dataMonitor_currentDisp(current);
    ScreenIF_SetCursor(9, 3);
    ScreenIF_PrintStr("Mode:");
    ScreenIF_SetCursor(14, 3);
    dataMonitor_modeDisp(mode);

#ifdef DATAMONITOR_DEBUG_DATA
    PRINTF("DataMonitor: Displayed all parameters - SOC: %d%%, SOH: %d%%, Current: %f A, Temp: %f C, Mode: %d, Fault: %d\r\r\n",
           soc, soh, current, temp, mode, fault);
#endif
}

/**
 * @brief      Displays the balancing status of cells on the LCD.
 * @details    Clears the LCD, prints a header, and displays the numbers of cells
 *             with balancing enabled (bits set in the input data). Cells are shown
 *             in a 2-per-row format, with a total count if any bits are set.
 * @param      data 16-bit value where each bit represents a cell's balancing status.
 * @return     None (void).
 */
void dataMonitor_balancingStatus(uint16_t data)
{
    ScreenIF_Clear();
    ScreenIF_SetCursor(2, 0);
    ScreenIF_PrintStr("Balancing status:");

    int found = 0;
    int row = 1;
    int column = 0;
    int count = 0;                // Count bits per row for LCD display
    int no_of_balanced_cells = 0; // Count of set bits

    for (int i = 15; i >= 0; i--)
    {
        if (data & (1u << i))
        {
            int bit_num = i + 1;
            char Buffer[16];
            sprintf(Buffer, "%d", bit_num);
            ScreenIF_SetCursor(column, row);
            ScreenIF_PrintStr("Cell:");
            ScreenIF_PrintStr(Buffer);
            column += 10; // Move cursor right
            count++;
            no_of_balanced_cells++;
            found = 1;

            if (count == 2)
            {
                row++;
                column = 0;
                count = 0;
            }
        }
    }

    if (!found)
    {
    	ScreenIF_SetCursor(0, 2);
    	ScreenIF_PrintStr("Balancing");
    	ScreenIF_SetCursor(8, 3);
    	ScreenIF_PrintStr("Deactivated");

#ifdef DATAMONITOR_DEBUG_STATUS
        PRINTF("DataMonitor: No cells are balancing\r\r\n");
#endif
    }
    else
    {
#ifdef DATAMONITOR_DEBUG_STATUS
        PRINTF("DataMonitor: Total balanced cells: %d\r\r\n", no_of_balanced_cells);
#endif
    }
}

/**
 * @brief      Displays the speed percentage on the LCD.
 * @details    Prints a "Speed:" label followed by the speed value and "%" suffix.
 * @param      speed Percentage speed value to display (0-100).
 * @return     None (void).
 */
static void dataMonitor_speedDisp(uint8_t speed)
{
    char Buffer[16];
    ScreenIF_PrintStr("Speed:");
    sprintf(Buffer, "%d", speed);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("%");

#ifdef DATAMONITOR_DEBUG_SPEED
    PRINTF("DataMonitor: Displayed speed: %d%%\r\r\n", speed);
#endif
}

/**
 * @brief      Displays fan status and speed on the LCD.
 * @details    Clears the LCD and shows status (ON/OFF) and speed for Fan 1 and Fan 2
 *             in a structured format across four rows.
 * @param      speed1 Speed percentage for Fan 1 (0-100).
 * @param      speed2 Speed percentage for Fan 2 (0-100).
 * @param      Fan1_mode Operational mode of Fan 1 (ON or OFF).
 * @param      Fan2_mode Operational mode of Fan 2 (ON or OFF).
 * @return     None (void).
 */
void dataMonitor_fanInfo(uint8_t speed1, uint8_t speed2, FanStatus_t Fan1_mode, FanStatus_t Fan2_mode)
{
    ScreenIF_Clear();
    ScreenIF_SetCursor(0, 0);
    ScreenIF_PrintStr("Fan 1:");
    ScreenIF_SetCursor(0, 1);
    ScreenIF_PrintStr("Mode:");
    if (Fan1_mode == ON)
    {
        ScreenIF_PrintStr("ON");
#ifdef DATAMONITOR_DEBUG_FAN
        PRINTF("DataMonitor: Fan 1 mode displayed: ON\r\r\n");
#endif
    }
    else if (Fan1_mode == OFF)
    {
        ScreenIF_PrintStr("OFF");
#ifdef DATAMONITOR_DEBUG_FAN
        PRINTF("DataMonitor: Fan 1 mode displayed: OFF\r\r\n");
#endif
    }
    ScreenIF_SetCursor(9, 1);
    dataMonitor_speedDisp(speed1);

    ScreenIF_SetCursor(0, 2);
    ScreenIF_PrintStr("Fan 2:");
    ScreenIF_SetCursor(0, 3);
    ScreenIF_PrintStr("Mode:");
    if (Fan2_mode == ON)
    {
        ScreenIF_PrintStr("ON");
#ifdef DATAMONITOR_DEBUG_FAN
        PRINTF("DataMonitor: Fan 2 mode displayed: ON\r\r\n");
#endif
    }
    else if (Fan2_mode == OFF)
    {
        ScreenIF_PrintStr("OFF");
#ifdef DATAMONITOR_DEBUG_FAN
        PRINTF("DataMonitor: Fan 2 mode displayed: OFF\r\r\n");
#endif
    }
    ScreenIF_SetCursor(9, 3);
    dataMonitor_speedDisp(speed2);
}

/**
 * @brief      Displays the pack voltage on the LCD.
 * @details    Clears the LCD, prints a "Pack Voltage:" label, and displays the
 *             voltage value with two decimal places followed by "V".
 * @param      voltage Float value representing the pack voltage.
 * @return     None (void).
 */
void dataMonitor_packvoltage(float voltage)
{
    ScreenIF_Clear();
    ScreenIF_SetCursor(0, 0);
    ScreenIF_PrintStr("Pack Voltage:");
    char Buffer[16];
    DataMonitor_float2str(voltage, Buffer, 2);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("V");

#ifdef DATAMONITOR_DEBUG_VOLTAGE
    PRINTF("DataMonitor: Displayed pack voltage: %.2f V\n", voltage);
#endif
}
//=============================================================================
// End of File
//=============================================================================
