/**
 * @file dataMonitor.c
 * @brief Implementation of the Data Monitor driver for LCD display.
 *
 * @details This file contains the implementation for displaying battery management
 *          system parameters such as State of Charge (SOC), State of Health (SOH),
 *          current, temperature, mode, and fault status on an LCD. It interfaces
 *          with the ScreenIF module for LCD operations and provides utility functions
 *          for formatting data for display.
 *
 * @note Project: Graduation Project - Battery Management System
 * @note Engineer: Amr Ahmed
 * @note Component: Data Monitor driver
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTs/BatteryStatusMonitor/Inc/dataMonitor.h>

//=============================================================================
// Public Function Definitions
//=============================================================================
/**
 * @brief Converts a float to a string with specified precision.
 * @details Converts a floating-point number to a string representation, handling
 *          negative numbers, integer and fractional parts, and rounding to the
 *          specified precision. The result is stored in the provided buffer.
 * @param num Floating-point number to convert.
 * @param str Pointer to the output string buffer.
 * @param precision Number of decimal places to include.
 */
void floatToString(float num, char *str, int precision)
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
    printf("DataMonitor: Converted float %f to string: %s\n", num, str);
#endif
}

/**
 * @brief Displays the State of Charge (SOC) on the LCD.
 * @details Formats the SOC value as a percentage and displays it on the LCD
 *          using the ScreenIF module.
 * @param soc State of Charge of the battery pack (0 to 100).
 */
void DataMonitor_soc_disp(uint8_t soc)
{
    char Buffer[16];
    ScreenIF_PrintStr("SOC:");
    sprintf(Buffer, "%d", soc);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("%");

#ifdef DATAMONITOR_DEBUG_SOC
    printf("DataMonitor: Displayed SOC: %d%%\n", soc);
#endif
}

/**
 * @brief Displays the State of Health (SOH) on the LCD.
 * @details Formats the SOH value as a percentage and displays it on the LCD
 *          using the ScreenIF module.
 * @param soh State of Health of the battery pack (0 to 100).
 */
void DataMonitor_soh_disp(uint8_t soh)
{
    char Buffer[16];
    ScreenIF_PrintStr("SOH:");
    sprintf(Buffer, "%d", soh);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("%");

#ifdef DATAMONITOR_DEBUG_SOH
    printf("DataMonitor: Displayed SOH: %d%%\n", soh);
#endif
}

/**
 * @brief Displays the current on the LCD.
 * @details Converts the current value to a string with two decimal places
 *          and displays it on the LCD with the unit "A".
 * @param current Current of the battery pack in amperes.
 */
void DataMonitor_current_disp(float current)
{
    char Buffer[16];
    ScreenIF_PrintStr("I=");
    floatToString(current, Buffer, 2);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("A");

#ifdef DATAMONITOR_DEBUG_CURRENT
    printf("DataMonitor: Displayed current: %f A\n", current);
#endif
}

/**
 * @brief Displays the fault status on the LCD.
 * @details Displays a warning message for fault condition (FAULT_STATUS_ACTIVE) or a
 *          no-fault message (FAULT_STATUS_NONE) on the LCD using the ScreenIF module.
 * @param fault Fault status of the battery pack.
 */
void DataMonitor_Fault_disp(BMSFaultStatus_t fault)
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
        printf("DataMonitor: Displayed fault status: Warning\n");
#endif
    }
    else if (fault == FaultStatusNone)
    {
        ScreenIF_SetCursor(0, 2);
        ScreenIF_PrintStr("Fault Status:");
        ScreenIF_SetCursor(0, 3);
        ScreenIF_PrintStr("No Fault");

#ifdef DATAMONITOR_DEBUG_FAULT
        printf("DataMonitor: Displayed fault status: No Fault\n");
#endif
    }
}
/**
 * @brief Displays the temperature on the LCD.
 * @details Converts the temperature value to a string with two decimal places
 *          and displays it on the LCD with the unit "C".
 * @param temp Temperature of the battery pack in degrees Celsius.
 */
void DataMonitor_Temp_disp(float temp)
{
    char Buffer[16];
    ScreenIF_PrintStr("T:");
    floatToString(temp, Buffer, 2);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("C");

#ifdef DATAMONITOR_DEBUG_TEMP
    printf("DataMonitor: Displayed temperature: %f C\n", temp);
#endif
}

/**
 * @brief Displays the operating mode on the LCD.
 * @details Displays the operating mode (Normal, Sleep, or Diagnostics) based
 *          on the input mode value.
 * @param mode Operating mode of the battery pack
 */
void DataMonitor_Mode_disp(BMSMode_t mode)
{
    ScreenIF_SetCursor(9, 3);
    ScreenIF_PrintStr("Mode:");
    if (mode == NormalMode)
    {
        ScreenIF_PrintStr("Normal");
#ifdef DATAMONITOR_DEBUG_MODE
        printf("DataMonitor: Displayed mode: Normal\n");
#endif
    }
    else if (mode == SleepMode)
    {
        ScreenIF_PrintStr("Sleep");
#ifdef DATAMONITOR_DEBUG_MODE
        printf("DataMonitor: Displayed mode: Sleep\n");
#endif
    }
    else if (mode == DiagnosticMode)
    {
        ScreenIF_PrintStr("Diagnostics");
#ifdef DATAMONITOR_DEBUG_MODE
        printf("DataMonitor: Displayed mode: Diagnostics\n");
#endif
    }
}

/**
 * @brief Displays all battery parameters on the LCD.
 * @details Initializes the LCD with 4 lines, clears the display, and calls
 *          individual display functions to show SOC, SOH, current, temperature,
 *          mode, and fault status.
 * @param soc State of Charge of the battery pack (0 to 100).
 * @param soh State of Health of the battery pack (0 to 100).
 * @param current Current of the battery pack in amperes.
 * @param temp Temperature of the battery pack in degrees Celsius.
 * @param mode Operating mode of the battery pack (0: Normal, 1: Sleep, 2: Diagnostics).
 * @param fault Fault status of the battery pack (0 for no fault, 1 for fault).
 */
void DataMonitor_lcd(uint8_t soc, uint8_t soh, float current, float temp, BMSMode_t mode, BMSFaultStatus_t fault)
{
    ScreenIF_Init(4);
    ScreenIF_Clear();
    ScreenIF_SetCursor(0, 0);
    DataMonitor_soc_disp(soc);
    ScreenIF_SetCursor(11, 0);
    DataMonitor_soh_disp(soh);
    ScreenIF_SetCursor(0, 1);
    DataMonitor_current_disp(current);
    ScreenIF_SetCursor(8, 1);
    DataMonitor_Temp_disp(temp);
    ScreenIF_SetCursor(9, 3);
    DataMonitor_Mode_disp(mode);
    DataMonitor_Fault_disp(fault);

#ifdef DATAMONITOR_DEBUG_DATA
    PRINTF("DataMonitor: Displayed all parameters - SOC: %d%%, SOH: %d%%, Current: %.2f A, Temp: %.2f C, Mode: %d, Fault: %d\r\r\n",
               soc, soh, current, temp, (int)mode, (int)fault);
#endif
}

//=============================================================================
// End of File
//=============================================================================
