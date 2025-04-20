/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Amr Ahmed
 *	Component:   LCD driver
 *	File: 		 LCD.c
 */
#include "COTS/ScreenIF/Inc/ScreenIF.h"
#include <COTS/BatteryStatusMonitor/Inc/DataMonitor.h>
#include <COTS/BatteryStatusMonitor/Inc/DataMonitor_Cfg.h>

/*................................AMR,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,*/
void floatToString(float num, char *str, int precision) {
    int32_t int_part = (int32_t)num;
    float frac_part = num - (float)int_part;
    char *ptr = str;

    // Handle negative numbers
    if (num < 0.0f) {
        *ptr++ = '-';
        int_part = -int_part;
        frac_part = -frac_part;
    }

    // Convert integer part
    if (int_part == 0) {
        *ptr++ = '0';
    } else {
        char *start = ptr;
        while (int_part > 0) {
            *ptr++ = (int_part % 10) + '0';
            int_part /= 10;
        }
        // Reverse the integer part
        char *end = ptr - 1;
        while (start < end) {
            char tmp = *start;
            *start++ = *end;
            *end-- = tmp;
        }
    }

    // Add decimal point
    *ptr++ = '.';

    // Convert fractional part with rounding
    float epsilon = 0.5f;
    for (int i = 0; i < precision; i++) {
        epsilon *= 0.1f;  // Create precision-specific epsilon
    }

    frac_part += epsilon;  // Add rounding factor

    while (precision--) {
        frac_part *= 10.0f;
        int digit = (int)frac_part;
        *ptr++ = digit + '0';
        frac_part -= digit;
    }

    // Null-terminate the string
    *ptr = '\0';
}
void DataMonitor_soc_disp(uint8_t soc){
    char Buffer[16];
    ScreenIF_SetCursor(0,0);
    ScreenIF_PrintStr("SOC:");
    sprintf(Buffer, "%d", soc);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("%");
}
void DataMonitor_soh_disp(uint8_t soh){
    char Buffer[16];
    ScreenIF_SetCursor(11,0);
    ScreenIF_PrintStr("SOH:");
    sprintf(Buffer, "%d", soh);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("%");
}
void DataMonitor_current_disp(float current){
    char Buffer[16];
    ScreenIF_SetCursor(0,1);
    ScreenIF_PrintStr("I=");
    floatToString(current,Buffer,2);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("A");
}
void DataMonitor_Fault_disp(uint8_t fault){
    if (fault==1){
    ScreenIF_Clear();
    ScreenIF_SetCursor(0,0);
    ScreenIF_PrintStr("Fault:");
    ScreenIF_SetCursor(5,1);
    ScreenIF_PrintStr("warning!!!");
    ScreenIF_SetCursor(0,2);
    ScreenIF_PrintStr("Check UART for INFO");

    }
    else if (fault==0){
    ScreenIF_SetCursor(0,2);
    ScreenIF_PrintStr("Fault Status:");
    ScreenIF_SetCursor(0,3);
    ScreenIF_PrintStr("No Fault");

    }

}
void DataMonitor_Temp_disp(float temp)
{
    char Buffer[16];
    ScreenIF_SetCursor(8,1);
    ScreenIF_PrintStr("T:");
    floatToString(temp,Buffer,2);
    ScreenIF_PrintStr(Buffer);
    ScreenIF_PrintStr("C");
}
void DataMonitor_Mode_disp(uint8_t mode){
    ScreenIF_SetCursor(9,3);
    ScreenIF_PrintStr("Mode:");
    if (mode==0){
    ScreenIF_PrintStr("Normal");
    }
    else if (mode==1){
    ScreenIF_PrintStr("Sleep");
    }
    else if (mode==2){
    ScreenIF_PrintStr("Diagnostics");
    }
}
void DataMonitor_lcd(uint8_t soc,uint8_t soh,float current,float temp,uint8_t mode,uint8_t fault){
	  ScreenIF_Init(4);
	  ScreenIF_Clear();
	  DataMonitor_soc_disp(soc);
	  DataMonitor_soh_disp(soh);
	  DataMonitor_current_disp(current);
	  DataMonitor_Temp_disp(temp);
	  DataMonitor_Mode_disp(mode);
	  DataMonitor_Fault_disp(fault);
}


