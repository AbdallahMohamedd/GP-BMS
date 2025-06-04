/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Amr Ahmed
 *	Component:   LCD driver
 *	File: 		 LCD.h
 */

#ifndef TEAM_TEAM_INC_LCD_H_
#define TEAM_TEAM_INC_LCD_H_
#include <COTs/DebugInfoManager/Inc/debugInfo.h>
#include <COTs/ScreenIF/Inc/ScreenIF.h>
#include "stdio.h"
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



#endif /* TEAM_TEAM_INC_LCD_H_ */
//=============================================================================
// End of File
//=============================================================================
