/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:   bit_operator driver
 *	File: 		 helpful.h
 */

#include "fsl_common.h"
#include "stdint.h"
#include "fsl_clock.h"


#ifndef COTS_PUBLIC_INC_HELPFUL_H_
#define COTS_PUBLIC_INC_HELPFUL_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SET_BIT(address, bit)			 address |= (1 << bit)
#define RESET_BIT(address, bit)		   	 address &= ~(1 << bit)
#define TOGGEL_BIT(address, bit)	   	 address ^= (1 << bit)
#define READ_BIT(address, bit)		     ((address & (1 << bit)) >> bit)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void delay_seconds(float seconds);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);


#endif /* COTS_PUBLIC_INC_HELPFUL_H_ */
