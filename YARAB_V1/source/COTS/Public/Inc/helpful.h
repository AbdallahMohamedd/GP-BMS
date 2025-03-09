/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdullah Mohamed
 *	Component:   bit_operator driver
 *	File: 		 helpful.h
 */

#ifndef COTS_PUBLIC_INC_HELPFUL_H_
#define COTS_PUBLIC_INC_HELPFUL_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SET_BIT(address, bit)			 address |= (1 << bit)
#define RESET_BIT(address, bit)		   	 address &= ~(1 << bit)
#define TOGGEL_BIT(address, bit)	   	 address ^= (1 << bit)
#define READ_BIT(address, bit)		     ((address & (1 << bit)) >> bit)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void delay_seconds(float seconds);


#endif /* COTS_PUBLIC_INC_HELPFUL_H_ */
