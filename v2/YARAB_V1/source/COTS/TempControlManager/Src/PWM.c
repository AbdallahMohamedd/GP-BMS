/*
 *	Graduation Project, Battery Management System
 *	Eng.: 	    Abdelrahman Mohamed
 *	Component:   PWM driver
 *	File: 		 PWM.h
 */


#include <COTS/TempControlManager/Inc/PWM.h>

#define PWM_PORT PORTA
#define PWM_PIN  13  // PTA13 (TPM1_CH1)
#define TPM_MODULE TPM1
#define TPM_CHANNEL 1

void PWM_Init(uint8_t prescaler, uint16_t period) {
    // Enable clock to PORTA and TPM1
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

    // Configure PTA13 as TPM1_CH1 (PWM output)
    PWM_PORT->PCR[PWM_PIN] = PORT_PCR_MUX(3);

    // Select MCGIRCLK (4 MHz) as TPM clock source
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(3);

    // Disable TPM1 while configuring
    TPM_MODULE->SC = 0;

    // Set Prescaler
    TPM_MODULE->SC |= TPM_SC_PS(prescaler);

    // Set MOD value (determines PWM period)
    TPM_MODULE->MOD = period - 1;

    // Configure TPM1 Channel 1 for edge-aligned PWM, high-true pulses
    TPM_MODULE->CONTROLS[TPM_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
}

void PWM_Start(uint8_t duty_cycle) {
    // Set duty cycle
    TPM_MODULE->CONTROLS[TPM_CHANNEL].CnV = (TPM_MODULE->MOD * duty_cycle) / 100;

    // Enable TPM1 in up-counting mode
    TPM_MODULE->SC |= TPM_SC_CMOD(1);
}

void PWM_SetDutyCycle(uint8_t duty_cycle) {
    // Adjust duty cycle dynamically
    TPM_MODULE->CONTROLS[TPM_CHANNEL].CnV = (TPM_MODULE->MOD * duty_cycle) / 100;
}

void PWM_Stop(void) {
    // Disable TPM1
    TPM_MODULE->SC &= ~TPM_SC_CMOD_MASK;
}
