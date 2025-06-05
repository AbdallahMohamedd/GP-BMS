#ifndef PWM_H_
#define PWM_H_

#include <MKL25Z4.h>
#include <stdint.h>

void PWM_Fan1_Init(void); // PTC2 → TPM0_CH1
void PWM_Fan2_Init(void); // PTC1 → TPM0_CH0

void PWM_Fan1_SetDuty(uint8_t duty);
void PWM_Fan2_SetDuty(uint8_t duty);

#endif // PWM_H_
