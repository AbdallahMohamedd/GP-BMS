#include <source/COTs/TempControlManager/Inc/pwm.h>


#define TPM_CLK_FREQ 48000000U
#define PWM_FREQ_HZ 1000U

static void CommonTPM0_Setup(void) {
    // Enable clock to TPM0 and PORTC once only
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Set TPM clock source to PLLFLL (48 MHz)
    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // Use PLLFLLCLK

    // Stop TPM0 for config
    TPM0->SC = 0;
    TPM0->MOD = (TPM_CLK_FREQ / PWM_FREQ_HZ) - 1; // 47999 for 1kHz
    TPM0->SC |= TPM_SC_PS(0); // Prescaler = 1
}

// ================= FAN 1: PTC2 → TPM0_CH1 =================
void PWM_Fan1_Init(void) {
    CommonTPM0_Setup();

    // Set PTC2 to ALT4 (TPM0_CH1)
    PORTC->PCR[2] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[2] |= PORT_PCR_MUX(4);

    TPM0->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // High-true PWM
    TPM0->CONTROLS[1].CnV = 0; // Start with 0% duty

    TPM0->SC |= TPM_SC_CMOD(1); // Start counter
}

void PWM_Fan1_SetDuty(uint8_t duty) {
    if (duty > 100) duty = 100;
    TPM0->CONTROLS[1].CnV = ((TPM0->MOD + 1) * duty) / 100;
}

// ================= FAN 2: PTC1 → TPM0_CH0 =================
void PWM_Fan2_Init(void) {
    CommonTPM0_Setup();

    // Set PTC1 to ALT4 (TPM0_CH0)
    PORTC->PCR[1] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[1] |= PORT_PCR_MUX(4);

    TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // High-true PWM
    TPM0->CONTROLS[0].CnV = 0; // Start with 0% duty

    TPM0->SC |= TPM_SC_CMOD(1); // Start counter
}

void PWM_Fan2_SetDuty(uint8_t duty) {
    if (duty > 100) duty = 100;
    TPM0->CONTROLS[0].CnV = ((TPM0->MOD + 1) * duty) / 100;
}
