/**
 * @file       FanControlManager.c
 * @brief      Implementation of the Fan Control Manager driver for PWM-based fan control.
 *
 * @details    This file provides the implementation for controlling two fans using
 *             PWM signals via TPM0 (Timer/PWM Module 0) on the MKL25Z4 microcontroller.
 *             It includes initialization and duty cycle adjustment functions for
 *             Fan 1 (PTC2/TPM0_CH1) and Fan 2 (PTC1/TPM0_CH0).
 *
 * @note       Project: Graduation Project - Battery Management System
 * @note       Engineer: Abdelrahman Mohamed
 * @note       Component: Fan Control Manager driver
 */

//=============================================================================
// Includes
//=============================================================================
#include <COTs/FanControlManager/Inc/fanCtrl.h>

//=============================================================================
// Definitions
//=============================================================================
#define TPM_CLK_FREQ    48000000U
#define PWM_FREQ_HZ     1000U

//=============================================================================
// Static Function Definitions
//=============================================================================
/**
 * @brief      Sets up the common TPM0 configuration.
 * @details    Enables the clock for TPM0 and PORTC, configures the TPM clock
 *             source to PLLFLL (48 MHz), stops the TPM0 counter, sets the
 *             modulus for 1 kHz PWM, and applies a prescaler of 1.
 */
static void fanCtrl_commonTpm0Setup(void)
{
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

#ifdef FANCTRL_DEBUG_INIT
    PRINTF("FanCtrl: TPM0 setup completed with MOD: %lu\r\r\n", TPM0->MOD);
#endif
}

//=============================================================================
// Public Function Definitions
//=============================================================================
/**
 * @brief      Initializes Fan 1 (PTC2/TPM0_CH1) for PWM control.
 * @details    Configures PTC2 as TPM0_CH1, sets up high-true PWM mode,
 *             starts with 0% duty cycle, and enables the TPM0 counter.
 */
void fanCtrl_fan1Init(void)
{
    fanCtrl_commonTpm0Setup();

    // Set PTC2 to ALT4 (TPM0_CH1)
    PORTC->PCR[2] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[2] |= PORT_PCR_MUX(4);

    TPM0->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // High-true PWM
    TPM0->CONTROLS[1].CnV = 0; // Start with 0% duty

    TPM0->SC |= TPM_SC_CMOD(1); // Start counter

#ifdef FANCTRL_DEBUG_INIT
    PRINTF("FanCtrl: Fan 1 (PTC2) initialized\r\r\n");
#endif
}

/**
 * @brief      Sets the duty cycle for Fan 1 (PTC2/TPM0_CH1).
 * @details    Adjusts the PWM duty cycle to the specified percentage (0-100),
 *             capping at 100% if the input exceeds it.
 * @param      duty Percentage duty cycle (0-100).
 */
void fanCtrl_fan1SetDuty(uint8_t duty)
{
    if (duty > 100) duty = 100;
    TPM0->CONTROLS[1].CnV = ((TPM0->MOD + 1) * duty) / 100;

#ifdef FANCTRL_DEBUG_PWM
    PRINTF("FanCtrl: Fan 1 duty set to %d%%\r\r\n", duty);
#endif
}

/**
 * @brief      Initializes Fan 2 (PTC1/TPM0_CH0) for PWM control.
 * @details    Configures PTC1 as TPM0_CH0, sets up high-true PWM mode,
 *             starts with 0% duty cycle, and enables the TPM0 counter.
 */
void fanCtrl_fan2Init(void)
{
    fanCtrl_commonTpm0Setup();

    // Set PTC1 to ALT4 (TPM0_CH0)
    PORTC->PCR[1] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[1] |= PORT_PCR_MUX(4);

    TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // High-true PWM
    TPM0->CONTROLS[0].CnV = 0; // Start with 0% duty

    TPM0->SC |= TPM_SC_CMOD(1); // Start counter

#ifdef FANCTRL_DEBUG_INIT
    PRINTF("FanCtrl: Fan 2 (PTC1) initialized\r\r\n");
#endif
}

/**
 * @brief      Sets the duty cycle for Fan 2 (PTC1/TPM0_CH0).
 * @details    Adjusts the PWM duty cycle to the specified percentage (0-100),
 *             capping at 100% if the input exceeds it.
 * @param      duty Percentage duty cycle (0-100).
 */
void fanCtrl_fan2SetDuty(uint8_t duty)
{
    if (duty > 100) duty = 100;
    TPM0->CONTROLS[0].CnV = ((TPM0->MOD + 1) * duty) / 100;

#ifdef FANCTRL_DEBUG_PWM
    PRINTF("FanCtrl: Fan 2 duty set to %d%%\r\r\n", duty);
#endif
}
//=============================================================================
// End of File
//=============================================================================
