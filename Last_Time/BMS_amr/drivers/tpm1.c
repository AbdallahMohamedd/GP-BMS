// --------------------------------------------------------------------
//  Copyright (c) 2015, NXP Semiconductors.
//  All rights reserved.
// 
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
// 
//  o Redistributions of source code must retain the above copyright notice, this list
//    of conditions and the following disclaimer.
// 
//  o Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
// 
//  o Neither the name of NXP Semiconductors nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// --------------------------------------------------------------------
#include <tpm1.h>

// ----------------------------------------------------------------------------
#define TPM1_Start()       (TPM1_SC = TPM_SC_PS(4) | TPM_SC_CMOD(1))			//!< set prescaler 1/16, mode use int clock, up counter
#define TPM1_Stop()        (TPM1_SC = TPM_SC_CMOD(0))							//!< disables timer
// ----------------------------------------------------------------------------
/*! \brief Initialises TPM1 to be used for delays
 * 
 * Sets up the TPM1 for software compare 
 * 
 */
void DelayInit(void)  {

	TPM1_C0SC = TPM_CnSC_MSA_MASK ;        										// ELSA and ELSB = 0 -> software compare (no pin action)
	TPM1_C0V = 0; 																
	TPM1_MOD = 0xFFFF;           												
}
// ----------------------------------------------------------------------------
/*! \brief Delay function for microseconds (1..21845) - 65535/3
 * 
 * This function generates a delay in microseconds using TPM1 (Timer/PWM Module 1).
 * The resolution of 1/3 microsecond is a result of a 48MHz clock with a prescaler of 16.
 * The maximum delay value is 21845 (65535/3) due to the 16-bit counter limit.
 * A timeout mechanism is added to prevent infinite loops if the CH0F flag is not set.
 * 
 * \param delay Value of delay in units of 3x microseconds (e.g., 100 for 300us).
 * 
 * \b Usage-Example:
 * \code  
 *         Delay(DELAY_100us); // Delays for 100 * 3 = 300 microseconds
 * \endcode
 */
void Delay(u16 delay) {
    // Clear the Channel 0 Flag (CH0F) and Timer Overflow Flag (TOF) to reset status
    TPM1_STATUS = TPM_STATUS_CH0F_MASK | TPM_STATUS_TOF_MASK;

    // Set the Channel 0 Value (C0V) register to the desired delay value
    // This value will be compared with the counter to trigger the CH0F flag
    TPM1_C0V = delay;

    // Reset the TPM1 counter to zero to start counting from the beginning
    TPM1_CNT = 0;

    // Start the TPM1 timer with a prescaler of 16 and internal clock source
    // TPM_SC_PS(4) sets the prescaler to divide by 16, TPM_SC_CMOD(1) enables the counter
    TPM1_Start();

    // Define a timeout counter to prevent an infinite loop
    // 0xFFFF is chosen as a sufficiently large value to allow the delay to complete
    uint32_t timeout = 0xFFFF;

    // Wait in a loop until the Channel 0 Flag (CH0F) is set or timeout occurs
    // CH0F is set when the counter (TPM1_CNT) matches the value in TPM1_C0V
    while (!(TPM1_STATUS & TPM_STATUS_CH0F_MASK) && timeout--) {
        DONOTHING(); // Placeholder to prevent compiler optimization; no operation performed
    }

    // Stop the TPM1 timer to prevent further counting
    TPM1_Stop();
}
// ----------------------------------------------------------------------------
/*! \brief Delay function for ms (1..65535)
 * 
 * @param msDelay  value of delay in ms
 */
void Delayms(u16 msDelay)  {

	for(; msDelay>0; msDelay--)
		Delay(DELAY_1000us);													
}
