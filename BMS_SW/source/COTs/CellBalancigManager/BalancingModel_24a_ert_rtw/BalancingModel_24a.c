/*
 * File: BalancingModel_24a.c
 *
 * Code generated for Simulink model 'BalancingModel_24a'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Tue Jul  1 17:19:48 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M0/M0+
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "BalancingModel_24a.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>

/* Named constants for Chart: '<S1>/Chart1' */
#define IN_Data_Check                  ((uint8_T)1U)
#define IN_Discharge_Time_interval     ((uint8_T)2U)
#define IN_Error_Check                 ((uint8_T)3U)
#define IN_Get_balance_reference       ((uint8_T)4U)
#define IN_Index_number                ((uint8_T)5U)
#define IN_initialization              ((uint8_T)6U)

/* Block signals and states (default storage) */
DW rtDW;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void BalancingModel_24a_step(void)
{
  real_T rtb_TmpSignalConversionAtSFunct[14];
  real_T tmp;
  int32_T i;
  int32_T tmp_0;

  /* SignalConversion generated from: '<S2>/ SFunction ' incorporates:
   *  Chart: '<S1>/Chart1'
   */
  rtb_TmpSignalConversionAtSFunct[0] = 0.0;
  rtb_TmpSignalConversionAtSFunct[1] = 0.0;
  rtb_TmpSignalConversionAtSFunct[2] = 0.0;
  rtb_TmpSignalConversionAtSFunct[3] = 0.0;
  rtb_TmpSignalConversionAtSFunct[4] = 0.0;
  rtb_TmpSignalConversionAtSFunct[5] = 0.0;
  rtb_TmpSignalConversionAtSFunct[6] = 0.0;
  rtb_TmpSignalConversionAtSFunct[7] = 0.0;
  rtb_TmpSignalConversionAtSFunct[8] = 0.0;
  rtb_TmpSignalConversionAtSFunct[9] = 0.0;
  rtb_TmpSignalConversionAtSFunct[10] = 0.0;
  rtb_TmpSignalConversionAtSFunct[11] = 0.0;
  rtb_TmpSignalConversionAtSFunct[12] = 0.0;
  rtb_TmpSignalConversionAtSFunct[13] = 0.0;

  /* Chart: '<S1>/Chart1' incorporates:
   *  SignalConversion generated from: '<S2>/ SFunction '
   */
  if (rtDW.temporalCounter_i1 < 32767U) {
    rtDW.temporalCounter_i1++;
  }

  if (rtDW.is_active_c7_BalancingModel_24a == 0U) {
    rtDW.is_active_c7_BalancingModel_24a = 1U;
    rtDW.is_c7_BalancingModel_24a = IN_initialization;
    memset(&rtDW.Ganna[0], 0, 14U * sizeof(real_T));
    rtDW.i = 1.0;
    rtDW.G = 1.0;
    rtDW.H = 1.0;
  } else {
    switch (rtDW.is_c7_BalancingModel_24a) {
     case IN_Data_Check:
      if (rtDW.i == 14.0) {
        rtDW.i = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Index_number;
        rtDW.idx = 1.0;
      } else {
        rtDW.i = 1.0;
        for (i = 0; i < 14; i++) {
          rtDW.i = (real_T)i + 1.0;
          tmp_0 = (int32_T)rtDW.i - 1;

          /* SignalConversion generated from: '<S2>/ SFunction ' */
          tmp = rtb_TmpSignalConversionAtSFunct[tmp_0];
          if (tmp > rtDW.cell_reference) {
            if (tmp > 1.6) {
              rtDW.Ganna[tmp_0] = (tmp - rtDW.cell_reference > 0.25);
            } else {
              rtDW.Ganna[tmp_0] = 0.0;
            }
          } else {
            rtDW.Ganna[tmp_0] = 0.0;
          }
        }
      }
      break;

     case IN_Discharge_Time_interval:
      if ((rtDW.temporalCounter_i1 >= 30000U) && (rtDW.H == 14.0)) {
        rtDW.H = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Get_balance_reference;
        rtDW.i = 1.0;
        rtDW.cell_reference = 10.0;
      } else {
        rtDW.H = 1.0;
        for (i = 0; i < 14; i++) {
          rtDW.H = (real_T)i + 1.0;
        }
      }
      break;

     case IN_Error_Check:
      if ((rtDW.G == 14.0) && (rtDW.abdullah == 0.0)) {
        rtDW.G = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Data_Check;
        rtDW.i = 1.0;
        for (i = 0; i < 14; i++) {
          rtDW.i = (real_T)i + 1.0;
          tmp_0 = (int32_T)rtDW.i - 1;

          /* SignalConversion generated from: '<S2>/ SFunction ' */
          tmp = rtb_TmpSignalConversionAtSFunct[tmp_0];
          if (tmp > rtDW.cell_reference) {
            if (tmp > 1.6) {
              rtDW.Ganna[tmp_0] = (tmp - rtDW.cell_reference > 0.25);
            } else {
              rtDW.Ganna[tmp_0] = 0.0;
            }
          } else {
            rtDW.Ganna[tmp_0] = 0.0;
          }
        }
      } else if (rtDW.abdullah == 1.0) {
        rtDW.G = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Get_balance_reference;
        rtDW.i = 1.0;
        rtDW.cell_reference = 10.0;
      } else {
        rtDW.G = 1.0;
        for (i = 0; i < 14; i++) {
          rtDW.G = (real_T)i + 1.0;
          if (fabs(rtb_TmpSignalConversionAtSFunct[(int32_T)rtDW.G - 1] -
                   rtDW.cell_reference) > 0.5) {
            rtDW.abdullah = 1.0;
          }
        }
      }
      break;

     case IN_Get_balance_reference:
      if (rtDW.i == 14.0) {
        rtDW.i = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Error_Check;
        rtDW.abdullah = 0.0;
      } else {
        rtDW.i = 1.0;
        for (i = 0; i < 14; i++) {
          rtDW.i = (real_T)i + 1.0;

          /* SignalConversion generated from: '<S2>/ SFunction ' */
          tmp = rtb_TmpSignalConversionAtSFunct[(int32_T)rtDW.i - 1];
          if ((tmp < rtDW.cell_reference) && (tmp > 1.5)) {
            rtDW.cell_reference = tmp;
          }
        }
      }
      break;

     case IN_Index_number:
      if (rtDW.G == 14.0) {
        rtDW.G = 1.0;
        rtDW.temporalCounter_i1 = 0U;
        rtDW.is_c7_BalancingModel_24a = IN_Discharge_Time_interval;
        rtDW.H = 1.0;
      } else {
        rtDW.G = 1.0;
        for (i = 0; i < 14; i++) {
          rtDW.G = (real_T)i + 1.0;
          if (rtDW.Ganna[(int32_T)rtDW.G - 1] == 1.0) {
            rtDW.idx++;
          }
        }
      }
      break;

     default:
      /* case IN_initialization: */
      rtDW.is_c7_BalancingModel_24a = IN_Get_balance_reference;
      rtDW.i = 1.0;
      rtDW.cell_reference = 10.0;
      break;
    }
  }
}

/* Model initialize function */
void BalancingModel_24a_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
