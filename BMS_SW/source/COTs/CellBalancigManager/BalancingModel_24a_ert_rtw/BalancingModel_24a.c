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
#define IN_Data_Check ((uint8_T)1U)
#define IN_Discharge_Time_interval ((uint8_T)2U)
#define IN_Error_Check ((uint8_T)3U)
#define IN_Get_balance_reference ((uint8_T)4U)
#define IN_Index_number ((uint8_T)5U)
#define IN_initialization ((uint8_T)6U)

/* Block signals and states (default storage) */
DW rtDW;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model steBalancingModel_24a_stepp function */
void BalancingModel_24a_step(TYPE_MEAS_RESULTS_RAW *rawResults)
{
  real_T Cell_Voltage[14];
  real_T tmp;
  int32_T i;
  int32_T tmp_0;

  /* SignalConversion generated from: '<S2>/ SFunction ' incorporates:
   *  Chart: '<S1>/Chart1'
   */
  Cell_Voltage[0] = rawResults->u16CellVoltage[0];
  Cell_Voltage[1] = rawResults->u16CellVoltage[1];
  Cell_Voltage[2] = rawResults->u16CellVoltage[2];
  Cell_Voltage[3] = rawResults->u16CellVoltage[3];
  Cell_Voltage[4] = rawResults->u16CellVoltage[4];
  Cell_Voltage[5] = rawResults->u16CellVoltage[5];
  Cell_Voltage[6] = rawResults->u16CellVoltage[6];
  Cell_Voltage[7] = rawResults->u16CellVoltage[7];
  Cell_Voltage[8] = rawResults->u16CellVoltage[8];
  Cell_Voltage[9] = rawResults->u16CellVoltage[9];
  Cell_Voltage[10] = rawResults->u16CellVoltage[10];
  Cell_Voltage[11] = rawResults->u16CellVoltage[11];
  Cell_Voltage[12] = rawResults->u16CellVoltage[12];
  Cell_Voltage[13] = rawResults->u16CellVoltage[13];

  /* Chart: '<S1>/Chart1' incorporates:
   *  SignalConversion generated from: '<S2>/ SFunction '
   */
  if (rtDW.temporalCounter_i1 < 32767U)
  {
    rtDW.temporalCounter_i1++;
  }

  if (rtDW.is_active_c7_BalancingModel_24a == 0U)
  {
    rtDW.is_active_c7_BalancingModel_24a = 1U;
    rtDW.is_c7_BalancingModel_24a = IN_initialization;
    memset(&rtDW.Ganna[0], 0, 14U * sizeof(real_T));
    rtDW.i = 1.0;
    rtDW.G = 1.0;
    rtDW.H = 1.0;
  }
  else
  {
    switch (rtDW.is_c7_BalancingModel_24a)
    {
    case IN_Data_Check:
      if (rtDW.i == 14.0)
      {
        rtDW.i = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Index_number;
        rtDW.idx = 1.0;
      }
      else
      {
        rtDW.i = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.i = (real_T)i + 1.0;
          tmp_0 = (int32_T)rtDW.i - 1;

          /* SignalConversion generated from: '<S2>/ SFunction ' */
          tmp = Cell_Voltage[tmp_0];
          if (tmp > rtDW.cell_reference)
          {
            if (tmp > 1.6)
            {
              rtDW.Ganna[tmp_0] = (tmp - rtDW.cell_reference > 0.25);
            }
            else
            {
              rtDW.Ganna[tmp_0] = 0.0;
            }
          }
          else
          {
            rtDW.Ganna[tmp_0] = 0.0;
          }
        }
      }
      break;

    case IN_Discharge_Time_interval:
      if ((rtDW.temporalCounter_i1 >= 30000U) && (rtDW.H == 14.0))
      {
        rtDW.H = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Get_balance_reference;
        rtDW.i = 1.0;
        rtDW.cell_reference = 10.0;
      }
      else
      {
        rtDW.H = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.H = (real_T)i + 1.0;
        }
      }
      break;

    case IN_Error_Check:
      if ((rtDW.G == 14.0) && (rtDW.abdullah == 0.0))
      {
        rtDW.G = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Data_Check;
        rtDW.i = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.i = (real_T)i + 1.0;
          tmp_0 = (int32_T)rtDW.i - 1;

          /* SignalConversion generated from: '<S2>/ SFunction ' */
          tmp = Cell_Voltage[tmp_0];
          if (tmp > rtDW.cell_reference)
          {
            if (tmp > 1.6)
            {
              rtDW.Ganna[tmp_0] = (tmp - rtDW.cell_reference > 0.25);
            }
            else
            {
              rtDW.Ganna[tmp_0] = 0.0;
            }
          }
          else
          {
            rtDW.Ganna[tmp_0] = 0.0;
          }
        }
      }
      else if (rtDW.abdullah == 1.0)
      {
        rtDW.G = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Get_balance_reference;
        rtDW.i = 1.0;
        rtDW.cell_reference = 10.0;
      }
      else
      {
        rtDW.G = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.G = (real_T)i + 1.0;
          if (fabs(Cell_Voltage[(int32_T)rtDW.G - 1] -
                   rtDW.cell_reference) > 0.5)
          {
            rtDW.abdullah = 1.0;
          }
        }
      }
      break;

    case IN_Get_balance_reference:
      if (rtDW.i == 14.0)
      {
        rtDW.i = 1.0;
        rtDW.is_c7_BalancingModel_24a = IN_Error_Check;
        rtDW.abdullah = 0.0;
      }
      else
      {
        rtDW.i = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.i = (real_T)i + 1.0;

          /* SignalConversion generated from: '<S2>/ SFunction ' */
          tmp = Cell_Voltage[(int32_T)rtDW.i - 1];
          if ((tmp < rtDW.cell_reference) && (tmp > 1.5))
          {
            rtDW.cell_reference = tmp;
          }
        }
      }
      break;

    case IN_Index_number:
      if (rtDW.G == 14.0)
      {
        rtDW.G = 1.0;
        rtDW.temporalCounter_i1 = 0U;
        rtDW.is_c7_BalancingModel_24a = IN_Discharge_Time_interval;
        rtDW.H = 1.0;
      }
      else
      {
        rtDW.G = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.G = (real_T)i + 1.0;
          if (rtDW.Ganna[(int32_T)rtDW.G - 1] == 1.0)
          {
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
