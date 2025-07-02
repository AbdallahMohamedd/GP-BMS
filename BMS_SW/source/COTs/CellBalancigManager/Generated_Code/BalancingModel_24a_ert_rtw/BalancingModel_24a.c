/*
 * File: BalancingModel_24a.c
 *
 * Code generated for Simulink model 'BalancingModel_24a'.
 *
 * Model version                  : 2.0
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed Jul  2 22:56:47 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M0/M0+
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "BalancingModel_24a.h"

/* Named constants for Chart: '<Root>/Stateflow' */
#define IN_Data_Check ((uint8_T)1U)
#define IN_Discharge_Time_interval ((uint8_T)2U)
#define IN_Error_Check ((uint8_T)3U)
#define IN_Get_balance_reference ((uint8_T)4U)
#define IN_Index_number ((uint8_T)5U)
#define IN_initialization ((uint8_T)6U)

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void BalancingModel_24a_step(TYPE_MEAS_RESULTS_RAW *rawResults)
{
  real_T Cell_Voltage[14];
  real_T tmp_0;
  int32_T i;
  int32_T tmp;

  /* SignalConversion generated from: '<S1>/ SFunction ' incorporates:
   *  Chart: '<Root>/Stateflow'
   *  Inport: '<Root>/CT1'
   *  Inport: '<Root>/CT10'
   *  Inport: '<Root>/CT11'
   *  Inport: '<Root>/CT12'
   *  Inport: '<Root>/CT13'
   *  Inport: '<Root>/CT14'
   *  Inport: '<Root>/CT2'
   *  Inport: '<Root>/CT3'
   *  Inport: '<Root>/CT4'
   *  Inport: '<Root>/CT5'
   *  Inport: '<Root>/CT6'
   *  Inport: '<Root>/CT7'
   *  Inport: '<Root>/CT8'
   *  Inport: '<Root>/CT9'
   */
  Cell_Voltage[0] = (rawResults->u16CellVoltage[0]) * CT_Resolution;
  Cell_Voltage[1] = (rawResults->u16CellVoltage[1]) * CT_Resolution;
  Cell_Voltage[2] = (rawResults->u16CellVoltage[2]) * CT_Resolution;
  Cell_Voltage[3] = (rawResults->u16CellVoltage[3]) * CT_Resolution;
  Cell_Voltage[4] = (rawResults->u16CellVoltage[4]) * CT_Resolution;
  Cell_Voltage[5] = (rawResults->u16CellVoltage[5]) * CT_Resolution;
  Cell_Voltage[6] = (rawResults->u16CellVoltage[6]) * CT_Resolution;
  Cell_Voltage[7] = (rawResults->u16CellVoltage[7]) * CT_Resolution;
  Cell_Voltage[8] = (rawResults->u16CellVoltage[8]) * CT_Resolution;
  Cell_Voltage[9] = (rawResults->u16CellVoltage[9]) * CT_Resolution;
  Cell_Voltage[10] = (rawResults->u16CellVoltage[10]) * CT_Resolution;
  Cell_Voltage[11] = (rawResults->u16CellVoltage[11]) * CT_Resolution;
  Cell_Voltage[12] = (rawResults->u16CellVoltage[12]) * CT_Resolution;
  Cell_Voltage[13] = (rawResults->u16CellVoltage[13]) * CT_Resolution;

  /* Chart: '<Root>/Stateflow' incorporates:
   *  Outport: '<Root>/Time_delay'
   *  Outport: '<Root>/active_indices'
   *  Outport: '<Root>/balance_enable'
   *  SignalConversion generated from: '<S1>/ SFunction '
   */
  if (rtDW.temporalCounter_i1 < 32767)
  {
    rtDW.temporalCounter_i1++;
  }

  if (rtDW.is_active_c7_BalancingModel_24a == 0)
  {
    rtDW.is_active_c7_BalancingModel_24a = 1U;
    rtDW.is_c7_BalancingModel_24a = IN_initialization;

    /* Outport: '<Root>/balance_enable' */
    memset(&rtY.balance_enable[0], 0, 14U * sizeof(real_T));
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

        /* Outport: '<Root>/active_indices' */
        memset(&rtY.active_indices[0], 0, 14U * sizeof(real_T));
        rtDW.idx = 1.0;
      }
      else
      {
        rtDW.i = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.i = (real_T)i + 1.0;
          tmp = (int32_T)rtDW.i - 1;

          /* SignalConversion generated from: '<S1>/ SFunction ' */
          tmp_0 = Cell_Voltage[tmp];
          if (tmp_0 > rtDW.cell_reference)
          {
            if (tmp_0 > 1.6)
            {
              rtY.balance_enable[tmp] = (tmp_0 - rtDW.cell_reference > 0.25);
            }
            else
            {
              rtY.balance_enable[tmp] = 0.0;
            }
          }
          else
          {
            rtY.balance_enable[tmp] = 0.0;
          }
        }
      }
      break;

    case IN_Discharge_Time_interval:
      if ((rtDW.temporalCounter_i1 >= 30000) && (rtDW.H == 14.0))
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
          tmp = (int32_T)rtDW.H - 1;
          if (rtY.balance_enable[tmp] == 1.0)
          {
            rtY.Time_delay[tmp] = 0.5;
#ifdef DebugInfoManager
            DebugInfo;
#endif

          }
          else
          {
            rtY.Time_delay[tmp] = 0.0;
#ifdef DebugInfoManager
            DebugInfo;
#endif

          }
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
          tmp = (int32_T)rtDW.i - 1;

          /* SignalConversion generated from: '<S1>/ SFunction ' */
          tmp_0 = Cell_Voltage[tmp];
          if (tmp_0 > rtDW.cell_reference)
          {
            if (tmp_0 > 1.6)
            {
              rtY.balance_enable[tmp] = ((tmp_0 - rtDW.cell_reference) > 1);
            }
            else
            {
              rtY.balance_enable[tmp] = 0.0;
            }
          }
          else
          {
            rtY.balance_enable[tmp] = 0.0;
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

          /* SignalConversion generated from: '<S1>/ SFunction ' */
          tmp_0 = Cell_Voltage[(int32_T)rtDW.i - 1];
          if ((tmp_0 < rtDW.cell_reference) && (tmp_0 > 1.5))
          {
            rtDW.cell_reference = tmp_0;
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

        /* Outport: '<Root>/Time_delay' */
        memset(&rtY.Time_delay[0], 0, 14U * sizeof(real_T));
      }
      else
      {
        rtDW.G = 1.0;
        for (i = 0; i < 14; i++)
        {
          rtDW.G = (real_T)i + 1.0;
            rtY.active_indices[(int32_T)rtDW.idx - 1] = rtDW.G;
            rtDW.idx++;
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
PRINTF("Cell Reference: %f\r\r\n", (float)rtDW.cell_reference);
  /* End of Chart: '<Root>/Stateflow' */
  for (int i = 0; i < 14; i++)
  {
      SlaveIF_enableCellBalancing((uint8_T)rtY.active_indices[i], (uint8_T)rtY.balance_enable[i], (float)rtY.Time_delay[i], 1);
    PRINTF("Active Index: %d, Balance Enable: %d, Time Delay: %f\r\r\n",
           (uint8_T)rtY.active_indices[i], (uint8_T)rtY.balance_enable[i], (float)rtY.Time_delay[i]);
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
