/*
 * File: BalancingModel_24a.h
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

#ifndef BalancingModel_24a_h_
#define BalancingModel_24a_h_
#ifndef BalancingModel_24a_COMMON_INCLUDES_
#define BalancingModel_24a_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include "COTs/BMSDataBase/Inc/database.h"
#include "COTs/SlaveControlIF/Inc/slaveIF.h"
#include "COTs/DebugInfoManager/Inc/debugInfo.h"
#endif                                 /* BalancingModel_24a_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif
#define CT_Resolution 0.00015258789  
/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T i;                            /* '<Root>/Stateflow' */
  real_T G;                            /* '<Root>/Stateflow' */
  real_T H;                            /* '<Root>/Stateflow' */
  real_T cell_reference;               /* '<Root>/Stateflow' */
  real_T idx;                          /* '<Root>/Stateflow' */
  real_T abdullah;                     /* '<Root>/Stateflow' */
  uint16_T temporalCounter_i1;         /* '<Root>/Stateflow' */
  uint8_T is_active_c7_BalancingModel_24a;/* '<Root>/Stateflow' */
  uint8_T is_c7_BalancingModel_24a;    /* '<Root>/Stateflow' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T CT1;                          /* '<Root>/CT1' */
  real_T CT2;                          /* '<Root>/CT2' */
  real_T CT3;                          /* '<Root>/CT3' */
  real_T CT4;                          /* '<Root>/CT4' */
  real_T CT5;                          /* '<Root>/CT5' */
  real_T CT6;                          /* '<Root>/CT6' */
  real_T CT7;                          /* '<Root>/CT7' */
  real_T CT8;                          /* '<Root>/CT8' */
  real_T CT9;                          /* '<Root>/CT9' */
  real_T CT10;                         /* '<Root>/CT10' */
  real_T CT11;                         /* '<Root>/CT11' */
  real_T CT12;                         /* '<Root>/CT12' */
  real_T CT13;                         /* '<Root>/CT13' */
  real_T CT14;                         /* '<Root>/CT14' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Time_delay[14];               /* '<Root>/Time_delay' */
  real_T active_indices[14];           /* '<Root>/active_indices' */
  real_T balance_enable[14];           /* '<Root>/balance_enable' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void BalancingModel_24a_initialize(void);
extern void BalancingModel_24a_step(TYPE_MEAS_RESULTS_RAW *rawResults);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BalancingModel_24a'
 * '<S1>'   : 'BalancingModel_24a/Stateflow'
 */
#endif                                 /* BalancingModel_24a_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
