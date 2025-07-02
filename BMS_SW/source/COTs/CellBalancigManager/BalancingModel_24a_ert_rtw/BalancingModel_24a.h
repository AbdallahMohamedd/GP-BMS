/*
 * File: BalancingModel_24a.h
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
#include <source/COTs/BMSDataBase/Inc/database.h>

#ifndef BalancingModel_24a_h_
#define BalancingModel_24a_h_
#ifndef BalancingModel_24a_COMMON_INCLUDES_
#define BalancingModel_24a_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* BalancingModel_24a_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;
#define CT_Resolution 0.00015258789 
/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Ganna[14];                    /* '<S1>/Chart1' */
  real_T i;                            /* '<S1>/Chart1' */
  real_T G;                            /* '<S1>/Chart1' */
  real_T H;                            /* '<S1>/Chart1' */
  real_T cell_reference;               /* '<S1>/Chart1' */
  real_T idx;                          /* '<S1>/Chart1' */
  real_T abdullah;                     /* '<S1>/Chart1' */
  uint16_T temporalCounter_i1;         /* '<S1>/Chart1' */
  uint8_T is_active_c7_BalancingModel_24a;/* '<S1>/Chart1' */
  uint8_T is_c7_BalancingModel_24a;    /* '<S1>/Chart1' */ //--> determine state in stateflow 
} DW;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

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
 * '<S1>'   : 'BalancingModel_24a/Cell Balancing Model '
 * '<S2>'   : 'BalancingModel_24a/Cell Balancing Model /Chart1'
 */
#endif                                 /* BalancingModel_24a_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
