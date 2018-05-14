/*
 * File: MATLAB.h
 *
 * Code generated for Simulink model 'MATLAB'.
 *
 * Model version                  : 1.63
 * Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
 * C/C++ source code generated on : Tue Jan 02 10:42:06 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_MATLAB_h_
#define RTW_HEADER_MATLAB_h_
#ifndef MATLAB_COMMON_INCLUDES_
# define MATLAB_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* MATLAB_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  real_T x_chap_ant[6];                /* '<Root>/_DataStoreBlk_1' */
  real_T v;                            /* '<Root>/_DataStoreBlk_2' */
  real_T w_chap;                       /* '<Root>/_DataStoreBlk_3' */
  real_T w_chap_2;                     /* '<Root>/_DataStoreBlk_4' */
} DW;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T r;                            /* '<Root>/r' */
  real_T y;                            /* '<Root>/y' */
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
  real_T x_chap_1;                     /* '<Root>/x_chap_1' */
  real_T x_chap_2;                     /* '<Root>/x_chap_2' */
  real_T w_out;                        /* '<Root>/w_out' */
  real_T w_out_2;                      /* '<Root>/w_out_2' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (auto storage) */
extern DW rtDW;

/* External inputs (root inport signals with auto storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void MATLAB_initialize(void);
extern void MATLAB_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('Simulink1_varias_freqs/MATLAB Function')    - opens subsystem Simulink1_varias_freqs/MATLAB Function
 * hilite_system('Simulink1_varias_freqs/MATLAB Function/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Simulink1_varias_freqs'
 * '<S1>'   : 'Simulink1_varias_freqs/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_MATLAB_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
