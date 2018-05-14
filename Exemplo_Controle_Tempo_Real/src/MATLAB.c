/*
 * File: MATLAB.c
 *
 * Code generated for Simulink model 'MATLAB'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
 * C/C++ source code generated on : Mon Mar 12 17:33:24 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "MATLAB.h"

/* Block signals and states (auto storage) */
DW rtDW;

/* External inputs (root inport signals with auto storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with auto storage) */
ExtY rtY;

/* Real-time model */
RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void MATLAB_step(void)
{
  real_T u;
  real_T x_chap[6];
  real_T b;
  static const int8_T a_0[6] = { 1, 0, 0, 0, 0, 0 };

  static const real_T b_a_0[36] = { 0.98919521524932863, 3.31993014621273, 0.0,
    0.0, 0.0, 0.0, -0.0049798952193190939, 0.991685162858988, 0.0, 0.0, 0.0, 0.0,
    -0.0049725067372275687, -0.0083086766794705467, 0.995561755643251,
    -177.39833746933076, 0.0, 0.0, -1.2463015019205817E-7, -1.386249035083449E-7,
    4.9926007350880053E-5, 0.995561755643251, 0.0, 0.0, -0.0049654199719701775,
    -0.0083027660694455028, 0.0, 0.0, 0.99130722209042965, -347.20647579675727,
    -1.245414910416825E-7, -1.3856576828203776E-7, 0.0, 0.0,
    4.985503623350465E-5, 0.9913072220904301 };

  static const real_T c_a_0[6] = { -0.0049798952193190948,
    -0.0083148371410119853, 0.0, 0.0, 0.0, 0.0 };

  static const real_T d_a_0[6] = { 0.30572422319580161, 2.744487847698645,
    -0.59609082226650667, 4997.71570549017, -5.2714074549290117,
    -536.40288451218089 };

  int32_T i;
  real_T tmp;
  int32_T i_0;

  /* Outport: '<Root>/w_out' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  /* MATLAB Function 'MATLAB Function': '<S1>:1' */
  /* '<S1>:1:10' w_out = w_chap; */
  rtY.w_out = rtDW.w_chap;

  /* Outport: '<Root>/w_out_2' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  /* '<S1>:1:11' w_out_2 = w_chap_2; */
  rtY.w_out_2 = rtDW.w_chap_2;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/r'
   *  Inport: '<Root>/y'
   */
  /* '<S1>:1:13' u=-K*x_chap_ant(1:2) +Ki*v - w_chap; */
  /* '<S1>:1:15' u=u*ligainv; */
  u = (( 299.66320244261738 * rtDW.x_chap_ant[0] + -1.1305702605185557 *
        rtDW.x_chap_ant[1]) + -101.24327468890826 * rtDW.v) - 0*rtDW.w_chap;

  /* '<S1>:1:17' y_chap = C_aum*x_chap_ant; */
  /* '<S1>:1:19' x_chap = Phi_aum*x_chap_ant + Gamma_aum*u + L*(y-y_chap); */
  tmp = 0.0;
  for (i = 0; i < 6; i++) {
    tmp += (real_T)a_0[i] * rtDW.x_chap_ant[i];
  }

  b = rtU.y - tmp;
  for (i = 0; i < 6; i++) {
    tmp = 0.0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      tmp += b_a_0[6 * i_0 + i] * rtDW.x_chap_ant[i_0];
    }

    x_chap[i] = (c_a_0[i] * u + tmp) + d_a_0[i] * b;
  }

  /* '<S1>:1:21' x_chap_1 = x_chap(1); */
  /* '<S1>:1:21' x_chap_2 = x_chap(2); */
  /* '<S1>:1:21' w_chap = x_chap(3); */
  rtDW.w_chap = x_chap[2];

  /* '<S1>:1:21' w_chap_2 = x_chap(5); */
  rtDW.w_chap_2 = x_chap[4];

  /* '<S1>:1:23' x_chap_ant = x_chap; */
  for (i = 0; i < 6; i++) {
    rtDW.x_chap_ant[i] = x_chap[i];
  }

  /* '<S1>:1:26' v = v +(r-y); */
  rtDW.v += rtU.r - rtU.y;

  /* Outport: '<Root>/u' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.u = u;

  /* Outport: '<Root>/x_chap_1' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.x_chap_1 = x_chap[0];

  /* Outport: '<Root>/x_chap_2' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.x_chap_2 = x_chap[1];
}

/* Model initialize function */
void MATLAB_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

