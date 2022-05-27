/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Position_State_Feedback_Motor_data.c
 *
 * Code generated for Simulink model 'Position_State_Feedback_Motor'.
 *
 * Model version                  : 1.16
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Fri May 27 15:46:21 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Position_State_Feedback_Motor.h"
#include "Position_State_Feedback_Motor_private.h"

/* Model block global parameters (default storage) */
real_T rtP_Ad[4] = { 1.0, 0.0, 0.0045507537789205793, 0.82585171582255734 } ;/* Variable: Ad
                                                                      * Referenced by: '<Root>/Model'
                                                                      */

real_T rtP_Bd[2] = { 0.00052852496597578856, 0.20488033432640315 } ;/* Variable: Bd
                                                                     * Referenced by: '<Root>/Model'
                                                                     */

real_T rtP_Cd[2] = { 1.0, 0.0 } ;      /* Variable: Cd
                                        * Referenced by: '<Root>/Model'
                                        */

real_T rtP_Kp[2] = { 83.34674735262773, 1.4373335931186066 } ;/* Variable: Kp
                                                               * Referenced by: '<Root>/Model'
                                                               */

real_T rtP_L[2] = { 0.61808536972954442, 0.005445473720324172 } ;/* Variable: L
                                                                  * Referenced by: '<Root>/Model'
                                                                  */

real_T rtP_Ts = 0.005;                 /* Variable: Ts
                                        * Referenced by:
                                        *   '<Root>/Ticks Delta'
                                        *   '<Root>/Qudrature Encoder1'
                                        */
real_T rtP_ki = 2.6333692309476358;    /* Variable: ki
                                        * Referenced by: '<Root>/Model'
                                        */

/* Block parameters (default storage) */
P_Position_State_Feedback_Mot_T Position_State_Feedback_Motor_P = {
  /* Expression: 0
   * Referenced by: '<Root>/Delay3'
   */
  0.0,

  /* Expression: 100/12
   * Referenced by: '<Root>/Gain1'
   */
  8.3333333333333339,

  /* Expression: 1
   * Referenced by: '<S1>/Constant2'
   */
  1.0,

  /* Expression: 2
   * Referenced by: '<S1>/Constant1'
   */
  2.0,

  /* Expression: 100
   * Referenced by: '<Root>/Saturation1'
   */
  100.0,

  /* Expression: -100
   * Referenced by: '<Root>/Saturation1'
   */
  -100.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant2'
   */
  1.0,

  /* Computed Parameter: Delay2_InitialCondition
   * Referenced by: '<Root>/Delay2'
   */
  0U
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
