/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Position_State_Feedback_Motor_types.h
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

#ifndef RTW_HEADER_Position_State_Feedback_Motor_types_h_
#define RTW_HEADER_Position_State_Feedback_Motor_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Model Code Variants */

/* Custom Type definition for MATLABSystem: '<Root>/IN2A1' */
#include "MW_SVD.h"
#ifndef struct_tag_KxFW01GBdhqk5JOEHU3GlD
#define struct_tag_KxFW01GBdhqk5JOEHU3GlD

struct tag_KxFW01GBdhqk5JOEHU3GlD
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_DIGITALIO_HANDLE;
};

#endif                                 /* struct_tag_KxFW01GBdhqk5JOEHU3GlD */

#ifndef typedef_mbed_DigitalWrite_Position_St_T
#define typedef_mbed_DigitalWrite_Position_St_T

typedef struct tag_KxFW01GBdhqk5JOEHU3GlD mbed_DigitalWrite_Position_St_T;

#endif                             /* typedef_mbed_DigitalWrite_Position_St_T */

#ifndef struct_tag_UndvUYqhBVOhRRpUse3fWF
#define struct_tag_UndvUYqhBVOhRRpUse3fWF

struct tag_UndvUYqhBVOhRRpUse3fWF
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_PWM_HANDLE;
};

#endif                                 /* struct_tag_UndvUYqhBVOhRRpUse3fWF */

#ifndef typedef_mbed_PWMOutput_Position_State_T
#define typedef_mbed_PWMOutput_Position_State_T

typedef struct tag_UndvUYqhBVOhRRpUse3fWF mbed_PWMOutput_Position_State_T;

#endif                             /* typedef_mbed_PWMOutput_Position_State_T */

#ifndef struct_tag_sGWFgQTjADKFs5f99dqloH
#define struct_tag_sGWFgQTjADKFs5f99dqloH

struct tag_sGWFgQTjADKFs5f99dqloH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T SampleTime;
};

#endif                                 /* struct_tag_sGWFgQTjADKFs5f99dqloH */

#ifndef typedef_soc_stm_QEP_Position_State_Fe_T
#define typedef_soc_stm_QEP_Position_State_Fe_T

typedef struct tag_sGWFgQTjADKFs5f99dqloH soc_stm_QEP_Position_State_Fe_T;

#endif                             /* typedef_soc_stm_QEP_Position_State_Fe_T */

/* Parameters (default storage) */
typedef struct P_Position_State_Feedback_Mot_T_ P_Position_State_Feedback_Mot_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_Position_State_Feedba_T RT_MODEL_Position_State_Feedb_T;

#endif                   /* RTW_HEADER_Position_State_Feedback_Motor_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
