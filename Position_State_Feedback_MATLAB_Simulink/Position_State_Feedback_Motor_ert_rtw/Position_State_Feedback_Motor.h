/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Position_State_Feedback_Motor.h
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

#ifndef RTW_HEADER_Position_State_Feedback_Motor_h_
#define RTW_HEADER_Position_State_Feedback_Motor_h_
#include <math.h>
#ifndef Position_State_Feedback_Motor_COMMON_INCLUDES_
#define Position_State_Feedback_Motor_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "MW_MbedPinInterface.h"
#include "MW_digitalIO.h"
#include "MW_PWM.h"
#include "soc_stm_encoder.h"
#endif                      /* Position_State_Feedback_Motor_COMMON_INCLUDES_ */

#include "Position_State_Feedback_Motor_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Child system includes */
#include "Controller_Observer_Model.h"
#include "MW_target_hardware_resources.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  ((const char_T **)(&((rtm)->errorStatus)))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

#define Position_State_Feedback_Motor_M (Position_State_Feedback_Moto_M)

/* Block signals (default storage) */
typedef struct {
  real_T u;                            /* '<Root>/Model' */
  real_T position;                     /* '<Root>/position' */
  real_T last_ticks_star;              /* '<Root>/modified counter' */
  real_T Reference;                    /* '<Root>/Reference Switching' */
  uint16_T QudratureEncoder1_o1;       /* '<Root>/Qudrature Encoder1' */
} B_Position_State_Feedback_Mot_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  soc_stm_QEP_Position_State_Fe_T obj; /* '<Root>/Qudrature Encoder1' */
  mbed_DigitalWrite_Position_St_T obj_n;/* '<Root>/EN_A1' */
  mbed_PWMOutput_Position_State_T obj_l;/* '<Root>/IN2A1' */
  mbed_PWMOutput_Position_State_T obj_k;/* '<Root>/IN1A1' */
  real_T Delay3_DSTATE;                /* '<Root>/Delay3' */
  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Refer;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Model;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_posit;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_modif;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Qudra;   /* synthesized block */

  uint32_T is_c3_Position_State_Feedback_M;/* '<Root>/Reference Switching' */
  uint16_T Delay2_DSTATE;              /* '<Root>/Delay2' */
  uint16_T temporalCounter_i1;         /* '<Root>/Reference Switching' */
  uint8_T is_active_c3_Position_State_Fee;/* '<Root>/Reference Switching' */
  MdlrefDW_Controller_Observer__T Model_InstanceData;/* '<Root>/Model' */
} DW_Position_State_Feedback_Mo_T;

/* Parameters (default storage) */
struct P_Position_State_Feedback_Mot_T_ {
  real_T Delay3_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<Root>/Delay3'
                                        */
  real_T Gain1_Gain;                   /* Expression: 100/12
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Constant2_Value;              /* Expression: 1
                                        * Referenced by: '<S1>/Constant2'
                                        */
  real_T Constant1_Value;              /* Expression: 2
                                        * Referenced by: '<S1>/Constant1'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 100
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -100
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Constant2_Value_c;            /* Expression: 1
                                        * Referenced by: '<Root>/Constant2'
                                        */
  uint16_T Delay2_InitialCondition;
                                  /* Computed Parameter: Delay2_InitialCondition
                                   * Referenced by: '<Root>/Delay2'
                                   */
};

/* Real-time Model Data Structure */
struct tag_RTM_Position_State_Feedba_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_Position_State_Feedback_Mot_T Position_State_Feedback_Motor_P;

/* Block signals (default storage) */
extern B_Position_State_Feedback_Mot_T Position_State_Feedback_Motor_B;

/* Block states (default storage) */
extern DW_Position_State_Feedback_Mo_T Position_State_Feedback_Moto_DW;

/* Model block global parameters (default storage) */
extern real_T rtP_Ad[4];               /* Variable: Ad
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_Bd[2];               /* Variable: Bd
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_Cd[2];               /* Variable: Cd
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_Kp[2];               /* Variable: Kp
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_L[2];                /* Variable: L
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_Ts;                  /* Variable: Ts
                                        * Referenced by:
                                        *   '<Root>/Ticks Delta'
                                        *   '<Root>/Qudrature Encoder1'
                                        */
extern real_T rtP_ki;                  /* Variable: ki
                                        * Referenced by: '<Root>/Model'
                                        */

/* Model entry point functions */
extern void Position_State_Feedback_Motor_initialize(void);
extern void Position_State_Feedback_Motor_step(void);
extern void Position_State_Feedback_Motor_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Position_State_Feedb_T *const Position_State_Feedback_Moto_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

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
 * '<Root>' : 'Position_State_Feedback_Motor'
 * '<S1>'   : 'Position_State_Feedback_Motor/1 if input > 0 0 otherwise1'
 * '<S2>'   : 'Position_State_Feedback_Motor/Reference Switching'
 * '<S3>'   : 'Position_State_Feedback_Motor/Ticks Delta'
 * '<S4>'   : 'Position_State_Feedback_Motor/modified counter'
 * '<S5>'   : 'Position_State_Feedback_Motor/position'
 */
#endif                         /* RTW_HEADER_Position_State_Feedback_Motor_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
