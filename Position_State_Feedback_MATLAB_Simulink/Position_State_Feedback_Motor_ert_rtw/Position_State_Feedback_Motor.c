/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Position_State_Feedback_Motor.c
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

/* Named constants for Chart: '<Root>/Reference Switching' */
#define Position_State_Feedback_M_IN_S1 (1U)
#define Position_State_Feedback_M_IN_S2 (2U)
#define Position_State_Feedback_M_IN_S3 (3U)
#define Position_State_Feedback_M_IN_s5 (4U)
#define Position_State_Feedback_M_IN_s6 (5U)

/* Block signals (default storage) */
B_Position_State_Feedback_Mot_T Position_State_Feedback_Motor_B;

/* Block states (default storage) */
DW_Position_State_Feedback_Mo_T Position_State_Feedback_Moto_DW;

/* Real-time model */
static RT_MODEL_Position_State_Feedb_T Position_State_Feedback_Moto_M_;
RT_MODEL_Position_State_Feedb_T *const Position_State_Feedback_Moto_M =
  &Position_State_Feedback_Moto_M_;

/* Model step function */
void Position_State_Feedback_Motor_step(void)
{
  real_T rtb_Divide;
  real_T sum;
  int32_T tmp;
  uint16_T b_varargout_2;

  /* Chart: '<Root>/Reference Switching' */
  if (Position_State_Feedback_Moto_DW.temporalCounter_i1 < 511U) {
    Position_State_Feedback_Moto_DW.temporalCounter_i1++;
  }

  if (Position_State_Feedback_Moto_DW.is_active_c3_Position_State_Fee == 0U) {
    Position_State_Feedback_Moto_DW.is_active_c3_Position_State_Fee = 1U;
    Position_State_Feedback_Moto_DW.is_c3_Position_State_Feedback_M =
      Position_State_Feedback_M_IN_S1;
    Position_State_Feedback_Moto_DW.temporalCounter_i1 = 0U;
    Position_State_Feedback_Motor_B.Reference = 0.0;
  } else {
    switch (Position_State_Feedback_Moto_DW.is_c3_Position_State_Feedback_M) {
     case Position_State_Feedback_M_IN_S1:
      if (Position_State_Feedback_Moto_DW.temporalCounter_i1 >= 400U) {
        Position_State_Feedback_Moto_DW.is_c3_Position_State_Feedback_M =
          Position_State_Feedback_M_IN_S2;
        Position_State_Feedback_Moto_DW.temporalCounter_i1 = 0U;
        Position_State_Feedback_Motor_B.Reference = -3.1415926535897931;
      } else {
        Position_State_Feedback_Motor_B.Reference = 0.0;
      }
      break;

     case Position_State_Feedback_M_IN_S2:
      if (Position_State_Feedback_Moto_DW.temporalCounter_i1 >= 400U) {
        Position_State_Feedback_Moto_DW.is_c3_Position_State_Feedback_M =
          Position_State_Feedback_M_IN_S3;
        Position_State_Feedback_Moto_DW.temporalCounter_i1 = 0U;
        Position_State_Feedback_Motor_B.Reference = 3.1415926535897931;
      } else {
        Position_State_Feedback_Motor_B.Reference = -3.1415926535897931;
      }
      break;

     case Position_State_Feedback_M_IN_S3:
      if (Position_State_Feedback_Moto_DW.temporalCounter_i1 >= 400U) {
        Position_State_Feedback_Moto_DW.is_c3_Position_State_Feedback_M =
          Position_State_Feedback_M_IN_s5;
        Position_State_Feedback_Moto_DW.temporalCounter_i1 = 0U;
        Position_State_Feedback_Motor_B.Reference = 6.2831853071795862;
      } else {
        Position_State_Feedback_Motor_B.Reference = 3.1415926535897931;
      }
      break;

     case Position_State_Feedback_M_IN_s5:
      if (Position_State_Feedback_Moto_DW.temporalCounter_i1 >= 400U) {
        Position_State_Feedback_Moto_DW.is_c3_Position_State_Feedback_M =
          Position_State_Feedback_M_IN_s6;
        Position_State_Feedback_Moto_DW.temporalCounter_i1 = 0U;
        Position_State_Feedback_Motor_B.Reference = 0.0;
      } else {
        Position_State_Feedback_Motor_B.Reference = 6.2831853071795862;
      }
      break;

     default:
      /* case IN_s6: */
      if (Position_State_Feedback_Moto_DW.temporalCounter_i1 >= 400U) {
        Position_State_Feedback_Moto_DW.is_c3_Position_State_Feedback_M =
          Position_State_Feedback_M_IN_S1;
        Position_State_Feedback_Moto_DW.temporalCounter_i1 = 0U;
        Position_State_Feedback_Motor_B.Reference = 0.0;
      } else {
        Position_State_Feedback_Motor_B.Reference = 0.0;
      }
      break;
    }
  }

  /* End of Chart: '<Root>/Reference Switching' */
  /* MATLABSystem: '<Root>/Qudrature Encoder1' */
  if (Position_State_Feedback_Moto_DW.obj.SampleTime != rtP_Ts) {
    Position_State_Feedback_Moto_DW.obj.SampleTime = rtP_Ts;
  }

  /* MATLABSystem: '<Root>/Qudrature Encoder1' */
  /* 		%% Define output properties */
  /*  Call C-function implementing device output */
  Position_State_Feedback_Motor_B.QudratureEncoder1_o1 = getEncoderCount();

  /* MATLABSystem: '<Root>/Qudrature Encoder1' */
  getIndexCount(&b_varargout_2);

  /* MATLAB Function: '<Root>/modified counter' incorporates:
   *  Delay: '<Root>/Delay3'
   */
  Position_State_Feedback_Motor_B.last_ticks_star =
    Position_State_Feedback_Moto_DW.Delay3_DSTATE;

  /* MATLAB Function: '<Root>/Ticks Delta' incorporates:
   *  Delay: '<Root>/Delay2'
   */
  if (fabs((real_T)(Position_State_Feedback_Motor_B.QudratureEncoder1_o1 -
                    Position_State_Feedback_Moto_DW.Delay2_DSTATE)) <= ceil
      (8400.0 * rtP_Ts)) {
    tmp = Position_State_Feedback_Motor_B.QudratureEncoder1_o1 -
      Position_State_Feedback_Moto_DW.Delay2_DSTATE;
  } else if (Position_State_Feedback_Moto_DW.Delay2_DSTATE >
             Position_State_Feedback_Motor_B.QudratureEncoder1_o1) {
    tmp = (Position_State_Feedback_Motor_B.QudratureEncoder1_o1 -
           Position_State_Feedback_Moto_DW.Delay2_DSTATE) + 65535;
  } else {
    tmp = (Position_State_Feedback_Motor_B.QudratureEncoder1_o1 -
           Position_State_Feedback_Moto_DW.Delay2_DSTATE) - 65535;
  }

  /* End of MATLAB Function: '<Root>/Ticks Delta' */

  /* MATLAB Function: '<Root>/modified counter' */
  sum = Position_State_Feedback_Motor_B.last_ticks_star + (real_T)tmp;
  rtb_Divide = fabs(sum);
  if (sum < 0.0) {
    sum = -1.0;
  } else if (sum > 0.0) {
    sum = 1.0;
  } else if (sum == 0.0) {
    sum = 0.0;
  } else {
    sum = (rtNaN);
  }

  if (rtIsNaN(rtb_Divide)) {
    rtb_Divide = (rtNaN);
  } else if (rtIsInf(rtb_Divide)) {
    rtb_Divide = (rtNaN);
  } else if (rtb_Divide == 0.0) {
    rtb_Divide = 0.0;
  } else {
    rtb_Divide = fmod(rtb_Divide, 89796.0);
  }

  Position_State_Feedback_Motor_B.last_ticks_star = sum * rtb_Divide;

  /* MATLAB Function: '<Root>/position' */
  Position_State_Feedback_Motor_B.position = 6.2831853071795862 *
    Position_State_Feedback_Motor_B.last_ticks_star / 3591.84;

  /* ModelReference: '<Root>/Model' */
  Controller_Observer_Model(&Position_State_Feedback_Motor_B.Reference,
    &Position_State_Feedback_Motor_B.position,
    &Position_State_Feedback_Motor_B.u,
    &(Position_State_Feedback_Moto_DW.Model_InstanceData.rtdw));

  /* Gain: '<Root>/Gain1' */
  sum = Position_State_Feedback_Motor_P.Gain1_Gain *
    Position_State_Feedback_Motor_B.u;

  /* Signum: '<S1>/Sign' */
  if (sum < 0.0) {
    rtb_Divide = -1.0;
  } else if (sum > 0.0) {
    rtb_Divide = 1.0;
  } else if (sum == 0.0) {
    rtb_Divide = 0.0;
  } else {
    rtb_Divide = (rtNaN);
  }

  /* End of Signum: '<S1>/Sign' */

  /* Product: '<S1>/Divide' incorporates:
   *  Constant: '<S1>/Constant1'
   *  Constant: '<S1>/Constant2'
   *  Sum: '<S1>/Sum4'
   */
  rtb_Divide = (rtb_Divide + Position_State_Feedback_Motor_P.Constant2_Value) /
    Position_State_Feedback_Motor_P.Constant1_Value;

  /* Saturate: '<Root>/Saturation1' */
  if (sum > Position_State_Feedback_Motor_P.Saturation1_UpperSat) {
    sum = Position_State_Feedback_Motor_P.Saturation1_UpperSat;
  } else if (sum < Position_State_Feedback_Motor_P.Saturation1_LowerSat) {
    sum = Position_State_Feedback_Motor_P.Saturation1_LowerSat;
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Abs: '<Root>/Abs1' */
  sum = fabs(sum);

  /* MATLABSystem: '<Root>/IN1A1' incorporates:
   *  Product: '<Root>/Product4'
   */
  MW_PWM_SetDutyCycle(Position_State_Feedback_Moto_DW.obj_k.MW_PWM_HANDLE,
                      rtb_Divide * sum);

  /* MATLABSystem: '<Root>/IN2A1' incorporates:
   *  Logic: '<Root>/NOT1'
   *  Product: '<Root>/Product3'
   */
  MW_PWM_SetDutyCycle(Position_State_Feedback_Moto_DW.obj_l.MW_PWM_HANDLE, sum *
                      (real_T)!(rtb_Divide != 0.0));

  /* MATLABSystem: '<Root>/EN_A1' incorporates:
   *  Constant: '<Root>/Constant2'
   */
  MW_digitalIO_write(Position_State_Feedback_Moto_DW.obj_n.MW_DIGITALIO_HANDLE,
                     Position_State_Feedback_Motor_P.Constant2_Value_c != 0.0);

  /* Update for Delay: '<Root>/Delay3' */
  Position_State_Feedback_Moto_DW.Delay3_DSTATE =
    Position_State_Feedback_Motor_B.last_ticks_star;

  /* Update for Delay: '<Root>/Delay2' */
  Position_State_Feedback_Moto_DW.Delay2_DSTATE =
    Position_State_Feedback_Motor_B.QudratureEncoder1_o1;

  {                                    /* Sample time: [0.005s, 0.0s] */
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  Position_State_Feedback_Moto_M->Timing.taskTime0 =
    ((time_T)(++Position_State_Feedback_Moto_M->Timing.clockTick0)) *
    Position_State_Feedback_Moto_M->Timing.stepSize0;
}

/* Model initialize function */
void Position_State_Feedback_Motor_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal(Position_State_Feedback_Moto_M, -1);
  Position_State_Feedback_Moto_M->Timing.stepSize0 = 0.005;

  /* External mode info */
  Position_State_Feedback_Moto_M->Sizes.checksums[0] = (3221227119U);
  Position_State_Feedback_Moto_M->Sizes.checksums[1] = (1472829747U);
  Position_State_Feedback_Moto_M->Sizes.checksums[2] = (3497975178U);
  Position_State_Feedback_Moto_M->Sizes.checksums[3] = (2273715960U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[9];
    Position_State_Feedback_Moto_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Position_State_Feedback_Moto_M->extModeInfo,
      &Position_State_Feedback_Moto_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Position_State_Feedback_Moto_M->extModeInfo,
                        Position_State_Feedback_Moto_M->Sizes.checksums);
    rteiSetTPtr(Position_State_Feedback_Moto_M->extModeInfo, rtmGetTPtr
                (Position_State_Feedback_Moto_M));
  }

  /* Model Initialize function for ModelReference Block: '<Root>/Model' */
  Controller_Observer__initialize(rtmGetErrorStatusPointer
    (Position_State_Feedback_Moto_M),
    &(Position_State_Feedback_Moto_DW.Model_InstanceData.rtm));

  {
    uint32_T pinname;
    mbed_DigitalWrite_Position_St_T *obj_0;
    mbed_PWMOutput_Position_State_T *obj;

    /* InitializeConditions for Delay: '<Root>/Delay3' */
    Position_State_Feedback_Moto_DW.Delay3_DSTATE =
      Position_State_Feedback_Motor_P.Delay3_InitialCondition;

    /* InitializeConditions for Delay: '<Root>/Delay2' */
    Position_State_Feedback_Moto_DW.Delay2_DSTATE =
      Position_State_Feedback_Motor_P.Delay2_InitialCondition;

    /* SystemInitialize for ModelReference: '<Root>/Model' */
    Controller_Observer_Model_Init
      (&(Position_State_Feedback_Moto_DW.Model_InstanceData.rtdw));

    /* Start for MATLABSystem: '<Root>/Qudrature Encoder1' */
    /*  Constructor */
    Position_State_Feedback_Moto_DW.obj.matlabCodegenIsDeleted = false;
    Position_State_Feedback_Moto_DW.obj.SampleTime = rtP_Ts;
    Position_State_Feedback_Moto_DW.obj.isInitialized = 1;

    /* 		%% Define output properties */
    /*  Call C-function implementing device initialization */
    initEncoder();
    Position_State_Feedback_Moto_DW.obj.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/IN1A1' */
    Position_State_Feedback_Moto_DW.obj_k.matlabCodegenIsDeleted = true;
    Position_State_Feedback_Moto_DW.obj_k.isInitialized = 0;
    Position_State_Feedback_Moto_DW.obj_k.matlabCodegenIsDeleted = false;
    obj = &Position_State_Feedback_Moto_DW.obj_k;
    Position_State_Feedback_Moto_DW.obj_k.isSetupComplete = false;
    Position_State_Feedback_Moto_DW.obj_k.isInitialized = 1;
    pinname = 5;
    obj->MW_PWM_HANDLE = MW_PWM_Open(pinname, 2000.0, 0.0);
    MW_PWM_Start(Position_State_Feedback_Moto_DW.obj_k.MW_PWM_HANDLE);
    Position_State_Feedback_Moto_DW.obj_k.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/IN2A1' */
    Position_State_Feedback_Moto_DW.obj_l.matlabCodegenIsDeleted = true;
    Position_State_Feedback_Moto_DW.obj_l.isInitialized = 0;
    Position_State_Feedback_Moto_DW.obj_l.matlabCodegenIsDeleted = false;
    obj = &Position_State_Feedback_Moto_DW.obj_l;
    Position_State_Feedback_Moto_DW.obj_l.isSetupComplete = false;
    Position_State_Feedback_Moto_DW.obj_l.isInitialized = 1;
    pinname = 4;
    obj->MW_PWM_HANDLE = MW_PWM_Open(pinname, 2000.0, 0.0);
    MW_PWM_Start(Position_State_Feedback_Moto_DW.obj_l.MW_PWM_HANDLE);
    Position_State_Feedback_Moto_DW.obj_l.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/EN_A1' */
    Position_State_Feedback_Moto_DW.obj_n.matlabCodegenIsDeleted = true;
    Position_State_Feedback_Moto_DW.obj_n.isInitialized = 0;
    Position_State_Feedback_Moto_DW.obj_n.matlabCodegenIsDeleted = false;
    obj_0 = &Position_State_Feedback_Moto_DW.obj_n;
    Position_State_Feedback_Moto_DW.obj_n.isSetupComplete = false;
    Position_State_Feedback_Moto_DW.obj_n.isInitialized = 1;
    pinname = 2;
    obj_0->MW_DIGITALIO_HANDLE = MW_digitalIO_open(pinname, 1);
    Position_State_Feedback_Moto_DW.obj_n.isSetupComplete = true;
  }
}

/* Model terminate function */
void Position_State_Feedback_Motor_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/Qudrature Encoder1' */
  if (!Position_State_Feedback_Moto_DW.obj.matlabCodegenIsDeleted) {
    Position_State_Feedback_Moto_DW.obj.matlabCodegenIsDeleted = true;
    if ((Position_State_Feedback_Moto_DW.obj.isInitialized == 1) &&
        Position_State_Feedback_Moto_DW.obj.isSetupComplete) {
      /*  Call C-function implementing device termination */
      releaseEncoder();
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Qudrature Encoder1' */
  /* Terminate for MATLABSystem: '<Root>/IN1A1' */
  if (!Position_State_Feedback_Moto_DW.obj_k.matlabCodegenIsDeleted) {
    Position_State_Feedback_Moto_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((Position_State_Feedback_Moto_DW.obj_k.isInitialized == 1) &&
        Position_State_Feedback_Moto_DW.obj_k.isSetupComplete) {
      MW_PWM_Stop(Position_State_Feedback_Moto_DW.obj_k.MW_PWM_HANDLE);
      MW_PWM_Close(Position_State_Feedback_Moto_DW.obj_k.MW_PWM_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/IN1A1' */

  /* Terminate for MATLABSystem: '<Root>/IN2A1' */
  if (!Position_State_Feedback_Moto_DW.obj_l.matlabCodegenIsDeleted) {
    Position_State_Feedback_Moto_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((Position_State_Feedback_Moto_DW.obj_l.isInitialized == 1) &&
        Position_State_Feedback_Moto_DW.obj_l.isSetupComplete) {
      MW_PWM_Stop(Position_State_Feedback_Moto_DW.obj_l.MW_PWM_HANDLE);
      MW_PWM_Close(Position_State_Feedback_Moto_DW.obj_l.MW_PWM_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/IN2A1' */
  /* Terminate for MATLABSystem: '<Root>/EN_A1' */
  if (!Position_State_Feedback_Moto_DW.obj_n.matlabCodegenIsDeleted) {
    Position_State_Feedback_Moto_DW.obj_n.matlabCodegenIsDeleted = true;
    if ((Position_State_Feedback_Moto_DW.obj_n.isInitialized == 1) &&
        Position_State_Feedback_Moto_DW.obj_n.isSetupComplete) {
      MW_digitalIO_close
        (Position_State_Feedback_Moto_DW.obj_n.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/EN_A1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
