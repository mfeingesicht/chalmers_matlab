/*
 * model.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "model".
 *
 * Model version              : 1.365
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon Mar 16 10:21:16 2020
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "model.h"
#include "model_private.h"

/* Block signals (default storage) */
B_model_T model_B;

/* Continuous states */
X_model_T model_X;

/* Block states (default storage) */
DW_model_T model_DW;

/* Real-time model */
RT_MODEL_model_T model_M_;
RT_MODEL_model_T *const model_M = &model_M_;
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (model_M->Timing.TaskCounters.TID[2])++;
  if ((model_M->Timing.TaskCounters.TID[2]) > 99) {/* Sample time: [0.01s, 0.0s] */
    model_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 10;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  model_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  model_step();
  model_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  model_step();
  model_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * Output and update for atomic system:
 *    '<S16>/MATLAB Function1'
 *    '<S32>/MATLAB Function1'
 */
void model_MATLABFunction1(real_T rtu_v, real_T rtu_psi, real_T rtu_beta,
  B_MATLABFunction1_model_T *localB)
{
  real_T Xdot_tmp;
  Xdot_tmp = rtu_psi + rtu_beta;
  localB->Xdot = cos(Xdot_tmp) * rtu_v;
  localB->Ydot = sin(Xdot_tmp) * rtu_v;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void model_step(void)
{
  /* local block i/o variables */
  real_T rtb_Integrator;
  real_T rtb_beta;
  real_T rtb_FilteredRollRate;
  real_T rtb_TmpSignalConversionAtDelay5[3];
  real_T rtb_IntegralGain;
  real_T rtb_DiscreteTimeIntegrator;
  real_T rtb_beta_l;
  if (rtmIsMajorTimeStep(model_M)) {
    /* set solver stop time */
    if (!(model_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&model_M->solverInfo, ((model_M->Timing.clockTickH0
        + 1) * model_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&model_M->solverInfo, ((model_M->Timing.clockTick0 +
        1) * model_M->Timing.stepSize0 + model_M->Timing.clockTickH0 *
        model_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(model_M)) {
    model_M->Timing.t[0] = rtsiGetT(&model_M->solverInfo);
  }

  {
    real_T *lastU;
    real_T rtb_Multiply_m;
    real_T rtb_TransferFcn1;
    real_T rtb_Sum6;
    real_T rtb_Gain1_m;
    real_T rtb_G2;
    real_T rtb_IMUGyroscope;
    real_T rtb_Divide;
    real_T rtb_Sum1;
    real_T rtb_Delay;
    real_T rtb_Multiply_n_tmp;
    real_T psi_d_tmp;

    /* TransferFcn: '<S8>/Transfer Fcn1' */
    rtb_TransferFcn1 = model_P.TransferFcn1_C * model_X.TransferFcn1_CSTATE;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Gain: '<S1>/Gain1' incorporates:
       *  Constant: '<Root>/True Steering Rate Perturbation'
       */
      model_B.Gain1 = model_P.Gain1_Gain *
        model_P.TrueSteeringRatePerturbation_Va;
    }

    /* Sum: '<Root>/Sum' */
    model_B.Sum = rtb_TransferFcn1 + model_B.Gain1;

    /* Outputs for Atomic SubSystem: '<Root>/IMU + Complementary Filter' */
    /* Outputs for Atomic SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */
    /* Gain: '<S28>/Gain1' incorporates:
     *  Gain: '<S23>/Gain'
     *  Gain: '<S3>/Gain1'
     */
    rtb_G2 = sin(model_P.lambda);

    /* End of Outputs for SubSystem: '<Root>/IMU + Complementary Filter' */

    /* MATLAB Function: '<S31>/MATLAB Function' incorporates:
     *  Constant: '<Root>/Forward Velocity'
     *  Constant: '<S31>/ Bike Parameters Model '
     *  Gain: '<S28>/Gain1'
     *  MATLAB Function: '<S15>/MATLAB Function'
     */
    rtb_Multiply_n_tmp = model_P.bike_params[3] / model_P.bike_params[2];

    /* End of Outputs for SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */
    rtb_Multiply_m = atan(rtb_Multiply_n_tmp * tan(rtb_G2 * model_B.Sum));

    /* Outputs for Atomic SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */
    psi_d_tmp = model_P.v / model_P.bike_params[3];

    /* End of Outputs for SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */
    model_B.psi_d = psi_d_tmp * sin(rtb_Multiply_m);
    rtb_beta = rtb_Multiply_m;

    /* Integrator: '<S31>/Integrator' */
    rtb_Integrator = model_X.Integrator_CSTATE;

    /* Gain: '<S10>/Gain' incorporates:
     *  Sum: '<S28>/Sum'
     */
    model_B.Gain = (rtb_beta + rtb_Integrator) * model_P.Gain_Gain;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Sum: '<S30>/Sum3' incorporates:
       *  Constant: '<S30>/Constant'
       *  Constant: '<S30>/Constant5'
       */
      model_B.DCBD[0] = model_P.D + model_P.Constant_Value;

      /* Product: '<S30>/Product8' incorporates:
       *  Constant: '<S30>/Constant1'
       *  Constant: '<S30>/Constant3'
       */
      rtb_Sum6 = model_P.C[0] * model_P.B[0] + model_P.C[1] * model_P.B[1];

      /* Sum: '<S30>/Sum4' incorporates:
       *  Constant: '<S30>/Constant6'
       *  Product: '<S30>/Product8'
       */
      model_B.DCBD[1] = rtb_Sum6 + model_P.D;

      /* Concatenate: '<S30>/Matrix Concatenate' incorporates:
       *  Constant: '<S30>/Constant1'
       *  Constant: '<S30>/Constant2'
       *  Product: '<S30>/Product1'
       */
      model_B.CCA[0] = model_P.C[0];
      model_B.CCA[1] = model_P.C[0] * model_P.A[0] + model_P.C[1] * model_P.A[1];
      model_B.CCA[2] = model_P.C[1];
      model_B.CCA[3] = model_P.C[0] * model_P.A[2] + model_P.C[1] * model_P.A[3];
    }

    /* Sum: '<S30>/Sum' incorporates:
     *  Integrator: '<S30>/Integrator1'
     *  Product: '<S30>/Product6'
     *  Product: '<S30>/Product7'
     */
    model_B.Sum_g[0] = (model_B.CCA[0] * model_X.Integrator1_CSTATE[0] +
                        model_B.CCA[2] * model_X.Integrator1_CSTATE[1]) +
      model_B.DCBD[0] * model_B.Sum;
    model_B.Sum_g[1] = (model_B.CCA[1] * model_X.Integrator1_CSTATE[0] +
                        model_B.CCA[3] * model_X.Integrator1_CSTATE[1]) +
      model_B.DCBD[1] * model_B.Sum;

    /* Gain: '<S12>/Gain' */
    model_B.Gain_i = model_P.Gain_Gain_b * model_B.Sum_g[0];
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
    }

    /* Gain: '<S11>/Gain' incorporates:
     *  Integrator: '<S5>/Integrator'
     */
    model_B.Gain_c = model_P.Gain_Gain_c * model_X.Integrator_CSTATE_b;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
    }

    /* Gain: '<S13>/Gain' */
    model_B.Gain_b = model_P.Gain_Gain_o * model_B.Sum;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
    }

    /* Gain: '<S14>/Gain' */
    model_B.Gain_n = model_P.Gain_Gain_g * model_B.Sum;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Gain: '<S2>/Gain1' incorporates:
       *  Constant: '<Root>/Measured Roll Rate Perturbation'
       */
      rtb_Gain1_m = model_P.Gain1_Gain_j *
        model_P.MeasuredRollRatePerturbation_Va;
    }

    /* Outputs for Atomic SubSystem: '<Root>/Steering Encoder' */
    /* ZeroOrderHold: '<S6>/Zero-Order Hold' */
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      model_B.ZeroOrderHold = model_B.Sum;

      /* Outputs for Atomic SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */
      /* DiscreteIntegrator: '<S15>/Discrete-Time Integrator' */
      rtb_DiscreteTimeIntegrator = model_DW.DiscreteTimeIntegrator_DSTATE;

      /* MATLAB Function: '<S15>/MATLAB Function' incorporates:
       *  Gain: '<S3>/Gain1'
       */
      rtb_Multiply_m = rtb_Multiply_n_tmp * (rtb_G2 * model_B.ZeroOrderHold);
      rtb_beta_l = rtb_Multiply_m;

      /* MATLAB Function: '<S16>/MATLAB Function1' incorporates:
       *  Constant: '<Root>/Forward Velocity'
       */
      model_MATLABFunction1(model_P.v, rtb_DiscreteTimeIntegrator, rtb_beta_l,
                            &model_B.sf_MATLABFunction1_k);

      /* Update for DiscreteIntegrator: '<S15>/Discrete-Time Integrator' incorporates:
       *  MATLAB Function: '<S15>/MATLAB Function'
       */
      model_DW.DiscreteTimeIntegrator_DSTATE += psi_d_tmp * rtb_Multiply_m *
        model_P.DiscreteTimeIntegrator_gainval;

      /* Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator' */
      model_DW.DiscreteTimeIntegrator_DSTATE_o +=
        model_P.DiscreteTimeIntegrator_gainva_o *
        model_B.sf_MATLABFunction1_k.Xdot;

      /* Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' */
      model_DW.DiscreteTimeIntegrator1_DSTATE +=
        model_P.DiscreteTimeIntegrator1_gainval *
        model_B.sf_MATLABFunction1_k.Ydot;

      /* End of Outputs for SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */
    }

    /* End of ZeroOrderHold: '<S6>/Zero-Order Hold' */
    /* End of Outputs for SubSystem: '<Root>/Steering Encoder' */
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Product: '<S27>/Divide' incorporates:
       *  Constant: '<Root>/Forward Velocity'
       *  Constant: '<S27>/b Real Bike'
       *  Math: '<S27>/Square'
       */
      model_B.Divide = model_P.v * model_P.v / model_P.b;
    }

    /* Product: '<S27>/Multiply' incorporates:
     *  Gain: '<S27>/Gain'
     *  Trigonometry: '<S27>/Tan'
     */
    rtb_Multiply_m = tan(sin(model_P.lambda) * model_B.Sum) * model_B.Divide;

    /* Outputs for Atomic SubSystem: '<Root>/IMU + Complementary Filter' */
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Trigonometry: '<S23>/Tan' incorporates:
       *  Delay: '<S19>/Delay2'
       *  Gain: '<S23>/Gain'
       */
      model_B.Tan = tan(rtb_G2 * model_DW.Delay2_DSTATE);
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Delay: '<S19>/Delay3' */
      model_B.Delay3 = model_DW.Delay3_DSTATE;

      /* Sum: '<S25>/Sum4' incorporates:
       *  Delay: '<S25>/Delay18'
       *  Gain: '<S25>/Gain4'
       *  Gain: '<S25>/Gain6'
       */
      rtb_FilteredRollRate = (1.0 - model_P.roll_rate_coef) * model_B.Delay3 +
        model_P.roll_rate_coef * model_DW.Delay18_DSTATE;

      /* Product: '<S24>/Multiply1' incorporates:
       *  Constant: '<S24>/Constant4'
       *  Delay: '<S26>/Delay18'
       *  Gain: '<S26>/Gain3'
       *  Sum: '<S26>/Sum3'
       */
      rtb_Sum6 = 1.0 / model_P.Ts * (rtb_FilteredRollRate -
        model_DW.Delay18_DSTATE_a) * model_P.IMU_height;

      /* Delay: '<S19>/Delay1' */
      rtb_Sum1 = model_DW.Delay1_DSTATE;

      /* Product: '<S23>/Divide' incorporates:
       *  Constant: '<S23>/Bike Model b'
       *  Delay: '<S19>/Delay1'
       *  Math: '<S23>/Square'
       */
      rtb_Divide = model_DW.Delay1_DSTATE * model_DW.Delay1_DSTATE / model_P.b;
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Delay: '<S19>/Delay' */
      rtb_Delay = model_DW.Delay_DSTATE;

      /* Trigonometry: '<S22>/Cos' */
      model_B.Cos = cos(rtb_Delay);
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Product: '<S23>/Multiply' */
      rtb_Sum1 = rtb_Divide * model_B.Tan;
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Trigonometry: '<S22>/Sin' */
      model_B.Sin = sin(rtb_Delay);
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Delay: '<S19>/Delay5' */
      rtb_G2 = model_DW.Delay5_DSTATE[2];

      /* Sum: '<S22>/Sum6' incorporates:
       *  Delay: '<S19>/Delay5'
       *  Product: '<S22>/Multiply1'
       */
      rtb_Sum6 = (rtb_Sum6 - rtb_Sum1 * model_B.Cos) + model_DW.Delay5_DSTATE[1];

      /* Product: '<S22>/Multiply' */
      rtb_Sum1 *= model_B.Sin;

      /* Sum: '<S22>/Sum1' */
      rtb_Sum1 = rtb_G2 - rtb_Sum1;

      /* Trigonometry: '<S22>/Atan2' */
      model_B.Roll_acc = rt_atan2d_snf(rtb_Sum6, rtb_Sum1);
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Outputs for Atomic SubSystem: '<S19>/Complementary Filter' */
      /* Sum: '<S21>/Sum4' incorporates:
       *  Delay: '<S21>/Delay'
       *  Gain: '<S21>/Gain2'
       *  Gain: '<S21>/Gain3'
       *  Gain: '<S21>/Gain4'
       *  Sum: '<S21>/Sum3'
       */
      model_B.Sum4 = (model_P.Ts * model_B.Delay3 + model_DW.Delay_DSTATE_a) *
        model_P.complementary + (1.0 - model_P.complementary) * model_B.Roll_acc;

      /* Update for Delay: '<S21>/Delay' */
      model_DW.Delay_DSTATE_a = model_B.Sum4;

      /* End of Outputs for SubSystem: '<S19>/Complementary Filter' */
    }

    /* Derivative: '<S20>/Derivative' incorporates:
     *  Constant: '<Root>/Forward Velocity'
     */
    rtb_G2 = model_M->Timing.t[0];
    if ((model_DW.TimeStampA >= rtb_G2) && (model_DW.TimeStampB >= rtb_G2)) {
      model_B.acc_x = 0.0;
    } else {
      rtb_Sum6 = model_DW.TimeStampA;
      lastU = &model_DW.LastUAtTimeA;
      if (model_DW.TimeStampA < model_DW.TimeStampB) {
        if (model_DW.TimeStampB < rtb_G2) {
          rtb_Sum6 = model_DW.TimeStampB;
          lastU = &model_DW.LastUAtTimeB;
        }
      } else {
        if (model_DW.TimeStampA >= rtb_G2) {
          rtb_Sum6 = model_DW.TimeStampB;
          lastU = &model_DW.LastUAtTimeB;
        }
      }

      model_B.acc_x = (model_P.v - *lastU) / (rtb_G2 - rtb_Sum6);
    }

    /* End of Derivative: '<S20>/Derivative' */

    /* Trigonometry: '<S20>/Cos' incorporates:
     *  Integrator: '<S5>/Integrator'
     */
    rtb_G2 = cos(model_X.Integrator_CSTATE_b);

    /* Trigonometry: '<S20>/Sin' incorporates:
     *  Integrator: '<S5>/Integrator'
     */
    rtb_Sum6 = sin(model_X.Integrator_CSTATE_b);

    /* Sum: '<S20>/Sum6' incorporates:
     *  Constant: '<S29>/Constant3'
     *  Gain: '<S20>/Gain1'
     *  Product: '<S20>/Multiply'
     *  Product: '<S29>/Multiply'
     */
    model_B.Sum6_m = (rtb_Multiply_m * rtb_G2 - model_P.IMU_height *
                      model_B.Sum_g[1]) + model_P.g * rtb_Sum6;

    /* Sum: '<S20>/Sum1' incorporates:
     *  Gain: '<S20>/Gain2'
     *  Product: '<S20>/Multiply1'
     */
    model_B.Sum1_m = model_P.g * rtb_G2 - rtb_Multiply_m * rtb_Sum6;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* SignalConversion generated from: '<S19>/Delay5' */
      rtb_TmpSignalConversionAtDelay5[0] = model_B.acc_x;
      rtb_TmpSignalConversionAtDelay5[1] = model_B.Sum6_m;
      rtb_TmpSignalConversionAtDelay5[2] = model_B.Sum1_m;

      /* ZeroOrderHold: '<S4>/IMU Gyroscope' */
      rtb_IMUGyroscope = model_B.Sum_g[0];
    }

    /* End of Outputs for SubSystem: '<Root>/IMU + Complementary Filter' */

    /* MATLAB Function: '<S32>/MATLAB Function1' incorporates:
     *  Constant: '<Root>/Forward Velocity'
     */
    model_MATLABFunction1(model_P.v, rtb_Integrator, rtb_beta,
                          &model_B.sf_MATLABFunction1);

    /* Sum: '<S30>/Sum2' incorporates:
     *  Constant: '<S30>/Constant10'
     *  Constant: '<S30>/Constant11'
     *  Integrator: '<S30>/Integrator1'
     *  Product: '<S30>/Product2'
     *  Product: '<S30>/Product5'
     */
    model_B.xdot[0] = (model_P.A[0] * model_X.Integrator1_CSTATE[0] + model_P.A
                       [2] * model_X.Integrator1_CSTATE[1]) + model_P.B[0] *
      model_B.Sum;
    model_B.xdot[1] = (model_P.A[1] * model_X.Integrator1_CSTATE[0] + model_P.A
                       [3] * model_X.Integrator1_CSTATE[1]) + model_P.B[1] *
      model_B.Sum;

    /* Gain: '<S8>/Gain' incorporates:
     *  TransferFcn: '<S8>/Transfer Fcn'
     */
    model_B.T = model_P.TransferFcn_C * model_X.TransferFcn_CSTATE * model_P.kt;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Sum: '<S9>/Sum6' incorporates:
       *  Constant: '<Root>/Reference Roll Angle'
       */
      model_B.Sum6 = model_P.ReferenceRollAngle_Value - model_B.Sum4;
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Sum: '<Root>/Sum5' */
      model_B.Sum5 = rtb_IMUGyroscope + rtb_Gain1_m;
    }

    /* TransferFcn: '<S9>/G1' */
    rtb_G2 = model_P.g1_num[0] / model_P.g1_den[0];

    /* Sum: '<Root>/Sum1' incorporates:
     *  Sum: '<S9>/Sum4'
     *  TransferFcn: '<S9>/G1'
     *  TransferFcn: '<S9>/G2'
     */
    model_B.Sum1 = (((model_P.g1_num[1] / model_P.g1_den[0] - rtb_G2 *
                      (model_P.g1_den[1] / model_P.g1_den[0])) *
                     model_X.G1_CSTATE + rtb_G2 * model_B.Sum6) - model_B.Sum5) *
      (model_P.g2_num / model_P.g2_den);

    /* Sum: '<S8>/Sum' */
    model_B.Sum_h = model_B.Sum1 - rtb_TransferFcn1;
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Gain: '<S62>/Integral Gain' */
      rtb_IntegralGain = model_P.P_steering_speedcontrol /
        model_P.Ti_steering_speedcontrol * model_B.Sum_h;

      /* Sum: '<S74>/Sum' incorporates:
       *  DiscreteIntegrator: '<S65>/Integrator'
       *  Gain: '<S70>/Proportional Gain'
       */
      model_B.Sum_d = model_P.P_steering_speedcontrol * model_B.Sum_h +
        model_DW.Integrator_DSTATE;
    }

    /* Sum: '<S8>/Sum1' incorporates:
     *  Gain: '<S8>/Gain1'
     */
    model_B.Sum1_d = model_B.Sum_d - model_P.kt * rtb_TransferFcn1;
  }

  if (rtmIsMajorTimeStep(model_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(model_M->rtwLogInfo, (model_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(model_M)) {
    real_T *lastU;

    /* Update for Atomic SubSystem: '<Root>/IMU + Complementary Filter' */
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[1] == 0) {
      /* Update for Delay: '<S19>/Delay3' */
      model_DW.Delay3_DSTATE = model_B.Sum_g[0];

      /* Update for Delay: '<S25>/Delay18' */
      model_DW.Delay18_DSTATE = rtb_FilteredRollRate;

      /* Update for Delay: '<S26>/Delay18' */
      model_DW.Delay18_DSTATE_a = rtb_FilteredRollRate;

      /* Update for Delay: '<S19>/Delay1' incorporates:
       *  Constant: '<Root>/Forward Velocity'
       */
      model_DW.Delay1_DSTATE = model_P.v;

      /* Update for Delay: '<S19>/Delay5' */
      model_DW.Delay5_DSTATE[0] = rtb_TmpSignalConversionAtDelay5[0];
      model_DW.Delay5_DSTATE[1] = rtb_TmpSignalConversionAtDelay5[1];
      model_DW.Delay5_DSTATE[2] = rtb_TmpSignalConversionAtDelay5[2];
    }

    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Update for Delay: '<S19>/Delay2' */
      model_DW.Delay2_DSTATE = model_B.ZeroOrderHold;

      /* Update for Delay: '<S19>/Delay' */
      model_DW.Delay_DSTATE = model_B.Sum4;
    }

    /* Update for Derivative: '<S20>/Derivative' incorporates:
     *  Constant: '<Root>/Forward Velocity'
     */
    if (model_DW.TimeStampA == (rtInf)) {
      model_DW.TimeStampA = model_M->Timing.t[0];
      lastU = &model_DW.LastUAtTimeA;
    } else if (model_DW.TimeStampB == (rtInf)) {
      model_DW.TimeStampB = model_M->Timing.t[0];
      lastU = &model_DW.LastUAtTimeB;
    } else if (model_DW.TimeStampA < model_DW.TimeStampB) {
      model_DW.TimeStampA = model_M->Timing.t[0];
      lastU = &model_DW.LastUAtTimeA;
    } else {
      model_DW.TimeStampB = model_M->Timing.t[0];
      lastU = &model_DW.LastUAtTimeB;
    }

    *lastU = model_P.v;

    /* End of Update for Derivative: '<S20>/Derivative' */
    /* End of Update for SubSystem: '<Root>/IMU + Complementary Filter' */
    if (rtmIsMajorTimeStep(model_M) &&
        model_M->Timing.TaskCounters.TID[2] == 0) {
      /* Update for DiscreteIntegrator: '<S65>/Integrator' */
      model_DW.Integrator_DSTATE += model_P.Integrator_gainval *
        rtb_IntegralGain;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(model_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(model_M)!=-1) &&
          !((rtmGetTFinal(model_M)-(((model_M->Timing.clockTick1+
               model_M->Timing.clockTickH1* 4294967296.0)) * 0.0001)) >
            (((model_M->Timing.clockTick1+model_M->Timing.clockTickH1*
               4294967296.0)) * 0.0001) * (DBL_EPSILON))) {
        rtmSetErrorStatus(model_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&model_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++model_M->Timing.clockTick0)) {
      ++model_M->Timing.clockTickH0;
    }

    model_M->Timing.t[0] = rtsiGetSolverStopTime(&model_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.0001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.0001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      model_M->Timing.clockTick1++;
      if (!model_M->Timing.clockTick1) {
        model_M->Timing.clockTickH1++;
      }
    }

    rate_scheduler();
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void model_derivatives(void)
{
  XDot_model_T *_rtXdot;
  _rtXdot = ((XDot_model_T *) model_M->derivs);

  /* Derivatives for TransferFcn: '<S8>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += model_P.TransferFcn1_A *
    model_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += model_B.T;

  /* Derivatives for Integrator: '<S31>/Integrator' */
  _rtXdot->Integrator_CSTATE = model_B.psi_d;

  /* Derivatives for Integrator: '<S30>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[0] = model_B.xdot[0];
  _rtXdot->Integrator1_CSTATE[1] = model_B.xdot[1];

  /* Derivatives for Integrator: '<S5>/Integrator' */
  _rtXdot->Integrator_CSTATE_b = model_B.Sum_g[0];

  /* Derivatives for Integrator: '<S32>/Integrator1' */
  _rtXdot->Integrator1_CSTATE_h = model_B.sf_MATLABFunction1.Xdot;

  /* Derivatives for Integrator: '<S32>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = model_B.sf_MATLABFunction1.Ydot;

  /* Derivatives for TransferFcn: '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate' */
  _rtXdot->SteeringmotortransferfunctionSt = 0.0;
  _rtXdot->SteeringmotortransferfunctionSt +=
    model_P.SteeringmotortransferfunctionSt *
    model_X.SteeringmotortransferfunctionSt;
  _rtXdot->SteeringmotortransferfunctionSt += model_B.Sum1;

  /* Derivatives for TransferFcn: '<S8>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += model_P.TransferFcn_A *
    model_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += model_B.Sum1_d;

  /* Derivatives for TransferFcn: '<S9>/G1' */
  _rtXdot->G1_CSTATE = 0.0;
  _rtXdot->G1_CSTATE += -model_P.g1_den[1] / model_P.g1_den[0] *
    model_X.G1_CSTATE;
  _rtXdot->G1_CSTATE += model_B.Sum6;
}

/* Model initialize function */
void model_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)model_M, 0,
                sizeof(RT_MODEL_model_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&model_M->solverInfo, &model_M->Timing.simTimeStep);
    rtsiSetTPtr(&model_M->solverInfo, &rtmGetTPtr(model_M));
    rtsiSetStepSizePtr(&model_M->solverInfo, &model_M->Timing.stepSize0);
    rtsiSetdXPtr(&model_M->solverInfo, &model_M->derivs);
    rtsiSetContStatesPtr(&model_M->solverInfo, (real_T **) &model_M->contStates);
    rtsiSetNumContStatesPtr(&model_M->solverInfo, &model_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&model_M->solverInfo,
      &model_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&model_M->solverInfo,
      &model_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&model_M->solverInfo,
      &model_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&model_M->solverInfo, (&rtmGetErrorStatus(model_M)));
    rtsiSetRTModelPtr(&model_M->solverInfo, model_M);
  }

  rtsiSetSimTimeStep(&model_M->solverInfo, MAJOR_TIME_STEP);
  model_M->intgData.y = model_M->odeY;
  model_M->intgData.f[0] = model_M->odeF[0];
  model_M->intgData.f[1] = model_M->odeF[1];
  model_M->intgData.f[2] = model_M->odeF[2];
  model_M->contStates = ((X_model_T *) &model_X);
  rtsiSetSolverData(&model_M->solverInfo, (void *)&model_M->intgData);
  rtsiSetSolverName(&model_M->solverInfo,"ode3");
  rtmSetTPtr(model_M, &model_M->Timing.tArray[0]);
  rtmSetTFinal(model_M, 20.0);
  model_M->Timing.stepSize0 = 0.0001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    model_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(model_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(model_M->rtwLogInfo, (NULL));
    rtliSetLogT(model_M->rtwLogInfo, "tout");
    rtliSetLogX(model_M->rtwLogInfo, "");
    rtliSetLogXFinal(model_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(model_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(model_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(model_M->rtwLogInfo, 0);
    rtliSetLogDecimation(model_M->rtwLogInfo, 1);
    rtliSetLogY(model_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(model_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(model_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &model_B), 0,
                sizeof(B_model_T));

  /* states (continuous) */
  {
    (void) memset((void *)&model_X, 0,
                  sizeof(X_model_T));
  }

  /* states (dwork) */
  (void) memset((void *)&model_DW, 0,
                sizeof(DW_model_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(model_M->rtwLogInfo, 0.0, rtmGetTFinal
    (model_M), model_M->Timing.stepSize0, (&rtmGetErrorStatus(model_M)));

  /* InitializeConditions for TransferFcn: '<S8>/Transfer Fcn1' */
  model_X.TransferFcn1_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S31>/Integrator' */
  model_X.Integrator_CSTATE = model_P.Integrator_IC;

  /* InitializeConditions for Integrator: '<S30>/Integrator1' */
  model_X.Integrator1_CSTATE[0] = model_P.Integrator1_IC[0];
  model_X.Integrator1_CSTATE[1] = model_P.Integrator1_IC[1];

  /* InitializeConditions for Integrator: '<S5>/Integrator' */
  model_X.Integrator_CSTATE_b = model_P.initial_roll;

  /* InitializeConditions for Integrator: '<S32>/Integrator1' */
  model_X.Integrator1_CSTATE_h = model_P.Integrator1_IC_e;

  /* InitializeConditions for Integrator: '<S32>/Integrator2' */
  model_X.Integrator2_CSTATE = model_P.Integrator2_IC;

  /* InitializeConditions for TransferFcn: '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate' */
  model_X.SteeringmotortransferfunctionSt = 0.0;

  /* InitializeConditions for TransferFcn: '<S8>/Transfer Fcn' */
  model_X.TransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S9>/G1' */
  model_X.G1_CSTATE = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S65>/Integrator' */
  model_DW.Integrator_DSTATE = model_P.PIDController_InitialConditionF;

  /* SystemInitialize for Atomic SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */
  /* InitializeConditions for DiscreteIntegrator: '<S15>/Discrete-Time Integrator' */
  model_DW.DiscreteTimeIntegrator_DSTATE = model_P.DiscreteTimeIntegrator_IC;

  /* InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator' */
  model_DW.DiscreteTimeIntegrator_DSTATE_o = model_P.initial_x;

  /* InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' */
  model_DW.DiscreteTimeIntegrator1_DSTATE = model_P.initial_y;

  /* End of SystemInitialize for SubSystem: '<Root>/Global Angles and Coordinates Calculator - Position' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/IMU + Complementary Filter' */
  /* InitializeConditions for Delay: '<S19>/Delay3' */
  model_DW.Delay3_DSTATE = model_P.Delay3_InitialCondition;

  /* InitializeConditions for Delay: '<S25>/Delay18' */
  model_DW.Delay18_DSTATE = model_P.Delay18_InitialCondition;

  /* InitializeConditions for Delay: '<S26>/Delay18' */
  model_DW.Delay18_DSTATE_a = model_P.Delay18_InitialCondition_h;

  /* InitializeConditions for Delay: '<S19>/Delay1' */
  model_DW.Delay1_DSTATE = model_P.Delay1_InitialCondition;

  /* InitializeConditions for Delay: '<S19>/Delay2' */
  model_DW.Delay2_DSTATE = model_P.Delay2_InitialCondition;

  /* InitializeConditions for Delay: '<S19>/Delay' */
  model_DW.Delay_DSTATE = model_P.Delay_InitialCondition;

  /* InitializeConditions for Delay: '<S19>/Delay5' */
  model_DW.Delay5_DSTATE[0] = model_P.Delay5_InitialCondition;
  model_DW.Delay5_DSTATE[1] = model_P.Delay5_InitialCondition;
  model_DW.Delay5_DSTATE[2] = model_P.Delay5_InitialCondition;

  /* InitializeConditions for Derivative: '<S20>/Derivative' */
  model_DW.TimeStampA = (rtInf);
  model_DW.TimeStampB = (rtInf);

  /* SystemInitialize for Atomic SubSystem: '<S19>/Complementary Filter' */
  /* InitializeConditions for Delay: '<S21>/Delay' */
  model_DW.Delay_DSTATE_a = model_P.initial_roll;

  /* End of SystemInitialize for SubSystem: '<S19>/Complementary Filter' */
  /* End of SystemInitialize for SubSystem: '<Root>/IMU + Complementary Filter' */
}

/* Model terminate function */
void model_terminate(void)
{
  /* (no terminate code required) */
}
