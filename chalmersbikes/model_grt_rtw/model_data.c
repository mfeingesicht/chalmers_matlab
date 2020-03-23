/*
 * model_data.c
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

/* Block parameters (default storage) */
P_model_T model_P = {
  /* Variable: A
   * Referenced by:
   *   '<S30>/Constant11'
   *   '<S30>/Constant2'
   */
  { -0.0, 1.0, 18.370786516853933, 0.0 },

  /* Variable: B
   * Referenced by:
   *   '<S30>/Constant10'
   *   '<S30>/Constant3'
   */
  { 1.0, 0.0 },

  /* Variable: C
   * Referenced by: '<S30>/Constant1'
   */
  { 4.3556424035173427, 26.558663007818225 },

  /* Variable: D
   * Referenced by:
   *   '<S30>/Constant5'
   *   '<S30>/Constant6'
   */
  0.027449803049343718,

  /* Variable: IMU_height
   * Referenced by:
   *   '<S29>/Constant3'
   *   '<S24>/Constant4'
   */
  0.215,

  /* Variable: P_steering_speedcontrol
   * Referenced by:
   *   '<S62>/Integral Gain'
   *   '<S70>/Proportional Gain'
   */
  1000.0,

  /* Variable: Ti_steering_speedcontrol
   * Referenced by: '<S62>/Integral Gain'
   */
  1.0,

  /* Variable: Ts
   * Referenced by:
   *   '<S21>/Gain4'
   *   '<S26>/Gain3'
   */
  0.01,

  /* Variable: b
   * Referenced by:
   *   '<S27>/b Real Bike'
   *   '<S23>/Bike Model b'
   */
  1.15,

  /* Variable: bike_params
   * Referenced by:
   *   '<S15>/ Bike Parameters Model '
   *   '<S31>/ Bike Parameters Model '
   */
  { 9.81, 0.534, 1.15, 0.6687, 1.1519173063162575, 31.3 },

  /* Variable: complementary
   * Referenced by:
   *   '<S21>/Gain2'
   *   '<S21>/Gain3'
   */
  0.985,

  /* Variable: g
   * Referenced by:
   *   '<S20>/Gain1'
   *   '<S20>/Gain2'
   */
  9.81,

  /* Variable: g1_den
   * Referenced by: '<S9>/G1'
   */
  { 1.0, 0.0 },

  /* Variable: g1_num
   * Referenced by: '<S9>/G1'
   */
  { 1.0, 0.05 },

  /* Variable: g2_den
   * Referenced by: '<S9>/G2'
   */
  1.0,

  /* Variable: g2_num
   * Referenced by: '<S9>/G2'
   */
  5.0,

  /* Variable: initial_roll
   * Referenced by:
   *   '<S5>/Integrator'
   *   '<S21>/Delay'
   */
  0.034906585039886591,

  /* Variable: initial_x
   * Referenced by: '<S16>/Discrete-Time Integrator'
   */
  0.0,

  /* Variable: initial_y
   * Referenced by: '<S16>/Discrete-Time Integrator1'
   */
  0.0,

  /* Variable: kt
   * Referenced by:
   *   '<S8>/Gain'
   *   '<S8>/Gain1'
   */
  0.0273,

  /* Variable: lambda
   * Referenced by:
   *   '<S3>/Gain1'
   *   '<S27>/Gain'
   *   '<S28>/Gain1'
   *   '<S23>/Gain'
   */
  1.1519173063162575,

  /* Variable: roll_rate_coef
   * Referenced by:
   *   '<S25>/Gain4'
   *   '<S25>/Gain6'
   */
  0.9,

  /* Variable: v
   * Referenced by: '<Root>/Forward Velocity'
   */
  4.0,

  /* Mask Parameter: PIDController_InitialConditionF
   * Referenced by: '<S65>/Integrator'
   */
  0.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainval
   * Referenced by: '<S15>/Discrete-Time Integrator'
   */
  0.01,

  /* Expression: 0
   * Referenced by: '<S15>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainva_o
   * Referenced by: '<S16>/Discrete-Time Integrator'
   */
  0.01,

  /* Computed Parameter: DiscreteTimeIntegrator1_gainval
   * Referenced by: '<S16>/Discrete-Time Integrator1'
   */
  0.01,

  /* Expression: 0.0
   * Referenced by: '<S19>/Delay3'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<S25>/Delay18'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<S26>/Delay18'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<S19>/Delay1'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<S19>/Delay2'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<S19>/Delay'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<S19>/Delay5'
   */
  0.0,

  /* Computed Parameter: TransferFcn1_A
   * Referenced by: '<S8>/Transfer Fcn1'
   */
  -36812.413668152512,

  /* Computed Parameter: TransferFcn1_C
   * Referenced by: '<S8>/Transfer Fcn1'
   */
  36812.413668152512,

  /* Expression: 0
   * Referenced by: '<Root>/True Steering Rate Perturbation'
   */
  0.0,

  /* Expression: pi/180
   * Referenced by: '<S1>/Gain1'
   */
  0.017453292519943295,

  /* Expression: 0
   * Referenced by: '<S31>/Integrator'
   */
  0.0,

  /* Expression: 180/pi
   * Referenced by: '<S10>/Gain'
   */
  57.295779513082323,

  /* Expression: zeros(size(C,1),size(B,2))
   * Referenced by: '<S30>/Constant'
   */
  0.0,

  /* Expression: [C ; C*A]\[initial_rollrate ; 0]
   * Referenced by: '<S30>/Integrator1'
   */
  { -0.0078273550510390248, 0.0025980127308090008 },

  /* Expression: 180/pi
   * Referenced by: '<S12>/Gain'
   */
  57.295779513082323,

  /* Expression: 180/pi
   * Referenced by: '<S11>/Gain'
   */
  57.295779513082323,

  /* Expression: 180/pi
   * Referenced by: '<S13>/Gain'
   */
  57.295779513082323,

  /* Expression: 180/pi
   * Referenced by: '<S14>/Gain'
   */
  57.295779513082323,

  /* Expression: 0
   * Referenced by: '<Root>/Measured Roll Rate Perturbation'
   */
  0.0,

  /* Expression: pi/180
   * Referenced by: '<S2>/Gain1'
   */
  0.017453292519943295,

  /* Expression: 0
   * Referenced by: '<S32>/Integrator1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S32>/Integrator2'
   */
  0.0,

  /* Expression: deg2rad(4)
   * Referenced by: '<Root>/Reference Roll Angle'
   */
  0.069813170079773182,

  /* Computed Parameter: SteeringmotortransferfunctionSt
   * Referenced by: '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate'
   */
  -245.9,

  /* Computed Parameter: Steeringmotortransferfunction_c
   * Referenced by: '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate'
   */
  253.4,

  /* Computed Parameter: TransferFcn_A
   * Referenced by: '<S8>/Transfer Fcn'
   */
  -3176.4705882352932,

  /* Computed Parameter: TransferFcn_C
   * Referenced by: '<S8>/Transfer Fcn'
   */
  29411.764705882346,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S65>/Integrator'
   */
  0.01
};
