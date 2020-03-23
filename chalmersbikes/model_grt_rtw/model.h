/*
 * model.h
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

#ifndef RTW_HEADER_model_h_
#define RTW_HEADER_model_h_
#include <math.h>
#include <float.h>
#include <string.h>
#include <stddef.h>
#ifndef model_COMMON_INCLUDES_
# define model_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* model_COMMON_INCLUDES_ */

#include "model_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals for system '<S16>/MATLAB Function1' */
typedef struct {
  real_T Xdot;                         /* '<S16>/MATLAB Function1' */
  real_T Ydot;                         /* '<S16>/MATLAB Function1' */
} B_MATLABFunction1_model_T;

/* Block signals (default storage) */
typedef struct {
  real_T Gain1;                        /* '<S1>/Gain1' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T Gain;                         /* '<S10>/Gain' */
  real_T DCBD[2];                      /* '<S30>/Matrix Concatenate1' */
  real_T CCA[4];                       /* '<S30>/Matrix Concatenate' */
  real_T Sum_g[2];                     /* '<S30>/Sum' */
  real_T Gain_i;                       /* '<S12>/Gain' */
  real_T Gain_c;                       /* '<S11>/Gain' */
  real_T Gain_b;                       /* '<S13>/Gain' */
  real_T Gain_n;                       /* '<S14>/Gain' */
  real_T Divide;                       /* '<S27>/Divide' */
  real_T xdot[2];                      /* '<S30>/Sum2' */
  real_T T;                            /* '<S8>/Gain' */
  real_T Sum6;                         /* '<S9>/Sum6' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T Sum1;                         /* '<Root>/Sum1' */
  real_T Sum_h;                        /* '<S8>/Sum' */
  real_T Sum_d;                        /* '<S74>/Sum' */
  real_T Sum1_d;                       /* '<S8>/Sum1' */
  real_T ZeroOrderHold;                /* '<S6>/Zero-Order Hold' */
  real_T psi_d;                        /* '<S31>/MATLAB Function' */
  real_T Delay3;                       /* '<S19>/Delay3' */
  real_T Tan;                          /* '<S23>/Tan' */
  real_T Cos;                          /* '<S22>/Cos' */
  real_T Sin;                          /* '<S22>/Sin' */
  real_T Roll_acc;                     /* '<S22>/Atan2' */
  real_T acc_x;                        /* '<S20>/Derivative' */
  real_T Sum6_m;                       /* '<S20>/Sum6' */
  real_T Sum1_m;                       /* '<S20>/Sum1' */
  real_T Sum4;                         /* '<S21>/Sum4' */
  B_MATLABFunction1_model_T sf_MATLABFunction1;/* '<S32>/MATLAB Function1' */
  B_MATLABFunction1_model_T sf_MATLABFunction1_k;/* '<S16>/MATLAB Function1' */
} B_model_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S65>/Integrator' */
  real_T Delay3_DSTATE;                /* '<S19>/Delay3' */
  real_T Delay18_DSTATE;               /* '<S25>/Delay18' */
  real_T Delay18_DSTATE_a;             /* '<S26>/Delay18' */
  real_T Delay1_DSTATE;                /* '<S19>/Delay1' */
  real_T Delay2_DSTATE;                /* '<S19>/Delay2' */
  real_T Delay_DSTATE;                 /* '<S19>/Delay' */
  real_T Delay5_DSTATE[3];             /* '<S19>/Delay5' */
  real_T Delay_DSTATE_a;               /* '<S21>/Delay' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S15>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_o;/* '<S16>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTATE;/* '<S16>/Discrete-Time Integrator1' */
  real_T TimeStampA;                   /* '<S20>/Derivative' */
  real_T LastUAtTimeA;                 /* '<S20>/Derivative' */
  real_T TimeStampB;                   /* '<S20>/Derivative' */
  real_T LastUAtTimeB;                 /* '<S20>/Derivative' */
  struct {
    void *LoggedData;
  } TrueHeadingScope_PWORK;            /* '<Root>/True Heading Scope' */

  struct {
    void *LoggedData;
  } TrueRollRateScope_PWORK;           /* '<Root>/True Roll Rate Scope' */

  struct {
    void *LoggedData;
  } TrueRollScope_PWORK;               /* '<Root>/True Roll Scope' */

  struct {
    void *LoggedData;
  } TrueSteeringAngleScope_PWORK;      /* '<Root>/True Steering Angle Scope' */

  struct {
    void *LoggedData;
  } TrueSteeringRateScope_PWORK;       /* '<Root>/True Steering Rate Scope' */
} DW_model_T;

/* Continuous states (default storage) */
typedef struct {
  real_T TransferFcn1_CSTATE;          /* '<S8>/Transfer Fcn1' */
  real_T Integrator_CSTATE;            /* '<S31>/Integrator' */
  real_T Integrator1_CSTATE[2];        /* '<S30>/Integrator1' */
  real_T Integrator_CSTATE_b;          /* '<S5>/Integrator' */
  real_T Integrator1_CSTATE_h;         /* '<S32>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S32>/Integrator2' */
  real_T SteeringmotortransferfunctionSt;
  /* '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate' */
  real_T TransferFcn_CSTATE;           /* '<S8>/Transfer Fcn' */
  real_T G1_CSTATE;                    /* '<S9>/G1' */
} X_model_T;

/* State derivatives (default storage) */
typedef struct {
  real_T TransferFcn1_CSTATE;          /* '<S8>/Transfer Fcn1' */
  real_T Integrator_CSTATE;            /* '<S31>/Integrator' */
  real_T Integrator1_CSTATE[2];        /* '<S30>/Integrator1' */
  real_T Integrator_CSTATE_b;          /* '<S5>/Integrator' */
  real_T Integrator1_CSTATE_h;         /* '<S32>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S32>/Integrator2' */
  real_T SteeringmotortransferfunctionSt;
  /* '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate' */
  real_T TransferFcn_CSTATE;           /* '<S8>/Transfer Fcn' */
  real_T G1_CSTATE;                    /* '<S9>/G1' */
} XDot_model_T;

/* State disabled  */
typedef struct {
  boolean_T TransferFcn1_CSTATE;       /* '<S8>/Transfer Fcn1' */
  boolean_T Integrator_CSTATE;         /* '<S31>/Integrator' */
  boolean_T Integrator1_CSTATE[2];     /* '<S30>/Integrator1' */
  boolean_T Integrator_CSTATE_b;       /* '<S5>/Integrator' */
  boolean_T Integrator1_CSTATE_h;      /* '<S32>/Integrator1' */
  boolean_T Integrator2_CSTATE;        /* '<S32>/Integrator2' */
  boolean_T SteeringmotortransferfunctionSt;
  /* '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate' */
  boolean_T TransferFcn_CSTATE;        /* '<S8>/Transfer Fcn' */
  boolean_T G1_CSTATE;                 /* '<S9>/G1' */
} XDis_model_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters (default storage) */
struct P_model_T_ {
  real_T A[4];                         /* Variable: A
                                        * Referenced by:
                                        *   '<S30>/Constant11'
                                        *   '<S30>/Constant2'
                                        */
  real_T B[2];                         /* Variable: B
                                        * Referenced by:
                                        *   '<S30>/Constant10'
                                        *   '<S30>/Constant3'
                                        */
  real_T C[2];                         /* Variable: C
                                        * Referenced by: '<S30>/Constant1'
                                        */
  real_T D;                            /* Variable: D
                                        * Referenced by:
                                        *   '<S30>/Constant5'
                                        *   '<S30>/Constant6'
                                        */
  real_T IMU_height;                   /* Variable: IMU_height
                                        * Referenced by:
                                        *   '<S29>/Constant3'
                                        *   '<S24>/Constant4'
                                        */
  real_T P_steering_speedcontrol;      /* Variable: P_steering_speedcontrol
                                        * Referenced by:
                                        *   '<S62>/Integral Gain'
                                        *   '<S70>/Proportional Gain'
                                        */
  real_T Ti_steering_speedcontrol;     /* Variable: Ti_steering_speedcontrol
                                        * Referenced by: '<S62>/Integral Gain'
                                        */
  real_T Ts;                           /* Variable: Ts
                                        * Referenced by:
                                        *   '<S21>/Gain4'
                                        *   '<S26>/Gain3'
                                        */
  real_T b;                            /* Variable: b
                                        * Referenced by:
                                        *   '<S27>/b Real Bike'
                                        *   '<S23>/Bike Model b'
                                        */
  real_T bike_params[6];               /* Variable: bike_params
                                        * Referenced by:
                                        *   '<S15>/ Bike Parameters Model '
                                        *   '<S31>/ Bike Parameters Model '
                                        */
  real_T complementary;                /* Variable: complementary
                                        * Referenced by:
                                        *   '<S21>/Gain2'
                                        *   '<S21>/Gain3'
                                        */
  real_T g;                            /* Variable: g
                                        * Referenced by:
                                        *   '<S20>/Gain1'
                                        *   '<S20>/Gain2'
                                        */
  real_T g1_den[2];                    /* Variable: g1_den
                                        * Referenced by: '<S9>/G1'
                                        */
  real_T g1_num[2];                    /* Variable: g1_num
                                        * Referenced by: '<S9>/G1'
                                        */
  real_T g2_den;                       /* Variable: g2_den
                                        * Referenced by: '<S9>/G2'
                                        */
  real_T g2_num;                       /* Variable: g2_num
                                        * Referenced by: '<S9>/G2'
                                        */
  real_T initial_roll;                 /* Variable: initial_roll
                                        * Referenced by:
                                        *   '<S5>/Integrator'
                                        *   '<S21>/Delay'
                                        */
  real_T initial_x;                    /* Variable: initial_x
                                        * Referenced by: '<S16>/Discrete-Time Integrator'
                                        */
  real_T initial_y;                    /* Variable: initial_y
                                        * Referenced by: '<S16>/Discrete-Time Integrator1'
                                        */
  real_T kt;                           /* Variable: kt
                                        * Referenced by:
                                        *   '<S8>/Gain'
                                        *   '<S8>/Gain1'
                                        */
  real_T lambda;                       /* Variable: lambda
                                        * Referenced by:
                                        *   '<S3>/Gain1'
                                        *   '<S27>/Gain'
                                        *   '<S28>/Gain1'
                                        *   '<S23>/Gain'
                                        */
  real_T roll_rate_coef;               /* Variable: roll_rate_coef
                                        * Referenced by:
                                        *   '<S25>/Gain4'
                                        *   '<S25>/Gain6'
                                        */
  real_T v;                            /* Variable: v
                                        * Referenced by: '<Root>/Forward Velocity'
                                        */
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S65>/Integrator'
                               */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S15>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<S15>/Discrete-Time Integrator'
                                        */
  real_T DiscreteTimeIntegrator_gainva_o;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_o
                           * Referenced by: '<S16>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator1_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator1_gainval
                           * Referenced by: '<S16>/Discrete-Time Integrator1'
                           */
  real_T Delay3_InitialCondition;      /* Expression: 0.0
                                        * Referenced by: '<S19>/Delay3'
                                        */
  real_T Delay18_InitialCondition;     /* Expression: 0.0
                                        * Referenced by: '<S25>/Delay18'
                                        */
  real_T Delay18_InitialCondition_h;   /* Expression: 0.0
                                        * Referenced by: '<S26>/Delay18'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0.0
                                        * Referenced by: '<S19>/Delay1'
                                        */
  real_T Delay2_InitialCondition;      /* Expression: 0.0
                                        * Referenced by: '<S19>/Delay2'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0.0
                                        * Referenced by: '<S19>/Delay'
                                        */
  real_T Delay5_InitialCondition;      /* Expression: 0.0
                                        * Referenced by: '<S19>/Delay5'
                                        */
  real_T TransferFcn1_A;               /* Computed Parameter: TransferFcn1_A
                                        * Referenced by: '<S8>/Transfer Fcn1'
                                        */
  real_T TransferFcn1_C;               /* Computed Parameter: TransferFcn1_C
                                        * Referenced by: '<S8>/Transfer Fcn1'
                                        */
  real_T TrueSteeringRatePerturbation_Va;/* Expression: 0
                                          * Referenced by: '<Root>/True Steering Rate Perturbation'
                                          */
  real_T Gain1_Gain;                   /* Expression: pi/180
                                        * Referenced by: '<S1>/Gain1'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S31>/Integrator'
                                        */
  real_T Gain_Gain;                    /* Expression: 180/pi
                                        * Referenced by: '<S10>/Gain'
                                        */
  real_T Constant_Value;               /* Expression: zeros(size(C,1),size(B,2))
                                        * Referenced by: '<S30>/Constant'
                                        */
  real_T Integrator1_IC[2];      /* Expression: [C ; C*A]\[initial_rollrate ; 0]
                                  * Referenced by: '<S30>/Integrator1'
                                  */
  real_T Gain_Gain_b;                  /* Expression: 180/pi
                                        * Referenced by: '<S12>/Gain'
                                        */
  real_T Gain_Gain_c;                  /* Expression: 180/pi
                                        * Referenced by: '<S11>/Gain'
                                        */
  real_T Gain_Gain_o;                  /* Expression: 180/pi
                                        * Referenced by: '<S13>/Gain'
                                        */
  real_T Gain_Gain_g;                  /* Expression: 180/pi
                                        * Referenced by: '<S14>/Gain'
                                        */
  real_T MeasuredRollRatePerturbation_Va;/* Expression: 0
                                          * Referenced by: '<Root>/Measured Roll Rate Perturbation'
                                          */
  real_T Gain1_Gain_j;                 /* Expression: pi/180
                                        * Referenced by: '<S2>/Gain1'
                                        */
  real_T Integrator1_IC_e;             /* Expression: 0
                                        * Referenced by: '<S32>/Integrator1'
                                        */
  real_T Integrator2_IC;               /* Expression: 0
                                        * Referenced by: '<S32>/Integrator2'
                                        */
  real_T ReferenceRollAngle_Value;     /* Expression: deg2rad(4)
                                        * Referenced by: '<Root>/Reference Roll Angle'
                                        */
  real_T SteeringmotortransferfunctionSt;
                          /* Computed Parameter: SteeringmotortransferfunctionSt
                           * Referenced by: '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate'
                           */
  real_T Steeringmotortransferfunction_c;
                          /* Computed Parameter: Steeringmotortransferfunction_c
                           * Referenced by: '<S7>/Steering motor transfer function Steering Rate reference to Steering Rate'
                           */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S8>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S8>/Transfer Fcn'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S65>/Integrator'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_model_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_model_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[10];
  real_T odeF[3][10];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* Block parameters (default storage) */
extern P_model_T model_P;

/* Block signals (default storage) */
extern B_model_T model_B;

/* Continuous states (default storage) */
extern X_model_T model_X;

/* Block states (default storage) */
extern DW_model_T model_DW;

/* Model entry point functions */
extern void model_initialize(void);
extern void model_step(void);
extern void model_terminate(void);

/* Real-time Model object */
extern RT_MODEL_model_T *const model_M;

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
 * '<Root>' : 'model'
 * '<S1>'   : 'model/Degrees to Radians'
 * '<S2>'   : 'model/Degrees to Radians6'
 * '<S3>'   : 'model/Global Angles and Coordinates Calculator - Position'
 * '<S4>'   : 'model/IMU + Complementary Filter'
 * '<S5>'   : 'model/Physical Bike'
 * '<S6>'   : 'model/Steering Encoder'
 * '<S7>'   : 'model/Steering Motor'
 * '<S8>'   : 'model/Steering Motor1'
 * '<S9>'   : 'model/Technion Controller'
 * '<S10>'  : 'model/True Heading'
 * '<S11>'  : 'model/True Roll'
 * '<S12>'  : 'model/True Roll Rate'
 * '<S13>'  : 'model/True Steering Angle'
 * '<S14>'  : 'model/True Steering Rate'
 * '<S15>'  : 'model/Global Angles and Coordinates Calculator - Position/Measured Bike Angles to Global Angles'
 * '<S16>'  : 'model/Global Angles and Coordinates Calculator - Position/Measured Global Angles  to Global Coordinates'
 * '<S17>'  : 'model/Global Angles and Coordinates Calculator - Position/Measured Bike Angles to Global Angles/MATLAB Function'
 * '<S18>'  : 'model/Global Angles and Coordinates Calculator - Position/Measured Global Angles  to Global Coordinates/MATLAB Function1'
 * '<S19>'  : 'model/IMU + Complementary Filter/Complementary filter'
 * '<S20>'  : 'model/IMU + Complementary Filter/IMU Accelerometer'
 * '<S21>'  : 'model/IMU + Complementary Filter/Complementary filter/Complementary Filter'
 * '<S22>'  : 'model/IMU + Complementary Filter/Complementary filter/Roll Angle Estimation of Accelerometer'
 * '<S23>'  : 'model/IMU + Complementary Filter/Complementary filter/Roll Angle Estimation of Accelerometer/Centripetal Acceleration Compensator'
 * '<S24>'  : 'model/IMU + Complementary Filter/Complementary filter/Roll Angle Estimation of Accelerometer/Lateral Acceleration Compensator'
 * '<S25>'  : 'model/IMU + Complementary Filter/Complementary filter/Roll Angle Estimation of Accelerometer/Lateral Acceleration Compensator/Low Pass Filter'
 * '<S26>'  : 'model/IMU + Complementary Filter/Complementary filter/Roll Angle Estimation of Accelerometer/Lateral Acceleration Compensator/Numerical Derivative'
 * '<S27>'  : 'model/Physical Bike/Centripetal Acceleration Calculator'
 * '<S28>'  : 'model/Physical Bike/Global Angles and Coordinates Calculator - Position'
 * '<S29>'  : 'model/Physical Bike/Lateral Acceleration (due to Roll) Calculator'
 * '<S30>'  : 'model/Physical Bike/Linear Bicycle Model with bike and box as 1 volume'
 * '<S31>'  : 'model/Physical Bike/Global Angles and Coordinates Calculator - Position/Measured Bike Angles to Global Angles'
 * '<S32>'  : 'model/Physical Bike/Global Angles and Coordinates Calculator - Position/Measured Global Angles  to Global Coordinates'
 * '<S33>'  : 'model/Physical Bike/Global Angles and Coordinates Calculator - Position/Measured Bike Angles to Global Angles/MATLAB Function'
 * '<S34>'  : 'model/Physical Bike/Global Angles and Coordinates Calculator - Position/Measured Global Angles  to Global Coordinates/MATLAB Function1'
 * '<S35>'  : 'model/Steering Motor1/PID Controller'
 * '<S36>'  : 'model/Steering Motor1/PID Controller/Anti-windup'
 * '<S37>'  : 'model/Steering Motor1/PID Controller/D Gain'
 * '<S38>'  : 'model/Steering Motor1/PID Controller/Filter'
 * '<S39>'  : 'model/Steering Motor1/PID Controller/Filter ICs'
 * '<S40>'  : 'model/Steering Motor1/PID Controller/I Gain'
 * '<S41>'  : 'model/Steering Motor1/PID Controller/Ideal P Gain'
 * '<S42>'  : 'model/Steering Motor1/PID Controller/Ideal P Gain Fdbk'
 * '<S43>'  : 'model/Steering Motor1/PID Controller/Integrator'
 * '<S44>'  : 'model/Steering Motor1/PID Controller/Integrator ICs'
 * '<S45>'  : 'model/Steering Motor1/PID Controller/N Copy'
 * '<S46>'  : 'model/Steering Motor1/PID Controller/N Gain'
 * '<S47>'  : 'model/Steering Motor1/PID Controller/P Copy'
 * '<S48>'  : 'model/Steering Motor1/PID Controller/Parallel P Gain'
 * '<S49>'  : 'model/Steering Motor1/PID Controller/Reset Signal'
 * '<S50>'  : 'model/Steering Motor1/PID Controller/Saturation'
 * '<S51>'  : 'model/Steering Motor1/PID Controller/Saturation Fdbk'
 * '<S52>'  : 'model/Steering Motor1/PID Controller/Sum'
 * '<S53>'  : 'model/Steering Motor1/PID Controller/Sum Fdbk'
 * '<S54>'  : 'model/Steering Motor1/PID Controller/Tracking Mode'
 * '<S55>'  : 'model/Steering Motor1/PID Controller/Tracking Mode Sum'
 * '<S56>'  : 'model/Steering Motor1/PID Controller/postSat Signal'
 * '<S57>'  : 'model/Steering Motor1/PID Controller/preSat Signal'
 * '<S58>'  : 'model/Steering Motor1/PID Controller/Anti-windup/Passthrough'
 * '<S59>'  : 'model/Steering Motor1/PID Controller/D Gain/Disabled'
 * '<S60>'  : 'model/Steering Motor1/PID Controller/Filter/Disabled'
 * '<S61>'  : 'model/Steering Motor1/PID Controller/Filter ICs/Disabled'
 * '<S62>'  : 'model/Steering Motor1/PID Controller/I Gain/Internal Parameters'
 * '<S63>'  : 'model/Steering Motor1/PID Controller/Ideal P Gain/Passthrough'
 * '<S64>'  : 'model/Steering Motor1/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S65>'  : 'model/Steering Motor1/PID Controller/Integrator/Discrete'
 * '<S66>'  : 'model/Steering Motor1/PID Controller/Integrator ICs/Internal IC'
 * '<S67>'  : 'model/Steering Motor1/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S68>'  : 'model/Steering Motor1/PID Controller/N Gain/Disabled'
 * '<S69>'  : 'model/Steering Motor1/PID Controller/P Copy/Disabled'
 * '<S70>'  : 'model/Steering Motor1/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S71>'  : 'model/Steering Motor1/PID Controller/Reset Signal/Disabled'
 * '<S72>'  : 'model/Steering Motor1/PID Controller/Saturation/Passthrough'
 * '<S73>'  : 'model/Steering Motor1/PID Controller/Saturation Fdbk/Disabled'
 * '<S74>'  : 'model/Steering Motor1/PID Controller/Sum/Sum_PI'
 * '<S75>'  : 'model/Steering Motor1/PID Controller/Sum Fdbk/Disabled'
 * '<S76>'  : 'model/Steering Motor1/PID Controller/Tracking Mode/Disabled'
 * '<S77>'  : 'model/Steering Motor1/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S78>'  : 'model/Steering Motor1/PID Controller/postSat Signal/Forward_Path'
 * '<S79>'  : 'model/Steering Motor1/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_model_h_ */
