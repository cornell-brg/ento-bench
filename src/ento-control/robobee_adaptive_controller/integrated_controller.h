/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: integrated_controller.h
 *
 * Code generated for Simulink model 'integrated_controller'.
 *
 * Model version                  : 23.14
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Wed Mar 22 13:27:48 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_integrated_controller_h_
#define RTW_HEADER_integrated_controller_h_
#ifndef integrated_controller_COMMON_INCLUDES_
#define integrated_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                              /* integrated_controller_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef DEFINED_TYPEDEF_FOR_struct_1IiccvmJoz48CkFlmxa7PB_
#define DEFINED_TYPEDEF_FOR_struct_1IiccvmJoz48CkFlmxa7PB_

typedef struct {
  real_T ax[10];
  real_T N;
  real_T T;
  real_T n_1[10];
  real_T n_2[10];
  real_T n_3[10];
  real_T n_4[10];
  real_T n_5[10];
} struct_1IiccvmJoz48CkFlmxa7PB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_B1Vk66atRZ2ph9uociFBiB_
#define DEFINED_TYPEDEF_FOR_struct_B1Vk66atRZ2ph9uociFBiB_

typedef struct {
  real_T en;
  real_T height;
  real_T time;
} struct_B1Vk66atRZ2ph9uociFBiB;

#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T integratorwithsat[6];         /* '<S16>/integrator with sat' */
  real_T FR[9];                        /* '<S20>/F*R' */
  real_T R[9];                         /* '<S31>/Rotation Matrix' */
  real_T a_hato[6];                    /* '<S16>/initial guess' */
  real_T lowpass_states[3];            /* '<S5>/low pass' */
  real_T lowpass1_states[3];           /* '<S5>/low pass1' */
  real_T lowpass2_states[2];           /* '<S5>/low pass2' */
  real_T DelayInput_DSTATE[3];         /* '<S39>/Delay Input' */
  real_T DelayOutput_DSTATE[3];        /* '<S39>/Delay Output' */
  real_T DelayInput_DSTATE_o[3];       /* '<S40>/Delay Input' */
  real_T DelayOutput_DSTATE_k[3];      /* '<S40>/Delay Output' */
  real_T DelayInput_DSTATE_i[3];       /* '<S41>/Delay Input' */
  real_T DelayOutput_DSTATE_a[3];      /* '<S41>/Delay Output' */
  real_T DelayInput_DSTATE_j[9];       /* '<S37>/Delay Input' */
  real_T DelayOutput_DSTATE_b[9];      /* '<S37>/Delay Output' */
  real_T lowpass3_states[18];          /* '<S35>/low pass3' */
  real_T lowpass3_states_b[2];         /* '<S36>/low pass3' */
  real_T UnitDelay_DSTATE[6];          /* '<S8>/Unit Delay' */
  real_T UnitDelay1_DSTATE[6];         /* '<S8>/Unit Delay1' */
  real_T lowpass3_states_c[12];        /* '<S8>/low pass3' */
  real_T filterderivative1_states[6];  /* '<S20>/filter derivative 1' */
  real_T filter1_states[18];           /* '<S20>/filter 1' */
  real_T filter1_states_j[18];         /* '<S20>/filter1' */
  real_T integratorwithsat_DSTATE[6];  /* '<S16>/integrator with sat' */
  real_T lowpass3_tmp[9];              /* '<S35>/low pass3' */
  real_T lowpass3_tmp_m[6];            /* '<S8>/low pass3' */
  real_T filterderivative1_tmp[3];     /* '<S20>/filter derivative 1' */
  real_T PrescaledAmp;                 /* '<S77>/PrescaledAmp' */
  real_T PrescaledOffset;              /* '<S77>/PrescaledOffset' */
  real_T PrescaledAmp_g;               /* '<S78>/PrescaledAmp' */
  real_T PrescaledOffset_p;            /* '<S78>/PrescaledOffset' */
  real_T GainBias;                     /* '<S6>/GainBias' */
  real_T F;                            /* '<S9>/Thrust Controller' */
  real_T SaturatingRamp_DSTATE;        /* '<S7>/SaturatingRamp' */
  real_T SaturatingRamp_DSTATE_j;      /* '<S76>/SaturatingRamp' */
  real_T DiscreteTransferFcn_states;   /* '<S81>/Discrete Transfer Fcn' */
  real_T RampUpOrDown_DSTATE;          /* '<S77>/RampUpOrDown' */
  real_T DiscreteTransferFcn_states_n; /* '<S86>/Discrete Transfer Fcn' */
  real_T omegat_DSTATE;                /* '<S89>/omega*t' */
  real_T DiscreteTransferFcn_states_a; /* '<S84>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn_states_ac;/* '<S85>/Discrete Transfer Fcn' */
  real_T RampUpOrDown_DSTATE_k;        /* '<S78>/RampUpOrDown' */
  real_T DiscreteTransferFcn_states_av;/* '<S95>/Discrete Transfer Fcn' */
  real_T SmoothClock_DSTATE;           /* '<S98>/SmoothClock' */
  real_T DiscreteTransferFcn_states_p; /* '<S93>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn_states_a5;/* '<S94>/Discrete Transfer Fcn' */
  real_T DelayInput_DSTATE_c;          /* '<S38>/Delay Input' */
  real_T DelayOutput_DSTATE_c;         /* '<S38>/Delay Output' */
  real_T time_DSTATE;                  /* '<S14>/time' */
  real_T SaturatingRamp1_DSTATE;       /* '<S73>/SaturatingRamp1' */
  real_T SaturatingRamp_DSTATE_p;      /* '<S5>/SaturatingRamp' */
  real_T SaturatingRamp_DSTATE_a;      /* '<S4>/SaturatingRamp' */
  real_T saturationramp_DSTATE;        /* '<S26>/saturation ramp' */
  real_T time_DSTATE_m;                /* '<S13>/time' */
  real_T filter1_states_p;             /* '<S18>/filter 1' */
  real_T counter_DSTATE;               /* '<S26>/counter' */
  real_T counter1_DSTATE;              /* '<S13>/counter1' */
  real_T counter2_DSTATE;              /* '<S13>/counter2' */
  real_T counter_DSTATE_f;             /* '<S47>/counter' */
  real_T lowpass_tmp;                  /* '<S5>/low pass' */
  real_T lowpass1_tmp;                 /* '<S5>/low pass1' */
  real_T lowpass2_tmp;                 /* '<S5>/low pass2' */
  real_T lowpass3_tmp_b;               /* '<S36>/low pass3' */
  int8_T SaturatingRamp_PrevResetState;/* '<S5>/SaturatingRamp' */
  int8_T counter1_PrevResetState;      /* '<S13>/counter1' */
  uint8_T integratorwithsat_SYSTEM_ENABLE;/* '<S16>/integrator with sat' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Xm;                           /* '<Root>/X(m)' */
  real_T Ym;                           /* '<Root>/Y(m)' */
  real_T Zm;                           /* '<Root>/Z(m)' */
  real_T alpharadians;                 /* '<Root>/alpha(radians)' */
  real_T betaradians;                  /* '<Root>/beta(radians)' */
  real_T gammaradians;                 /* '<Root>/gamma(radians)' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T time;                         /* '<Root>/time' */
  real_T Xm1;                          /* '<Root>/X (m)1' */
  real_T Ym1;                          /* '<Root>/Y (m)1' */
  real_T Zm1;                          /* '<Root>/Z (m)1' */
  real_T alpharadians1;                /* '<Root>/alpha (radians)1' */
  real_T betaradians1;                 /* '<Root>/beta (radians)1' */
  real_T gammaradians1;                /* '<Root>/gamma (radians)1' */
  real_T bias;                         /* '<Root>/bias' */
  real_T LWsignal;                     /* '<Root>/LW signal' */
  real_T RWsignal;                     /* '<Root>/RW signal' */
  real_T LW_NoRamp_NoBias;             /* '<Root>/LW_NoRamp_NoBias' */
  real_T RW_NoRamp_NoBias;             /* '<Root>/RW_NoRamp_NoBias' */
  real_T signalampv;                   /* '<Root>/signal amp (v)' */
  real_T pitchbiasv;                   /* '<Root>/pitch bias (v)' */
  real_T muv;                          /* '<Root>/mu v' */
  real_T rollbiasv;                    /* '<Root>/roll bias (v)' */
  real_T flappingampdeg;               /* '<Root>/flapping amp (deg)' */
  real_T pitchbiasdeg;                 /* '<Root>/pitch bias (deg)' */
  real_T mu;                           /* '<Root>/mu' */
  real_T rollbiasdeg;                  /* '<Root>/roll bias (deg)' */
  real_T enable_i;                     /* '<Root>/enable' */
  real_T pitch;                        /* '<Root>/pitch' */
  real_T roll;                         /* '<Root>/roll' */
  real_T yaw;                          /* '<Root>/yaw' */
  real_T force;                        /* '<Root>/force' */
  real_T Fo_j;                         /* '<Root>/Fo' */
  real_T Theta[2];                     /* '<Root>/Theta' */
  real_T To[3];                        /* '<Root>/To' */
  real_T To1;                          /* '<Root>/To1' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  struct_1IiccvmJoz48CkFlmxa7PB agg_s; /* Variable: agg_s
                                        * Referenced by: '<S1>/maneuver setpoint rd1'
                                        */
  struct_B1Vk66atRZ2ph9uociFBiB landing_par;/* Variable: landing_par
                                             * Referenced by:
                                             *   '<S1>/maneuver setpoint rd1'
                                             *   '<S73>/landing_cond'
                                             */
  real_T A;                            /* Variable: A
                                        * Referenced by:
                                        *   '<S52>/lift gain'
                                        *   '<S53>/lift gain'
                                        *   '<S53>/pitch gain'
                                        *   '<S54>/1//A'
                                        */
  real_T Bv;                           /* Variable: Bv
                                        * Referenced by:
                                        *   '<S49>/Bv1'
                                        *   '<S54>/Bv'
                                        */
  real_T Cv;                           /* Variable: Cv
                                        * Referenced by:
                                        *   '<S49>/Cv'
                                        *   '<S54>/Cv'
                                        */
  real_T Pvin[4];                      /* Variable: Pvin
                                        * Referenced by:
                                        *   '<S57>/1'
                                        *   '<S57>/Gain2'
                                        *   '<S57>/Gain3'
                                        *   '<S57>/Gain8'
                                        */
  real_T Pvmax[5];                     /* Variable: Pvmax
                                        * Referenced by:
                                        *   '<S55>/Gain2'
                                        *   '<S55>/Gain3'
                                        *   '<S55>/Gain7'
                                        *   '<S55>/Gain8'
                                        *   '<S56>/Gain2'
                                        *   '<S56>/Gain3'
                                        *   '<S56>/Gain7'
                                        *   '<S56>/Gain8'
                                        *   '<S92>/Gain2'
                                        *   '<S92>/Gain3'
                                        *   '<S92>/Gain7'
                                        *   '<S92>/Gain8'
                                        *   '<S101>/Gain2'
                                        *   '<S101>/Gain3'
                                        *   '<S101>/Gain7'
                                        *   '<S101>/Gain8'
                                        */
  real_T adpt_gamma[36];               /* Variable: adpt_gamma
                                        * Referenced by: '<S15>/adaptive_estimation'
                                        */
  real_T att_gain[3];                  /* Variable: att_gain
                                        * Referenced by: '<S9>/Attitude Controleller'
                                        */
  real_T att_s;                        /* Variable: att_s
                                        * Referenced by: '<S35>/att_s'
                                        */
  real_T att_s2;                       /* Variable: att_s2
                                        * Referenced by: '<S36>/att_s'
                                        */
  real_T base_pitch;                   /* Variable: base_pitch
                                        * Referenced by: '<Root>/base4'
                                        */
  real_T base_roll;                    /* Variable: base_roll
                                        * Referenced by: '<Root>/base3'
                                        */
  real_T base_thrust;                  /* Variable: base_thrust
                                        * Referenced by: '<Root>/base'
                                        */
  real_T base_yaw;                     /* Variable: base_yaw
                                        * Referenced by: '<Root>/base2'
                                        */
  real_T bee_mass;                     /* Variable: bee_mass
                                        * Referenced by: '<S1>/saturation'
                                        */
  real_T closed_loop_en;               /* Variable: closed_loop_en
                                        * Referenced by: '<S3>/enable'
                                        */
  real_T est_lambda[9];                /* Variable: est_lambda
                                        * Referenced by: '<S15>/adaptive_estimation'
                                        */
  real_T freq;                         /* Variable: freq
                                        * Referenced by: '<Root>/FlappingFreq(hz)'
                                        */
  real_T gamma;                        /* Variable: gamma
                                        * Referenced by:
                                        *   '<S9>/Thrust Controller'
                                        *   '<S15>/adaptive_estimation'
                                        */
  real_T ixx;                          /* Variable: ixx
                                        * Referenced by: '<S9>/Attitude Controleller'
                                        */
  real_T iyy;                          /* Variable: iyy
                                        * Referenced by: '<S9>/Attitude Controleller'
                                        */
  real_T izz;                          /* Variable: izz
                                        * Referenced by: '<S9>/Attitude Controleller'
                                        */
  real_T k_da;                         /* Variable: k_da
                                        * Referenced by: '<S37>/GainPole'
                                        */
  real_T k_da2;                        /* Variable: k_da2
                                        * Referenced by: '<S38>/GainPole'
                                        */
  real_T k_eq;                         /* Variable: k_eq
                                        * Referenced by: '<S53>/pitch gain'
                                        */
  real_T lambda[5];                    /* Variable: lambda
                                        * Referenced by:
                                        *   '<S9>/Attitude Controleller'
                                        *   '<S9>/Thrust Controller'
                                        *   '<S15>/adaptive_estimation'
                                        */
  real_T lat_dd_limit;                 /* Variable: lat_dd_limit
                                        * Referenced by:
                                        *   '<S42>/lw1'
                                        *   '<S42>/up1'
                                        */
  real_T lat_k_da;                     /* Variable: lat_k_da
                                        * Referenced by: '<S39>/GainPole'
                                        */
  real_T lat_k_da2;                    /* Variable: lat_k_da2
                                        * Referenced by:
                                        *   '<S40>/GainPole'
                                        *   '<S41>/GainPole'
                                        */
  real_T lat_s;                        /* Variable: lat_s
                                        * Referenced by: '<S32>/lat_s'
                                        */
  real_T lat_s2;                       /* Variable: lat_s2
                                        * Referenced by:
                                        *   '<S32>/lat_s2'
                                        *   '<S32>/lat_s2_ '
                                        */
  real_T lp_den[3];                    /* Variable: lp_den
                                        * Referenced by:
                                        *   '<S5>/low pass2'
                                        *   '<S8>/low pass3'
                                        *   '<S35>/low pass3'
                                        *   '<S36>/low pass3'
                                        */
  real_T lp_num[3];                    /* Variable: lp_num
                                        * Referenced by:
                                        *   '<S5>/low pass2'
                                        *   '<S8>/low pass3'
                                        *   '<S35>/low pass3'
                                        *   '<S36>/low pass3'
                                        */
  real_T manoeuvre_en;                 /* Variable: manoeuvre_en
                                        * Referenced by: '<S1>/maneuver setpoint rd1'
                                        */
  real_T phase_offset;                 /* Variable: phase_offset
                                        * Referenced by: '<S89>/phase offset'
                                        */
  real_T running_time;                 /* Variable: running_time
                                        * Referenced by:
                                        *   '<Root>/running time'
                                        *   '<S1>/maneuver setpoint rd1'
                                        */
  real_T second_delay;                 /* Variable: second_delay
                                        * Referenced by: '<S26>/0.8'
                                        */
  real_T start_delay;                  /* Variable: start_delay
                                        * Referenced by:
                                        *   '<S67>/Constant'
                                        *   '<S73>/constant2'
                                        *   '<S47>/2'
                                        *   '<S26>/2'
                                        */
  real_T th_xo;                        /* Variable: th_xo
                                        * Referenced by: '<S8>/th_xo'
                                        */
  real_T th_yo;                        /* Variable: th_yo
                                        * Referenced by: '<S8>/th_yo1'
                                        */
  real_T thrust_gain;                  /* Variable: thrust_gain
                                        * Referenced by: '<S9>/Thrust Controller'
                                        */
  real_T thrust_offset;                /* Variable: thrust_offset
                                        * Referenced by: '<S8>/Fo'
                                        */
  real_T vlp_den[4];                   /* Variable: vlp_den
                                        * Referenced by:
                                        *   '<S5>/low pass'
                                        *   '<S5>/low pass1'
                                        */
  real_T vlp_num[4];                   /* Variable: vlp_num
                                        * Referenced by:
                                        *   '<S5>/low pass'
                                        *   '<S5>/low pass1'
                                        */
  real_T derivativefilter_ICPrevInput;
                                 /* Mask Parameter: derivativefilter_ICPrevInput
                                  * Referenced by: '<S39>/Delay Input'
                                  */
  real_T derivativefilter1_ICPrevInput;
                                /* Mask Parameter: derivativefilter1_ICPrevInput
                                 * Referenced by: '<S40>/Delay Input'
                                 */
  real_T derivativefilter2_ICPrevInput;
                                /* Mask Parameter: derivativefilter2_ICPrevInput
                                 * Referenced by: '<S41>/Delay Input'
                                 */
  real_T filteredderivative_ICPrevInput;
                               /* Mask Parameter: filteredderivative_ICPrevInput
                                * Referenced by: '<S37>/Delay Input'
                                */
  real_T filteredderivative_ICPrevInpu_e;
                              /* Mask Parameter: filteredderivative_ICPrevInpu_e
                               * Referenced by: '<S38>/Delay Input'
                               */
  real_T derivativefilter_ICPrevOutput;
                                /* Mask Parameter: derivativefilter_ICPrevOutput
                                 * Referenced by: '<S39>/Delay Output'
                                 */
  real_T derivativefilter1_ICPrevOutput;
                               /* Mask Parameter: derivativefilter1_ICPrevOutput
                                * Referenced by: '<S40>/Delay Output'
                                */
  real_T derivativefilter2_ICPrevOutput;
                               /* Mask Parameter: derivativefilter2_ICPrevOutput
                                * Referenced by: '<S41>/Delay Output'
                                */
  real_T filteredderivative_ICPrevOutput;
                              /* Mask Parameter: filteredderivative_ICPrevOutput
                               * Referenced by: '<S37>/Delay Output'
                               */
  real_T filteredderivative_ICPrevOutp_k;
                              /* Mask Parameter: filteredderivative_ICPrevOutp_k
                               * Referenced by: '<S38>/Delay Output'
                               */
  real_T derivativefilter_ZeroZ;       /* Mask Parameter: derivativefilter_ZeroZ
                                        * Referenced by: '<S39>/GainZero'
                                        */
  real_T derivativefilter1_ZeroZ;     /* Mask Parameter: derivativefilter1_ZeroZ
                                       * Referenced by: '<S40>/GainZero'
                                       */
  real_T derivativefilter2_ZeroZ;     /* Mask Parameter: derivativefilter2_ZeroZ
                                       * Referenced by: '<S41>/GainZero'
                                       */
  real_T filteredderivative_ZeroZ;   /* Mask Parameter: filteredderivative_ZeroZ
                                      * Referenced by: '<S37>/GainZero'
                                      */
  real_T filteredderivative_ZeroZ_g;
                                   /* Mask Parameter: filteredderivative_ZeroZ_g
                                    * Referenced by: '<S38>/GainZero'
                                    */
  real_T CompareToConstant1_const;   /* Mask Parameter: CompareToConstant1_const
                                      * Referenced by: '<S65>/Constant'
                                      */
  real_T CompareToConstant1_const_i;
                                   /* Mask Parameter: CompareToConstant1_const_i
                                    * Referenced by: '<S62>/Constant'
                                    */
  real_T CompareToConstant1_const_e;
                                   /* Mask Parameter: CompareToConstant1_const_e
                                    * Referenced by: '<S64>/Constant'
                                    */
  real_T CompareToConstant1_const_c;
                                   /* Mask Parameter: CompareToConstant1_const_c
                                    * Referenced by: '<S63>/Constant'
                                    */
  real_T CompareToConstant1_const_e1;
                                  /* Mask Parameter: CompareToConstant1_const_e1
                                   * Referenced by: '<S69>/Constant'
                                   */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S68>/Constant'
                                       */
  real_T torelancetime_const;          /* Mask Parameter: torelancetime_const
                                        * Referenced by: '<S74>/Constant'
                                        */
  real_T CompareToConstant1_const_m;
                                   /* Mask Parameter: CompareToConstant1_const_m
                                    * Referenced by: '<S23>/Constant'
                                    */
  real_T CompareToConstant1_const_e0;
                                  /* Mask Parameter: CompareToConstant1_const_e0
                                   * Referenced by: '<S27>/Constant'
                                   */
  real_T zero1_Value;                  /* Expression: 0
                                        * Referenced by: '<S58>/zero1'
                                        */
  real_T zero1_Value_o;                /* Expression: 0
                                        * Referenced by: '<S59>/zero1'
                                        */
  real_T zero1_Value_m;                /* Expression: 0
                                        * Referenced by: '<S60>/zero1'
                                        */
  real_T zero1_Value_e;                /* Expression: 0
                                        * Referenced by: '<S61>/zero1'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S28>/Constant'
                                        */
  real_T Constant_Value_o;             /* Expression: 0
                                        * Referenced by: '<S43>/Constant'
                                        */
  real_T Constant_Value_f;             /* Expression: 0
                                        * Referenced by: '<S44>/Constant'
                                        */
  real_T Constant_Value_h;             /* Expression: 0
                                        * Referenced by: '<S46>/Constant'
                                        */
  real_T Constant_Value_l;             /* Expression: 0
                                        * Referenced by: '<S48>/Constant'
                                        */
  real_T Constant_Value_p;             /* Expression: 0
                                        * Referenced by: '<S66>/Constant'
                                        */
  real_T Constant_Value_o5;            /* Expression: 0
                                        * Referenced by: '<S71>/Constant'
                                        */
  real_T Constant_Value_c;             /* Expression: 0
                                        * Referenced by: '<S91>/Constant'
                                        */
  real_T Constant_Value_i;             /* Expression: 0
                                        * Referenced by: '<S100>/Constant'
                                        */
  real_T SaturatingRamp_gainval;   /* Computed Parameter: SaturatingRamp_gainval
                                    * Referenced by: '<S7>/SaturatingRamp'
                                    */
  real_T SaturatingRamp_IC;            /* Expression: 0
                                        * Referenced by: '<S7>/SaturatingRamp'
                                        */
  real_T constant_Value;               /* Expression: 1
                                        * Referenced by: '<S7>/constant'
                                        */
  real_T SaturatingRamp_gainval_l;
                                 /* Computed Parameter: SaturatingRamp_gainval_l
                                  * Referenced by: '<S76>/SaturatingRamp'
                                  */
  real_T SaturatingRamp_IC_d;          /* Expression: 0
                                        * Referenced by: '<S76>/SaturatingRamp'
                                        */
  real_T SaturatingRamp_UpperSat;      /* Expression: 1
                                        * Referenced by: '<S76>/SaturatingRamp'
                                        */
  real_T SaturatingRamp_LowerSat;      /* Expression: 0
                                        * Referenced by: '<S76>/SaturatingRamp'
                                        */
  real_T DiscreteTransferFcn_NumCoef;  /* Expression: [0.05]
                                        * Referenced by: '<S81>/Discrete Transfer Fcn'
                                        */
  real_T DiscreteTransferFcn_DenCoef[2];/* Expression: [1 -.95]
                                         * Referenced by: '<S81>/Discrete Transfer Fcn'
                                         */
  real_T DiscreteTransferFcn_InitialStat;/* Expression: 0
                                          * Referenced by: '<S81>/Discrete Transfer Fcn'
                                          */
  real_T PreGain_Gain;                 /* Expression: 1
                                        * Referenced by: '<S76>/PreGain'
                                        */
  real_T u01_Gain;                     /* Expression: 0.01
                                        * Referenced by: '<S6>/0.01'
                                        */
  real_T GainBias_Gain;                /* Expression: 1
                                        * Referenced by: '<S79>/GainBias'
                                        */
  real_T RampUpOrDown_gainval;       /* Computed Parameter: RampUpOrDown_gainval
                                      * Referenced by: '<S77>/RampUpOrDown'
                                      */
  real_T RampUpOrDown_IC;              /* Expression: 0
                                        * Referenced by: '<S77>/RampUpOrDown'
                                        */
  real_T RampUpOrDown_UpperSat;        /* Expression: 1
                                        * Referenced by: '<S77>/RampUpOrDown'
                                        */
  real_T RampUpOrDown_LowerSat;        /* Expression: 0
                                        * Referenced by: '<S77>/RampUpOrDown'
                                        */
  real_T Constant_Value_b;             /* Expression: 1
                                        * Referenced by: '<S89>/Constant'
                                        */
  real_T DiscreteTransferFcn_NumCoef_g;/* Expression: [0.05]
                                        * Referenced by: '<S86>/Discrete Transfer Fcn'
                                        */
  real_T DiscreteTransferFcn_DenCoef_c[2];/* Expression: [1, -0.95]
                                           * Referenced by: '<S86>/Discrete Transfer Fcn'
                                           */
  real_T DiscreteTransferFcn_InitialSt_m;/* Expression: 0
                                          * Referenced by: '<S86>/Discrete Transfer Fcn'
                                          */
  real_T degtorad_Gain;                /* Expression: pi/180
                                        * Referenced by: '<S89>/deg to rad'
                                        */
  real_T omegat_gainval;               /* Computed Parameter: omegat_gainval
                                        * Referenced by: '<S89>/omega*t'
                                        */
  real_T omegat_IC;                    /* Expression: 0
                                        * Referenced by: '<S89>/omega*t'
                                        */
  real_T f_Gain;                       /* Expression: -1
                                        * Referenced by: '<S89>/f'
                                        */
  real_T uf_Gain;                      /* Expression: 2
                                        * Referenced by: '<S89>/2f'
                                        */
  real_T u_Value;                      /* Expression: 4
                                        * Referenced by: '<S92>/4'
                                        */
  real_T u_Value_d;                    /* Expression: 3
                                        * Referenced by: '<S92>/5'
                                        */
  real_T constant1_Value;              /* Expression: 1
                                        * Referenced by: '<S92>/constant1'
                                        */
  real_T DiscreteTransferFcn_NumCoef_m;/* Expression: [0.05]
                                        * Referenced by: '<S84>/Discrete Transfer Fcn'
                                        */
  real_T DiscreteTransferFcn_DenCoef_f[2];/* Expression: [1 -.95]
                                           * Referenced by: '<S84>/Discrete Transfer Fcn'
                                           */
  real_T DiscreteTransferFcn_InitialSt_p;/* Expression: 0
                                          * Referenced by: '<S84>/Discrete Transfer Fcn'
                                          */
  real_T DiscreteTransferFcn_NumCoef_j;/* Expression: [0.05]
                                        * Referenced by: '<S85>/Discrete Transfer Fcn'
                                        */
  real_T DiscreteTransferFcn_DenCoef_n[2];/* Expression: [1 -.95]
                                           * Referenced by: '<S85>/Discrete Transfer Fcn'
                                           */
  real_T DiscreteTransferFcn_InitialSt_a;/* Expression: 0
                                          * Referenced by: '<S85>/Discrete Transfer Fcn'
                                          */
  real_T ground_Value;                 /* Expression: 1
                                        * Referenced by: '<S79>/ground'
                                        */
  real_T u1_Gain;                      /* Expression: 0.01
                                        * Referenced by: '<S6>/0.1'
                                        */
  real_T GainBias_Gain_h;              /* Expression: 1
                                        * Referenced by: '<S80>/GainBias'
                                        */
  real_T RampUpOrDown_gainval_d;   /* Computed Parameter: RampUpOrDown_gainval_d
                                    * Referenced by: '<S78>/RampUpOrDown'
                                    */
  real_T RampUpOrDown_IC_d;            /* Expression: 0
                                        * Referenced by: '<S78>/RampUpOrDown'
                                        */
  real_T RampUpOrDown_UpperSat_j;      /* Expression: 1
                                        * Referenced by: '<S78>/RampUpOrDown'
                                        */
  real_T RampUpOrDown_LowerSat_n;      /* Expression: 0
                                        * Referenced by: '<S78>/RampUpOrDown'
                                        */
  real_T Constant_Value_hm;            /* Expression: 1
                                        * Referenced by: '<S98>/Constant'
                                        */
  real_T DiscreteTransferFcn_NumCoef_d;/* Expression: [0.05]
                                        * Referenced by: '<S95>/Discrete Transfer Fcn'
                                        */
  real_T DiscreteTransferFcn_DenCoef_i[2];/* Expression: [1 -.95]
                                           * Referenced by: '<S95>/Discrete Transfer Fcn'
                                           */
  real_T DiscreteTransferFcn_InitialSt_n;/* Expression: 0
                                          * Referenced by: '<S95>/Discrete Transfer Fcn'
                                          */
  real_T SmoothClock_gainval;         /* Computed Parameter: SmoothClock_gainval
                                       * Referenced by: '<S98>/SmoothClock'
                                       */
  real_T SmoothClock_IC;               /* Expression: 0
                                        * Referenced by: '<S98>/SmoothClock'
                                        */
  real_T f_Gain_l;                     /* Expression: 1
                                        * Referenced by: '<S98>/f'
                                        */
  real_T uf_Gain_p;                    /* Expression: 2
                                        * Referenced by: '<S98>/2f'
                                        */
  real_T u_Value_k;                    /* Expression: 4
                                        * Referenced by: '<S101>/4'
                                        */
  real_T u_Value_n;                    /* Expression: 3
                                        * Referenced by: '<S101>/5'
                                        */
  real_T constant1_Value_c;            /* Expression: 1
                                        * Referenced by: '<S101>/constant1'
                                        */
  real_T DiscreteTransferFcn_NumCoef_o;/* Expression: [0.05]
                                        * Referenced by: '<S93>/Discrete Transfer Fcn'
                                        */
  real_T DiscreteTransferFcn_DenCoef_d[2];/* Expression: [1 -.95]
                                           * Referenced by: '<S93>/Discrete Transfer Fcn'
                                           */
  real_T DiscreteTransferFcn_InitialSt_g;/* Expression: 0
                                          * Referenced by: '<S93>/Discrete Transfer Fcn'
                                          */
  real_T DiscreteTransferFcn_NumCoef_oh;/* Expression: [0.05]
                                         * Referenced by: '<S94>/Discrete Transfer Fcn'
                                         */
  real_T DiscreteTransferFcn_DenCoef_fy[2];/* Expression: [1 -.95]
                                            * Referenced by: '<S94>/Discrete Transfer Fcn'
                                            */
  real_T DiscreteTransferFcn_InitialSt_l;/* Expression: 0
                                          * Referenced by: '<S94>/Discrete Transfer Fcn'
                                          */
  real_T ground_Value_o;               /* Expression: 1
                                        * Referenced by: '<S80>/ground'
                                        */
  real_T u2_Gain;                      /* Expression: 0.01
                                        * Referenced by: '<S6>/0.2'
                                        */
  real_T lowpass_InitialStates;        /* Expression: 0
                                        * Referenced by: '<S5>/low pass'
                                        */
  real_T lowpass1_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S5>/low pass1'
                                        */
  real_T lowpass2_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S5>/low pass2'
                                        */
  real_T lowpass3_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S35>/low pass3'
                                        */
  real_T lowpass3_InitialStates_l;     /* Expression: 0
                                        * Referenced by: '<S36>/low pass3'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 30
                                        * Referenced by: '<S36>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -30
                                        * Referenced by: '<S36>/Saturation'
                                        */
  real_T time_gainval;                 /* Computed Parameter: time_gainval
                                        * Referenced by: '<S14>/time'
                                        */
  real_T time_IC;                      /* Expression: 0
                                        * Referenced by: '<S14>/time'
                                        */
  real_T adaptive_en_Value;            /* Expression: 1
                                        * Referenced by: '<S8>/adaptive_en'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<S8>/Unit Delay'
                                        */
  real_T UnitDelay1_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<S8>/Unit Delay1'
                                        */
  real_T lowpass3_InitialStates_g;     /* Expression: 0
                                        * Referenced by: '<S8>/low pass3'
                                        */
  real_T correction_Gain;              /* Expression: 1/0.8
                                        * Referenced by: '<S50>/correction'
                                        */
  real_T mg_Gain;                      /* Expression: 10^6/9.81
                                        * Referenced by: '<S50>/mg'
                                        */
  real_T constant_Value_f;             /* Expression: 209.13
                                        * Referenced by: '<S50>/constant'
                                        */
  real_T gain1_Gain;                   /* Expression: 1/3.7645
                                        * Referenced by: '<S50>/gain1'
                                        */
  real_T half_Gain;                    /* Expression: 0.5
                                        * Referenced by: '<S52>/half'
                                        */
  real_T gain2_Gain;                   /* Expression: 0.0069148
                                        * Referenced by: '<S50>/gain2'
                                        */
  real_T constant1_Value_b;            /* Expression: -0.36459
                                        * Referenced by: '<S50>/constant1'
                                        */
  real_T Gain_Gain;                    /* Expression: 1.2
                                        * Referenced by: '<S2>/Gain'
                                        */
  real_T correction1_Gain;             /* Expression: 1
                                        * Referenced by: '<S50>/correction1'
                                        */
  real_T uNm2_Gain;                    /* Expression: 10^6
                                        * Referenced by: '<S50>/uNm2'
                                        */
  real_T pitchgain_Gain;               /* Expression: 4.25
                                        * Referenced by: '<S52>/pitch gain'
                                        */
  real_T gain6_Gain;                   /* Expression: 0.016834
                                        * Referenced by: '<S50>/gain6'
                                        */
  real_T constant2_Value;              /* Expression: -0.31839
                                        * Referenced by: '<S50>/constant2'
                                        */
  real_T correction2_Gain;             /* Expression: 1
                                        * Referenced by: '<S50>/correction2'
                                        */
  real_T uNm_Gain;                     /* Expression: 10^6
                                        * Referenced by: '<S50>/uNm'
                                        */
  real_T uNm3_Gain;                    /* Expression: 2*pi*0.05
                                        * Referenced by: '<S50>/uNm3'
                                        */
  real_T u_Value_kx;                   /* Expression: 4
                                        * Referenced by: '<S55>/4'
                                        */
  real_T u_Value_l;                    /* Expression: 3
                                        * Referenced by: '<S55>/5'
                                        */
  real_T constant1_Value_b4;           /* Expression: 1
                                        * Referenced by: '<S55>/constant1'
                                        */
  real_T u_Value_f;                    /* Expression: 1
                                        * Referenced by: '<S54>/1'
                                        */
  real_T u_Value_i;                    /* Expression: 4
                                        * Referenced by: '<S56>/4'
                                        */
  real_T u_Value_lh;                   /* Expression: 3
                                        * Referenced by: '<S56>/5'
                                        */
  real_T constant1_Value_l;            /* Expression: 1
                                        * Referenced by: '<S56>/constant1'
                                        */
  real_T half1_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S52>/half1'
                                        */
  real_T gain7_Gain;                   /* Expression: 0.0027082
                                        * Referenced by: '<S50>/gain7'
                                        */
  real_T constant3_Value;              /* Expression: 0.13859
                                        * Referenced by: '<S50>/constant3'
                                        */
  real_T Gain_Gain_c;                  /* Expression: 1.00
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T correction3_Gain;             /* Expression: 2
                                        * Referenced by: '<S50>/correction3'
                                        */
  real_T uNm1_Gain;                    /* Expression: 10^6
                                        * Referenced by: '<S50>/uNm1'
                                        */
  real_T half2_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S52>/half2'
                                        */
  real_T maxallowed_Value;             /* Expression: 149
                                        * Referenced by: '<S51>/max allowed'
                                        */
  real_T pp_Gain;                      /* Expression: 2
                                        * Referenced by: '<S2>/p-p'
                                        */
  real_T pp_Gain_a;                    /* Expression: 2
                                        * Referenced by: '<S53>/p-p'
                                        */
  real_T u_Gain;                       /* Expression: 2
                                        * Referenced by: '<S53>/2'
                                        */
  real_T u_Value_j;                    /* Expression: 3
                                        * Referenced by: '<S57>/3'
                                        */
  real_T u_Value_jl;                   /* Expression: 1
                                        * Referenced by: '<S49>/1'
                                        */
  real_T Gain1_Gain;                   /* Expression: 0
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T pp1_Gain;                     /* Expression: 2
                                        * Referenced by: '<S2>/p-p1'
                                        */
  real_T u_Gain_a;                     /* Expression: 2
                                        * Referenced by: '<S53>/1'
                                        */
  real_T SaturatingRamp1_gainval; /* Computed Parameter: SaturatingRamp1_gainval
                                   * Referenced by: '<S73>/SaturatingRamp1'
                                   */
  real_T SaturatingRamp1_IC;           /* Expression: 0
                                        * Referenced by: '<S73>/SaturatingRamp1'
                                        */
  real_T constant1_Value_d;            /* Expression: 2
                                        * Referenced by: '<S73>/constant1'
                                        */
  real_T SaturatingRamp_gainval_g;
                                 /* Computed Parameter: SaturatingRamp_gainval_g
                                  * Referenced by: '<S5>/SaturatingRamp'
                                  */
  real_T SaturatingRamp_IC_a;          /* Expression: 0
                                        * Referenced by: '<S5>/SaturatingRamp'
                                        */
  real_T SaturatingRamp_gainval_b;
                                 /* Computed Parameter: SaturatingRamp_gainval_b
                                  * Referenced by: '<S4>/SaturatingRamp'
                                  */
  real_T SaturatingRamp_IC_c;          /* Expression: 0
                                        * Referenced by: '<S4>/SaturatingRamp'
                                        */
  real_T saturationramp_gainval;   /* Computed Parameter: saturationramp_gainval
                                    * Referenced by: '<S26>/saturation ramp'
                                    */
  real_T saturationramp_IC;            /* Expression: 0
                                        * Referenced by: '<S26>/saturation ramp'
                                        */
  real_T saturationramp_UpperSat;      /* Expression: 1
                                        * Referenced by: '<S26>/saturation ramp'
                                        */
  real_T saturationramp_LowerSat;      /* Expression: -1
                                        * Referenced by: '<S26>/saturation ramp'
                                        */
  real_T filterderivative1_NumCoef[3]; /* Expression: est_fd1.num{1}
                                        * Referenced by: '<S20>/filter derivative 1'
                                        */
  real_T filterderivative1_DenCoef[3]; /* Expression: est_fd1.den{1}
                                        * Referenced by: '<S20>/filter derivative 1'
                                        */
  real_T filterderivative1_InitialStates;/* Expression: 0
                                          * Referenced by: '<S20>/filter derivative 1'
                                          */
  real_T filter1_NumCoef[3];           /* Expression: est_ft1.num{1}
                                        * Referenced by: '<S20>/filter 1'
                                        */
  real_T filter1_DenCoef[3];           /* Expression: est_ft1.den{1}
                                        * Referenced by: '<S20>/filter 1'
                                        */
  real_T filter1_InitialStates;        /* Expression: 0
                                        * Referenced by: '<S20>/filter 1'
                                        */
  real_T filter1_NumCoef_f[3];         /* Expression: est_ft1.num{1}
                                        * Referenced by: '<S20>/filter1'
                                        */
  real_T filter1_DenCoef_f[3];         /* Expression: est_ft1.den{1}
                                        * Referenced by: '<S20>/filter1'
                                        */
  real_T filter1_InitialStates_n;      /* Expression: 0
                                        * Referenced by: '<S20>/filter1'
                                        */
  real_T zero1_Value_a;                /* Expression: 0
                                        * Referenced by: '<S22>/zero1'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: 0.06
                                        * Referenced by: '<S19>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: -0.06
                                        * Referenced by: '<S19>/Saturation'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 0.06
                                        * Referenced by: '<S19>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -0.06
                                        * Referenced by: '<S19>/Saturation1'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 0.06
                                        * Referenced by: '<S19>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: -0.06
                                        * Referenced by: '<S19>/Saturation2'
                                        */
  real_T zero1_Value_f;                /* Expression: 0
                                        * Referenced by: '<S25>/zero1'
                                        */
  real_T integratorwithsat_gainval;
                                /* Computed Parameter: integratorwithsat_gainval
                                 * Referenced by: '<S16>/integrator with sat'
                                 */
  real_T integratorwithsat_IC;         /* Expression: 0
                                        * Referenced by: '<S16>/integrator with sat'
                                        */
  real_T integratorwithsat_UpperSat[6];
                                /* Expression: [1, 0.25, 0.25, 5e-8, 5e-8, 3e-8]
                                 * Referenced by: '<S16>/integrator with sat'
                                 */
  real_T integratorwithsat_LowerSat[6];
                          /* Expression: [-1, -0.25, -0.25, -5e-8, -5e-8, -3e-8]
                           * Referenced by: '<S16>/integrator with sat'
                           */
  real_T time_gainval_g;               /* Computed Parameter: time_gainval_g
                                        * Referenced by: '<S13>/time'
                                        */
  real_T time_IC_f;                    /* Expression: 0
                                        * Referenced by: '<S13>/time'
                                        */
  real_T filter1_NumCoef_b[2];         /* Expression: Ff_filter.num{1}
                                        * Referenced by: '<S18>/filter 1'
                                        */
  real_T filter1_DenCoef_c[2];         /* Expression: Ff_filter.den{1}
                                        * Referenced by: '<S18>/filter 1'
                                        */
  real_T filter1_InitialStates_i;      /* Expression: 0
                                        * Referenced by: '<S18>/filter 1'
                                        */
  real_T counter_gainval;              /* Computed Parameter: counter_gainval
                                        * Referenced by: '<S26>/counter'
                                        */
  real_T counter_IC;                   /* Expression: 0
                                        * Referenced by: '<S26>/counter'
                                        */
  real_T constant_Value_e;             /* Expression: 1
                                        * Referenced by: '<S26>/constant'
                                        */
  real_T att_stability_threshold_Value;/* Expression: 25
                                        * Referenced by: '<S13>/att_stability_threshold'
                                        */
  real_T counter1_gainval;             /* Computed Parameter: counter1_gainval
                                        * Referenced by: '<S13>/counter1'
                                        */
  real_T counter1_IC;                  /* Expression: 0
                                        * Referenced by: '<S13>/counter1'
                                        */
  real_T steadytime_Value;             /* Expression: 1
                                        * Referenced by: '<S13>/steady time'
                                        */
  real_T counter2_gainval;             /* Computed Parameter: counter2_gainval
                                        * Referenced by: '<S13>/counter2'
                                        */
  real_T counter2_IC;                  /* Expression: 0
                                        * Referenced by: '<S13>/counter2'
                                        */
  real_T counter_gainval_o;            /* Computed Parameter: counter_gainval_o
                                        * Referenced by: '<S47>/counter'
                                        */
  real_T counter_IC_n;                 /* Expression: 0
                                        * Referenced by: '<S47>/counter'
                                        */
  real_T u5_Value;                     /* Expression: 1.2
                                        * Referenced by: '<S47>/1.5'
                                        */
  real_T constant_Value_m;             /* Expression: 1
                                        * Referenced by: '<S47>/constant'
                                        */
  real_T constant_Value_h;             /* Expression: 1
                                        * Referenced by: '<S14>/constant'
                                        */
  real_T biasvoltage_Value;            /* Expression: 300
                                        * Referenced by: '<Root>/bias voltage'
                                        */
  real_T constant_Value_n;             /* Expression: 1
                                        * Referenced by: '<S4>/constant'
                                        */
  real_T Gain_Gain_n;                  /* Expression: 1
                                        * Referenced by: '<S5>/Gain'
                                        */
  real_T constant_Value_a;             /* Expression: 1
                                        * Referenced by: '<S73>/constant'
                                        */
  real_T Gain_Gain_h;                  /* Expression: 2
                                        * Referenced by: '<S82>/Gain'
                                        */
  real_T Constant_Value_iq;            /* Expression: 1
                                        * Referenced by: '<S82>/Constant'
                                        */
  real_T PreGain_Value;                /* Expression: 1
                                        * Referenced by: '<S77>/PreGain'
                                        */
  real_T p2ptoamp_Gain;                /* Expression: 1/2
                                        * Referenced by: '<S6>/p2p to amp'
                                        */
  real_T half_Gain_h;                  /* Expression: 0.5
                                        * Referenced by: '<S6>/half'
                                        */
  real_T Gain_Gain_b;                  /* Expression: 2
                                        * Referenced by: '<S87>/Gain'
                                        */
  real_T Constant_Value_p5;            /* Expression: 1
                                        * Referenced by: '<S87>/Constant'
                                        */
  real_T upi_Gain;                     /* Expression: 2*pi
                                        * Referenced by: '<S89>/2pi'
                                        */
  real_T PreGain_Value_a;              /* Expression: 1
                                        * Referenced by: '<S78>/PreGain'
                                        */
  real_T Gain_Gain_o;                  /* Expression: 2
                                        * Referenced by: '<S96>/Gain'
                                        */
  real_T Constant_Value_oz;            /* Expression: 1
                                        * Referenced by: '<S96>/Constant'
                                        */
  real_T upi_Gain_o;                   /* Expression: 2*pi
                                        * Referenced by: '<S98>/2pi'
                                        */
  real_T GainBias_Gain_p;              /* Expression: 1
                                        * Referenced by: '<S6>/GainBias'
                                        */
  uint32_T Constant_Value_pz;          /* Computed Parameter: Constant_Value_pz
                                        * Referenced by: '<S83>/Constant'
                                        */
  uint32_T Constant_Value_d;           /* Computed Parameter: Constant_Value_d
                                        * Referenced by: '<S90>/Constant'
                                        */
  uint32_T Constant_Value_ij;          /* Computed Parameter: Constant_Value_ij
                                        * Referenced by: '<S99>/Constant'
                                        */
  uint16_T Constant_Value_i5;          /* Computed Parameter: Constant_Value_i5
                                        * Referenced by: '<S70>/Constant'
                                        */
  uint8_T Switch1_Threshold;           /* Computed Parameter: Switch1_Threshold
                                        * Referenced by: '<S61>/Switch1'
                                        */
  uint8_T Switch1_Threshold_i;        /* Computed Parameter: Switch1_Threshold_i
                                       * Referenced by: '<S58>/Switch1'
                                       */
  uint8_T Switch1_Threshold_l;        /* Computed Parameter: Switch1_Threshold_l
                                       * Referenced by: '<S60>/Switch1'
                                       */
  uint8_T Switch1_Threshold_c;        /* Computed Parameter: Switch1_Threshold_c
                                       * Referenced by: '<S59>/Switch1'
                                       */
  uint8_T Switch1_Threshold_cr;      /* Computed Parameter: Switch1_Threshold_cr
                                      * Referenced by: '<S22>/Switch1'
                                      */
  uint8_T Switch1_Threshold_e;        /* Computed Parameter: Switch1_Threshold_e
                                       * Referenced by: '<S25>/Switch1'
                                       */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P rtP;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void integrated_controller_initialize(void);
extern void integrated_controller_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S37>/DataTypeProp' : Unused code path elimination
 * Block '<S38>/DataTypeProp' : Unused code path elimination
 * Block '<S39>/DataTypeProp' : Unused code path elimination
 * Block '<S40>/DataTypeProp' : Unused code path elimination
 * Block '<S41>/DataTypeProp' : Unused code path elimination
 * Block '<Root>/Sine Wave1' : Unused code path elimination
 * Block '<Root>/Step1' : Unused code path elimination
 * Block '<Root>/Uniform Random Number1' : Unused code path elimination
 * Block '<S37>/Downcast' : Eliminate redundant data type conversion
 * Block '<S38>/Downcast' : Eliminate redundant data type conversion
 * Block '<S39>/Downcast' : Eliminate redundant data type conversion
 * Block '<S40>/Downcast' : Eliminate redundant data type conversion
 * Block '<S41>/Downcast' : Eliminate redundant data type conversion
 * Block '<S47>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S5>/Data Type Conversion1' : Eliminate redundant data type conversion
 */

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
 * '<Root>' : 'integrated_controller'
 * '<S1>'   : 'integrated_controller/Adaptive Controller'
 * '<S2>'   : 'integrated_controller/Force//Torque to Trajectory Specs'
 * '<S3>'   : 'integrated_controller/closed loop en'
 * '<S4>'   : 'integrated_controller/counter'
 * '<S5>'   : 'integrated_controller/measurement manager'
 * '<S6>'   : 'integrated_controller/signal generator'
 * '<S7>'   : 'integrated_controller/timer'
 * '<S8>'   : 'integrated_controller/Adaptive Controller/adaptive system'
 * '<S9>'   : 'integrated_controller/Adaptive Controller/attitude and thrust controller'
 * '<S10>'  : 'integrated_controller/Adaptive Controller/maneuver setpoint rd1'
 * '<S11>'  : 'integrated_controller/Adaptive Controller/observer'
 * '<S12>'  : 'integrated_controller/Adaptive Controller/saturation'
 * '<S13>'  : 'integrated_controller/Adaptive Controller/start motion'
 * '<S14>'  : 'integrated_controller/Adaptive Controller/time'
 * '<S15>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive'
 * '<S16>'  : 'integrated_controller/Adaptive Controller/adaptive system/integrator'
 * '<S17>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive/adaptive_estimation'
 * '<S18>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive/estimator'
 * '<S19>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive/saturation'
 * '<S20>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive/estimator/Filter1'
 * '<S21>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive/estimator/estimating W1'
 * '<S22>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive/saturation/safety1'
 * '<S23>'  : 'integrated_controller/Adaptive Controller/adaptive system/adaptive/saturation/safety1/Compare To Constant1'
 * '<S24>'  : 'integrated_controller/Adaptive Controller/adaptive system/integrator/initial guess'
 * '<S25>'  : 'integrated_controller/Adaptive Controller/adaptive system/integrator/safety1'
 * '<S26>'  : 'integrated_controller/Adaptive Controller/adaptive system/integrator/start ramp'
 * '<S27>'  : 'integrated_controller/Adaptive Controller/adaptive system/integrator/safety1/Compare To Constant1'
 * '<S28>'  : 'integrated_controller/Adaptive Controller/adaptive system/integrator/start ramp/Compare To Zero'
 * '<S29>'  : 'integrated_controller/Adaptive Controller/attitude and thrust controller/Attitude Controleller'
 * '<S30>'  : 'integrated_controller/Adaptive Controller/attitude and thrust controller/Thrust Controller'
 * '<S31>'  : 'integrated_controller/Adaptive Controller/observer/Rotaion'
 * '<S32>'  : 'integrated_controller/Adaptive Controller/observer/lateral observer'
 * '<S33>'  : 'integrated_controller/Adaptive Controller/observer/Rotaion/Rotation Matrix'
 * '<S34>'  : 'integrated_controller/Adaptive Controller/observer/Rotaion/Rotation Matrix1'
 * '<S35>'  : 'integrated_controller/Adaptive Controller/observer/Rotaion/filtered derivative'
 * '<S36>'  : 'integrated_controller/Adaptive Controller/observer/Rotaion/filtered derivative1'
 * '<S37>'  : 'integrated_controller/Adaptive Controller/observer/Rotaion/filtered derivative/filtered derivative'
 * '<S38>'  : 'integrated_controller/Adaptive Controller/observer/Rotaion/filtered derivative1/filtered derivative'
 * '<S39>'  : 'integrated_controller/Adaptive Controller/observer/lateral observer/derivative filter'
 * '<S40>'  : 'integrated_controller/Adaptive Controller/observer/lateral observer/derivative filter1'
 * '<S41>'  : 'integrated_controller/Adaptive Controller/observer/lateral observer/derivative filter2'
 * '<S42>'  : 'integrated_controller/Adaptive Controller/observer/lateral observer/saturation'
 * '<S43>'  : 'integrated_controller/Adaptive Controller/start motion/Compare To Zero1'
 * '<S44>'  : 'integrated_controller/Adaptive Controller/start motion/Compare To Zero2'
 * '<S45>'  : 'integrated_controller/Adaptive Controller/start motion/metric l2 norm'
 * '<S46>'  : 'integrated_controller/Adaptive Controller/start motion/start maneouvre'
 * '<S47>'  : 'integrated_controller/Adaptive Controller/start motion/time_enable'
 * '<S48>'  : 'integrated_controller/Adaptive Controller/start motion/time_enable/Compare To Zero'
 * '<S49>'  : 'integrated_controller/Force//Torque to Trajectory Specs/MuV'
 * '<S50>'  : 'integrated_controller/Force//Torque to Trajectory Specs/Nominal desired wing trajectory'
 * '<S51>'  : 'integrated_controller/Force//Torque to Trajectory Specs/Proportion (out of one)'
 * '<S52>'  : 'integrated_controller/Force//Torque to Trajectory Specs/nominal desired voltage'
 * '<S53>'  : 'integrated_controller/Force//Torque to Trajectory Specs/v to deg (for ref)'
 * '<S54>'  : 'integrated_controller/Force//Torque to Trajectory Specs/nominal desired voltage/Vmax//Phi'
 * '<S55>'  : 'integrated_controller/Force//Torque to Trajectory Specs/nominal desired voltage/Vmax//Phi/norm(mu)'
 * '<S56>'  : 'integrated_controller/Force//Torque to Trajectory Specs/nominal desired voltage/Vmax//Phi/norm(mu)1'
 * '<S57>'  : 'integrated_controller/Force//Torque to Trajectory Specs/v to deg (for ref)/norm(mu)'
 * '<S58>'  : 'integrated_controller/closed loop en/safety1'
 * '<S59>'  : 'integrated_controller/closed loop en/safety2'
 * '<S60>'  : 'integrated_controller/closed loop en/safety3'
 * '<S61>'  : 'integrated_controller/closed loop en/safety4'
 * '<S62>'  : 'integrated_controller/closed loop en/safety1/Compare To Constant1'
 * '<S63>'  : 'integrated_controller/closed loop en/safety2/Compare To Constant1'
 * '<S64>'  : 'integrated_controller/closed loop en/safety3/Compare To Constant1'
 * '<S65>'  : 'integrated_controller/closed loop en/safety4/Compare To Constant1'
 * '<S66>'  : 'integrated_controller/counter/Compare To Zero'
 * '<S67>'  : 'integrated_controller/counter/start delay compare'
 * '<S68>'  : 'integrated_controller/measurement manager/Compare To Constant'
 * '<S69>'  : 'integrated_controller/measurement manager/Compare To Constant1'
 * '<S70>'  : 'integrated_controller/measurement manager/Compare To Zero2'
 * '<S71>'  : 'integrated_controller/measurement manager/cp'
 * '<S72>'  : 'integrated_controller/measurement manager/envelope'
 * '<S73>'  : 'integrated_controller/measurement manager/landing condition'
 * '<S74>'  : 'integrated_controller/measurement manager/torelance time'
 * '<S75>'  : 'integrated_controller/measurement manager/landing condition/landing_cond'
 * '<S76>'  : 'integrated_controller/signal generator/Bias_Signal'
 * '<S77>'  : 'integrated_controller/signal generator/Drive_Signal'
 * '<S78>'  : 'integrated_controller/signal generator/Drive_Signal1'
 * '<S79>'  : 'integrated_controller/signal generator/SignalSaturationLimit'
 * '<S80>'  : 'integrated_controller/signal generator/SignalSaturationLimit1'
 * '<S81>'  : 'integrated_controller/signal generator/Bias_Signal/AveragedBias'
 * '<S82>'  : 'integrated_controller/signal generator/Bias_Signal/SIgnalForRamp_binary'
 * '<S83>'  : 'integrated_controller/signal generator/Bias_Signal/SIgnalForRamp_binary/Compare To Zero'
 * '<S84>'  : 'integrated_controller/signal generator/Drive_Signal/AveragedAmp_filtered'
 * '<S85>'  : 'integrated_controller/signal generator/Drive_Signal/AveragedOffset_filtered'
 * '<S86>'  : 'integrated_controller/signal generator/Drive_Signal/AveragedTau_filtered'
 * '<S87>'  : 'integrated_controller/signal generator/Drive_Signal/SignalForRamp_binary'
 * '<S88>'  : 'integrated_controller/signal generator/Drive_Signal/SignalForSin_binary'
 * '<S89>'  : 'integrated_controller/signal generator/Drive_Signal/Subsystem'
 * '<S90>'  : 'integrated_controller/signal generator/Drive_Signal/SignalForRamp_binary/Compare To Zero'
 * '<S91>'  : 'integrated_controller/signal generator/Drive_Signal/SignalForSin_binary/Compare To Zero'
 * '<S92>'  : 'integrated_controller/signal generator/Drive_Signal/Subsystem/norm(mu)'
 * '<S93>'  : 'integrated_controller/signal generator/Drive_Signal1/AveragedAmp_filtered'
 * '<S94>'  : 'integrated_controller/signal generator/Drive_Signal1/AveragedOffset_filtered'
 * '<S95>'  : 'integrated_controller/signal generator/Drive_Signal1/AveragedTau_filtered'
 * '<S96>'  : 'integrated_controller/signal generator/Drive_Signal1/SignalForRamp_binary'
 * '<S97>'  : 'integrated_controller/signal generator/Drive_Signal1/SignalForSin_binary'
 * '<S98>'  : 'integrated_controller/signal generator/Drive_Signal1/Subsystem'
 * '<S99>'  : 'integrated_controller/signal generator/Drive_Signal1/SignalForRamp_binary/Compare To Zero'
 * '<S100>' : 'integrated_controller/signal generator/Drive_Signal1/SignalForSin_binary/Compare To Zero'
 * '<S101>' : 'integrated_controller/signal generator/Drive_Signal1/Subsystem/norm(mu)'
 */
#endif                                 /* RTW_HEADER_integrated_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
