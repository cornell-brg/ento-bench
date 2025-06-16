/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: integrated_controller_data.c
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

#include "integrated_controller.h"

/* Block parameters (default storage) */
P rtP = {
  /* Variable: agg_s
   * Referenced by: '<S1>/maneuver setpoint rd1'
   */
  {
    { 0.0, -1.4802973661668755e-15, 1.6283271027835628e-14, 0.0,
      -2.9605947323337509e-15, 0.05703703703703162, -0.063374485596702515,
      0.027160493827158658, -0.0052812071330586947, 0.00039120052837472193 },
    9.0,
    3.0,

    { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 },

    { -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 },

    { -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 },

    { -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 },

    { -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 }
  },

  /* Variable: landing_par
   * Referenced by:
   *   '<S1>/maneuver setpoint rd1'
   *   '<S73>/landing_cond'
   */
  {
    1.0,
    0.0,
    0.5
  },

  /* Variable: A
   * Referenced by:
   *   '<S52>/lift gain'
   *   '<S53>/lift gain'
   *   '<S53>/pitch gain'
   *   '<S54>/1//A'
   */
  154.0981473908416,

  /* Variable: Bv
   * Referenced by:
   *   '<S49>/Bv1'
   *   '<S54>/Bv'
   */
  273.95226202816286,

  /* Variable: Cv
   * Referenced by:
   *   '<S49>/Cv'
   *   '<S54>/Cv'
   */
  984.19804187420937,

  /* Variable: Pvin
   * Referenced by:
   *   '<S57>/1'
   *   '<S57>/Gain2'
   *   '<S57>/Gain3'
   *   '<S57>/Gain8'
   */
  { 1.7657456936854083, -8.98505989631795, 17.560050272285082, -12.6819043889045
  },

  /* Variable: Pvmax
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
  { 0.74800427611635556, -2.12530539744174, 2.3591391387333607,
    -0.97809696390547385, 0.998343369397654 },

  /* Variable: adpt_gamma
   * Referenced by: '<S15>/adaptive_estimation'
   */
  { 5.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0E-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    5.0E-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-9 },

  /* Variable: att_gain
   * Referenced by: '<S9>/Attitude Controleller'
   */
  { 7.4E-8, 7.4E-8, 2.22E-8 },

  /* Variable: att_s
   * Referenced by: '<S35>/att_s'
   */
  500.0,

  /* Variable: att_s2
   * Referenced by: '<S36>/att_s'
   */
  60.0,

  /* Variable: base_pitch
   * Referenced by: '<Root>/base4'
   */
  -9.7E-7,

  /* Variable: base_roll
   * Referenced by: '<Root>/base3'
   */
  5.9E-7,

  /* Variable: base_thrust
   * Referenced by: '<Root>/base'
   */
  0.0,

  /* Variable: base_yaw
   * Referenced by: '<Root>/base2'
   */
  -8.0E-8,

  /* Variable: bee_mass
   * Referenced by: '<S1>/saturation'
   */
  8.0E-5,

  /* Variable: closed_loop_en
   * Referenced by: '<S3>/enable'
   */
  1.0,

  /* Variable: est_lambda
   * Referenced by: '<S15>/adaptive_estimation'
   */
  { 70.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0 },

  /* Variable: freq
   * Referenced by: '<Root>/FlappingFreq(hz)'
   */
  120.0,

  /* Variable: gamma
   * Referenced by:
   *   '<S9>/Thrust Controller'
   *   '<S15>/adaptive_estimation'
   */
  100.0,

  /* Variable: ixx
   * Referenced by: '<S9>/Attitude Controleller'
   */
  1.8746E-9,

  /* Variable: iyy
   * Referenced by: '<S9>/Attitude Controleller'
   */
  1.9824E-9,

  /* Variable: izz
   * Referenced by: '<S9>/Attitude Controleller'
   */
  6.37E-10,

  /* Variable: k_da
   * Referenced by: '<S37>/GainPole'
   */
  0.951229424500714,

  /* Variable: k_da2
   * Referenced by: '<S38>/GainPole'
   */
  0.951229424500714,

  /* Variable: k_eq
   * Referenced by: '<S53>/pitch gain'
   */
  356.0,

  /* Variable: lambda
   * Referenced by:
   *   '<S9>/Attitude Controleller'
   *   '<S9>/Thrust Controller'
   *   '<S15>/adaptive_estimation'
   */
  { 11.0, 176.0, 704.0, 20.0, 100.0 },

  /* Variable: lat_dd_limit
   * Referenced by:
   *   '<S42>/lw1'
   *   '<S42>/up1'
   */
  3.0,

  /* Variable: lat_k_da
   * Referenced by: '<S39>/GainPole'
   */
  0.951229424500714,

  /* Variable: lat_k_da2
   * Referenced by:
   *   '<S40>/GainPole'
   *   '<S41>/GainPole'
   */
  0.99004983374916811,

  /* Variable: lat_s
   * Referenced by: '<S32>/lat_s'
   */
  500.0,

  /* Variable: lat_s2
   * Referenced by:
   *   '<S32>/lat_s2'
   *   '<S32>/lat_s2_ '
   */
  100.0,

  /* Variable: lp_den
   * Referenced by:
   *   '<S5>/low pass2'
   *   '<S8>/low pass3'
   *   '<S35>/low pass3'
   *   '<S36>/low pass3'
   */
  { 1.0, -1.9644605802052322, 0.96508117389913506 },

  /* Variable: lp_num
   * Referenced by:
   *   '<S5>/low pass2'
   *   '<S8>/low pass3'
   *   '<S35>/low pass3'
   *   '<S36>/low pass3'
   */
  { 0.00015514842347569949, 0.000310296846951399, 0.00015514842347569949 },

  /* Variable: manoeuvre_en
   * Referenced by: '<S1>/maneuver setpoint rd1'
   */
  1.0,

  /* Variable: phase_offset
   * Referenced by: '<S89>/phase offset'
   */
  0.0,

  /* Variable: running_time
   * Referenced by:
   *   '<Root>/running time'
   *   '<S1>/maneuver setpoint rd1'
   */
  14.0,

  /* Variable: second_delay
   * Referenced by: '<S26>/0.8'
   */
  0.8,

  /* Variable: start_delay
   * Referenced by:
   *   '<S67>/Constant'
   *   '<S73>/constant2'
   *   '<S47>/2'
   *   '<S26>/2'
   */
  0.000,

  /* Variable: th_xo
   * Referenced by: '<S8>/th_xo'
   */
  -0.01,

  /* Variable: th_yo
   * Referenced by: '<S8>/th_yo1'
   */
  0.08,

  /* Variable: thrust_gain
   * Referenced by: '<S9>/Thrust Controller'
   */
  270.0,

  /* Variable: thrust_offset
   * Referenced by: '<S8>/Fo'
   */
  0.0,

  /* Variable: vlp_den
   * Referenced by:
   *   '<S5>/low pass'
   *   '<S5>/low pass1'
   */
  { 1.0, -2.96230144608566, 2.9253101348486945, -0.96300211588509221 },

  /* Variable: vlp_num
   * Referenced by:
   *   '<S5>/low pass'
   *   '<S5>/low pass1'
   */
  { 8.21609742768361E-7, 2.4648292283050829E-6, 2.4648292283050829E-6,
    8.21609742768361E-7 },

  /* Mask Parameter: derivativefilter_ICPrevInput
   * Referenced by: '<S39>/Delay Input'
   */
  0.0,

  /* Mask Parameter: derivativefilter1_ICPrevInput
   * Referenced by: '<S40>/Delay Input'
   */
  0.0,

  /* Mask Parameter: derivativefilter2_ICPrevInput
   * Referenced by: '<S41>/Delay Input'
   */
  0.0,

  /* Mask Parameter: filteredderivative_ICPrevInput
   * Referenced by: '<S37>/Delay Input'
   */
  0.0,

  /* Mask Parameter: filteredderivative_ICPrevInpu_e
   * Referenced by: '<S38>/Delay Input'
   */
  0.0,

  /* Mask Parameter: derivativefilter_ICPrevOutput
   * Referenced by: '<S39>/Delay Output'
   */
  0.0,

  /* Mask Parameter: derivativefilter1_ICPrevOutput
   * Referenced by: '<S40>/Delay Output'
   */
  0.0,

  /* Mask Parameter: derivativefilter2_ICPrevOutput
   * Referenced by: '<S41>/Delay Output'
   */
  0.0,

  /* Mask Parameter: filteredderivative_ICPrevOutput
   * Referenced by: '<S37>/Delay Output'
   */
  0.0,

  /* Mask Parameter: filteredderivative_ICPrevOutp_k
   * Referenced by: '<S38>/Delay Output'
   */
  0.0,

  /* Mask Parameter: derivativefilter_ZeroZ
   * Referenced by: '<S39>/GainZero'
   */
  1.0,

  /* Mask Parameter: derivativefilter1_ZeroZ
   * Referenced by: '<S40>/GainZero'
   */
  1.0,

  /* Mask Parameter: derivativefilter2_ZeroZ
   * Referenced by: '<S41>/GainZero'
   */
  1.0,

  /* Mask Parameter: filteredderivative_ZeroZ
   * Referenced by: '<S37>/GainZero'
   */
  1.0,

  /* Mask Parameter: filteredderivative_ZeroZ_g
   * Referenced by: '<S38>/GainZero'
   */
  1.0,

  /* Mask Parameter: CompareToConstant1_const
   * Referenced by: '<S65>/Constant'
   */
  0.0,

  /* Mask Parameter: CompareToConstant1_const_i
   * Referenced by: '<S62>/Constant'
   */
  0.0,

  /* Mask Parameter: CompareToConstant1_const_e
   * Referenced by: '<S64>/Constant'
   */
  0.0,

  /* Mask Parameter: CompareToConstant1_const_c
   * Referenced by: '<S63>/Constant'
   */
  0.0,

  /* Mask Parameter: CompareToConstant1_const_e1
   * Referenced by: '<S69>/Constant'
   */
  -0.027,

  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S68>/Constant'
   */
  0.5,

  /* Mask Parameter: torelancetime_const
   * Referenced by: '<S74>/Constant'
   */
  0.05,

  /* Mask Parameter: CompareToConstant1_const_m
   * Referenced by: '<S23>/Constant'
   */
  0.0,

  /* Mask Parameter: CompareToConstant1_const_e0
   * Referenced by: '<S27>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S58>/zero1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S59>/zero1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S60>/zero1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S61>/zero1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S28>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S43>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S44>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S46>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S48>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S66>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S71>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S91>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S100>/Constant'
   */
  0.0,

  /* Computed Parameter: SaturatingRamp_gainval
   * Referenced by: '<S7>/SaturatingRamp'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S7>/SaturatingRamp'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S7>/constant'
   */
  1.0,

  /* Computed Parameter: SaturatingRamp_gainval_l
   * Referenced by: '<S76>/SaturatingRamp'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S76>/SaturatingRamp'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S76>/SaturatingRamp'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S76>/SaturatingRamp'
   */
  0.0,

  /* Expression: [0.05]
   * Referenced by: '<S81>/Discrete Transfer Fcn'
   */
  0.05,

  /* Expression: [1 -.95]
   * Referenced by: '<S81>/Discrete Transfer Fcn'
   */
  { 1.0, -0.95 },

  /* Expression: 0
   * Referenced by: '<S81>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S76>/PreGain'
   */
  1.0,

  /* Expression: 0.01
   * Referenced by: '<S6>/0.01'
   */
  0.01,

  /* Expression: 1
   * Referenced by: '<S79>/GainBias'
   */
  1.0,

  /* Computed Parameter: RampUpOrDown_gainval
   * Referenced by: '<S77>/RampUpOrDown'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S77>/RampUpOrDown'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S77>/RampUpOrDown'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S77>/RampUpOrDown'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S89>/Constant'
   */
  1.0,

  /* Expression: [0.05]
   * Referenced by: '<S86>/Discrete Transfer Fcn'
   */
  0.05,

  /* Expression: [1, -0.95]
   * Referenced by: '<S86>/Discrete Transfer Fcn'
   */
  { 1.0, -0.95 },

  /* Expression: 0
   * Referenced by: '<S86>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: pi/180
   * Referenced by: '<S89>/deg to rad'
   */
  0.017453292519943295,

  /* Computed Parameter: omegat_gainval
   * Referenced by: '<S89>/omega*t'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S89>/omega*t'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S89>/f'
   */
  -1.0,

  /* Expression: 2
   * Referenced by: '<S89>/2f'
   */
  2.0,

  /* Expression: 4
   * Referenced by: '<S92>/4'
   */
  4.0,

  /* Expression: 3
   * Referenced by: '<S92>/5'
   */
  3.0,

  /* Expression: 1
   * Referenced by: '<S92>/constant1'
   */
  1.0,

  /* Expression: [0.05]
   * Referenced by: '<S84>/Discrete Transfer Fcn'
   */
  0.05,

  /* Expression: [1 -.95]
   * Referenced by: '<S84>/Discrete Transfer Fcn'
   */
  { 1.0, -0.95 },

  /* Expression: 0
   * Referenced by: '<S84>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: [0.05]
   * Referenced by: '<S85>/Discrete Transfer Fcn'
   */
  0.05,

  /* Expression: [1 -.95]
   * Referenced by: '<S85>/Discrete Transfer Fcn'
   */
  { 1.0, -0.95 },

  /* Expression: 0
   * Referenced by: '<S85>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S79>/ground'
   */
  1.0,

  /* Expression: 0.01
   * Referenced by: '<S6>/0.1'
   */
  0.01,

  /* Expression: 1
   * Referenced by: '<S80>/GainBias'
   */
  1.0,

  /* Computed Parameter: RampUpOrDown_gainval_d
   * Referenced by: '<S78>/RampUpOrDown'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S78>/RampUpOrDown'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S78>/RampUpOrDown'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S78>/RampUpOrDown'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S98>/Constant'
   */
  1.0,

  /* Expression: [0.05]
   * Referenced by: '<S95>/Discrete Transfer Fcn'
   */
  0.05,

  /* Expression: [1 -.95]
   * Referenced by: '<S95>/Discrete Transfer Fcn'
   */
  { 1.0, -0.95 },

  /* Expression: 0
   * Referenced by: '<S95>/Discrete Transfer Fcn'
   */
  0.0,

  /* Computed Parameter: SmoothClock_gainval
   * Referenced by: '<S98>/SmoothClock'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S98>/SmoothClock'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S98>/f'
   */
  1.0,

  /* Expression: 2
   * Referenced by: '<S98>/2f'
   */
  2.0,

  /* Expression: 4
   * Referenced by: '<S101>/4'
   */
  4.0,

  /* Expression: 3
   * Referenced by: '<S101>/5'
   */
  3.0,

  /* Expression: 1
   * Referenced by: '<S101>/constant1'
   */
  1.0,

  /* Expression: [0.05]
   * Referenced by: '<S93>/Discrete Transfer Fcn'
   */
  0.05,

  /* Expression: [1 -.95]
   * Referenced by: '<S93>/Discrete Transfer Fcn'
   */
  { 1.0, -0.95 },

  /* Expression: 0
   * Referenced by: '<S93>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: [0.05]
   * Referenced by: '<S94>/Discrete Transfer Fcn'
   */
  0.05,

  /* Expression: [1 -.95]
   * Referenced by: '<S94>/Discrete Transfer Fcn'
   */
  { 1.0, -0.95 },

  /* Expression: 0
   * Referenced by: '<S94>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S80>/ground'
   */
  1.0,

  /* Expression: 0.01
   * Referenced by: '<S6>/0.2'
   */
  0.01,

  /* Expression: 0
   * Referenced by: '<S5>/low pass'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S5>/low pass1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S5>/low pass2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S35>/low pass3'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S36>/low pass3'
   */
  0.0,

  /* Expression: 30
   * Referenced by: '<S36>/Saturation'
   */
  30.0,

  /* Expression: -30
   * Referenced by: '<S36>/Saturation'
   */
  -30.0,

  /* Computed Parameter: time_gainval
   * Referenced by: '<S14>/time'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S14>/time'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S8>/adaptive_en'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S8>/Unit Delay'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S8>/Unit Delay1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S8>/low pass3'
   */
  0.0,

  /* Expression: 1/0.8
   * Referenced by: '<S50>/correction'
   */
  1.25,

  /* Expression: 10^6/9.81
   * Referenced by: '<S50>/mg'
   */
  101936.79918450559,

  /* Expression: 209.13
   * Referenced by: '<S50>/constant'
   */
  209.13,

  /* Expression: 1/3.7645
   * Referenced by: '<S50>/gain1'
   */
  0.26563952716164163,

  /* Expression: 0.5
   * Referenced by: '<S52>/half'
   */
  0.5,

  /* Expression: 0.0069148
   * Referenced by: '<S50>/gain2'
   */
  0.0069148,

  /* Expression: -0.36459
   * Referenced by: '<S50>/constant1'
   */
  -0.36459,

  /* Expression: 1.2
   * Referenced by: '<S2>/Gain'
   */
  1.2,

  /* Expression: 1
   * Referenced by: '<S50>/correction1'
   */
  1.0,

  /* Expression: 10^6
   * Referenced by: '<S50>/uNm2'
   */
  1.0E+6,

  /* Expression: 4.25
   * Referenced by: '<S52>/pitch gain'
   */
  4.25,

  /* Expression: 0.016834
   * Referenced by: '<S50>/gain6'
   */
  0.016834,

  /* Expression: -0.31839
   * Referenced by: '<S50>/constant2'
   */
  -0.31839,

  /* Expression: 1
   * Referenced by: '<S50>/correction2'
   */
  1.0,

  /* Expression: 10^6
   * Referenced by: '<S50>/uNm'
   */
  1.0E+6,

  /* Expression: 2*pi*0.05
   * Referenced by: '<S50>/uNm3'
   */
  0.31415926535897931,

  /* Expression: 4
   * Referenced by: '<S55>/4'
   */
  4.0,

  /* Expression: 3
   * Referenced by: '<S55>/5'
   */
  3.0,

  /* Expression: 1
   * Referenced by: '<S55>/constant1'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S54>/1'
   */
  1.0,

  /* Expression: 4
   * Referenced by: '<S56>/4'
   */
  4.0,

  /* Expression: 3
   * Referenced by: '<S56>/5'
   */
  3.0,

  /* Expression: 1
   * Referenced by: '<S56>/constant1'
   */
  1.0,

  /* Expression: 0.5
   * Referenced by: '<S52>/half1'
   */
  0.5,

  /* Expression: 0.0027082
   * Referenced by: '<S50>/gain7'
   */
  0.0027082,

  /* Expression: 0.13859
   * Referenced by: '<S50>/constant3'
   */
  0.13859,

  /* Expression: 1.00
   * Referenced by: '<Root>/Gain'
   */
  1.0,

  /* Expression: 2
   * Referenced by: '<S50>/correction3'
   */
  2.0,

  /* Expression: 10^6
   * Referenced by: '<S50>/uNm1'
   */
  1.0E+6,

  /* Expression: 0.5
   * Referenced by: '<S52>/half2'
   */
  0.5,

  /* Expression: 149
   * Referenced by: '<S51>/max allowed'
   */
  149.0,

  /* Expression: 2
   * Referenced by: '<S2>/p-p'
   */
  2.0,

  /* Expression: 2
   * Referenced by: '<S53>/p-p'
   */
  2.0,

  /* Expression: 2
   * Referenced by: '<S53>/2'
   */
  2.0,

  /* Expression: 3
   * Referenced by: '<S57>/3'
   */
  3.0,

  /* Expression: 1
   * Referenced by: '<S49>/1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Gain1'
   */
  0.0,

  /* Expression: 2
   * Referenced by: '<S2>/p-p1'
   */
  2.0,

  /* Expression: 2
   * Referenced by: '<S53>/1'
   */
  2.0,

  /* Computed Parameter: SaturatingRamp1_gainval
   * Referenced by: '<S73>/SaturatingRamp1'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S73>/SaturatingRamp1'
   */
  0.0,

  /* Expression: 2
   * Referenced by: '<S73>/constant1'
   */
  2.0,

  /* Computed Parameter: SaturatingRamp_gainval_g
   * Referenced by: '<S5>/SaturatingRamp'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S5>/SaturatingRamp'
   */
  0.0,

  /* Computed Parameter: SaturatingRamp_gainval_b
   * Referenced by: '<S4>/SaturatingRamp'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S4>/SaturatingRamp'
   */
  0.0,

  /* Computed Parameter: saturationramp_gainval
   * Referenced by: '<S26>/saturation ramp'
   */
  0.0005,

  /* Expression: 0
   * Referenced by: '<S26>/saturation ramp'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S26>/saturation ramp'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S26>/saturation ramp'
   */
  -1.0,

  /* Expression: est_fd1.num{1}
   * Referenced by: '<S20>/filter derivative 1'
   */
  { 90000.0, -179960.3009599467, 89960.30095994669 },

  /* Expression: est_fd1.den{1}
   * Referenced by: '<S20>/filter derivative 1'
   */
  { 1.0, -1.9408910670970161, 0.9417645335842485 },

  /* Expression: 0
   * Referenced by: '<S20>/filter derivative 1'
   */
  0.0,

  /* Expression: est_ft1.num{1}
   * Referenced by: '<S20>/filter 1'
   */
  { 0.0, 0.00044110044503657778, 0.000432366042195778 },

  /* Expression: est_ft1.den{1}
   * Referenced by: '<S20>/filter 1'
   */
  { 1.0, -1.9408910670970161, 0.9417645335842485 },

  /* Expression: 0
   * Referenced by: '<S20>/filter 1'
   */
  0.0,

  /* Expression: est_ft1.num{1}
   * Referenced by: '<S20>/filter1'
   */
  { 0.0, 0.00044110044503657778, 0.000432366042195778 },

  /* Expression: est_ft1.den{1}
   * Referenced by: '<S20>/filter1'
   */
  { 1.0, -1.9408910670970161, 0.9417645335842485 },

  /* Expression: 0
   * Referenced by: '<S20>/filter1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S22>/zero1'
   */
  0.0,

  /* Expression: 0.06
   * Referenced by: '<S19>/Saturation'
   */
  0.06,

  /* Expression: -0.06
   * Referenced by: '<S19>/Saturation'
   */
  -0.06,

  /* Expression: 0.06
   * Referenced by: '<S19>/Saturation1'
   */
  0.06,

  /* Expression: -0.06
   * Referenced by: '<S19>/Saturation1'
   */
  -0.06,

  /* Expression: 0.06
   * Referenced by: '<S19>/Saturation2'
   */
  0.06,

  /* Expression: -0.06
   * Referenced by: '<S19>/Saturation2'
   */
  -0.06,

  /* Expression: 0
   * Referenced by: '<S25>/zero1'
   */
  0.0,

  /* Computed Parameter: integratorwithsat_gainval
   * Referenced by: '<S16>/integrator with sat'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S16>/integrator with sat'
   */
  0.0,

  /* Expression: [1, 0.25, 0.25, 5e-8, 5e-8, 3e-8]
   * Referenced by: '<S16>/integrator with sat'
   */
  { 1.0, 0.25, 0.25, 5.0E-8, 5.0E-8, 3.0E-8 },

  /* Expression: [-1, -0.25, -0.25, -5e-8, -5e-8, -3e-8]
   * Referenced by: '<S16>/integrator with sat'
   */
  { -1.0, -0.25, -0.25, -5.0E-8, -5.0E-8, -3.0E-8 },

  /* Computed Parameter: time_gainval_g
   * Referenced by: '<S13>/time'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S13>/time'
   */
  0.0,

  /* Expression: Ff_filter.num{1}
   * Referenced by: '<S18>/filter 1'
   */
  { 0.0, 0.0099501662508319488 },

  /* Expression: Ff_filter.den{1}
   * Referenced by: '<S18>/filter 1'
   */
  { 1.0, -0.99004983374916811 },

  /* Expression: 0
   * Referenced by: '<S18>/filter 1'
   */
  0.0,

  /* Computed Parameter: counter_gainval
   * Referenced by: '<S26>/counter'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S26>/counter'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S26>/constant'
   */
  1.0,

  /* Expression: 25
   * Referenced by: '<S13>/att_stability_threshold'
   */
  25.0,

  /* Computed Parameter: counter1_gainval
   * Referenced by: '<S13>/counter1'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S13>/counter1'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S13>/steady time'
   */
  1.0,

  /* Computed Parameter: counter2_gainval
   * Referenced by: '<S13>/counter2'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S13>/counter2'
   */
  0.0,

  /* Computed Parameter: counter_gainval_o
   * Referenced by: '<S47>/counter'
   */
  0.0001,

  /* Expression: 0
   * Referenced by: '<S47>/counter'
   */
  0.0,

  /* Expression: 1.2
   * Referenced by: '<S47>/1.5'
   */
  1.2,

  /* Expression: 1
   * Referenced by: '<S47>/constant'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S14>/constant'
   */
  1.0,

  /* Expression: 300
   * Referenced by: '<Root>/bias voltage'
   */
  300.0,

  /* Expression: 1
   * Referenced by: '<S4>/constant'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S73>/constant'
   */
  1.0,

  /* Expression: 2
   * Referenced by: '<S82>/Gain'
   */
  2.0,

  /* Expression: 1
   * Referenced by: '<S82>/Constant'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S77>/PreGain'
   */
  1.0,

  /* Expression: 1/2
   * Referenced by: '<S6>/p2p to amp'
   */
  0.5,

  /* Expression: 0.5
   * Referenced by: '<S6>/half'
   */
  0.5,

  /* Expression: 2
   * Referenced by: '<S87>/Gain'
   */
  2.0,

  /* Expression: 1
   * Referenced by: '<S87>/Constant'
   */
  1.0,

  /* Expression: 2*pi
   * Referenced by: '<S89>/2pi'
   */
  6.2831853071795862,

  /* Expression: 1
   * Referenced by: '<S78>/PreGain'
   */
  1.0,

  /* Expression: 2
   * Referenced by: '<S96>/Gain'
   */
  2.0,

  /* Expression: 1
   * Referenced by: '<S96>/Constant'
   */
  1.0,

  /* Expression: 2*pi
   * Referenced by: '<S98>/2pi'
   */
  6.2831853071795862,

  /* Expression: 1
   * Referenced by: '<S6>/GainBias'
   */
  1.0,

  /* Computed Parameter: Constant_Value_pz
   * Referenced by: '<S83>/Constant'
   */
  0U,

  /* Computed Parameter: Constant_Value_d
   * Referenced by: '<S90>/Constant'
   */
  0U,

  /* Computed Parameter: Constant_Value_ij
   * Referenced by: '<S99>/Constant'
   */
  0U,

  /* Computed Parameter: Constant_Value_i5
   * Referenced by: '<S70>/Constant'
   */
  0U,

  /* Computed Parameter: Switch1_Threshold
   * Referenced by: '<S61>/Switch1'
   */
  1U,

  /* Computed Parameter: Switch1_Threshold_i
   * Referenced by: '<S58>/Switch1'
   */
  1U,

  /* Computed Parameter: Switch1_Threshold_l
   * Referenced by: '<S60>/Switch1'
   */
  1U,

  /* Computed Parameter: Switch1_Threshold_c
   * Referenced by: '<S59>/Switch1'
   */
  1U,

  /* Computed Parameter: Switch1_Threshold_cr
   * Referenced by: '<S22>/Switch1'
   */
  1U,

  /* Computed Parameter: Switch1_Threshold_e
   * Referenced by: '<S25>/Switch1'
   */
  1U
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
