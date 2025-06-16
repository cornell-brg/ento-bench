/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: integrated_controller.c
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
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_powd_snf(real_T u0, real_T u1);
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T tmp;
  real_T tmp_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void integrated_controller_step(void)
{
  /* local block i/o variables */
  real_T rtb_TmpSignalConversionAtfilter[3];
  real_T rtb_PoleYk1UkZeroUk1[3];
  real_T rtb_PoleYk1UkZeroUk1_j[3];
  real_T rtb_PoleYk1UkZeroUk1_h[3];
  real_T rtb_att_s[9];
  real_T rtb_PoleYk1UkZeroUk1_n[9];
  real_T rtb_att_s_a;
  real_T rtb_PoleYk1UkZeroUk1_e;
  real_T rtb_typeconv;
  real_T rtb_Switch1_p[6];
  real_T rtb_Saturation_f;
  real_T rtb_Saturation1;
  real_T rtb_Saturation2;
  real_T rtb_DataTypeConversion_d;
  real_T rtb_DataTypeConversion_p;
  real_T rtb_DataTypeConversion1;
  real_T rtb_DataTypeConversion2;
  real_T rtb_Gain_i;
  real_T rtb_Add_c;
  real_T rtb_Add_en;
  real_T rtb_upi;
  real_T rtb_Add_io;
  real_T rtb_upi_m;

  {
    real_T tmp[36];
    real_T Y[24];
    real_T tmp_0[24];
    real_T Ya_t[18];
    real_T rtb_Ya[18];
    real_T Wm[9];
    real_T filter1[9];
    real_T filter1_f[9];
    real_T lowpass3[9];
    real_T rtb_UnitDelay[6];
    real_T rtb_Yt[6];
    real_T rtb_a_hat_dot[6];
    real_T rtb_enable1[6];
    real_T rtb_enable2[6];
    real_T tmp_1[6];
    real_T S[4];
    real_T b_gamma[3];
    real_T b_tmp[3];
    real_T rtb_Sa[3];
    real_T rtb_rd_ddd[3];
    real_T rtb_rd_dddd[3];
    real_T torque_xyz[3];
    real_T xhat[3];
    real_T xhatd[3];
    real_T yhat[3];
    real_T zhat[3];
    real_T absxk;
    real_T b_a;
    real_T e_c;
    real_T f_c;
    real_T rel_time;
    real_T rel_time_0;
    real_T rel_time_1;
    real_T rtb_Abs_n;
    real_T rtb_Add;
    real_T rtb_BC;
    real_T rtb_CBC;
    real_T rtb_Cv;
    real_T rtb_Cv_j;
    real_T rtb_Force;
    real_T rtb_MinMax3_idx_0;
    real_T rtb_MinMax3_idx_1;
    real_T rtb_MinMax3_idx_2;
    real_T rtb_PreGain;
    real_T rtb_Saturation;
    real_T rtb_Sum;
    real_T rtb_Sum_j;
    real_T rtb_Tpitch;
    real_T rtb_Troll;
    real_T rtb_Tyaw;
    real_T rtb_VmaxPhi;
    real_T rtb_add_b;
    real_T rtb_divide1;
    real_T rtb_divide_l;
    real_T rtb_enable1_n;
    real_T rtb_enable2_a;
    real_T rtb_enable3;
    real_T rtb_enable4;
    real_T rtb_gain1;
    real_T rtb_half;
    real_T rtb_half2;
    real_T rtb_lat_s_idx_1;
    real_T rtb_lowpass2;
    real_T rtb_multiply1;
    real_T rtb_multiply3;
    real_T rtb_multiply4;
    real_T rtb_multiply_c;
    real_T rtb_p2ptoamp;
    real_T rtb_pitchgain;
    real_T rtb_pow2_a;
    real_T rtb_pp;
    real_T rtb_pp1;
    real_T rtb_pp_o;
    real_T rtb_rd_d_idx_0;
    real_T rtb_rd_d_idx_2;
    real_T rtb_rd_dd_idx_0;
    real_T rtb_rd_ddd_f;
    real_T rtb_rd_dddd_b;
    real_T rtb_rd_idx_0;
    real_T rtb_totalvolthalfside;
    real_T rtb_uA;
    real_T rtb_w_idx_0;
    real_T rtb_w_idx_1;
    real_T rtb_w_idx_2;
    real_T scale;
    real_T t;
    real_T xhatd_0;
    real_T y_tmp;
    real_T y_tmp_0;
    real_T y_tmp_1;
    real_T zhat_0;
    int32_T nz[3];
    int32_T r1;
    int32_T r2;
    int32_T rtb_times_j;
    int32_T rtemp;
    int32_T xpageoffset;
    uint8_T rtb_en;
    uint8_T rtb_en_g;
    boolean_T x[9];
    static const real_T y[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25 };

    /* Outport: '<Root>/time' incorporates:
     *  DiscreteIntegrator: '<S7>/SaturatingRamp'
     */
    rtY.time = rtDW.SaturatingRamp_DSTATE;

    /* Outport: '<Root>/X (m)1' incorporates:
     *  Inport: '<Root>/X(m)'
     */
    rtY.Xm1 = rtU.Xm;

    /* Outport: '<Root>/Y (m)1' incorporates:
     *  Inport: '<Root>/Y(m)'
     */
    rtY.Ym1 = rtU.Ym;

    /* Outport: '<Root>/Z (m)1' incorporates:
     *  Inport: '<Root>/Z(m)'
     */
    rtY.Zm1 = rtU.Zm;

    /* Outport: '<Root>/alpha (radians)1' incorporates:
     *  Inport: '<Root>/alpha(radians)'
     */
    rtY.alpharadians1 = rtU.alpharadians;

    /* Outport: '<Root>/beta (radians)1' incorporates:
     *  Inport: '<Root>/beta(radians)'
     */
    rtY.betaradians1 = rtU.betaradians;

    /* Outport: '<Root>/gamma (radians)1' incorporates:
     *  Inport: '<Root>/gamma(radians)'
     */
    rtY.gammaradians1 = rtU.gammaradians;

    /* Gain: '<S76>/PreGain' incorporates:
     *  DiscreteIntegrator: '<S76>/SaturatingRamp'
     *  DiscreteTransferFcn: '<S81>/Discrete Transfer Fcn'
     *  Product: '<S76>/Product'
     */
    rtb_PreGain = rtP.DiscreteTransferFcn_NumCoef *
      rtDW.DiscreteTransferFcn_states * rtDW.SaturatingRamp_DSTATE_j *
      rtP.PreGain_Gain;

    /* Outport: '<Root>/bias' incorporates:
     *  Gain: '<S6>/0.01'
     */
    rtY.bias = rtP.u01_Gain * rtb_PreGain;

    /* DiscreteTransferFcn: '<S86>/Discrete Transfer Fcn' */
    rel_time = rtP.DiscreteTransferFcn_NumCoef_g *
      rtDW.DiscreteTransferFcn_states_n;

    /* Sum: '<S89>/Add' incorporates:
     *  Constant: '<S89>/phase offset'
     *  DiscreteIntegrator: '<S89>/omega*t'
     *  Gain: '<S89>/deg to rad'
     */
    rtb_Add = rtP.degtorad_Gain * rtP.phase_offset + rtDW.omegat_DSTATE;

    /* Abs: '<S89>/Abs' incorporates:
     *  Abs: '<S89>/Abs1'
     *  DiscreteTransferFcn: '<S86>/Discrete Transfer Fcn'
     */
    scale = fabs(rel_time);

    /* Sum: '<S77>/Sum' incorporates:
     *  Abs: '<S89>/Abs'
     *  Constant: '<S89>/Constant'
     *  Constant: '<S92>/4'
     *  Constant: '<S92>/5'
     *  Constant: '<S92>/constant1'
     *  DiscreteTransferFcn: '<S84>/Discrete Transfer Fcn'
     *  DiscreteTransferFcn: '<S85>/Discrete Transfer Fcn'
     *  DiscreteTransferFcn: '<S86>/Discrete Transfer Fcn'
     *  Gain: '<S89>/2f'
     *  Gain: '<S89>/f'
     *  Gain: '<S92>/Gain2'
     *  Gain: '<S92>/Gain3'
     *  Gain: '<S92>/Gain7'
     *  Gain: '<S92>/Gain8'
     *  Math: '<S92>/pow2'
     *  Math: '<S92>/pow3'
     *  Math: '<S92>/square1'
     *  Product: '<S77>/signal with gain'
     *  Product: '<S89>/normalised'
     *  Product: '<S89>/signal with gain1'
     *  Product: '<S89>/signal with gain2'
     *  Sum: '<S89>/1-mu'
     *  Sum: '<S89>/combined sines'
     *  Sum: '<S92>/Sum1'
     *  Trigonometry: '<S89>/2nd harmonic'
     *  Trigonometry: '<S89>/first harmonic'
     */
    rtb_Sum = ((rtP.Constant_Value_b - scale) * sin(rtb_Add) + rtP.f_Gain *
               rel_time * sin(rtP.uf_Gain * rtb_Add)) / ((((rtP.Pvmax[0] *
      rt_powd_snf(scale, rtP.u_Value) + rtP.Pvmax[1] * rt_powd_snf(scale,
      rtP.u_Value_d)) + scale * scale * rtP.Pvmax[2]) + rtP.Pvmax[3] * scale) +
      rtP.constant1_Value) * (rtP.DiscreteTransferFcn_NumCoef_m *
      rtDW.DiscreteTransferFcn_states_a) + rtP.DiscreteTransferFcn_NumCoef_j *
      rtDW.DiscreteTransferFcn_states_ac;

    /* Gain: '<S79>/GainBias' */
    rtb_MinMax3_idx_2 = rtP.GainBias_Gain * rtb_PreGain;

    /* Product: '<S77>/OutputWithRamping' incorporates:
     *  DiscreteIntegrator: '<S77>/RampUpOrDown'
     */
    rtb_w_idx_0 = rtDW.RampUpOrDown_DSTATE * rtb_Sum;

    /* MinMax: '<S79>/MinOf' */
    if ((rtb_MinMax3_idx_2 <= rtb_w_idx_0) || rtIsNaN(rtb_w_idx_0)) {
      rtb_w_idx_0 = rtb_MinMax3_idx_2;
    }

    /* End of MinMax: '<S79>/MinOf' */

    /* MinMax: '<S79>/MaxOf' incorporates:
     *  Constant: '<S79>/ground'
     */
    if ((!(rtb_w_idx_0 >= rtP.ground_Value)) && (!rtIsNaN(rtP.ground_Value))) {
      rtb_w_idx_0 = rtP.ground_Value;
    }

    /* Outport: '<Root>/LW signal' incorporates:
     *  Gain: '<S6>/0.1'
     *  MinMax: '<S79>/MaxOf'
     */
    rtY.LWsignal = rtP.u1_Gain * rtb_w_idx_0;

    /* DiscreteTransferFcn: '<S95>/Discrete Transfer Fcn' */
    rel_time = rtP.DiscreteTransferFcn_NumCoef_d *
      rtDW.DiscreteTransferFcn_states_av;

    /* Abs: '<S98>/Abs' incorporates:
     *  Abs: '<S98>/Abs1'
     *  DiscreteTransferFcn: '<S95>/Discrete Transfer Fcn'
     */
    scale = fabs(rel_time);

    /* Sum: '<S78>/Sum' incorporates:
     *  Abs: '<S98>/Abs'
     *  Constant: '<S101>/4'
     *  Constant: '<S101>/5'
     *  Constant: '<S101>/constant1'
     *  Constant: '<S98>/Constant'
     *  DiscreteIntegrator: '<S98>/SmoothClock'
     *  DiscreteTransferFcn: '<S93>/Discrete Transfer Fcn'
     *  DiscreteTransferFcn: '<S94>/Discrete Transfer Fcn'
     *  DiscreteTransferFcn: '<S95>/Discrete Transfer Fcn'
     *  Gain: '<S101>/Gain2'
     *  Gain: '<S101>/Gain3'
     *  Gain: '<S101>/Gain7'
     *  Gain: '<S101>/Gain8'
     *  Gain: '<S98>/2f'
     *  Gain: '<S98>/f'
     *  Math: '<S101>/pow2'
     *  Math: '<S101>/pow3'
     *  Math: '<S101>/square1'
     *  Product: '<S78>/signal with gain'
     *  Product: '<S98>/normalised'
     *  Product: '<S98>/signal with gain1'
     *  Product: '<S98>/signal with gain2'
     *  Sum: '<S101>/Sum1'
     *  Sum: '<S98>/1-mu'
     *  Sum: '<S98>/combined sines'
     *  Trigonometry: '<S98>/2nd harmonic'
     *  Trigonometry: '<S98>/first harmonic'
     */
    rtb_Sum_j = ((rtP.Constant_Value_hm - scale) * sin(rtDW.SmoothClock_DSTATE)
                 + rtP.f_Gain_l * rel_time * sin(rtP.uf_Gain_p *
      rtDW.SmoothClock_DSTATE)) / ((((rtP.Pvmax[0] * rt_powd_snf(scale,
      rtP.u_Value_k) + rtP.Pvmax[1] * rt_powd_snf(scale, rtP.u_Value_n)) + scale
      * scale * rtP.Pvmax[2]) + rtP.Pvmax[3] * scale) + rtP.constant1_Value_c) *
      (rtP.DiscreteTransferFcn_NumCoef_o * rtDW.DiscreteTransferFcn_states_p) +
      rtP.DiscreteTransferFcn_NumCoef_oh * rtDW.DiscreteTransferFcn_states_a5;

    /* Gain: '<S80>/GainBias' */
    rtb_MinMax3_idx_2 = rtP.GainBias_Gain_h * rtb_PreGain;

    /* Product: '<S78>/OutputWithRamping' incorporates:
     *  DiscreteIntegrator: '<S78>/RampUpOrDown'
     */
    rtb_w_idx_0 = rtDW.RampUpOrDown_DSTATE_k * rtb_Sum_j;

    /* MinMax: '<S80>/MinOf' */
    if ((rtb_MinMax3_idx_2 <= rtb_w_idx_0) || rtIsNaN(rtb_w_idx_0)) {
      rtb_w_idx_0 = rtb_MinMax3_idx_2;
    }

    /* End of MinMax: '<S80>/MinOf' */

    /* MinMax: '<S80>/MaxOf' incorporates:
     *  Constant: '<S80>/ground'
     */
    if ((!(rtb_w_idx_0 >= rtP.ground_Value_o)) && (!rtIsNaN(rtP.ground_Value_o)))
    {
      rtb_w_idx_0 = rtP.ground_Value_o;
    }

    /* Outport: '<Root>/RW signal' incorporates:
     *  Gain: '<S6>/0.2'
     *  MinMax: '<S80>/MaxOf'
     */
    rtY.RWsignal = rtP.u2_Gain * rtb_w_idx_0;

    /* Outport: '<Root>/LW_NoRamp_NoBias' */
    rtY.LW_NoRamp_NoBias = rtb_Sum;

    /* Outport: '<Root>/RW_NoRamp_NoBias' */
    rtY.RW_NoRamp_NoBias = rtb_Sum_j;

    /* DiscreteTransferFcn: '<S5>/low pass' incorporates:
     *  Inport: '<Root>/X(m)'
     */
    rtDW.lowpass_tmp = (((rtU.Xm - rtDW.lowpass_states[0] * rtP.vlp_den[1]) -
                         rtDW.lowpass_states[1] * rtP.vlp_den[2]) -
                        rtDW.lowpass_states[2] * rtP.vlp_den[3]) / rtP.vlp_den[0];
    rel_time_0 = ((rtP.vlp_num[0] * rtDW.lowpass_tmp + rtDW.lowpass_states[0] *
                   rtP.vlp_num[1]) + rtDW.lowpass_states[1] * rtP.vlp_num[2]) +
      rtDW.lowpass_states[2] * rtP.vlp_num[3];

    /* DiscreteTransferFcn: '<S5>/low pass1' incorporates:
     *  Inport: '<Root>/Y(m)'
     */
    rtDW.lowpass1_tmp = (((rtU.Ym - rtDW.lowpass1_states[0] * rtP.vlp_den[1]) -
                          rtDW.lowpass1_states[1] * rtP.vlp_den[2]) -
                         rtDW.lowpass1_states[2] * rtP.vlp_den[3]) /
      rtP.vlp_den[0];
    rel_time_1 = ((rtP.vlp_num[0] * rtDW.lowpass1_tmp + rtDW.lowpass1_states[0] *
                   rtP.vlp_num[1]) + rtDW.lowpass1_states[1] * rtP.vlp_num[2]) +
      rtDW.lowpass1_states[2] * rtP.vlp_num[3];

    /* DiscreteTransferFcn: '<S5>/low pass2' incorporates:
     *  Inport: '<Root>/Z(m)'
     */
    rtDW.lowpass2_tmp = ((rtU.Zm - rtDW.lowpass2_states[0] * rtP.lp_den[1]) -
                         rtDW.lowpass2_states[1] * rtP.lp_den[2]) / rtP.lp_den[0];
    rel_time = (rtP.lp_num[0] * rtDW.lowpass2_tmp + rtDW.lowpass2_states[0] *
                rtP.lp_num[1]) + rtDW.lowpass2_states[1] * rtP.lp_num[2];
    rtb_lowpass2 = rel_time;

    /* SignalConversion generated from: '<S20>/filter derivative 1' incorporates:
     *  DiscreteTransferFcn: '<S5>/low pass'
     *  DiscreteTransferFcn: '<S5>/low pass1'
     *  DiscreteTransferFcn: '<S5>/low pass2'
     */
    rtb_TmpSignalConversionAtfilter[0] = rel_time_0;
    rtb_TmpSignalConversionAtfilter[1] = rel_time_1;
    rtb_TmpSignalConversionAtfilter[2] = rel_time;

    /* Sum: '<S39>/Sum' incorporates:
     *  Gain: '<S39>/GainPole'
     *  Gain: '<S39>/GainZero'
     *  UnitDelay: '<S39>/Delay Input'
     *  UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S39>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S39>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1[0] = (rtb_TmpSignalConversionAtfilter[0] -
      rtP.derivativefilter_ZeroZ * rtDW.DelayInput_DSTATE[0]) + rtP.lat_k_da *
      rtDW.DelayOutput_DSTATE[0];

    /* Sum: '<S40>/Sum' incorporates:
     *  Gain: '<S40>/GainPole'
     *  Gain: '<S40>/GainZero'
     *  UnitDelay: '<S40>/Delay Input'
     *  UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S40>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S40>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1_j[0] = (rtb_TmpSignalConversionAtfilter[0] -
      rtP.derivativefilter1_ZeroZ * rtDW.DelayInput_DSTATE_o[0]) + rtP.lat_k_da2
      * rtDW.DelayOutput_DSTATE_k[0];

    /* Sum: '<S41>/Sum' incorporates:
     *  Gain: '<S41>/GainPole'
     *  Gain: '<S41>/GainZero'
     *  UnitDelay: '<S41>/Delay Input'
     *  UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S41>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S41>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1_h[0] = (rtb_PoleYk1UkZeroUk1_j[0] -
      rtP.derivativefilter2_ZeroZ * rtDW.DelayInput_DSTATE_i[0]) + rtP.lat_k_da2
      * rtDW.DelayOutput_DSTATE_a[0];

    /* Gain: '<S32>/lat_s2_ ' incorporates:
     *  Gain: '<S32>/lat_s2'
     */
    rtb_MinMax3_idx_0 = rtP.lat_s2 * rtb_PoleYk1UkZeroUk1_h[0] * rtP.lat_s2;

    /* MinMax: '<S42>/MinMax2' */
    if ((rtP.lat_dd_limit <= rtb_MinMax3_idx_0) || rtIsNaN(rtb_MinMax3_idx_0)) {
      rtb_MinMax3_idx_0 = rtP.lat_dd_limit;
    }

    /* MinMax: '<S42>/MinMax3' incorporates:
     *  Constant: '<S42>/lw1'
     */
    if ((-rtP.lat_dd_limit >= rtb_MinMax3_idx_0) || rtIsNaN(rtb_MinMax3_idx_0))
    {
      rtb_MinMax3_idx_0 = -rtP.lat_dd_limit;
    }

    /* Sum: '<S39>/Sum' incorporates:
     *  Gain: '<S39>/GainPole'
     *  Gain: '<S39>/GainZero'
     *  UnitDelay: '<S39>/Delay Input'
     *  UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S39>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S39>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1[1] = (rtb_TmpSignalConversionAtfilter[1] -
      rtP.derivativefilter_ZeroZ * rtDW.DelayInput_DSTATE[1]) + rtP.lat_k_da *
      rtDW.DelayOutput_DSTATE[1];

    /* Gain: '<S32>/lat_s' */
    rtb_lat_s_idx_1 = rtP.lat_s * rtb_PoleYk1UkZeroUk1[1];

    /* Sum: '<S40>/Sum' incorporates:
     *  Gain: '<S40>/GainPole'
     *  Gain: '<S40>/GainZero'
     *  UnitDelay: '<S40>/Delay Input'
     *  UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S40>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S40>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1_j[1] = (rtb_TmpSignalConversionAtfilter[1] -
      rtP.derivativefilter1_ZeroZ * rtDW.DelayInput_DSTATE_o[1]) + rtP.lat_k_da2
      * rtDW.DelayOutput_DSTATE_k[1];

    /* Sum: '<S41>/Sum' incorporates:
     *  Gain: '<S41>/GainPole'
     *  Gain: '<S41>/GainZero'
     *  UnitDelay: '<S41>/Delay Input'
     *  UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S41>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S41>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1_h[1] = (rtb_PoleYk1UkZeroUk1_j[1] -
      rtP.derivativefilter2_ZeroZ * rtDW.DelayInput_DSTATE_i[1]) + rtP.lat_k_da2
      * rtDW.DelayOutput_DSTATE_a[1];

    /* Gain: '<S32>/lat_s2_ ' incorporates:
     *  Gain: '<S32>/lat_s2'
     */
    rtb_MinMax3_idx_1 = rtP.lat_s2 * rtb_PoleYk1UkZeroUk1_h[1] * rtP.lat_s2;

    /* MinMax: '<S42>/MinMax2' */
    if ((rtP.lat_dd_limit <= rtb_MinMax3_idx_1) || rtIsNaN(rtb_MinMax3_idx_1)) {
      rtb_MinMax3_idx_1 = rtP.lat_dd_limit;
    }

    /* MinMax: '<S42>/MinMax3' incorporates:
     *  Constant: '<S42>/lw1'
     */
    if ((-rtP.lat_dd_limit >= rtb_MinMax3_idx_1) || rtIsNaN(rtb_MinMax3_idx_1))
    {
      rtb_MinMax3_idx_1 = -rtP.lat_dd_limit;
    }

    /* Sum: '<S39>/Sum' incorporates:
     *  Gain: '<S39>/GainPole'
     *  Gain: '<S39>/GainZero'
     *  UnitDelay: '<S39>/Delay Input'
     *  UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S39>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S39>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1[2] = (rtb_TmpSignalConversionAtfilter[2] -
      rtP.derivativefilter_ZeroZ * rtDW.DelayInput_DSTATE[2]) + rtP.lat_k_da *
      rtDW.DelayOutput_DSTATE[2];

    /* Sum: '<S40>/Sum' incorporates:
     *  Gain: '<S40>/GainPole'
     *  Gain: '<S40>/GainZero'
     *  UnitDelay: '<S40>/Delay Input'
     *  UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S40>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S40>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1_j[2] = (rtb_TmpSignalConversionAtfilter[2] -
      rtP.derivativefilter1_ZeroZ * rtDW.DelayInput_DSTATE_o[2]) + rtP.lat_k_da2
      * rtDW.DelayOutput_DSTATE_k[2];

    /* Sum: '<S41>/Sum' incorporates:
     *  Gain: '<S41>/GainPole'
     *  Gain: '<S41>/GainZero'
     *  UnitDelay: '<S41>/Delay Input'
     *  UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S41>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S41>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1_h[2] = (rtb_PoleYk1UkZeroUk1_j[2] -
      rtP.derivativefilter2_ZeroZ * rtDW.DelayInput_DSTATE_i[2]) + rtP.lat_k_da2
      * rtDW.DelayOutput_DSTATE_a[2];

    /* Gain: '<S32>/lat_s2_ ' incorporates:
     *  Gain: '<S32>/lat_s2'
     */
    rtb_MinMax3_idx_2 = rtP.lat_s2 * rtb_PoleYk1UkZeroUk1_h[2] * rtP.lat_s2;

    /* MinMax: '<S42>/MinMax2' */
    if ((rtP.lat_dd_limit <= rtb_MinMax3_idx_2) || rtIsNaN(rtb_MinMax3_idx_2)) {
      rtb_MinMax3_idx_2 = rtP.lat_dd_limit;
    }

    /* MinMax: '<S42>/MinMax3' incorporates:
     *  Constant: '<S42>/lw1'
     */
    if ((-rtP.lat_dd_limit >= rtb_MinMax3_idx_2) || rtIsNaN(rtb_MinMax3_idx_2))
    {
      rtb_MinMax3_idx_2 = -rtP.lat_dd_limit;
    }

    /* MATLAB Function: '<S31>/Rotation Matrix' incorporates:
     *  Inport: '<Root>/alpha(radians)'
     *  Inport: '<Root>/beta(radians)'
     *  Inport: '<Root>/gamma(radians)'
     */
    /* :  eu_x = euler_angles(1); */
    /* :  eu_y = euler_angles(2); */
    /* :  eu_z = euler_angles(3); */
    /* :  R = [cos(eu_y)*cos(eu_z); -cos(eu_y)*sin(eu_z); sin(eu_y); ... */
    /* :      cos(eu_x)*sin(eu_z)+sin(eu_x)*sin(eu_y)*cos(eu_z); cos(eu_x)*cos(eu_z)-sin(eu_x)*sin(eu_y)*sin(eu_z); -sin(eu_x)*cos(eu_y); ... */
    /* :      sin(eu_x)*sin(eu_z)-cos(eu_x)*sin(eu_y)*cos(eu_z); sin(eu_x)*cos(eu_z)+cos(eu_x)*sin(eu_y)*sin(eu_z); cos(eu_x)*cos(eu_y)]; */
    rel_time = cos(rtU.betaradians);
    scale = sin(rtU.gammaradians);
    absxk = cos(rtU.gammaradians);
    t = cos(rtU.alpharadians);
    rtb_rd_d_idx_2 = sin(rtU.alpharadians);
    rtb_rd_idx_0 = sin(rtU.betaradians);
    rtDW.R[0] = rel_time * absxk;
    rtDW.R[1] = -rel_time * scale;
    rtDW.R[2] = rtb_rd_idx_0;
    rtDW.R[3] = rtb_rd_d_idx_2 * rtb_rd_idx_0 * absxk + t * scale;
    rtDW.R[4] = t * absxk - sin(rtU.alpharadians) * sin(rtU.betaradians) * scale;
    rtDW.R[5] = -rtb_rd_d_idx_2 * rel_time;
    rtDW.R[6] = rtb_rd_d_idx_2 * scale - t * rtb_rd_idx_0 * absxk;
    rtDW.R[7] = cos(rtU.alpharadians) * sin(rtU.betaradians) * scale +
      rtb_rd_d_idx_2 * absxk;
    rtDW.R[8] = t * rel_time;
    for (r1 = 0; r1 < 9; r1++) {
      /* Gain: '<S35>/att_s' */
      rtb_att_s[r1] = rtP.att_s * rtDW.R[r1];

      /* Sum: '<S37>/Sum' incorporates:
       *  Gain: '<S37>/GainPole'
       *  Gain: '<S37>/GainZero'
       *  UnitDelay: '<S37>/Delay Input'
       *  UnitDelay: '<S37>/Delay Output'
       *
       * Block description for '<S37>/Sum':
       *
       *  Add in CPU
       *
       * Block description for '<S37>/GainPole':
       *
       *  Multiply in CPU
       *
       * Block description for '<S37>/GainZero':
       *
       *  Multiply in CPU
       *
       * Block description for '<S37>/Delay Input':
       *
       *  Store in Global RAM
       *
       * Block description for '<S37>/Delay Output':
       *
       *  Store in Global RAM
       */
      rtb_PoleYk1UkZeroUk1_n[r1] = (rtb_att_s[r1] - rtP.filteredderivative_ZeroZ
        * rtDW.DelayInput_DSTATE_j[r1]) + rtP.k_da *
        rtDW.DelayOutput_DSTATE_b[r1];

      /* DiscreteTransferFcn: '<S35>/low pass3' */
      xpageoffset = r1 << 1;
      rtb_w_idx_0 = rtDW.lowpass3_states[xpageoffset + 1];
      rtDW.lowpass3_tmp[r1] = ((rtb_PoleYk1UkZeroUk1_n[r1] - rtP.lp_den[1] *
        rtDW.lowpass3_states[xpageoffset]) - rtb_w_idx_0 * rtP.lp_den[2]) /
        rtP.lp_den[0];
      lowpass3[r1] = (rtP.lp_num[0] * rtDW.lowpass3_tmp[r1] + rtP.lp_num[1] *
                      rtDW.lowpass3_states[xpageoffset]) + rtb_w_idx_0 *
        rtP.lp_num[2];
    }

    /* MATLAB Function: '<S31>/Rotation Matrix1' */
    /* :  w = zeros(3,1); */
    /* :  w(1) = R(3)*Rdot(2) + R(6)*Rdot(5) + R(9)*Rdot(8); */
    rtb_w_idx_0 = (lowpass3[1] * rtDW.R[2] + lowpass3[4] * rtDW.R[5]) +
      lowpass3[7] * rtDW.R[8];

    /* :  w(2) = R(1)*Rdot(3) + R(4)*Rdot(6) + R(7)*Rdot(9); */
    rtb_w_idx_1 = (rtDW.R[0] * lowpass3[2] + rtDW.R[3] * lowpass3[5]) + rtDW.R[6]
      * lowpass3[8];

    /* :  w(3) = R(2)*Rdot(1) + R(5)*Rdot(4) + R(8)*Rdot(7); */
    rtb_w_idx_2 = (lowpass3[0] * rtDW.R[1] + lowpass3[3] * rtDW.R[4]) +
      lowpass3[6] * rtDW.R[7];

    /* Gain: '<S36>/att_s' incorporates:
     *  MATLAB Function: '<S31>/Rotation Matrix1'
     */
    /* :  wz = w(3); */
    rtb_att_s_a = rtP.att_s2 * rtb_w_idx_2;

    /* Sum: '<S38>/Sum' incorporates:
     *  Gain: '<S38>/GainPole'
     *  Gain: '<S38>/GainZero'
     *  UnitDelay: '<S38>/Delay Input'
     *  UnitDelay: '<S38>/Delay Output'
     *
     * Block description for '<S38>/Sum':
     *
     *  Add in CPU
     *
     * Block description for '<S38>/GainPole':
     *
     *  Multiply in CPU
     *
     * Block description for '<S38>/GainZero':
     *
     *  Multiply in CPU
     *
     * Block description for '<S38>/Delay Input':
     *
     *  Store in Global RAM
     *
     * Block description for '<S38>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtb_PoleYk1UkZeroUk1_e = (rtb_att_s_a - rtP.filteredderivative_ZeroZ_g *
      rtDW.DelayInput_DSTATE_c) + rtP.k_da2 * rtDW.DelayOutput_DSTATE_c;

    /* DiscreteTransferFcn: '<S36>/low pass3' */
    rtDW.lowpass3_tmp_b = ((rtb_PoleYk1UkZeroUk1_e - rtDW.lowpass3_states_b[0] *
      rtP.lp_den[1]) - rtDW.lowpass3_states_b[1] * rtP.lp_den[2]) / rtP.lp_den[0];
    rel_time = (rtP.lp_num[0] * rtDW.lowpass3_tmp_b + rtDW.lowpass3_states_b[0] *
                rtP.lp_num[1]) + rtDW.lowpass3_states_b[1] * rtP.lp_num[2];

    /* Saturate: '<S36>/Saturation' incorporates:
     *  DiscreteTransferFcn: '<S36>/low pass3'
     */
    if (rel_time > rtP.Saturation_UpperSat) {
      rtb_Saturation = rtP.Saturation_UpperSat;
    } else if (rel_time < rtP.Saturation_LowerSat) {
      rtb_Saturation = rtP.Saturation_LowerSat;
    } else {
      rtb_Saturation = rel_time;
    }

    /* End of Saturate: '<S36>/Saturation' */

    /* MATLAB Function: '<S1>/maneuver setpoint rd1' incorporates:
     *  DiscreteIntegrator: '<S14>/time'
     */
    /* :  ori_x = -0.01; */
    /* :  ori_y = 0.01; */
    /* :  ori_z = 0.04; */
    /* :  rd = [ori_x; ori_y; ori_z]; */
    /* :  rd_d = zeros(3,1); */
    /* :  rd_dd = zeros(3,1); */
    /* :  rd_ddd = zeros(3,1); */
    /* :  rd_dddd = zeros(3,1); */
    rtb_rd_idx_0 = -0.01;
    rtb_rd_d_idx_0 = 0.0;
    rtb_rd_dd_idx_0 = 0.0;
    rtb_rd_ddd[0] = 0.0;
    rtb_rd_dddd[0] = 0.0;
    rtb_rd_ddd[1] = 0.0;
    rtb_rd_dddd[1] = 0.0;
    absxk = 0.04;
    rtb_rd_d_idx_2 = 0.0;
    rtb_rd_ddd[2] = 0.0;
    rtb_rd_dddd[2] = 0.0;

    /* :  if manoeuvre_en */
    if ((rtP.manoeuvre_en != 0.0) && (rtDW.time_DSTATE > 3.0)) {
      /* :  Tinit = 3; */
      /* :  if real_time > Tinit */
      /* :  rel_time = real_time - Tinit; */
      rel_time = rtDW.time_DSTATE - 3.0;

      /* :  if rel_time > agg_s.T */
      if (rtDW.time_DSTATE - 3.0 > rtP.agg_s.T) {
        /* :  rel_time = agg_s.T; */
        rel_time = rtP.agg_s.T;
      }

      /* :  agg_ax = agg_s.ax; */
      /* :  agg_T = agg_s.T; */
      /* :  agg_N = agg_s.N; */
      /* :  n_1 = agg_s.n_1; */
      /* :  n_2 = agg_s.n_2; */
      /* :  n_3 = agg_s.n_3; */
      /* :  n_4 = agg_s.n_4; */
      /* :  n_5 = agg_s.n_5; */
      /* :  rd(1) = rd(1) + agg_ax'*(rel_time.^n_1)'; */
      scale = 0.0;

      /* :  rd_d(1) = (n_1.*agg_ax') * (rel_time.^n_2)'; */
      rtb_rd_d_idx_0 = 0.0;

      /* :  rd_dd(1) = (n_1.*n_2.*agg_ax') * (rel_time.^n_3)'; */
      rtb_rd_dd_idx_0 = 0.0;

      /* :  rd_ddd(1) = (n_1.*n_2.*n_3.*agg_ax') * (rel_time.^n_4)'; */
      y_tmp = 0.0;

      /* :  rd_dddd(1) = (n_1.*n_2.*n_3.*n_4.*agg_ax') * (rel_time.^n_5)'; */
      y_tmp_0 = 0.0;
      for (r1 = 0; r1 < 10; r1++) {
        scale += rtP.agg_s.ax[r1] * rt_powd_snf(rel_time, rtP.agg_s.n_1[r1]);
        rtb_rd_d_idx_0 += rtP.agg_s.n_1[r1] * rtP.agg_s.ax[r1] * rt_powd_snf
          (rel_time, rtP.agg_s.n_2[r1]);
        y_tmp_1 = rtP.agg_s.n_1[r1] * rtP.agg_s.n_2[r1];
        rtb_rd_dd_idx_0 += y_tmp_1 * rtP.agg_s.ax[r1] * rt_powd_snf(rel_time,
          rtP.agg_s.n_3[r1]);
        y_tmp_1 *= rtP.agg_s.n_3[r1];
        y_tmp += y_tmp_1 * rtP.agg_s.ax[r1] * rt_powd_snf(rel_time,
          rtP.agg_s.n_4[r1]);
        y_tmp_0 += y_tmp_1 * rtP.agg_s.n_4[r1] * rtP.agg_s.ax[r1] * rt_powd_snf
          (rel_time, rtP.agg_s.n_5[r1]);
      }

      rtb_rd_idx_0 = scale - 0.01;
      rtb_rd_ddd[0] = y_tmp;
      rtb_rd_dddd[0] = y_tmp_0;
    }

    /* :  if landing_par.en */
    rel_time = rtP.running_time - rtP.landing_par.time;
    if ((rtP.landing_par.en != 0.0) && (rtDW.time_DSTATE > rel_time)) {
      /* :  if real_time > (running_time-landing_par.time) */
      /* :  rd_d(3) = (landing_par.height-ori_z)/landing_par.time; */
      rtb_rd_d_idx_2 = (rtP.landing_par.height - 0.04) / rtP.landing_par.time;

      /* :  rd(3) = ori_z + ( (landing_par.height-ori_z)/landing_par.time ) * (real_time-(running_time-landing_par.time) ); */
      absxk = (rtDW.time_DSTATE - rel_time) * rtb_rd_d_idx_2 + 0.04;
    }

    for (r1 = 0; r1 < 6; r1++) {
      /* UnitDelay: '<S8>/Unit Delay' */
      scale = rtDW.UnitDelay_DSTATE[r1];

      /* Product: '<S8>/enable1' incorporates:
       *  Constant: '<S8>/adaptive_en'
       */
      rtb_enable1[r1] = rtP.adaptive_en_Value * scale;

      /* DiscreteTransferFcn: '<S8>/low pass3' incorporates:
       *  UnitDelay: '<S8>/Unit Delay1'
       */
      xpageoffset = r1 << 1;
      rel_time = rtDW.lowpass3_states_c[xpageoffset + 1];
      rtDW.lowpass3_tmp_m[r1] = ((rtDW.UnitDelay1_DSTATE[r1] - rtP.lp_den[1] *
        rtDW.lowpass3_states_c[xpageoffset]) - rel_time * rtP.lp_den[2]) /
        rtP.lp_den[0];

      /* Product: '<S8>/enable2' incorporates:
       *  Constant: '<S8>/adaptive_en'
       *  DiscreteTransferFcn: '<S8>/low pass3'
       */
      rtb_enable2[r1] = ((rtP.lp_num[0] * rtDW.lowpass3_tmp_m[r1] + rtP.lp_num[1]
                          * rtDW.lowpass3_states_c[xpageoffset]) + rel_time *
                         rtP.lp_num[2]) * rtP.adaptive_en_Value;

      /* UnitDelay: '<S8>/Unit Delay' */
      rtb_UnitDelay[r1] = scale;
    }

    /* MATLAB Function: '<S9>/Attitude Controleller' incorporates:
     *  DiscreteTransferFcn: '<S5>/low pass'
     *  DiscreteTransferFcn: '<S5>/low pass1'
     *  Gain: '<S32>/lat_s'
     *  MATLAB Function: '<S1>/maneuver setpoint rd1'
     *  MATLAB Function: '<S9>/Thrust Controller'
     *  SignalConversion generated from: '<S29>/ SFunction '
     */
    /* :  g = 9.8; */
    /* :  T = norm(r_dotdot + [0;0;g]); */
    scale = 3.3121686421112381E-170;
    y_tmp = fabs(rtb_MinMax3_idx_0);
    if (y_tmp > 3.3121686421112381E-170) {
      rel_time = 1.0;
      scale = y_tmp;
    } else {
      t = y_tmp / 3.3121686421112381E-170;
      rel_time = t * t;
    }

    y_tmp_0 = fabs(rtb_MinMax3_idx_1);
    if (y_tmp_0 > scale) {
      t = scale / y_tmp_0;
      rel_time = rel_time * t * t + 1.0;
      scale = y_tmp_0;
    } else {
      t = y_tmp_0 / scale;
      rel_time += t * t;
    }

    y_tmp_1 = fabs(rtb_MinMax3_idx_2 + 9.8);
    if (y_tmp_1 > scale) {
      t = scale / y_tmp_1;
      rel_time = rel_time * t * t + 1.0;
      scale = y_tmp_1;
    } else {
      t = y_tmp_1 / scale;
      rel_time += t * t;
    }

    rel_time = scale * sqrt(rel_time);

    /* :  R11 = R(1); */
    /* :  R12 = R(2); */
    /* :  R13 = R(3); */
    /* :  R21 = R(4); */
    /* :  R22 = R(5); */
    /* :  R23 = R(6); */
    /* :  R31 = R(7); */
    /* :  R32 = R(8); */
    /* :  R33 = R(9); */
    /* :  R11d = Rdot(1); */
    /* :  R12d = Rdot(2); */
    /* :  R13d = Rdot(3); */
    /* :  R21d = Rdot(4); */
    /* :  R22d = Rdot(5); */
    /* :  R23d = Rdot(6); */
    /* :  R31d = Rdot(7); */
    /* :  R32d = Rdot(8); */
    /* :  R33d = Rdot(9); */
    /* :  theta_xh = a_hat(2); */
    /* :  theta_yh = a_hat(3); */
    /* :  wx = Omega(1); */
    /* :  wy = Omega(2); */
    /* :  wz = Omega(3); */
    /* :  torque_xoh = a_hat(4); */
    /* :  torque_yoh = a_hat(5); */
    /* :  torque_zoh = a_hat(6); */
    /* :  lambda1 = lambda(1); */
    /* :  lambda2 = lambda(2); */
    /* :  lambda3 = lambda(3); */
    /* :  J = diag([ixx, iyy, izz]); */
    memset(&lowpass3[0], 0, 9U * sizeof(real_T));
    lowpass3[0] = rtP.ixx;
    lowpass3[4] = rtP.iyy;
    lowpass3[8] = rtP.izz;

    /* :  xhat = [R11; R21; R31]; */
    xhat[0] = rtDW.R[0];
    xhat[1] = rtDW.R[3];
    xhat[2] = rtDW.R[6];

    /* :  yhat = [R12; R22; R32]; */
    /* :  zhat = [R13; R23; R33]; */
    zhat[0] = rtDW.R[2];
    zhat[1] = rtDW.R[5];
    zhat[2] = rtDW.R[8];

    /* :  xhatd = yhat*wz-zhat*wy; */
    /* :  yhatd = -xhat*wz+zhat*wx; */
    /* :  theta_xd = a_hatd(2); */
    /* :  theta_yd = a_hatd(3); */
    /* :  gamma = lambda1*(r_dotdot-rd_dd) + lambda2*(r_dot-rd_d) + lambda3*(r-rd); */
    xhatd[0] = rtDW.R[1] * rtb_w_idx_2 - rtDW.R[2] * rtb_w_idx_1;
    b_tmp[0] = rtb_MinMax3_idx_0 - rtb_rd_dd_idx_0;
    torque_xyz[0] = rtP.lat_s * rtb_PoleYk1UkZeroUk1[0] - rtb_rd_d_idx_0;
    xhatd[1] = rtDW.R[4] * rtb_w_idx_2 - rtDW.R[5] * rtb_w_idx_1;
    rtb_rd_d_idx_0 = rtDW.R[7] * rtb_w_idx_2;
    xhatd[2] = rtb_rd_d_idx_0 - rtDW.R[8] * rtb_w_idx_1;
    rtb_MinMax3_idx_0 = rtP.lat_s * rtb_PoleYk1UkZeroUk1[2] - rtb_rd_d_idx_2;
    b_gamma[0] = (rtP.lambda[0] * b_tmp[0] + torque_xyz[0] * rtP.lambda[1]) +
      (rel_time_0 - rtb_rd_idx_0) * rtP.lambda[2];
    b_gamma[1] = (rtP.lambda[0] * rtb_MinMax3_idx_1 + rtP.lambda[1] *
                  rtb_lat_s_idx_1) + (rel_time_1 - 0.01) * rtP.lambda[2];
    rel_time_0 = rtb_lowpass2 - absxk;
    b_gamma[2] = (rtP.lambda[0] * rtb_MinMax3_idx_2 + rtP.lambda[1] *
                  rtb_MinMax3_idx_0) + rel_time_0 * rtP.lambda[2];

    /* :  gammad_d_myhat = lambda1*T*(wx-wz*theta_yh) + dot(-lambda1*rd_ddd + lambda2*(r_dotdot-rd_dd) + lambda3*(r_dot-rd_d), -yhat); */
    /* :  gammad_d_xhat = lambda1*T*(wy+wz*theta_xh) + dot(-lambda1*rd_ddd + lambda2*(r_dotdot-rd_dd) + lambda3*(r_dot-rd_d), xhat); */
    /* :  S = [wx;wy;wz] + [-wz*theta_yh;wz*theta_xh;0] - (1/T)*[dot(rd_ddd,-yhat); dot(rd_ddd,xhat); 0] + (1/T)*[dot(gamma,-yhat); dot(gamma,xhat);0]; */
    t = 1.0 / rel_time;
    rel_time_1 = (-rtP.lambda[0] * rtb_rd_ddd[0] + b_tmp[0] * rtP.lambda[1]) +
      torque_xyz[0] * rtP.lambda[2];
    scale = rel_time_1 * -rtDW.R[1];
    absxk = rel_time_1 * rtDW.R[0];
    yhat[0] = -rtDW.R[1];
    rtb_rd_idx_0 = -rtP.lambda[0] * 0.0;
    rel_time_1 = (rtP.lambda[1] * rtb_MinMax3_idx_1 + rtb_rd_idx_0) +
      rtP.lambda[2] * rtb_lat_s_idx_1;
    scale += rel_time_1 * -rtDW.R[4];
    absxk += rel_time_1 * rtDW.R[3];
    yhat[1] = -rtDW.R[4];
    rel_time_1 = (rtP.lambda[1] * rtb_MinMax3_idx_2 + rtb_rd_idx_0) +
      rtP.lambda[2] * rtb_MinMax3_idx_0;
    yhat[2] = -rtDW.R[7];
    rtb_Sa[0] = ((-rtb_w_idx_2 * rtb_enable1[2] + rtb_w_idx_0) - ((rtb_rd_ddd[0]
      * -rtDW.R[1] + 0.0 * -rtDW.R[4]) + 0.0 * -rtDW.R[7]) * t) + ((b_gamma[0] *
      -rtDW.R[1] + b_gamma[1] * -rtDW.R[4]) + b_gamma[2] * -rtDW.R[7]) * t;
    rtb_rd_dd_idx_0 = rtb_enable1[1] * rtb_w_idx_2 + rtb_w_idx_1;
    rtb_Sa[1] = (rtb_rd_dd_idx_0 - ((rtb_rd_ddd[0] * rtDW.R[0] + 0.0 * rtDW.R[3])
      + 0.0 * rtDW.R[6]) * t) + ((b_gamma[0] * rtDW.R[0] + b_gamma[1] * rtDW.R[3])
      + b_gamma[2] * rtDW.R[6]) * t;
    rtb_Sa[2] = (rtb_w_idx_2 - t * 0.0) + t * 0.0;

    /* :  torque_xyz = [torque_xoh;torque_yoh;torque_zoh] - J*[-wzd*theta_yh;wzd*theta_xh;0] + ... */
    /* :              (J/T)*[dot(rd_dddd,-yhat)+dot(rd_ddd,-yhatd); dot(rd_dddd,xhat)+dot(rd_ddd,xhatd); 0] - ... */
    /* :              (J/T)*[gammad_d_myhat+dot(gamma,-yhatd); gammad_d_xhat+dot(gamma,xhatd);0] - ... */
    /* :              J*[-wz*theta_yd;wz*theta_xd;0] - cross(S-[wx;wy;wz], J*[wx;wy;wz]) - diag(att_gain)*S; */
    for (r1 = 0; r1 < 9; r1++) {
      Wm[r1] = lowpass3[r1] / rel_time;
    }

    t = 0.0;
    rtb_rd_d_idx_2 = 0.0;
    rtb_rd_idx_0 = 0.0;
    b_a = 0.0;
    e_c = 0.0;
    f_c = 0.0;
    for (r1 = 0; r1 < 3; r1++) {
      rtb_lat_s_idx_1 = xhat[r1];
      rtb_MinMax3_idx_1 = b_gamma[r1];
      xhatd_0 = xhatd[r1];
      rtb_rd_ddd_f = rtb_rd_ddd[r1];
      rtb_rd_dddd_b = rtb_rd_dddd[r1];
      t += rtb_rd_dddd_b * yhat[r1];
      zhat_0 = -(-rtb_lat_s_idx_1 * rtb_w_idx_2 + zhat[r1] * rtb_w_idx_0);
      rtb_rd_d_idx_2 += rtb_rd_ddd_f * zhat_0;
      rtb_rd_idx_0 += rtb_rd_dddd_b * rtb_lat_s_idx_1;
      b_a += rtb_rd_ddd_f * xhatd_0;
      e_c += rtb_MinMax3_idx_1 * zhat_0;
      f_c += rtb_MinMax3_idx_1 * xhatd_0;
      xhat[r1] = (lowpass3[r1 + 3] * rtb_w_idx_1 + lowpass3[r1] * rtb_w_idx_0) +
        lowpass3[r1 + 6] * rtb_w_idx_2;
      zhat[r1] = zhat_0;
    }

    memset(&filter1[0], 0, 9U * sizeof(real_T));
    b_tmp[0] = rtb_Sa[0] - rtb_w_idx_0;
    b_tmp[1] = rtb_Sa[1] - rtb_w_idx_1;
    b_tmp[2] = rtb_Sa[2] - rtb_w_idx_2;
    rtb_lat_s_idx_1 = -rtb_Saturation * rtb_enable1[2];
    rtb_MinMax3_idx_1 = rtb_Saturation * rtb_enable1[1];
    torque_xyz[0] = rtb_enable1[3];
    torque_xyz[1] = rtb_enable1[4];
    torque_xyz[2] = rtb_enable1[5];
    t += rtb_rd_d_idx_2;
    rtb_rd_idx_0 += b_a;
    rel_time *= rtP.lambda[0];
    scale = ((rtb_w_idx_0 - rtb_w_idx_2 * rtb_enable1[2]) * rel_time +
             (rel_time_1 * -rtDW.R[7] + scale)) + e_c;
    rel_time_1 = ((rel_time_1 * rtDW.R[6] + absxk) + rtb_rd_dd_idx_0 * rel_time)
      + f_c;
    rtb_rd_d_idx_2 = -rtb_w_idx_2 * rtb_enable2[2];
    rtb_rd_dd_idx_0 = rtb_enable2[1] * rtb_w_idx_2;
    for (r1 = 0; r1 < 3; r1++) {
      filter1[r1 + 3 * r1] = rtP.att_gain[r1];
      f_c = Wm[r1];
      e_c = lowpass3[r1];
      rel_time = f_c * t;
      absxk = e_c * rtb_rd_d_idx_2;
      b_a = e_c * rtb_lat_s_idx_1;
      xhatd_0 = f_c * scale;
      f_c = Wm[r1 + 3];
      e_c = lowpass3[r1 + 3];
      rel_time += f_c * rtb_rd_idx_0;
      absxk += e_c * rtb_rd_dd_idx_0;
      b_a += e_c * rtb_MinMax3_idx_1;
      xhatd_0 += f_c * rel_time_1;
      f_c = Wm[r1 + 6];
      e_c = lowpass3[r1 + 6];
      zhat[r1] = (((torque_xyz[r1] - (e_c * 0.0 + b_a)) + (f_c * 0.0 + rel_time))
                  - (f_c * 0.0 + xhatd_0)) - (e_c * 0.0 + absxk);
    }

    xhatd[0] = b_tmp[1] * xhat[2] - xhat[1] * b_tmp[2];
    xhatd[1] = xhat[0] * b_tmp[2] - b_tmp[0] * xhat[2];
    xhatd[2] = b_tmp[0] * xhat[1] - xhat[0] * b_tmp[1];
    for (r1 = 0; r1 < 3; r1++) {
      torque_xyz[r1] = (zhat[r1] - xhatd[r1]) - ((filter1[r1 + 3] * rtb_Sa[1] +
        filter1[r1] * rtb_Sa[0]) + filter1[r1 + 6] * rtb_Sa[2]);
    }

    /* :  torque_x = torque_xyz(1); */
    /* :  torque_y = torque_xyz(2); */
    /* :  torque_z = torque_xyz(3); */
    /* :  Sa = S; */
    /* :  Ya_t = zeros(3,6); */
    memset(&Ya_t[0], 0, 18U * sizeof(real_T));

    /* :  Ya_t(1:3,4:6) = eye(3); */
    memset(&lowpass3[0], 0, 9U * sizeof(real_T));
    lowpass3[0] = 1.0;
    lowpass3[4] = 1.0;
    lowpass3[8] = 1.0;
    for (r1 = 0; r1 < 3; r1++) {
      xpageoffset = (r1 + 3) * 3;
      Ya_t[xpageoffset] = lowpass3[3 * r1];
      Ya_t[xpageoffset + 1] = lowpass3[3 * r1 + 1];
      Ya_t[xpageoffset + 2] = lowpass3[3 * r1 + 2];
    }

    /* :  Ya_t(1,3) = ixx*lambda1*wz; */
    Ya_t[6] = rtP.ixx * rtP.lambda[0] * rtb_w_idx_2;

    /* :  Ya_t(2,2) = -iyy*lambda1*wz; */
    Ya_t[4] = -rtP.iyy * rtP.lambda[0] * rtb_w_idx_2;

    /* :  Ya = zeros(18,1); */
    memset(&rtb_Ya[0], 0, 18U * sizeof(real_T));

    /* :  Ya(1:6) = Ya_t(1,:); */
    /* :  Ya(7:12) = Ya_t(2,:); */
    /* :  Ya(13:18) = Ya_t(3,:); */
    for (r1 = 0; r1 < 6; r1++) {
      rtb_Ya[r1] = Ya_t[3 * r1];
      rtb_Ya[r1 + 6] = Ya_t[3 * r1 + 1];
      rtb_Ya[r1 + 12] = Ya_t[3 * r1 + 2];
    }

    /* MATLAB Function: '<S9>/Thrust Controller' */
    /* :  g = 9.8; */
    /* :  T = norm(r_dotdot + [0;0;g]); */
    scale = 3.3121686421112381E-170;
    if (y_tmp > 3.3121686421112381E-170) {
      rel_time = 1.0;
      scale = y_tmp;
    } else {
      t = y_tmp / 3.3121686421112381E-170;
      rel_time = t * t;
    }

    if (y_tmp_0 > scale) {
      t = scale / y_tmp_0;
      rel_time = rel_time * t * t + 1.0;
      scale = y_tmp_0;
    } else {
      t = y_tmp_0 / scale;
      rel_time += t * t;
    }

    if (y_tmp_1 > scale) {
      t = scale / y_tmp_1;
      rel_time = rel_time * t * t + 1.0;
      scale = y_tmp_1;
    } else {
      t = y_tmp_1 / scale;
      rel_time += t * t;
    }

    rel_time = scale * sqrt(rel_time);

    /* :  R11 = R(1); */
    /* :  R12 = R(2); */
    /* :  R13 = R(3); */
    /* :  R21 = R(4); */
    /* :  R22 = R(5); */
    /* :  R23 = R(6); */
    /* :  R31 = R(7); */
    /* :  R32 = R(8); */
    /* :  R33 = R(9); */
    /* :  R11d = Rdot(1); */
    /* :  R12d = Rdot(2); */
    /* :  R13d = Rdot(3); */
    /* :  R21d = Rdot(4); */
    /* :  R22d = Rdot(5); */
    /* :  R23d = Rdot(6); */
    /* :  R31d = Rdot(7); */
    /* :  R32d = Rdot(8); */
    /* :  R33d = Rdot(9); */
    /* :  wx = Omega(1); */
    /* :  wy = Omega(2); */
    /* :  wz = Omega(3); */
    /* :  Fo_h = a_hat(1); */
    /* :  theta_xh = a_hat(2); */
    /* :  theta_yh = a_hat(3); */
    /* :  torque_xoh = a_hat(4); */
    /* :  torque_yoh = a_hat(5); */
    /* :  torque_zoh = a_hat(6); */
    /* :  Lambda1 = lambda(4); */
    /* :  Lambda2 = lambda(5); */
    /* :  St = (r_dotdot(3)-rd_dd(3)) + Lambda1*(r_dot(3)-rd_d(3)) + Lambda2*(r(3)-rd(3)); */
    scale = (rtb_MinMax3_idx_0 * rtP.lambda[3] + rtb_MinMax3_idx_2) + rel_time_0
      * rtP.lambda[4];

    /* :  F = T - (1/(gamma*R33))*(1+theta_xh*R32/R33-theta_yh*R31/R33) * ... */
    /* :   ( T*(theta_xh*(-R33*wx+R31*wz)+theta_yh*(-R33*wy+R32*wz)-R32*wx+R31*wy)  - rd_ddd(3) + ... */
    /* :   (Lambda1)*(r_dotdot(3)-rd_dd(3)) + (Lambda2)*(r_dot(3)-rd_d(3)) + thrust_gain*St ); */
    /* :  F = max(min(F, g+1), g-1); */
    /* :  F = F + Fo_h; */
    y_tmp = rtb_enable1[1] * rtDW.R[7];
    y_tmp_0 = rtb_enable1[2] * rtDW.R[6];
    rtb_w_idx_2 = -rtDW.R[8] * rtb_w_idx_0 + rtb_w_idx_2 * rtDW.R[6];
    y_tmp_1 = -rtDW.R[8] * rtb_w_idx_1 + rtb_rd_d_idx_0;
    rtb_rd_d_idx_0 = rtb_MinMax3_idx_2 * rtP.lambda[3];
    rtb_MinMax3_idx_0 *= rtP.lambda[4];
    rel_time_0 = rtP.thrust_gain * scale;
    rtb_MinMax3_idx_2 = rel_time - ((((((rtb_w_idx_2 * rtb_enable1[1] + y_tmp_1 *
      rtb_enable1[2]) - rtb_w_idx_0 * rtDW.R[7]) + rtb_w_idx_1 * rtDW.R[6]) *
      rel_time + rtb_rd_d_idx_0) + rtb_MinMax3_idx_0) + rel_time_0) * (((y_tmp /
      rtDW.R[8] + 1.0) - y_tmp_0 / rtDW.R[8]) * (1.0 / (rtP.gamma * rtDW.R[8])));
    if (!(rtb_MinMax3_idx_2 <= 10.8)) {
      rtb_MinMax3_idx_2 = 10.8;
    }

    if (!(rtb_MinMax3_idx_2 >= 8.8)) {
      rtb_MinMax3_idx_2 = 8.8;
    }

    rtDW.F = rtb_MinMax3_idx_2 + rtb_enable1[0];

    /* :  Yt = zeros(6,1); */
    for (r1 = 0; r1 < 6; r1++) {
      rtb_Yt[r1] = 0.0;
    }

    /* :  Yt(1) = gamma*(R33-R32*theta_xh+R31*theta_yh); */
    rtb_Yt[0] = ((rtDW.R[8] - y_tmp) + y_tmp_0) * rtP.gamma;

    /* :  temp = T*theta_xh*(-R33*wx+R31*wz) + T*theta_yh*(-R33*wy+R32*wz) - T*R32*wx + T*R31*wy + ...  */
    /* :   - rd_dd(3) + Lambda1*(r_dotdot(3)-rd_dd(3)) + Lambda2*(r_dot(3)-rd_d(3)) + thrust_gain*St; */
    absxk = (((((rel_time * rtb_enable1[1] * rtb_w_idx_2 + rel_time *
                 rtb_enable1[2] * y_tmp_1) - rel_time * rtDW.R[7] * rtb_w_idx_0)
               + rel_time * rtDW.R[6] * rtb_w_idx_1) + rtb_rd_d_idx_0) +
             rtb_MinMax3_idx_0) + rel_time_0;

    /* :  Yt(2) = - T*(-R33*wx+R31*wz) - (R32/R33)*temp; */
    rtb_Yt[1] = rtb_w_idx_2 * -rel_time - rtDW.R[7] / rtDW.R[8] * absxk;

    /* :  Yt(3) = - T*(-R33*wy+R32*wz) + (R31/R33)*temp; */
    rtb_Yt[2] = rtDW.R[6] / rtDW.R[8] * absxk + y_tmp_1 * -rel_time;

    /* MATLAB Function: '<S1>/saturation' incorporates:
     *  MATLAB Function: '<S9>/Attitude Controleller'
     */
    /* :  Tpitch = max(min(pitch, 1e-6), -1e-6); */
    if (torque_xyz[0] <= 1.0E-6) {
      rtb_w_idx_0 = torque_xyz[0];
    } else {
      rtb_w_idx_0 = 1.0E-6;
    }

    if (rtb_w_idx_0 >= -1.0E-6) {
      rtb_Tpitch = rtb_w_idx_0;
    } else {
      rtb_Tpitch = -1.0E-6;
    }

    /* :  Troll = max(min(roll, 1e-6), -1e-6); */
    if (torque_xyz[1] <= 1.0E-6) {
      rtb_w_idx_0 = torque_xyz[1];
    } else {
      rtb_w_idx_0 = 1.0E-6;
    }

    if (rtb_w_idx_0 >= -1.0E-6) {
      rtb_Troll = rtb_w_idx_0;
    } else {
      rtb_Troll = -1.0E-6;
    }

    /* :  Tyaw = max(min(yaw, 5.0e-8), -5.0e-8); */
    if (torque_xyz[2] <= 5.0E-8) {
      rtb_w_idx_0 = torque_xyz[2];
    } else {
      rtb_w_idx_0 = 5.0E-8;
    }

    if (rtb_w_idx_0 >= -5.0E-8) {
      rtb_Tyaw = rtb_w_idx_0;
    } else {
      rtb_Tyaw = -5.0E-8;
    }

    /* :  Force = force*bee_mass; */
    rtb_Force = rtDW.F * rtP.bee_mass;

    /* End of MATLAB Function: '<S1>/saturation' */

    /* Product: '<S3>/enable4' incorporates:
     *  Constant: '<S3>/enable'
     */
    rtb_enable4 = rtP.closed_loop_en * rtb_Force;

    /* Switch: '<S61>/Switch1' incorporates:
     *  Constant: '<S61>/zero1'
     *  Constant: '<S65>/Constant'
     *  RelationalOperator: '<S65>/Compare'
     */
    if ((rtb_enable4 <= rtP.CompareToConstant1_const) >= rtP.Switch1_Threshold)
    {
      rel_time = rtb_enable4;
    } else {
      rel_time = rtP.zero1_Value_e;
    }

    /* Gain: '<S50>/gain1' incorporates:
     *  Constant: '<Root>/base'
     *  Constant: '<S50>/constant'
     *  Gain: '<S50>/correction'
     *  Gain: '<S50>/mg'
     *  Sum: '<Root>/add'
     *  Sum: '<S50>/Sum'
     *  Switch: '<S61>/Switch1'
     */
    rtb_gain1 = ((rel_time + rtP.base_thrust) * rtP.correction_Gain *
                 rtP.mg_Gain + rtP.constant_Value_f) * rtP.gain1_Gain;

    /* Gain: '<S52>/half' incorporates:
     *  Gain: '<S52>/lift gain'
     */
    rtb_half = 1.0 / rtP.A * 273.95226202816286 * rtb_gain1 * rtP.half_Gain;

    /* Product: '<S3>/enable1' incorporates:
     *  Constant: '<S3>/enable'
     */
    rtb_enable1_n = rtP.closed_loop_en * rtb_Tpitch;

    /* Switch: '<S58>/Switch1' incorporates:
     *  Constant: '<S58>/zero1'
     *  Constant: '<S62>/Constant'
     *  RelationalOperator: '<S62>/Compare'
     */
    if ((rtb_enable1_n <= rtP.CompareToConstant1_const_i) >=
        rtP.Switch1_Threshold_i) {
      rel_time = rtb_enable1_n;
    } else {
      rel_time = rtP.zero1_Value;
    }

    /* Gain: '<S52>/pitch gain' incorporates:
     *  Constant: '<Root>/base4'
     *  Constant: '<S50>/constant1'
     *  Gain: '<S2>/Gain'
     *  Gain: '<S50>/correction1'
     *  Gain: '<S50>/gain2'
     *  Gain: '<S50>/uNm2'
     *  Product: '<S50>/divide'
     *  Sum: '<Root>/add1'
     *  Sum: '<S50>/Sum1'
     *  Switch: '<S58>/Switch1'
     */
    rtb_pitchgain = (rel_time + rtP.base_pitch) * rtP.Gain_Gain *
      rtP.correction1_Gain * rtP.uNm2_Gain * (1.0 / (rtP.gain2_Gain * rtb_gain1
      + rtP.constant1_Value_b)) * rtP.pitchgain_Gain;

    /* Product: '<S3>/enable3' incorporates:
     *  Constant: '<S3>/enable'
     */
    rtb_enable3 = rtP.closed_loop_en * rtb_Tyaw;

    /* Switch: '<S60>/Switch1' incorporates:
     *  Constant: '<S60>/zero1'
     *  Constant: '<S64>/Constant'
     *  RelationalOperator: '<S64>/Compare'
     */
    if ((rtb_enable3 <= rtP.CompareToConstant1_const_e) >=
        rtP.Switch1_Threshold_l) {
      rel_time = rtb_enable3;
    } else {
      rel_time = rtP.zero1_Value_m;
    }

    /* Product: '<S50>/divide1' incorporates:
     *  Constant: '<Root>/base2'
     *  Constant: '<S50>/constant2'
     *  Gain: '<S50>/correction2'
     *  Gain: '<S50>/gain6'
     *  Gain: '<S50>/uNm'
     *  Gain: '<S50>/uNm3'
     *  Sum: '<Root>/add2'
     *  Sum: '<S50>/Sum2'
     *  Switch: '<S60>/Switch1'
     */
    rtb_divide1 = (rel_time + rtP.base_yaw) * rtP.correction2_Gain *
      rtP.uNm_Gain * rtP.uNm3_Gain * (1.0 / (rtP.gain6_Gain * rtb_gain1 +
      rtP.constant2_Value));

    /* Abs: '<S52>/Abs' */
    rtb_Abs_n = fabs(rtb_divide1);

    /* Gain: '<S54>/Cv' */
    rtb_Cv = rtP.Cv * rtb_Abs_n;

    /* Sum: '<S54>/B+C' incorporates:
     *  Constant: '<S54>/1'
     *  Gain: '<S54>/Bv'
     *  Sum: '<S54>/Sum'
     */
    rtb_BC = (rtP.u_Value_f - rtb_Abs_n) * rtP.Bv + rtb_Cv;

    /* Product: '<S54>/C//(B+C)' */
    rtb_CBC = 1.0 / rtb_BC * rtb_Cv;

    /* Math: '<S56>/pow3' incorporates:
     *  Constant: '<S56>/4'
     */
    if ((rtb_CBC < 0.0) && (rtP.u_Value_i > floor(rtP.u_Value_i))) {
      rel_time = -rt_powd_snf(-rtb_CBC, rtP.u_Value_i);
    } else {
      rel_time = rt_powd_snf(rtb_CBC, rtP.u_Value_i);
    }

    /* Math: '<S56>/pow2' incorporates:
     *  Constant: '<S56>/5'
     */
    if ((rtb_CBC < 0.0) && (rtP.u_Value_lh > floor(rtP.u_Value_lh))) {
      absxk = -rt_powd_snf(-rtb_CBC, rtP.u_Value_lh);
    } else {
      absxk = rt_powd_snf(rtb_CBC, rtP.u_Value_lh);
    }

    /* Gain: '<S54>/1//A' incorporates:
     *  Constant: '<S55>/4'
     *  Constant: '<S55>/5'
     *  Constant: '<S55>/constant1'
     *  Constant: '<S56>/constant1'
     *  Gain: '<S55>/Gain2'
     *  Gain: '<S55>/Gain3'
     *  Gain: '<S55>/Gain7'
     *  Gain: '<S55>/Gain8'
     *  Gain: '<S56>/Gain2'
     *  Gain: '<S56>/Gain3'
     *  Gain: '<S56>/Gain7'
     *  Gain: '<S56>/Gain8'
     *  Math: '<S55>/pow2'
     *  Math: '<S55>/pow3'
     *  Math: '<S55>/square1'
     *  Math: '<S56>/pow2'
     *  Math: '<S56>/pow3'
     *  Math: '<S56>/square1'
     *  Product: '<S54>/op'
     *  Sum: '<S55>/Sum1'
     *  Sum: '<S56>/Sum1'
     */
    rtb_uA = 1.0 / ((((rtP.Pvmax[0] * rt_powd_snf(rtb_Abs_n, rtP.u_Value_kx) +
                       rtP.Pvmax[1] * rt_powd_snf(rtb_Abs_n, rtP.u_Value_l)) +
                      rtb_Abs_n * rtb_Abs_n * rtP.Pvmax[2]) + rtP.Pvmax[3] *
                     rtb_Abs_n) + rtP.constant1_Value_b4) * rtb_BC *
      ((((rtP.Pvmax[0] * rel_time + rtP.Pvmax[1] * absxk) + rtb_CBC * rtb_CBC *
         rtP.Pvmax[2]) + rtP.Pvmax[3] * rtb_CBC) + rtP.constant1_Value_l) * (1.0
      / rtP.A);

    /* Sum: '<S52>/add' incorporates:
     *  Gain: '<S52>/half1'
     *  Product: '<S52>/multply'
     */
    rtb_add_b = rtb_gain1 * rtb_uA * rtP.half1_Gain - rtb_half;

    /* Product: '<S3>/enable2' incorporates:
     *  Constant: '<S3>/enable'
     */
    rtb_enable2_a = rtP.closed_loop_en * rtb_Troll;

    /* Switch: '<S59>/Switch1' incorporates:
     *  Constant: '<S59>/zero1'
     *  Constant: '<S63>/Constant'
     *  RelationalOperator: '<S63>/Compare'
     */
    if ((rtb_enable2_a <= rtP.CompareToConstant1_const_c) >=
        rtP.Switch1_Threshold_c) {
      rel_time = rtb_enable2_a;
    } else {
      rel_time = rtP.zero1_Value_o;
    }

    /* Gain: '<S52>/half2' incorporates:
     *  Constant: '<Root>/base3'
     *  Constant: '<S50>/constant3'
     *  Gain: '<Root>/Gain'
     *  Gain: '<S50>/correction3'
     *  Gain: '<S50>/gain7'
     *  Gain: '<S50>/uNm1'
     *  Product: '<S50>/divide2'
     *  Product: '<S52>/multiply'
     *  Sum: '<Root>/add3'
     *  Sum: '<S50>/Sum3'
     *  Switch: '<S59>/Switch1'
     */
    rtb_half2 = (rtP.Gain_Gain_c * rel_time + rtP.base_roll) *
      rtP.correction3_Gain * rtP.uNm1_Gain * (1.0 / (rtP.gain7_Gain * rtb_gain1
      + rtP.constant3_Value)) * rtb_uA * rtP.half2_Gain;

    /* Sum: '<S51>/total volt (half side)' incorporates:
     *  Abs: '<S51>/Abs'
     *  Abs: '<S51>/Abs1'
     *  Abs: '<S51>/Abs2'
     *  Abs: '<S51>/Abs3'
     */
    rtb_totalvolthalfside = ((fabs(rtb_half) + fabs(rtb_pitchgain)) + fabs
      (rtb_add_b)) + fabs(rtb_half2);

    /* MinMax: '<S51>/MinOf' incorporates:
     *  Constant: '<S51>/max allowed'
     */
    if ((rtb_totalvolthalfside <= rtP.maxallowed_Value) || rtIsNaN
        (rtP.maxallowed_Value)) {
      rel_time = rtb_totalvolthalfside;
    } else {
      rel_time = rtP.maxallowed_Value;
    }

    /* Product: '<S51>/divide' incorporates:
     *  MinMax: '<S51>/MinOf'
     */
    rtb_divide_l = 1.0 / rtb_totalvolthalfside * rel_time;

    /* Product: '<S2>/multiply' */
    rtb_multiply_c = rtb_divide_l * rtb_half;

    /* Sum: '<S2>/B+C1' incorporates:
     *  Product: '<S2>/multiply2'
     *  Sum: '<S53>/total volt (half side)1'
     */
    rtb_w_idx_0 = rtb_divide_l * rtb_add_b + rtb_multiply_c;

    /* Gain: '<S2>/p-p' incorporates:
     *  Sum: '<S2>/B+C1'
     */
    rtb_pp = rtb_w_idx_0 * rtP.pp_Gain;

    /* Outport: '<Root>/signal amp (v)' */
    rtY.signalampv = rtb_pp;

    /* Product: '<S2>/multiply1' */
    rtb_multiply1 = rtb_divide_l * rtb_pitchgain;

    /* Outport: '<Root>/pitch bias (v)' */
    rtY.pitchbiasv = rtb_multiply1;

    /* Gain: '<S53>/p-p' incorporates:
     *  Gain: '<S53>/lift gain'
     */
    rtb_pp_o = rtP.A / 273.95226202816286 * rtb_multiply_c * rtP.pp_Gain_a;

    /* Product: '<S53>/Vmax//Phi' incorporates:
     *  Gain: '<S53>/2'
     */
    rtb_VmaxPhi = rtb_w_idx_0 * rtP.u_Gain * (1.0 / rtb_pp_o);

    /* Math: '<S57>/pow2' incorporates:
     *  Constant: '<S57>/3'
     */
    if ((rtb_VmaxPhi < 0.0) && (rtP.u_Value_j > floor(rtP.u_Value_j))) {
      rtb_pow2_a = -rt_powd_snf(-rtb_VmaxPhi, rtP.u_Value_j);
    } else {
      rtb_pow2_a = rt_powd_snf(rtb_VmaxPhi, rtP.u_Value_j);
    }

    /* End of Math: '<S57>/pow2' */

    /* Signum: '<S53>/Sign' */
    if (rtIsNaN(rtb_divide1)) {
      rel_time = (rtNaN);
    } else if (rtb_divide1 < 0.0) {
      rel_time = -1.0;
    } else {
      rel_time = (rtb_divide1 > 0.0);
    }

    /* Product: '<S53>/multiply4' incorporates:
     *  Constant: '<S57>/1'
     *  Gain: '<S57>/Gain2'
     *  Gain: '<S57>/Gain3'
     *  Gain: '<S57>/Gain8'
     *  Math: '<S57>/square1'
     *  Product: '<S57>/divide3'
     *  Signum: '<S53>/Sign'
     *  Sum: '<S57>/Sum1'
     */
    rtb_multiply4 = (((rtb_VmaxPhi * rtb_VmaxPhi * rtP.Pvin[1] + rtP.Pvin[0] *
                       rtb_pow2_a) + rtP.Pvin[2] * rtb_VmaxPhi) + rtP.Pvin[3]) *
      (1.0 / rtb_pow2_a) * rel_time;

    /* Gain: '<S49>/Cv' */
    rtb_Cv_j = rtP.Cv * rtb_multiply4;

    /* Outport: '<Root>/mu v' incorporates:
     *  Abs: '<S49>/Abs'
     *  Constant: '<S49>/1'
     *  Gain: '<Root>/Gain1'
     *  Gain: '<S49>/Bv1'
     *  Product: '<S49>/C//(B+C)'
     *  Sum: '<S49>/1-mu'
     *  Sum: '<S49>/B+C'
     */
    rtY.muv = 1.0 / ((rtP.u_Value_jl - fabs(rtb_multiply4)) * rtP.Bv + rtb_Cv_j)
      * rtb_Cv_j * rtP.Gain1_Gain;

    /* Product: '<S2>/multiply3' */
    rtb_multiply3 = rtb_divide_l * rtb_half2;

    /* Gain: '<S2>/p-p1' */
    rtb_pp1 = rtP.pp1_Gain * rtb_multiply3;

    /* Outport: '<Root>/roll bias (v)' */
    rtY.rollbiasv = rtb_pp1;

    /* Outport: '<Root>/flapping amp (deg)' */
    rtY.flappingampdeg = rtb_pp_o;

    /* Outport: '<Root>/pitch bias (deg)' incorporates:
     *  Gain: '<S53>/pitch gain'
     */
    rtY.pitchbiasdeg = rtP.A / rtP.k_eq * rtb_multiply1;

    /* Outport: '<Root>/mu' */
    rtY.mu = rtb_multiply4;

    /* Outport: '<Root>/roll bias (deg)' incorporates:
     *  Gain: '<S53>/1'
     *  Product: '<S53>/divide3'
     */
    rtY.rollbiasdeg = 1.0 / rtb_VmaxPhi * rtb_multiply3 * rtP.u_Gain_a;

    /* MATLAB Function: '<S5>/envelope' incorporates:
     *  Inport: '<Root>/X(m)'
     *  Inport: '<Root>/Z(m)'
     *  Inport: '<Root>/alpha(radians)'
     *  Inport: '<Root>/beta(radians)'
     */
    /* :  en = uint8(1); */
    rtb_en_g = 1U;

    /* :  if x < -0.20 */
    if (rtU.Xm < -0.2) {
      /* :  en = uint8(0); */
      rtb_en_g = 0U;
    } else if (rtU.Zm < -0.024) {
      /* :  elseif z < -0.024 */
      /* :  en = uint8(0); */
      rtb_en_g = 0U;
    } else if (fabs(rtU.alpharadians) > 1.5707963267948966) {
      /* :  elseif abs(alpha) > pi/2 */
      /* :  ; */
      /* :  en = uint8(0); */
      rtb_en_g = 0U;
    } else if (fabs(rtU.betaradians) > 1.5707963267948966) {
      /* :  elseif abs(beta) >pi/2 */
      /* :  ; */
      /* :  en = uint8(0); */
      rtb_en_g = 0U;
    }

    /* End of MATLAB Function: '<S5>/envelope' */

    /* MATLAB Function: '<S73>/landing_cond' incorporates:
     *  Constant: '<S73>/constant1'
     *  Constant: '<S73>/constant2'
     *  DiscreteIntegrator: '<S73>/SaturatingRamp1'
     *  Inport: '<Root>/Z(m)'
     *  Sum: '<S73>/Add'
     */
    /* :  en = en; */
    /* :  if ( (time_cond<0) | (alt>landing_par.height) ) */
    if (((rtDW.SaturatingRamp1_DSTATE - rtP.constant1_Value_d) - rtP.start_delay
         < 0.0) || (rtU.Zm > rtP.landing_par.height)) {
      /* :  en = uint8(1); */
      rtb_en = 1U;
    } else {
      /* :  else */
      /* :  en = uint8(0); */
      rtb_en = 0U;
    }

    /* End of MATLAB Function: '<S73>/landing_cond' */

    /* DataTypeConversion: '<S5>/type conv' incorporates:
     *  Constant: '<S68>/Constant'
     *  Constant: '<S69>/Constant'
     *  Constant: '<S70>/Constant'
     *  Constant: '<S71>/Constant'
     *  Inport: '<Root>/X(m)'
     *  Inport: '<Root>/Y(m)'
     *  Inport: '<Root>/Z(m)'
     *  Inport: '<Root>/alpha(radians)'
     *  Inport: '<Root>/beta(radians)'
     *  Inport: '<Root>/gamma(radians)'
     *  Product: '<S5>/OutputWithRamping1'
     *  Product: '<S5>/times'
     *  RelationalOperator: '<S68>/Compare'
     *  RelationalOperator: '<S69>/Compare'
     *  RelationalOperator: '<S70>/Compare'
     *  RelationalOperator: '<S71>/Compare'
     *  Sum: '<S5>/total'
     *  Trigonometry: '<S5>/c_x'
     *  Trigonometry: '<S5>/c_y'
     */
    rtb_typeconv = ((int32_T)((uint32_T)(((((rtU.Xm + rtU.Ym) + rtU.Zm) +
      rtU.alpharadians) + rtU.betaradians) + rtU.gammaradians !=
      rtP.Constant_Value_o5) * ((uint32_T)rtb_en_g * rtb_en * (uint32_T)(rtU.Zm >=
      rtP.CompareToConstant1_const_e1)) * (uint32_T)(cos(rtU.alpharadians) * cos
      (rtU.betaradians) > rtP.CompareToConstant_const)) == rtP.Constant_Value_i5);

    /* DiscreteIntegrator: '<S5>/SaturatingRamp' */
    if ((rtb_typeconv <= 0.0) && (rtDW.SaturatingRamp_PrevResetState == 1)) {
      rtDW.SaturatingRamp_DSTATE_p = rtP.SaturatingRamp_IC_a;
    }

    /* Product: '<Root>/times' incorporates:
     *  Constant: '<Root>/running time'
     *  Constant: '<S66>/Constant'
     *  Constant: '<S67>/Constant'
     *  Constant: '<S74>/Constant'
     *  DiscreteIntegrator: '<S4>/SaturatingRamp'
     *  DiscreteIntegrator: '<S5>/SaturatingRamp'
     *  Product: '<S4>/Product2'
     *  RelationalOperator: '<S66>/Compare'
     *  RelationalOperator: '<S67>/Compare'
     *  RelationalOperator: '<S74>/Compare'
     *  Sum: '<S4>/Add'
     */
    rtb_times_j = (int32_T)((uint32_T)(rtDW.SaturatingRamp_DSTATE_a -
      rtP.running_time < rtP.Constant_Value_p) * (uint32_T)
      (rtDW.SaturatingRamp_DSTATE_a > rtP.start_delay) * (uint32_T)
      (rtDW.SaturatingRamp_DSTATE_p <= rtP.torelancetime_const));

    /* Outport: '<Root>/enable' incorporates:
     *  DataTypeConversion: '<Root>/Data Type Conversion'
     */
    rtY.enable_i = rtb_times_j;

    /* Outport: '<Root>/pitch' */
    rtY.pitch = rtb_Tpitch;

    /* Outport: '<Root>/roll' */
    rtY.roll = rtb_Troll;

    /* Outport: '<Root>/yaw' */
    rtY.yaw = rtb_Tyaw;

    /* Outport: '<Root>/force' */
    rtY.force = rtb_Force;

    /* DiscreteTransferFcn: '<S20>/filter derivative 1' */
    for (r1 = 0; r1 < 3; r1++) {
      xpageoffset = r1 << 1;
      rtb_w_idx_0 = rtDW.filterderivative1_states[xpageoffset + 1];
      rtDW.filterderivative1_tmp[r1] = ((rtb_TmpSignalConversionAtfilter[r1] -
        rtP.filterderivative1_DenCoef[1] *
        rtDW.filterderivative1_states[xpageoffset]) - rtb_w_idx_0 *
        rtP.filterderivative1_DenCoef[2]) / rtP.filterderivative1_DenCoef[0];
      torque_xyz[r1] = (rtP.filterderivative1_NumCoef[0] *
                        rtDW.filterderivative1_tmp[r1] +
                        rtP.filterderivative1_NumCoef[1] *
                        rtDW.filterderivative1_states[xpageoffset]) +
        rtb_w_idx_0 * rtP.filterderivative1_NumCoef[2];
    }

    /* End of DiscreteTransferFcn: '<S20>/filter derivative 1' */
    /* :  W = zeros(9,1); */
    for (r1 = 0; r1 < 9; r1++) {
      /* DiscreteTransferFcn: '<S20>/filter 1' */
      xpageoffset = r1 << 1;
      filter1[r1] = rtDW.filter1_states[xpageoffset + 1] * rtP.filter1_NumCoef[2]
        + rtP.filter1_NumCoef[1] * rtDW.filter1_states[xpageoffset];

      /* DiscreteTransferFcn: '<S20>/filter1' */
      xpageoffset = r1 << 1;
      filter1_f[r1] = rtDW.filter1_states_j[xpageoffset + 1] *
        rtP.filter1_NumCoef_f[2] + rtP.filter1_NumCoef_f[1] *
        rtDW.filter1_states_j[xpageoffset];
    }

    /* MATLAB Function: '<S18>/estimating W1' */
    /* :  g = 9.8; */
    /* :  R11f = Rf(1); */
    /* :  R12f = Rf(2); */
    /* :  R13f = Rf(3); */
    /* :  R21f = Rf(4); */
    /* :  R22f = Rf(5); */
    /* :  R23f = Rf(6); */
    /* :  R31f = Rf(7); */
    /* :  R32f = Rf(8); */
    /* :  R33f = Rf(9); */
    /* :  FR11 = RFf(1); */
    /* :  FR12 = RFf(2); */
    /* :  FR13 = RFf(3); */
    /* :  FR21 = RFf(4); */
    /* :  FR22 = RFf(5); */
    /* :  FR23 = RFf(6); */
    /* :  FR31 = RFf(7); */
    /* :  FR32 = RFf(8); */
    /* :  FR33 = RFf(9); */
    /* :  xf = rdd(1)-RFf(3); */
    /* :  yf = rdd(2)-RFf(6); */
    /* :  zf = rdd(3)-RFf(9)+g; */
    /* :  alpha_x = -RFf(2); */
    /* :  alpha_y = -RFf(5); */
    /* :  alpha_z = -RFf(8); */
    /* :  beta_x = RFf(1); */
    /* :  beta_y = RFf(4); */
    /* :  beta_z = RFf(7); */
    /* :  A = [-R13f, -FR12, FR11; -R23f, -FR22, FR21; -R33f, -FR32, FR31]; */
    lowpass3[0] = -filter1[2];
    lowpass3[3] = -filter1_f[1];
    lowpass3[6] = filter1_f[0];
    lowpass3[1] = -filter1[5];
    lowpass3[4] = -filter1_f[4];
    lowpass3[7] = filter1_f[3];
    lowpass3[2] = -filter1[8];
    lowpass3[5] = -filter1_f[7];
    lowpass3[8] = filter1_f[6];

    /* MATLAB Function: '<S15>/adaptive_estimation' incorporates:
     *  MATLAB Function: '<S18>/estimating W1'
     *  MATLAB Function: '<S9>/Thrust Controller'
     */
    /* :  e = [xf; yf; zf]-A*[a_hat(1); a_hat(2); a_hat(3)]; */
    /* :  W(1:3) = -A(1,:); */
    /* :  W(4:6) = -A(2,:); */
    /* :  W(7:9) = -A(3,:); */
    /* :  S = [Sa; St]; */
    S[0] = rtb_Sa[0];
    S[1] = rtb_Sa[1];
    S[2] = rtb_Sa[2];
    S[3] = scale;

    /* :  Y = zeros(4,6); */
    /* :  Y(1,:) = Ya(1:6); */
    /* :  Y(2,:) = Ya(7:12); */
    /* :  Y(3,:) = Ya(13:18); */
    /* :  Y(4,:) = Yt; */
    for (r1 = 0; r1 < 6; r1++) {
      xpageoffset = r1 << 2;
      Y[xpageoffset] = rtb_Ya[r1];
      Y[xpageoffset + 1] = rtb_Ya[r1 + 6];
      Y[xpageoffset + 2] = rtb_Ya[r1 + 12];
      Y[xpageoffset + 3] = rtb_Yt[r1];
    }

    /* :  R31= R(7); */
    /* :  R32 = R(8); */
    /* :  Wm = zeros(3,3); */
    /* :  Wm(1,:) = W(1:3); */
    /* :  Wm(2,:) = W(4:6); */
    /* :  Wm(3,:) = W(7:9); */
    Wm[0] = filter1[2];
    Wm[1] = filter1[5];
    Wm[2] = filter1[8];
    Wm[3] = filter1_f[1];
    Wm[4] = filter1_f[4];
    Wm[5] = filter1_f[7];
    Wm[6] = -filter1_f[0];
    Wm[7] = -filter1_f[3];
    Wm[8] = -filter1_f[6];

    /* :  Phi = est_lambda + [0,0.5*gamma*R32,-0.5*gamma*R31;0.5*gamma*R32,0,0;-0.5*gamma*R31,0,0]*St; */
    /* :  if sum(sum(isnan(Wm))) */
    for (r1 = 0; r1 < 9; r1++) {
      x[r1] = rtIsNaN(Wm[r1]);
    }

    for (r2 = 0; r2 < 3; r2++) {
      xpageoffset = r2 * 3;
      nz[r2] = (x[xpageoffset + 2] + x[xpageoffset + 1]) + x[xpageoffset];
    }

    if (((real_T)nz[0] + (real_T)nz[1]) + (real_T)nz[2] != 0.0) {
      /* :  a_hat_dot = -adpt_gamma*transpose(Y)*S; */
      for (r1 = 0; r1 < 36; r1++) {
        tmp[r1] = -rtP.adpt_gamma[r1];
      }

      for (r1 = 0; r1 < 6; r1++) {
        rtb_a_hat_dot[r1] = 0.0;
        for (xpageoffset = 0; xpageoffset < 4; xpageoffset++) {
          r2 = 6 * xpageoffset + r1;
          tmp_0[r2] = 0.0;
          for (rtemp = 0; rtemp < 6; rtemp++) {
            tmp_0[r2] += tmp[6 * rtemp + r1] * Y[(rtemp << 2) + xpageoffset];
          }

          rtb_a_hat_dot[r1] += tmp_0[r2] * S[xpageoffset];
        }
      }
    } else {
      /* :  else */
      /* :  Win = pinv(Wm); */
      /* :  a_hat_dot = -adpt_gamma*transpose(Y)*S-[adpt_gamma(1:3,1:3)*Phi*((transpose(Wm)*Wm)\(transpose(Wm)*e));0;0;0]; */
      for (r1 = 0; r1 < 3; r1++) {
        filter1[3 * r1] = Wm[r1];
        filter1[3 * r1 + 1] = Wm[r1 + 3];
        filter1[3 * r1 + 2] = Wm[r1 + 6];
      }

      xhat[0] = rtb_UnitDelay[0];
      xhat[1] = rtb_UnitDelay[1];
      xhat[2] = rtb_UnitDelay[2];
      b_tmp[0] = torque_xyz[0] - filter1_f[2];
      b_tmp[1] = torque_xyz[1] - filter1_f[5];
      b_tmp[2] = (torque_xyz[2] - filter1_f[8]) + 9.8;
      for (r1 = 0; r1 < 3; r1++) {
        zhat[r1] = 0.0;
        for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
          r2 = 3 * xpageoffset + r1;
          filter1_f[r2] = 0.0;
          filter1_f[r2] += Wm[3 * xpageoffset] * filter1[r1];
          filter1_f[r2] += Wm[3 * xpageoffset + 1] * filter1[r1 + 3];
          filter1_f[r2] += Wm[3 * xpageoffset + 2] * filter1[r1 + 6];
          zhat[r1] += lowpass3[r2] * xhat[xpageoffset];
        }

        torque_xyz[r1] = b_tmp[r1] - zhat[r1];
      }

      for (r1 = 0; r1 < 3; r1++) {
        b_tmp[r1] = (filter1[r1 + 3] * torque_xyz[1] + filter1[r1] * torque_xyz
                     [0]) + filter1[r1 + 6] * torque_xyz[2];
      }

      r1 = 0;
      r2 = 1;
      xpageoffset = 2;
      rel_time = fabs(filter1_f[0]);
      absxk = fabs(filter1_f[1]);
      if (absxk > rel_time) {
        rel_time = absxk;
        r1 = 1;
        r2 = 0;
      }

      if (fabs(filter1_f[2]) > rel_time) {
        r1 = 2;
        r2 = 1;
        xpageoffset = 0;
      }

      filter1_f[r2] /= filter1_f[r1];
      filter1_f[xpageoffset] /= filter1_f[r1];
      filter1_f[r2 + 3] -= filter1_f[r1 + 3] * filter1_f[r2];
      filter1_f[xpageoffset + 3] -= filter1_f[r1 + 3] * filter1_f[xpageoffset];
      filter1_f[r2 + 6] -= filter1_f[r1 + 6] * filter1_f[r2];
      filter1_f[xpageoffset + 6] -= filter1_f[r1 + 6] * filter1_f[xpageoffset];
      if (fabs(filter1_f[xpageoffset + 3]) > fabs(filter1_f[r2 + 3])) {
        rtemp = r2;
        r2 = xpageoffset;
        xpageoffset = rtemp;
      }

      filter1_f[xpageoffset + 3] /= filter1_f[r2 + 3];
      filter1_f[xpageoffset + 6] -= filter1_f[xpageoffset + 3] * filter1_f[r2 +
        6];
      torque_xyz[1] = b_tmp[r2] - b_tmp[r1] * filter1_f[r2];
      torque_xyz[2] = (b_tmp[xpageoffset] - b_tmp[r1] * filter1_f[xpageoffset])
        - filter1_f[xpageoffset + 3] * torque_xyz[1];
      torque_xyz[2] /= filter1_f[xpageoffset + 6];
      torque_xyz[0] = b_tmp[r1] - filter1_f[r1 + 6] * torque_xyz[2];
      torque_xyz[1] -= filter1_f[r2 + 6] * torque_xyz[2];
      torque_xyz[1] /= filter1_f[r2 + 3];
      torque_xyz[0] -= filter1_f[r1 + 3] * torque_xyz[1];
      torque_xyz[0] /= filter1_f[r1];
      for (r1 = 0; r1 < 36; r1++) {
        tmp[r1] = -rtP.adpt_gamma[r1];
      }

      for (r1 = 0; r1 < 6; r1++) {
        for (xpageoffset = 0; xpageoffset < 4; xpageoffset++) {
          r2 = 6 * xpageoffset + r1;
          tmp_0[r2] = 0.0;
          for (rtemp = 0; rtemp < 6; rtemp++) {
            tmp_0[r2] += tmp[6 * rtemp + r1] * Y[(rtemp << 2) + xpageoffset];
          }
        }
      }

      lowpass3[0] = 0.0 * scale + rtP.est_lambda[0];
      rel_time = 0.5 * rtP.gamma * rtDW.R[7] * scale;
      lowpass3[1] = rel_time + rtP.est_lambda[1];
      absxk = -0.5 * rtP.gamma * rtDW.R[6] * scale;
      lowpass3[2] = absxk + rtP.est_lambda[2];
      lowpass3[3] = rel_time + rtP.est_lambda[3];
      lowpass3[4] = 0.0 * scale + rtP.est_lambda[4];
      lowpass3[5] = 0.0 * scale + rtP.est_lambda[5];
      lowpass3[6] = absxk + rtP.est_lambda[6];
      lowpass3[7] = 0.0 * scale + rtP.est_lambda[7];
      lowpass3[8] = 0.0 * scale + rtP.est_lambda[8];
      for (r1 = 0; r1 < 3; r1++) {
        b_tmp[r1] = 0.0;
        for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
          r2 = 3 * xpageoffset + r1;
          Wm[r2] = 0.0;
          Wm[r2] += lowpass3[3 * xpageoffset] * rtP.adpt_gamma[r1];
          Wm[r2] += lowpass3[3 * xpageoffset + 1] * rtP.adpt_gamma[r1 + 6];
          Wm[r2] += lowpass3[3 * xpageoffset + 2] * rtP.adpt_gamma[r1 + 12];
          b_tmp[r1] += Wm[r2] * torque_xyz[xpageoffset];
        }
      }

      tmp_1[0] = b_tmp[0];
      tmp_1[1] = b_tmp[1];
      tmp_1[2] = b_tmp[2];
      tmp_1[3] = 0.0;
      tmp_1[4] = 0.0;
      tmp_1[5] = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        rtb_a_hat_dot[r1] = (((tmp_0[r1 + 6] * S[1] + tmp_0[r1] * S[0]) +
                              tmp_0[r1 + 12] * S[2]) + tmp_0[r1 + 18] * scale) -
          tmp_1[r1];
      }
    }

    /* End of MATLAB Function: '<S15>/adaptive_estimation' */
    for (r1 = 0; r1 < 6; r1++) {
      /* RelationalOperator: '<S23>/Compare' */
      rtb_w_idx_0 = rtb_a_hat_dot[r1];

      /* Switch: '<S22>/Switch1' incorporates:
       *  Constant: '<S23>/Constant'
       *  RelationalOperator: '<S23>/Compare'
       */
      if ((rtb_w_idx_0 <= rtP.CompareToConstant1_const_m) >=
          rtP.Switch1_Threshold_cr) {
        /* Switch: '<S22>/Switch1' */
        rtb_Switch1_p[r1] = rtb_w_idx_0;
      } else {
        /* Switch: '<S22>/Switch1' incorporates:
         *  Constant: '<S22>/zero1'
         */
        rtb_Switch1_p[r1] = rtP.zero1_Value_a;
      }

      /* End of Switch: '<S22>/Switch1' */
    }

    /* Saturate: '<S19>/Saturation' */
    if (rtb_Switch1_p[0] > rtP.Saturation_UpperSat_e) {
      /* Saturate: '<S19>/Saturation' */
      rtb_Saturation_f = rtP.Saturation_UpperSat_e;
    } else if (rtb_Switch1_p[0] < rtP.Saturation_LowerSat_f) {
      /* Saturate: '<S19>/Saturation' */
      rtb_Saturation_f = rtP.Saturation_LowerSat_f;
    } else {
      /* Saturate: '<S19>/Saturation' */
      rtb_Saturation_f = rtb_Switch1_p[0];
    }

    /* End of Saturate: '<S19>/Saturation' */

    /* Saturate: '<S19>/Saturation1' */
    if (rtb_Switch1_p[1] > rtP.Saturation1_UpperSat) {
      /* Saturate: '<S19>/Saturation1' */
      rtb_Saturation1 = rtP.Saturation1_UpperSat;
    } else if (rtb_Switch1_p[1] < rtP.Saturation1_LowerSat) {
      /* Saturate: '<S19>/Saturation1' */
      rtb_Saturation1 = rtP.Saturation1_LowerSat;
    } else {
      /* Saturate: '<S19>/Saturation1' */
      rtb_Saturation1 = rtb_Switch1_p[1];
    }

    /* End of Saturate: '<S19>/Saturation1' */

    /* Saturate: '<S19>/Saturation2' */
    if (rtb_Switch1_p[2] > rtP.Saturation2_UpperSat) {
      /* Saturate: '<S19>/Saturation2' */
      rtb_Saturation2 = rtP.Saturation2_UpperSat;
    } else if (rtb_Switch1_p[2] < rtP.Saturation2_LowerSat) {
      /* Saturate: '<S19>/Saturation2' */
      rtb_Saturation2 = rtP.Saturation2_LowerSat;
    } else {
      /* Saturate: '<S19>/Saturation2' */
      rtb_Saturation2 = rtb_Switch1_p[2];
    }

    /* End of Saturate: '<S19>/Saturation2' */

    /* DiscreteIntegrator: '<S16>/integrator with sat' incorporates:
     *  Constant: '<S27>/Constant'
     *  RelationalOperator: '<S27>/Compare'
     *  Switch: '<S25>/Switch1'
     */
    if (rtDW.integratorwithsat_SYSTEM_ENABLE != 0) {
      /* DiscreteIntegrator: '<S16>/integrator with sat' */
      rtDW.integratorwithsat[0] = rtDW.integratorwithsat_DSTATE[0];
      rtDW.integratorwithsat[1] = rtDW.integratorwithsat_DSTATE[1];
      rtDW.integratorwithsat[2] = rtDW.integratorwithsat_DSTATE[2];
      rtDW.integratorwithsat[3] = rtDW.integratorwithsat_DSTATE[3];
      rtDW.integratorwithsat[4] = rtDW.integratorwithsat_DSTATE[4];
      rtDW.integratorwithsat[5] = rtDW.integratorwithsat_DSTATE[5];
    } else {
      if ((rtb_Saturation_f <= rtP.CompareToConstant1_const_e0) >=
          rtP.Switch1_Threshold_e) {
        /* Switch: '<S25>/Switch1' */
        rel_time = rtb_Saturation_f;
      } else {
        /* Switch: '<S25>/Switch1' incorporates:
         *  Constant: '<S25>/zero1'
         */
        rel_time = rtP.zero1_Value_f;
      }

      /* DiscreteIntegrator: '<S16>/integrator with sat' incorporates:
       *  DiscreteIntegrator: '<S26>/saturation ramp'
       *  Product: '<S16>/enable1'
       *  Switch: '<S25>/Switch1'
       */
      rtDW.integratorwithsat[0] = rtDW.saturationramp_DSTATE * rel_time *
        rtP.integratorwithsat_gainval + rtDW.integratorwithsat_DSTATE[0];

      /* Switch: '<S25>/Switch1' incorporates:
       *  Constant: '<S25>/zero1'
       *  Constant: '<S27>/Constant'
       *  RelationalOperator: '<S27>/Compare'
       */
      if ((rtb_Saturation1 <= rtP.CompareToConstant1_const_e0) >=
          rtP.Switch1_Threshold_e) {
        rel_time = rtb_Saturation1;
      } else {
        rel_time = rtP.zero1_Value_f;
      }

      /* DiscreteIntegrator: '<S16>/integrator with sat' incorporates:
       *  DiscreteIntegrator: '<S26>/saturation ramp'
       *  Product: '<S16>/enable1'
       *  Switch: '<S25>/Switch1'
       */
      rtDW.integratorwithsat[1] = rtDW.saturationramp_DSTATE * rel_time *
        rtP.integratorwithsat_gainval + rtDW.integratorwithsat_DSTATE[1];

      /* Switch: '<S25>/Switch1' incorporates:
       *  Constant: '<S25>/zero1'
       *  Constant: '<S27>/Constant'
       *  RelationalOperator: '<S27>/Compare'
       */
      if ((rtb_Saturation2 <= rtP.CompareToConstant1_const_e0) >=
          rtP.Switch1_Threshold_e) {
        rel_time = rtb_Saturation2;
      } else {
        rel_time = rtP.zero1_Value_f;
      }

      /* DiscreteIntegrator: '<S16>/integrator with sat' incorporates:
       *  DiscreteIntegrator: '<S26>/saturation ramp'
       *  Product: '<S16>/enable1'
       *  Switch: '<S25>/Switch1'
       */
      rtDW.integratorwithsat[2] = rtDW.saturationramp_DSTATE * rel_time *
        rtP.integratorwithsat_gainval + rtDW.integratorwithsat_DSTATE[2];

      /* Switch: '<S25>/Switch1' incorporates:
       *  Constant: '<S25>/zero1'
       *  Constant: '<S27>/Constant'
       *  RelationalOperator: '<S27>/Compare'
       */
      if ((rtb_Switch1_p[3] <= rtP.CompareToConstant1_const_e0) >=
          rtP.Switch1_Threshold_e) {
        rel_time = rtb_Switch1_p[3];
      } else {
        rel_time = rtP.zero1_Value_f;
      }

      /* DiscreteIntegrator: '<S16>/integrator with sat' incorporates:
       *  DiscreteIntegrator: '<S26>/saturation ramp'
       *  Product: '<S16>/enable1'
       *  Switch: '<S25>/Switch1'
       */
      rtDW.integratorwithsat[3] = rtDW.saturationramp_DSTATE * rel_time *
        rtP.integratorwithsat_gainval + rtDW.integratorwithsat_DSTATE[3];

      /* Switch: '<S25>/Switch1' incorporates:
       *  Constant: '<S25>/zero1'
       *  Constant: '<S27>/Constant'
       *  RelationalOperator: '<S27>/Compare'
       */
      if ((rtb_Switch1_p[4] <= rtP.CompareToConstant1_const_e0) >=
          rtP.Switch1_Threshold_e) {
        rel_time = rtb_Switch1_p[4];
      } else {
        rel_time = rtP.zero1_Value_f;
      }

      /* DiscreteIntegrator: '<S16>/integrator with sat' incorporates:
       *  DiscreteIntegrator: '<S26>/saturation ramp'
       *  Product: '<S16>/enable1'
       *  Switch: '<S25>/Switch1'
       */
      rtDW.integratorwithsat[4] = rtDW.saturationramp_DSTATE * rel_time *
        rtP.integratorwithsat_gainval + rtDW.integratorwithsat_DSTATE[4];

      /* Switch: '<S25>/Switch1' incorporates:
       *  Constant: '<S25>/zero1'
       *  Constant: '<S27>/Constant'
       *  RelationalOperator: '<S27>/Compare'
       */
      if ((rtb_Switch1_p[5] <= rtP.CompareToConstant1_const_e0) >=
          rtP.Switch1_Threshold_e) {
        rel_time = rtb_Switch1_p[5];
      } else {
        rel_time = rtP.zero1_Value_f;
      }

      /* DiscreteIntegrator: '<S16>/integrator with sat' incorporates:
       *  DiscreteIntegrator: '<S26>/saturation ramp'
       *  Product: '<S16>/enable1'
       *  Switch: '<S25>/Switch1'
       */
      rtDW.integratorwithsat[5] = rtDW.saturationramp_DSTATE * rel_time *
        rtP.integratorwithsat_gainval + rtDW.integratorwithsat_DSTATE[5];
    }

    for (r2 = 0; r2 < 6; r2++) {
      if (rtDW.integratorwithsat[r2] >= rtP.integratorwithsat_UpperSat[r2]) {
        rtDW.integratorwithsat[r2] = rtP.integratorwithsat_UpperSat[r2];
      } else if (rtDW.integratorwithsat[r2] <= rtP.integratorwithsat_LowerSat[r2])
      {
        rtDW.integratorwithsat[r2] = rtP.integratorwithsat_LowerSat[r2];
      }
    }

    /* End of DiscreteIntegrator: '<S16>/integrator with sat' */

    /* MATLAB Function: '<S16>/initial guess' incorporates:
     *  Constant: '<S8>/Fo'
     *  Constant: '<S8>/th_xo'
     *  Constant: '<S8>/th_yo1'
     */
    /* :  a_hato = zeros(6,1); */
    /* :  a_hato(1) = a_hat(1)+init_Fo; */
    rtDW.a_hato[0] = rtDW.integratorwithsat[0] + rtP.thrust_offset;

    /* :  a_hato(2) = a_hat(2)+init_thx; */
    rtDW.a_hato[1] = rtDW.integratorwithsat[1] + rtP.th_xo;

    /* :  a_hato(3) = a_hat(3)+init_thy; */
    rtDW.a_hato[2] = rtDW.integratorwithsat[2] + rtP.th_yo;

    /* :  a_hato(4) = a_hat(4); */
    rtDW.a_hato[3] = rtDW.integratorwithsat[3];

    /* :  a_hato(5) = a_hat(5); */
    rtDW.a_hato[4] = rtDW.integratorwithsat[4];

    /* :  a_hato(6) = a_hat(6); */
    rtDW.a_hato[5] = rtDW.integratorwithsat[5];

    /* Outport: '<Root>/Fo' incorporates:
     *  MATLAB Function: '<S16>/initial guess'
     */
    /* :  Fo = a_hato(1); */
    /* :  theta = [a_hato(2);a_hato(3)]; */
    /* :  torque_o = [a_hato(4);a_hato(5);a_hato(6)]; */
    rtY.Fo_j = rtDW.a_hato[0];

    /* Outport: '<Root>/Theta' incorporates:
     *  MATLAB Function: '<S16>/initial guess'
     */
    rtY.Theta[0] = rtDW.a_hato[1];
    rtY.Theta[1] = rtDW.a_hato[2];

    /* Outport: '<Root>/To' incorporates:
     *  MATLAB Function: '<S16>/initial guess'
     */
    rtY.To[0] = rtDW.integratorwithsat[3];
    rtY.To[1] = rtDW.integratorwithsat[4];
    rtY.To[2] = rtDW.integratorwithsat[5];

    /* Outport: '<Root>/To1' incorporates:
     *  DiscreteIntegrator: '<S13>/time'
     */
    rtY.To1 = rtDW.time_DSTATE_m;

    /* DiscreteTransferFcn: '<S18>/filter 1' */
    rel_time = rtP.filter1_NumCoef_b[1] * rtDW.filter1_states_p;
    for (r1 = 0; r1 < 9; r1++) {
      /* Product: '<S20>/F*R' incorporates:
       *  DiscreteTransferFcn: '<S18>/filter 1'
       */
      rtDW.FR[r1] = rtDW.R[r1] * rel_time;
    }

    /* DataTypeConversion: '<S26>/Data Type Conversion' incorporates:
     *  Constant: '<S26>/0.8'
     *  Constant: '<S26>/2'
     *  Constant: '<S28>/Constant'
     *  DiscreteIntegrator: '<S26>/counter'
     *  RelationalOperator: '<S28>/Compare'
     *  Sum: '<S26>/Add1'
     *  Sum: '<S26>/Add2'
     */
    rtb_DataTypeConversion_d = (rtDW.counter_DSTATE - (rtP.start_delay +
      rtP.second_delay) > rtP.Constant_Value);

    /* MATLAB Function: '<S13>/metric l2 norm' */
    /* :  metric = diag([1,1,0.25]); */
    /* :  SaTSa = Sa' * metric * Sa; */
    rtb_w_idx_0 = 0.0;
    for (r1 = 0; r1 < 3; r1++) {
      rtb_w_idx_0 += ((y[3 * r1 + 1] * rtb_Sa[1] + y[3 * r1] * rtb_Sa[0]) + y[3 *
                      r1 + 2] * rtb_Sa[2]) * rtb_Sa[r1];
    }

    /* DataTypeConversion: '<S13>/Data Type Conversion' incorporates:
     *  Constant: '<S13>/att_stability_threshold'
     *  Constant: '<S43>/Constant'
     *  MATLAB Function: '<S13>/metric l2 norm'
     *  RelationalOperator: '<S43>/Compare'
     *  Sum: '<S13>/Add3'
     */
    rtb_DataTypeConversion_p = (rtb_w_idx_0 - rtP.att_stability_threshold_Value <
      rtP.Constant_Value_o);

    /* DiscreteIntegrator: '<S13>/counter1' */
    if ((rtb_DataTypeConversion_p <= 0.0) && (rtDW.counter1_PrevResetState == 1))
    {
      rtDW.counter1_DSTATE = rtP.counter1_IC;
    }

    /* DataTypeConversion: '<S13>/Data Type Conversion1' incorporates:
     *  Constant: '<S13>/steady time'
     *  Constant: '<S46>/Constant'
     *  Constant: '<S47>/1.5'
     *  Constant: '<S47>/2'
     *  Constant: '<S48>/Constant'
     *  DiscreteIntegrator: '<S13>/counter1'
     *  DiscreteIntegrator: '<S47>/counter'
     *  Product: '<S13>/Product'
     *  RelationalOperator: '<S46>/Compare'
     *  RelationalOperator: '<S48>/Compare'
     *  Sum: '<S13>/Add4'
     *  Sum: '<S47>/Add1'
     *  Sum: '<S47>/Add2'
     */
    rtb_DataTypeConversion1 = (rtDW.counter_DSTATE_f - (rtP.start_delay +
      rtP.u5_Value) > rtP.Constant_Value_l) * (rtDW.counter1_DSTATE -
      rtP.steadytime_Value > rtP.Constant_Value_h);

    /* DataTypeConversion: '<S13>/Data Type Conversion2' incorporates:
     *  Constant: '<S44>/Constant'
     *  DiscreteIntegrator: '<S13>/counter2'
     *  RelationalOperator: '<S44>/Compare'
     */
    rtb_DataTypeConversion2 = (rtDW.counter2_DSTATE > rtP.Constant_Value_f);

    /* Gain: '<S5>/Gain' */
    rtb_Gain_i = rtP.Gain_Gain_n * rtb_typeconv;

    /* Sum: '<S82>/Add' incorporates:
     *  Constant: '<S82>/Constant'
     *  Constant: '<S83>/Constant'
     *  DataTypeConversion: '<S82>/Data Type Conversion'
     *  Gain: '<S82>/Gain'
     *  RelationalOperator: '<S83>/Compare'
     */
    rtb_Add_c = (real_T)((uint32_T)rtb_times_j > rtP.Constant_Value_pz) *
      rtP.Gain_Gain_h - rtP.Constant_Value_iq;

    /* Gain: '<S6>/p2p to amp' */
    rtb_p2ptoamp = rtP.p2ptoamp_Gain * rtb_pp;

    /* Product: '<S77>/PrescaledAmp' incorporates:
     *  Constant: '<S77>/PreGain'
     *  Sum: '<S6>/Sum'
     */
    rtDW.PrescaledAmp = (rtb_p2ptoamp + rtb_pp1) * rtP.PreGain_Value;

    /* Sum: '<S77>/Sum1' incorporates:
     *  Constant: '<Root>/bias voltage'
     *  Gain: '<S6>/half'
     *  Sum: '<S78>/Sum1'
     */
    rtb_w_idx_0 = rtP.half_Gain_h * rtP.biasvoltage_Value + rtb_multiply1;

    /* Product: '<S77>/PrescaledOffset' incorporates:
     *  Constant: '<S77>/PreGain'
     *  Sum: '<S77>/Sum1'
     */
    rtDW.PrescaledOffset = rtb_w_idx_0 * rtP.PreGain_Value;

    /* Sum: '<S87>/Add' incorporates:
     *  Constant: '<S87>/Constant'
     *  Constant: '<S90>/Constant'
     *  DataTypeConversion: '<S87>/Data Type Conversion'
     *  Gain: '<S87>/Gain'
     *  RelationalOperator: '<S90>/Compare'
     */
    rtb_Add_en = (real_T)((uint32_T)rtb_times_j > rtP.Constant_Value_d) *
      rtP.Gain_Gain_b - rtP.Constant_Value_p5;

    /* Gain: '<S89>/2pi' incorporates:
     *  Constant: '<Root>/FlappingFreq(hz)'
     *  Constant: '<S91>/Constant'
     *  DataTypeConversion: '<S88>/Data Type Conversion'
     *  DiscreteIntegrator: '<S77>/RampUpOrDown'
     *  Product: '<S77>/ProductPhase'
     *  RelationalOperator: '<S91>/Compare'
     */
    rtb_upi = (real_T)(rtDW.RampUpOrDown_DSTATE > rtP.Constant_Value_c) *
      rtP.freq * rtP.upi_Gain;

    /* Product: '<S78>/PrescaledAmp' incorporates:
     *  Constant: '<S78>/PreGain'
     *  Sum: '<S6>/Sum1'
     */
    rtDW.PrescaledAmp_g = (rtb_p2ptoamp - rtb_pp1) * rtP.PreGain_Value_a;

    /* Product: '<S78>/PrescaledOffset' incorporates:
     *  Constant: '<S78>/PreGain'
     */
    rtDW.PrescaledOffset_p = rtb_w_idx_0 * rtP.PreGain_Value_a;

    /* Sum: '<S96>/Add' incorporates:
     *  Constant: '<S96>/Constant'
     *  Constant: '<S99>/Constant'
     *  DataTypeConversion: '<S96>/Data Type Conversion'
     *  Gain: '<S96>/Gain'
     *  RelationalOperator: '<S99>/Compare'
     */
    rtb_Add_io = (real_T)((uint32_T)rtb_times_j > rtP.Constant_Value_ij) *
      rtP.Gain_Gain_o - rtP.Constant_Value_oz;

    /* Gain: '<S98>/2pi' incorporates:
     *  Constant: '<Root>/FlappingFreq(hz)'
     *  Constant: '<S100>/Constant'
     *  DataTypeConversion: '<S97>/Data Type Conversion'
     *  DiscreteIntegrator: '<S78>/RampUpOrDown'
     *  Product: '<S78>/ProductPhase'
     *  RelationalOperator: '<S100>/Compare'
     */
    rtb_upi_m = (real_T)(rtDW.RampUpOrDown_DSTATE_k > rtP.Constant_Value_i) *
      rtP.freq * rtP.upi_Gain_o;

    /* Gain: '<S6>/GainBias' incorporates:
     *  Constant: '<Root>/bias voltage'
     */
    rtDW.GainBias = rtP.GainBias_Gain_p * rtP.biasvoltage_Value;
  }

  {
    real_T denAccum;
    int32_T i;
    int32_T memOffset;

    /* Update for DiscreteIntegrator: '<S7>/SaturatingRamp' incorporates:
     *  Constant: '<S7>/constant'
     */
    rtDW.SaturatingRamp_DSTATE += rtP.SaturatingRamp_gainval *
      rtP.constant_Value;

    /* Update for DiscreteIntegrator: '<S76>/SaturatingRamp' */
    rtDW.SaturatingRamp_DSTATE_j += rtP.SaturatingRamp_gainval_l * rtb_Add_c;
    if (rtDW.SaturatingRamp_DSTATE_j >= rtP.SaturatingRamp_UpperSat) {
      rtDW.SaturatingRamp_DSTATE_j = rtP.SaturatingRamp_UpperSat;
    } else if (rtDW.SaturatingRamp_DSTATE_j <= rtP.SaturatingRamp_LowerSat) {
      rtDW.SaturatingRamp_DSTATE_j = rtP.SaturatingRamp_LowerSat;
    }

    /* End of Update for DiscreteIntegrator: '<S76>/SaturatingRamp' */

    /* Update for DiscreteTransferFcn: '<S81>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states = (rtDW.GainBias -
      rtP.DiscreteTransferFcn_DenCoef[1] * rtDW.DiscreteTransferFcn_states) /
      rtP.DiscreteTransferFcn_DenCoef[0];

    /* Update for DiscreteIntegrator: '<S77>/RampUpOrDown' */
    rtDW.RampUpOrDown_DSTATE += rtP.RampUpOrDown_gainval * rtb_Add_en;
    if (rtDW.RampUpOrDown_DSTATE >= rtP.RampUpOrDown_UpperSat) {
      rtDW.RampUpOrDown_DSTATE = rtP.RampUpOrDown_UpperSat;
    } else if (rtDW.RampUpOrDown_DSTATE <= rtP.RampUpOrDown_LowerSat) {
      rtDW.RampUpOrDown_DSTATE = rtP.RampUpOrDown_LowerSat;
    }

    /* End of Update for DiscreteIntegrator: '<S77>/RampUpOrDown' */

    /* Update for DiscreteTransferFcn: '<S86>/Discrete Transfer Fcn' incorporates:
     *  Outport: '<Root>/mu v'
     */
    rtDW.DiscreteTransferFcn_states_n = (rtY.muv -
      rtP.DiscreteTransferFcn_DenCoef_c[1] * rtDW.DiscreteTransferFcn_states_n) /
      rtP.DiscreteTransferFcn_DenCoef_c[0];

    /* Update for DiscreteIntegrator: '<S89>/omega*t' */
    rtDW.omegat_DSTATE += rtP.omegat_gainval * rtb_upi;

    /* Update for DiscreteTransferFcn: '<S84>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_a = (rtDW.PrescaledAmp -
      rtP.DiscreteTransferFcn_DenCoef_f[1] * rtDW.DiscreteTransferFcn_states_a) /
      rtP.DiscreteTransferFcn_DenCoef_f[0];

    /* Update for DiscreteTransferFcn: '<S85>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_ac = (rtDW.PrescaledOffset -
      rtP.DiscreteTransferFcn_DenCoef_n[1] * rtDW.DiscreteTransferFcn_states_ac)
      / rtP.DiscreteTransferFcn_DenCoef_n[0];

    /* Update for DiscreteIntegrator: '<S78>/RampUpOrDown' */
    rtDW.RampUpOrDown_DSTATE_k += rtP.RampUpOrDown_gainval_d * rtb_Add_io;
    if (rtDW.RampUpOrDown_DSTATE_k >= rtP.RampUpOrDown_UpperSat_j) {
      rtDW.RampUpOrDown_DSTATE_k = rtP.RampUpOrDown_UpperSat_j;
    } else if (rtDW.RampUpOrDown_DSTATE_k <= rtP.RampUpOrDown_LowerSat_n) {
      rtDW.RampUpOrDown_DSTATE_k = rtP.RampUpOrDown_LowerSat_n;
    }

    /* End of Update for DiscreteIntegrator: '<S78>/RampUpOrDown' */

    /* Update for DiscreteTransferFcn: '<S95>/Discrete Transfer Fcn' incorporates:
     *  Outport: '<Root>/mu v'
     */
    rtDW.DiscreteTransferFcn_states_av = (rtY.muv -
      rtP.DiscreteTransferFcn_DenCoef_i[1] * rtDW.DiscreteTransferFcn_states_av)
      / rtP.DiscreteTransferFcn_DenCoef_i[0];

    /* Update for DiscreteIntegrator: '<S98>/SmoothClock' */
    rtDW.SmoothClock_DSTATE += rtP.SmoothClock_gainval * rtb_upi_m;

    /* Update for DiscreteTransferFcn: '<S93>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_p = (rtDW.PrescaledAmp_g -
      rtP.DiscreteTransferFcn_DenCoef_d[1] * rtDW.DiscreteTransferFcn_states_p) /
      rtP.DiscreteTransferFcn_DenCoef_d[0];

    /* Update for DiscreteTransferFcn: '<S94>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_a5 = (rtDW.PrescaledOffset_p -
      rtP.DiscreteTransferFcn_DenCoef_fy[1] * rtDW.DiscreteTransferFcn_states_a5)
      / rtP.DiscreteTransferFcn_DenCoef_fy[0];

    /* Update for DiscreteTransferFcn: '<S5>/low pass' */
    rtDW.lowpass_states[2] = rtDW.lowpass_states[1];
    rtDW.lowpass_states[1] = rtDW.lowpass_states[0];
    rtDW.lowpass_states[0] = rtDW.lowpass_tmp;

    /* Update for DiscreteTransferFcn: '<S5>/low pass1' */
    rtDW.lowpass1_states[2] = rtDW.lowpass1_states[1];
    rtDW.lowpass1_states[1] = rtDW.lowpass1_states[0];
    rtDW.lowpass1_states[0] = rtDW.lowpass1_tmp;

    /* Update for DiscreteTransferFcn: '<S5>/low pass2' */
    rtDW.lowpass2_states[1] = rtDW.lowpass2_states[0];
    rtDW.lowpass2_states[0] = rtDW.lowpass2_tmp;

    /* Update for UnitDelay: '<S39>/Delay Input'
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE[0] = rtb_TmpSignalConversionAtfilter[0];

    /* Update for UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE[0] = rtb_PoleYk1UkZeroUk1[0];

    /* Update for UnitDelay: '<S40>/Delay Input'
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_o[0] = rtb_TmpSignalConversionAtfilter[0];

    /* Update for UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_k[0] = rtb_PoleYk1UkZeroUk1_j[0];

    /* Update for UnitDelay: '<S41>/Delay Input'
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_i[0] = rtb_PoleYk1UkZeroUk1_j[0];

    /* Update for UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_a[0] = rtb_PoleYk1UkZeroUk1_h[0];

    /* Update for UnitDelay: '<S39>/Delay Input'
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE[1] = rtb_TmpSignalConversionAtfilter[1];

    /* Update for UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE[1] = rtb_PoleYk1UkZeroUk1[1];

    /* Update for UnitDelay: '<S40>/Delay Input'
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_o[1] = rtb_TmpSignalConversionAtfilter[1];

    /* Update for UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_k[1] = rtb_PoleYk1UkZeroUk1_j[1];

    /* Update for UnitDelay: '<S41>/Delay Input'
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_i[1] = rtb_PoleYk1UkZeroUk1_j[1];

    /* Update for UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_a[1] = rtb_PoleYk1UkZeroUk1_h[1];

    /* Update for UnitDelay: '<S39>/Delay Input'
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE[2] = rtb_TmpSignalConversionAtfilter[2];

    /* Update for UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE[2] = rtb_PoleYk1UkZeroUk1[2];

    /* Update for UnitDelay: '<S40>/Delay Input'
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_o[2] = rtb_TmpSignalConversionAtfilter[2];

    /* Update for UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_k[2] = rtb_PoleYk1UkZeroUk1_j[2];

    /* Update for UnitDelay: '<S41>/Delay Input'
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_i[2] = rtb_PoleYk1UkZeroUk1_j[2];

    /* Update for UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_a[2] = rtb_PoleYk1UkZeroUk1_h[2];
    for (i = 0; i < 9; i++) {
      /* Update for UnitDelay: '<S37>/Delay Input'
       *
       * Block description for '<S37>/Delay Input':
       *
       *  Store in Global RAM
       */
      rtDW.DelayInput_DSTATE_j[i] = rtb_att_s[i];

      /* Update for UnitDelay: '<S37>/Delay Output'
       *
       * Block description for '<S37>/Delay Output':
       *
       *  Store in Global RAM
       */
      rtDW.DelayOutput_DSTATE_b[i] = rtb_PoleYk1UkZeroUk1_n[i];

      /* Update for DiscreteTransferFcn: '<S35>/low pass3' */
      memOffset = i << 1;
      rtDW.lowpass3_states[memOffset - -1] = rtDW.lowpass3_states[memOffset];
      rtDW.lowpass3_states[memOffset] = rtDW.lowpass3_tmp[i];
    }

    /* Update for UnitDelay: '<S38>/Delay Input'
     *
     * Block description for '<S38>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_c = rtb_att_s_a;

    /* Update for UnitDelay: '<S38>/Delay Output'
     *
     * Block description for '<S38>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_c = rtb_PoleYk1UkZeroUk1_e;

    /* Update for DiscreteTransferFcn: '<S36>/low pass3' */
    rtDW.lowpass3_states_b[1] = rtDW.lowpass3_states_b[0];
    rtDW.lowpass3_states_b[0] = rtDW.lowpass3_tmp_b;

    /* Update for DiscreteIntegrator: '<S14>/time' incorporates:
     *  Constant: '<S14>/constant'
     */
    rtDW.time_DSTATE += rtP.time_gainval * rtP.constant_Value_h;

    /* Update for UnitDelay: '<S8>/Unit Delay1' */
    rtDW.UnitDelay1_DSTATE[0] = rtb_Saturation_f;
    rtDW.UnitDelay1_DSTATE[1] = rtb_Saturation1;
    rtDW.UnitDelay1_DSTATE[2] = rtb_Saturation2;
    rtDW.UnitDelay1_DSTATE[3] = rtb_Switch1_p[3];
    rtDW.UnitDelay1_DSTATE[4] = rtb_Switch1_p[4];
    rtDW.UnitDelay1_DSTATE[5] = rtb_Switch1_p[5];
    for (i = 0; i < 6; i++) {
      /* Update for UnitDelay: '<S8>/Unit Delay' */
      rtDW.UnitDelay_DSTATE[i] = rtDW.a_hato[i];

      /* Update for DiscreteTransferFcn: '<S8>/low pass3' */
      memOffset = i << 1;
      rtDW.lowpass3_states_c[memOffset - -1] = rtDW.lowpass3_states_c[memOffset];
      rtDW.lowpass3_states_c[memOffset] = rtDW.lowpass3_tmp_m[i];
    }

    /* Update for DiscreteIntegrator: '<S73>/SaturatingRamp1' incorporates:
     *  Constant: '<S73>/constant'
     */
    rtDW.SaturatingRamp1_DSTATE += rtP.SaturatingRamp1_gainval *
      rtP.constant_Value_a;

    /* Update for DiscreteIntegrator: '<S5>/SaturatingRamp' */
    rtDW.SaturatingRamp_DSTATE_p += rtP.SaturatingRamp_gainval_g * rtb_Gain_i;
    if (rtb_typeconv > 0.0) {
      rtDW.SaturatingRamp_PrevResetState = 1;
    } else if (rtb_typeconv < 0.0) {
      rtDW.SaturatingRamp_PrevResetState = -1;
    } else if (rtb_typeconv == 0.0) {
      rtDW.SaturatingRamp_PrevResetState = 0;
    } else {
      rtDW.SaturatingRamp_PrevResetState = 2;
    }

    /* End of Update for DiscreteIntegrator: '<S5>/SaturatingRamp' */

    /* Update for DiscreteIntegrator: '<S4>/SaturatingRamp' incorporates:
     *  Constant: '<S4>/constant'
     */
    rtDW.SaturatingRamp_DSTATE_a += rtP.SaturatingRamp_gainval_b *
      rtP.constant_Value_n;

    /* Update for DiscreteIntegrator: '<S26>/saturation ramp' */
    rtDW.saturationramp_DSTATE += rtP.saturationramp_gainval *
      rtb_DataTypeConversion_d;
    if (rtDW.saturationramp_DSTATE >= rtP.saturationramp_UpperSat) {
      rtDW.saturationramp_DSTATE = rtP.saturationramp_UpperSat;
    } else if (rtDW.saturationramp_DSTATE <= rtP.saturationramp_LowerSat) {
      rtDW.saturationramp_DSTATE = rtP.saturationramp_LowerSat;
    }

    /* End of Update for DiscreteIntegrator: '<S26>/saturation ramp' */

    /* Update for DiscreteTransferFcn: '<S20>/filter derivative 1' */
    rtDW.filterderivative1_states[1] = rtDW.filterderivative1_states[0];
    rtDW.filterderivative1_states[0] = rtDW.filterderivative1_tmp[0];
    rtDW.filterderivative1_states[3] = rtDW.filterderivative1_states[2];
    rtDW.filterderivative1_states[2] = rtDW.filterderivative1_tmp[1];
    rtDW.filterderivative1_states[5] = rtDW.filterderivative1_states[4];
    rtDW.filterderivative1_states[4] = rtDW.filterderivative1_tmp[2];
    for (i = 0; i < 9; i++) {
      /* Update for DiscreteTransferFcn: '<S20>/filter 1' incorporates:
       *  DiscreteTransferFcn: '<S20>/filter1'
       */
      memOffset = i << 1;
      denAccum = (rtDW.R[i] - rtP.filter1_DenCoef[1] *
                  rtDW.filter1_states[memOffset]) -
        rtDW.filter1_states[memOffset + 1] * rtP.filter1_DenCoef[2];
      rtDW.filter1_states[memOffset - -1] = rtDW.filter1_states[memOffset];
      rtDW.filter1_states[memOffset] = denAccum / rtP.filter1_DenCoef[0];

      /* Update for DiscreteTransferFcn: '<S20>/filter1' */
      denAccum = (rtDW.FR[i] - rtP.filter1_DenCoef_f[1] *
                  rtDW.filter1_states_j[memOffset]) -
        rtDW.filter1_states_j[memOffset + 1] * rtP.filter1_DenCoef_f[2];
      rtDW.filter1_states_j[memOffset - -1] = rtDW.filter1_states_j[memOffset];
      rtDW.filter1_states_j[memOffset] = denAccum / rtP.filter1_DenCoef_f[0];
    }

    /* Update for DiscreteIntegrator: '<S16>/integrator with sat' */
    rtDW.integratorwithsat_SYSTEM_ENABLE = 0U;
    for (i = 0; i < 6; i++) {
      rtDW.integratorwithsat_DSTATE[i] = rtDW.integratorwithsat[i];
    }

    /* End of Update for DiscreteIntegrator: '<S16>/integrator with sat' */

    /* Update for DiscreteIntegrator: '<S13>/time' */
    rtDW.time_DSTATE_m += rtP.time_gainval_g * rtb_DataTypeConversion2;

    /* Update for DiscreteTransferFcn: '<S18>/filter 1' */
    rtDW.filter1_states_p = (rtDW.F - rtP.filter1_DenCoef_c[1] *
      rtDW.filter1_states_p) / rtP.filter1_DenCoef_c[0];

    /* Update for DiscreteIntegrator: '<S26>/counter' incorporates:
     *  Constant: '<S26>/constant'
     */
    rtDW.counter_DSTATE += rtP.counter_gainval * rtP.constant_Value_e;

    /* Update for DiscreteIntegrator: '<S13>/counter1' */
    rtDW.counter1_DSTATE += rtP.counter1_gainval * rtb_DataTypeConversion_p;
    if (rtb_DataTypeConversion_p > 0.0) {
      rtDW.counter1_PrevResetState = 1;
    } else if (rtb_DataTypeConversion_p < 0.0) {
      rtDW.counter1_PrevResetState = -1;
    } else if (rtb_DataTypeConversion_p == 0.0) {
      rtDW.counter1_PrevResetState = 0;
    } else {
      rtDW.counter1_PrevResetState = 2;
    }

    /* End of Update for DiscreteIntegrator: '<S13>/counter1' */

    /* Update for DiscreteIntegrator: '<S13>/counter2' */
    rtDW.counter2_DSTATE += rtP.counter2_gainval * rtb_DataTypeConversion1;

    /* Update for DiscreteIntegrator: '<S47>/counter' incorporates:
     *  Constant: '<S47>/constant'
     */
    rtDW.counter_DSTATE_f += rtP.counter_gainval_o * rtP.constant_Value_m;
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  rtM->Timing.t[0] =
    ((time_T)(++rtM->Timing.clockTick0)) * rtM->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.0001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.0001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    rtM->Timing.clockTick1++;
  }
}

/* Model initialize function */
void integrated_controller_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  rtP.CompareToConstant1_const = rtInf;
  rtP.CompareToConstant1_const_i = rtInf;
  rtP.CompareToConstant1_const_e = rtInf;
  rtP.CompareToConstant1_const_c = rtInf;
  rtP.CompareToConstant1_const_m = rtInf;
  rtP.CompareToConstant1_const_e0 = rtInf;

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&rtM->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.0001;

  {
    int32_T i;

    /* InitializeConditions for DiscreteIntegrator: '<S7>/SaturatingRamp' */
    rtDW.SaturatingRamp_DSTATE = rtP.SaturatingRamp_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S76>/SaturatingRamp' */
    rtDW.SaturatingRamp_DSTATE_j = rtP.SaturatingRamp_IC_d;

    /* InitializeConditions for DiscreteTransferFcn: '<S81>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states = rtP.DiscreteTransferFcn_InitialStat;

    /* InitializeConditions for DiscreteIntegrator: '<S77>/RampUpOrDown' */
    rtDW.RampUpOrDown_DSTATE = rtP.RampUpOrDown_IC;

    /* InitializeConditions for DiscreteTransferFcn: '<S86>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_n = rtP.DiscreteTransferFcn_InitialSt_m;

    /* InitializeConditions for DiscreteIntegrator: '<S89>/omega*t' */
    rtDW.omegat_DSTATE = rtP.omegat_IC;

    /* InitializeConditions for DiscreteTransferFcn: '<S84>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_a = rtP.DiscreteTransferFcn_InitialSt_p;

    /* InitializeConditions for DiscreteTransferFcn: '<S85>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_ac = rtP.DiscreteTransferFcn_InitialSt_a;

    /* InitializeConditions for DiscreteIntegrator: '<S78>/RampUpOrDown' */
    rtDW.RampUpOrDown_DSTATE_k = rtP.RampUpOrDown_IC_d;

    /* InitializeConditions for DiscreteTransferFcn: '<S95>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_av = rtP.DiscreteTransferFcn_InitialSt_n;

    /* InitializeConditions for DiscreteIntegrator: '<S98>/SmoothClock' */
    rtDW.SmoothClock_DSTATE = rtP.SmoothClock_IC;

    /* InitializeConditions for DiscreteTransferFcn: '<S93>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_p = rtP.DiscreteTransferFcn_InitialSt_g;

    /* InitializeConditions for DiscreteTransferFcn: '<S94>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_states_a5 = rtP.DiscreteTransferFcn_InitialSt_l;

    /* InitializeConditions for DiscreteTransferFcn: '<S5>/low pass' */
    rtDW.lowpass_states[0] = rtP.lowpass_InitialStates;

    /* InitializeConditions for DiscreteTransferFcn: '<S5>/low pass1' */
    rtDW.lowpass1_states[0] = rtP.lowpass1_InitialStates;

    /* InitializeConditions for DiscreteTransferFcn: '<S5>/low pass' */
    rtDW.lowpass_states[1] = rtP.lowpass_InitialStates;

    /* InitializeConditions for DiscreteTransferFcn: '<S5>/low pass1' */
    rtDW.lowpass1_states[1] = rtP.lowpass1_InitialStates;

    /* InitializeConditions for DiscreteTransferFcn: '<S5>/low pass' */
    rtDW.lowpass_states[2] = rtP.lowpass_InitialStates;

    /* InitializeConditions for DiscreteTransferFcn: '<S5>/low pass1' */
    rtDW.lowpass1_states[2] = rtP.lowpass1_InitialStates;

    /* InitializeConditions for DiscreteTransferFcn: '<S5>/low pass2' */
    rtDW.lowpass2_states[0] = rtP.lowpass2_InitialStates;
    rtDW.lowpass2_states[1] = rtP.lowpass2_InitialStates;

    /* InitializeConditions for UnitDelay: '<S39>/Delay Input'
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE[0] = rtP.derivativefilter_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE[0] = rtP.derivativefilter_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S40>/Delay Input'
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_o[0] = rtP.derivativefilter1_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_k[0] = rtP.derivativefilter1_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S41>/Delay Input'
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_i[0] = rtP.derivativefilter2_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_a[0] = rtP.derivativefilter2_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S39>/Delay Input'
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE[1] = rtP.derivativefilter_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE[1] = rtP.derivativefilter_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S40>/Delay Input'
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_o[1] = rtP.derivativefilter1_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_k[1] = rtP.derivativefilter1_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S41>/Delay Input'
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_i[1] = rtP.derivativefilter2_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_a[1] = rtP.derivativefilter2_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S39>/Delay Input'
     *
     * Block description for '<S39>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE[2] = rtP.derivativefilter_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S39>/Delay Output'
     *
     * Block description for '<S39>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE[2] = rtP.derivativefilter_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S40>/Delay Input'
     *
     * Block description for '<S40>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_o[2] = rtP.derivativefilter1_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S40>/Delay Output'
     *
     * Block description for '<S40>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_k[2] = rtP.derivativefilter1_ICPrevOutput;

    /* InitializeConditions for UnitDelay: '<S41>/Delay Input'
     *
     * Block description for '<S41>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_i[2] = rtP.derivativefilter2_ICPrevInput;

    /* InitializeConditions for UnitDelay: '<S41>/Delay Output'
     *
     * Block description for '<S41>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_a[2] = rtP.derivativefilter2_ICPrevOutput;
    for (i = 0; i < 9; i++) {
      /* InitializeConditions for UnitDelay: '<S37>/Delay Input'
       *
       * Block description for '<S37>/Delay Input':
       *
       *  Store in Global RAM
       */
      rtDW.DelayInput_DSTATE_j[i] = rtP.filteredderivative_ICPrevInput;

      /* InitializeConditions for UnitDelay: '<S37>/Delay Output'
       *
       * Block description for '<S37>/Delay Output':
       *
       *  Store in Global RAM
       */
      rtDW.DelayOutput_DSTATE_b[i] = rtP.filteredderivative_ICPrevOutput;
    }

    /* InitializeConditions for DiscreteTransferFcn: '<S35>/low pass3' */
    for (i = 0; i < 18; i++) {
      rtDW.lowpass3_states[i] = rtP.lowpass3_InitialStates;
    }

    /* End of InitializeConditions for DiscreteTransferFcn: '<S35>/low pass3' */

    /* InitializeConditions for UnitDelay: '<S38>/Delay Input'
     *
     * Block description for '<S38>/Delay Input':
     *
     *  Store in Global RAM
     */
    rtDW.DelayInput_DSTATE_c = rtP.filteredderivative_ICPrevInpu_e;

    /* InitializeConditions for UnitDelay: '<S38>/Delay Output'
     *
     * Block description for '<S38>/Delay Output':
     *
     *  Store in Global RAM
     */
    rtDW.DelayOutput_DSTATE_c = rtP.filteredderivative_ICPrevOutp_k;

    /* InitializeConditions for DiscreteTransferFcn: '<S36>/low pass3' */
    rtDW.lowpass3_states_b[0] = rtP.lowpass3_InitialStates_l;
    rtDW.lowpass3_states_b[1] = rtP.lowpass3_InitialStates_l;

    /* InitializeConditions for DiscreteIntegrator: '<S14>/time' */
    rtDW.time_DSTATE = rtP.time_IC;
    for (i = 0; i < 6; i++) {
      /* InitializeConditions for UnitDelay: '<S8>/Unit Delay' */
      rtDW.UnitDelay_DSTATE[i] = rtP.UnitDelay_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S8>/Unit Delay1' */
      rtDW.UnitDelay1_DSTATE[i] = rtP.UnitDelay1_InitialCondition;
    }

    /* InitializeConditions for DiscreteTransferFcn: '<S8>/low pass3' */
    for (i = 0; i < 12; i++) {
      rtDW.lowpass3_states_c[i] = rtP.lowpass3_InitialStates_g;
    }

    /* End of InitializeConditions for DiscreteTransferFcn: '<S8>/low pass3' */

    /* InitializeConditions for DiscreteIntegrator: '<S73>/SaturatingRamp1' */
    rtDW.SaturatingRamp1_DSTATE = rtP.SaturatingRamp1_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S5>/SaturatingRamp' */
    rtDW.SaturatingRamp_DSTATE_p = rtP.SaturatingRamp_IC_a;
    rtDW.SaturatingRamp_PrevResetState = 2;

    /* InitializeConditions for DiscreteIntegrator: '<S4>/SaturatingRamp' */
    rtDW.SaturatingRamp_DSTATE_a = rtP.SaturatingRamp_IC_c;

    /* InitializeConditions for DiscreteIntegrator: '<S26>/saturation ramp' */
    rtDW.saturationramp_DSTATE = rtP.saturationramp_IC;

    /* InitializeConditions for DiscreteTransferFcn: '<S20>/filter derivative 1' */
    for (i = 0; i < 6; i++) {
      rtDW.filterderivative1_states[i] = rtP.filterderivative1_InitialStates;
    }

    /* End of InitializeConditions for DiscreteTransferFcn: '<S20>/filter derivative 1' */
    for (i = 0; i < 18; i++) {
      /* InitializeConditions for DiscreteTransferFcn: '<S20>/filter 1' */
      rtDW.filter1_states[i] = rtP.filter1_InitialStates;

      /* InitializeConditions for DiscreteTransferFcn: '<S20>/filter1' */
      rtDW.filter1_states_j[i] = rtP.filter1_InitialStates_n;
    }

    /* InitializeConditions for DiscreteIntegrator: '<S16>/integrator with sat' */
    for (i = 0; i < 6; i++) {
      rtDW.integratorwithsat_DSTATE[i] = rtP.integratorwithsat_IC;
    }

    /* End of InitializeConditions for DiscreteIntegrator: '<S16>/integrator with sat' */

    /* InitializeConditions for DiscreteIntegrator: '<S13>/time' */
    rtDW.time_DSTATE_m = rtP.time_IC_f;

    /* InitializeConditions for DiscreteTransferFcn: '<S18>/filter 1' */
    rtDW.filter1_states_p = rtP.filter1_InitialStates_i;

    /* InitializeConditions for DiscreteIntegrator: '<S26>/counter' */
    rtDW.counter_DSTATE = rtP.counter_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S13>/counter1' */
    rtDW.counter1_DSTATE = rtP.counter1_IC;
    rtDW.counter1_PrevResetState = 2;

    /* InitializeConditions for DiscreteIntegrator: '<S13>/counter2' */
    rtDW.counter2_DSTATE = rtP.counter2_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S47>/counter' */
    rtDW.counter_DSTATE_f = rtP.counter_IC_n;

    /* Enable for DiscreteIntegrator: '<S16>/integrator with sat' */
    rtDW.integratorwithsat_SYSTEM_ENABLE = 1U;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
