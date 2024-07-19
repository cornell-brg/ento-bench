#ifndef LINALG_CORE_H
#define LINALG_CORE_H

#ifdef __cpluplus
extern "C" {
#endif

#include "blas.h"
#include <stdio.h>

void        vv_proj_f32(Vec_f32* v1, Vec_f32* v2, Vec_f32* proj);
blas_status m_inv2x2_f32(Mat_f32* mat, Mat_f32* inv);
blas_status m_inv3x3_f32(Mat_f32* mat, Mat_f32* inv);
void        v_normalize_f32(Vec_f32* v1);
void        v_mean_f32(Vec_f32* v, float32_t* mean);
blas_status givens_rhs_f32(Vec_f32* vi, Vec_f32* vj, float32_t c, float32_t s);
blas_status diag_mult_mat_f32(Vec_f32* diag, Mat_f32* mat, Mat_f32* dst);
blas_status eye_f32(Mat_f32* mat);

static inline void v_zeros_f32(Vec_f32* vec) { for (uint32_t i = 0; i < vec->n; i++) vec->data[i] = 0; }
static inline void zeros_f32(float32_t* v, uint32_t N) { for (uint32_t i = 0; i < N; i++) v[i] = 0; }
static inline void m_zeros_f32(Mat_f32* mat) { for (uint32_t i = 0; i < mat->m*mat->n; i++) mat->data[i] = 0;}

#ifdef __cpluplus
}
#endif

#endif
