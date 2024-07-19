#ifndef DECOMP_H
#define DECOMP_H

#ifdef _cplusplus
extern "C" {
#endif

#include "blas.h"
#include "core.h"

uint32_t svd_osj_f32(Mat_f32* A, Mat_f32* V, Vec_f32* min_vec);
uint32_t svd_osj_transpose_f32(Mat_f32* A, Vec_f32* min_vec);

#ifdef _cplusplus
}
#endif

#endif // DECOMP_H
