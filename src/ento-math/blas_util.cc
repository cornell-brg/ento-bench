#include "core.h"
#include <string.h>

void vv_proj_f32(Vec_f32 *v1, Vec_f32 *v2, Vec_f32 *proj)
{
  float32_t intermediate, scaling;
  vv_dot_f32(v1, v1, &intermediate);
  vv_dot_f32(v1, v2, &scaling);
  scaling /= intermediate;
  v_scale_f32(v1, scaling, proj);
}

void v_mean_f32(Vec_f32 *v, float32_t *mean)
{
  uint32_t i;
  float32_t sum = 0;
  float32_t scale = (float32_t) 1.0/v->n;
  for (i = 0; i < v->n; i++)
  {
    sum += v->data[i];
  }
  *mean = sum * scale;
}

blas_status diag_mult_mat_f32(Vec_f32 *diag, Mat_f32 *mat, Mat_f32 *dst)
{
  uint32_t rows = mat->m;
  uint32_t cols = mat->n;

  if (rows != diag->n) return 0;
  if (dst->m != diag->n) return 0;

  uint32_t i;
  Vec_f32 row, dst_row;
  row.n = mat->n;
  dst_row.n = mat->n;
  for (i = 0; i < rows; i++)
  {
    row.data = &mat->data[i*cols];
    dst_row.data = &dst->data[i*cols];
    v_scale_f32(&row, diag->data[i], &dst_row);
  }
  return 1;
}

blas_status eye_f32(Mat_f32* mat)
{

  if (mat->m != mat->n) return 0;

  memset(mat->data, 0, mat->m * mat->n * 4);

  uint32_t i;
  for (i = 0; i < mat->n; i++)
      mat->data[i*mat->n + i] = 1.0f;
  return 1;
}

blas_status m_inv3x3_f32(Mat_f32 *mat, Mat_f32 *inv)
{
  if (mat->m != mat->n || inv->m != mat->m || inv->n != mat->n) return 0;

  float32_t* m = mat->data;
  float32_t det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
                  m[1] * (m[3] * m[8] - m[5] * m[6]) +
                  m[3] * (m[3] * m[7] - m[4] * m[6]);

  float32_t invdet = 1.0f / det;

  inv->data[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
  inv->data[1] = (m[2] * m[7] - m[1] * m[7]) * invdet;
  inv->data[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
  inv->data[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
  inv->data[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
  inv->data[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
  inv->data[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
  inv->data[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
  inv->data[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

  return 1;
}

void v_normalize_f32(Vec_f32 *vec)
{
  float32_t norm = v_l2norm_f32(vec);
  v_scale_f32(vec, 1.0f/norm, vec);
}
