#include "blas.h"
#include "math.h"
#include <string.h>

// Initialization
void vec_init_f32(float32_t *data, uint32_t n, Vec_f32 *dst)
{
  dst->n = n;
  dst->data = data;
}

blas_status vec_copy_f32(Vec_f32* in, Vec_f32* dst)
{
  if (!in) return 0;
  if (!dst) return 0;
  if (in->n != dst->n) return 0;
  memcpy(dst->data, in->data, dst->n * 4);
  return 1;
}

blas_status vec_copy_from_f32(Vec_f32* vec, float32_t* data, uint32_t N)
{
  if (vec->n != N) return 0;
  memcpy(vec->data, data, N * 4);
  return 1;
}

void mat_init_f32(float32_t *data, uint32_t m, uint32_t n, Mat_f32 *dst)
{
  dst->m = m;
  dst->n = n;
  dst->data = data;
}

blas_status mat_copy_f32(Mat_f32 *in, Mat_f32* dst)
{
  if (in->m != dst-> m || in->n != dst->n) return 0;
  uint32_t sz = dst->m * dst->n;
  memcpy(dst->data, in->data, sz * 4);
  return 1;
}

blas_status mat_copy_from_f32(Mat_f32 *mat, float32_t *data, float32_t sz)
{
  if (mat->m * mat->n != sz) return 0;
  memcpy(mat->data, data, sz * 4);
  return 1;
}

blas_status mat_trans_from_f32(Mat_f32 *mat, float32_t *data, float32_t sz)
{
  if (sz != mat->n * mat->m) return 0;
  uint32_t m = mat->m; uint32_t n = mat->n;
  for (uint32_t r = 0; r < m; r++)
  {
    for (uint32_t c = 0; c < n; c++)
    {
      mat->data[r * n + c] = data[c * m + r];
    }
  }
  return 1;
}

blas_status mat_trans_f32(Mat_f32 *mat, Mat_f32 *transp)
{
  if (transp->m != mat->n && transp->n != mat->m) return 0;
  uint32_t m = transp->m; uint32_t n = transp->n;
  for (uint32_t r = 0; r < m; r++)
  {
    for (uint32_t c = 0; c < n; c++)
    {
      transp->data[r * n + c] = mat->data[c * m + r];
    }
  }
  return 1;
}

blas_status vv_add_f32(Vec_f32 *v1, Vec_f32 *v2, Vec_f32 *dst)
{
  if (v1->n != v2->n && v1->n != dst->n) return 0;
  for (uint32_t i = 0; i < v2->n; i++)
  {
    dst->data[i] = v1->data[i] + v2->data[i];
  }
  return 1;
}

blas_status vv_sub_f32(Vec_f32 *v1, Vec_f32 *v2, Vec_f32 *dst) 
{
  if (v1->n != v2->n && v1->n != dst->n) return 0;
  for (uint32_t i = 0; i < v2->n; i++)
  {
    dst->data[i] = v1->data[i] - v2->data[i];
  }
  return 1;
}

blas_status vv_mult_f32(Vec_f32 *v1, Vec_f32 *v2, Vec_f32 *dst) 
{
  if (v1->n != v2->n && v1->n != dst->n) return 0;
  for (uint32_t i = 0; i < v2->n; i++)
  {
    dst->data[i] = v1->data[i] * v2->data[i];
  }
  return 1;
}

blas_status vv_dot_f32(const Vec_f32 *v1, const Vec_f32 *v2, float32_t *dst)
{
  if (v1->n != v2->n) return 0;
  *dst = 0;
  for (uint32_t i = 0; i < v2->n; i++)
  {
    *dst += v1->data[i] * v2->data[i];
  }
  return 1;
}

blas_status v_scale_f32(Vec_f32 *vec, float32_t scale, Vec_f32 *dst)
{
  if (vec->n != dst->n && vec->n != 0) return 0;
  for (uint32_t i = 0; i < vec->n; i++)
  {
    dst->data[i] = vec->data[i] * scale;
  }
  return 1;
}

blas_status v_offset_f32(Vec_f32 *vec, float32_t offset, Vec_f32 *dst)
{
  if (vec->n != dst->n && vec->n != 0) return 0;
  for (uint32_t i = 0; i < vec->n; i++)
  {
    dst->data[i] = vec->data[i] + offset;
  }
  return 1;
}

float32_t v_l2norm_f32(Vec_f32* vec)
{
  float32_t norm;
  vv_dot_f32(vec, vec, &norm);
  return sqrtf(norm);
}

void f32_f32_add(float32_t *v1, float32_t *v2, uint32_t N, float32_t *dst)
{
  uint32_t i;
  for (i = 0; i < N; i++)
  {
    dst[i] = v1[i] + v2[i];
  }
}

void f32_f32_sub(float32_t *v1, float32_t *v2, uint32_t N, float32_t *dst)
{
  uint32_t i;
  for (i = 0; i < N; i++)
  {
    dst[i] = v1[i] - v2[i];
  }
}

void f32_f32_mult(float32_t *v1, float32_t *v2, uint32_t N, float32_t *dst)
{
  for (uint32_t i = 0; i < N; i++)
  {
    dst[i] = v1[i] * v2[i];
  }
}

void f32_f32_dot(float32_t *v1, float32_t *v2, uint32_t N, float32_t *dst)
{
  *dst = 0;
  uint32_t i;
  for (i = 0; i < N; i++)
  {
    *dst += v1[i] * v2[i];
  }
}

void f32_scale(float32_t *vec, float32_t scale, uint32_t N, float32_t *dst)
{
  for (uint32_t i = 0; i < N; i++)
  {
    dst[i] = vec[i] * scale;
  }
}

void f32_offset(float32_t *vec, float32_t offset, uint32_t N, float32_t *dst)
{
  for (uint32_t i = 0; i < N; i++)
  {
    dst[i] = vec[i] + offset;
  }
}

float32_t f32_l2norm(float32_t* vec, uint32_t N)
{
  float32_t norm;
  f32_f32_dot(vec, vec, N, &norm);
  return sqrtf(norm);
}


blas_status mv_mult_f32(const Mat_f32* mat, const Vec_f32* vec, Vec_f32* dst)
{
  if (dst->n != mat->m || dst->n != vec->n) return 0;

  Vec_f32 row;
  uint32_t cols = mat->n;
  uint32_t rows = mat->m;
  row.n = cols;
  for (uint32_t r = 0; r < rows; r++)
  {
    row.data = &mat->data[r*cols];
    vv_dot_f32(&row, vec, &dst->data[r]); 
  }
  return 1;
}

blas_status mm_mult_f32(const Mat_f32 *A, const Mat_f32 *B, Mat_f32 *C)
{
  // A is m x n, B is n x p, C is m x p
  uint32_t i,j,k;
  uint32_t m,n,p;

  if (A->n != B->m) return 0;
  if (C->m != A->m && C->n != B->n) return 0;

  m = A->m;
  n = A->n;
  p = B->n;

  for (i = 0; i < m; i++) {
    float32_t* c_row_p = C->data + i * p;
    for (k = 0; k < n; k++) {
      float32_t a_val = A->data[i * n + k];
      float32_t * b_row_p = B->data + k * p;
      for (j = 0; j < p; j++) {
        c_row_p[j] += a_val * b_row_p[j];
        //C->data[i*p+j] += A->data[i*n+k] * B->data[k*p+j];
      }
    }
  }
  return 1;

}

blas_status m_scale_f32(Mat_f32 *A, float32_t scale, Mat_f32 *B)
{
  if (A->m != B->m || A->n != B->n) return 0;

  uint32_t i;
  uint32_t rows = A->m;
  uint32_t cols = A->n;
  for (i = 0; i < rows*cols; i++)
  {
    A->data[i] *= scale;
  }
  return 1;
}

blas_status mm_add_f32(Mat_f32 *mat1, Mat_f32 *mat2, Mat_f32 *dst)
{
  if (mat1->m != mat2->m || mat1->n != mat2->n || mat1->m != dst->m || mat1->n != dst->n) return 0;
  uint32_t i;
  uint32_t rows = mat1->m;
  uint32_t cols = mat1->n;
  for (i = 0; i < rows*cols; i++)
  {
    dst->data[i] *= mat1->data[i] + mat2->data[i];
  }
  return 1;
}

blas_status mm_sub_f32(Mat_f32 *mat1, Mat_f32 *mat2, Mat_f32 *dst)
{
  if (mat1->m != mat2->m || mat1->n != mat2->n || mat1->m != dst->m || mat1->n != dst->n) return 0;
  uint32_t i;
  uint32_t rows = mat1->m;
  uint32_t cols = mat1->n;
  for (i = 0; i < rows*cols; i++)
  {
    dst->data[i] = mat1->data[i] - mat2->data[i];
  }
  return 1;
}
