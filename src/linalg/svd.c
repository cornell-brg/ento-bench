#include "svd.h"
#include "blas.h"
#include "linalg/core.h"
#include <stdint.h>

float sign (const float num){
  if (num >= 0)
    return 1.0f;
  else
		return -1.0f;
}

uint32_t svd_osj_transpose_f32(Mat_f32 *A, Vec_f32* min_vec)
{
  static const float eps = 1e-6;
  float32_t alpha = 0, beta = 0, gamma = 0, tmp = 0, t = 0, s = 0, c = 0;
  uint8_t exitFlag = 0;
  uint32_t N = A->n;
  uint32_t M = A->m;
  float32_t offA = 0;

  Vec_f32 ai = { 9, A->data };
  Vec_f32 aj = { 9, A->data };
  uint32_t iterations = 0;
  while (!exitFlag)
  {
    exitFlag = 1;
    iterations++;
    for (int j = M - 1; j >= 1; --j)
    {
      for (int i = j - 1; i >= 0; --i)
      {
        ai.data = A->data + N * i;
        aj.data = A->data + N * j;
        vv_dot_f32(&ai, &ai, &alpha);
        vv_dot_f32(&aj, &aj, &beta);
        vv_dot_f32(&ai, &aj, &gamma);
        offA = sqrtf(alpha * beta);

        if (fabsf(gamma) >= eps * offA)
        {
          exitFlag = 0;
          tmp = (beta - alpha) / (2 * gamma);
          t = sign(tmp) / (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          c = expf(-0.5f * log1pf(t*t));
          s = c * t;

          float32_t* aid = ai.data;
          float32_t* ajd = aj.data;
          for (uint32_t k = 0; k < N; ++k)
          {
            tmp = *aid;
            *aid = c * tmp - s * *(ajd); ++aid;
            *ajd = s * tmp + c * *ajd; ++ajd;
          }
        }
      }
    }
  }

  for (uint8_t i=0; i < M; i++)
  {
    aj.data = A->data + N * i;
    v_normalize_f32(&aj);
  }

  // Loop over rows, extracting the singular value from the norm of the row
  // and then using it to normalize the rows and acquire the right singular vectors.
  float32_t ortho_vec[9] = {0}; ortho_vec[0] = 1;
  float32_t projection[9] = {0};
  float32_t accum_projection[9] = {0};
  
  Vec_f32 a9 = {9, ortho_vec};
  Vec_f32 vproj = {9, projection};
  Vec_f32 vaccum = {9, accum_projection};
  for (int i = 0; i < 8; i++)
  {
    ai.data = A->data + 9 * i;
    vv_proj_f32(&ai, &a9, &vproj);
    vv_add_f32(&vaccum, &vproj, &vaccum);
  }
  vv_sub_f32(&a9, &vaccum, min_vec);
  v_normalize_f32(min_vec);
  return iterations;
}

// What do we do if the input has M < N?
// We have a driver function outside of this that will handle setting up the input matrix.
// We want to keep column major input, therefore we keep this agnostic to the fact of
// finding the SVD of transpose and then fixing the right singular vectors.
// Assume MatrixFloat input V is already an identity matrix to calculate 
// the right singular vectors.
uint32_t svd_osj_f32(Mat_f32* A, Mat_f32* V, Vec_f32* min_vec)
{
  static const float eps = 1e-6;
  float32_t alpha = 0, beta = 0, gamma = 0, tmp = 0, t = 0, s = 0, c = 0;
  float minSingValSq = 10000;
  uint8_t minSingValIdx = 0;
  uint8_t exitFlag = 0;
  uint32_t M = A->m;
  uint32_t N = A->n;
  Vec_f32 ai = {N, A->data};
  Vec_f32 aj = {N, A->data};
  Vec_f32 vi = {V->m, V->data};
  Vec_f32 vj = {V->m, V->data};
  float32_t offA = 0;
  uint32_t iterations = 0;
  while (!exitFlag)
  {
    exitFlag = 1;
    iterations++;
    for (int j = M - 1; j >= 1; --j)
    {
      for (int i = j - 1; i >= 0; --i)
      {
        
        ai.data = A->data + N * i; aj.data = A->data + N * j;
        vv_dot_f32(&ai, &ai, &alpha);
        vv_dot_f32(&aj, &aj, &beta);
        vv_dot_f32(&ai, &aj, &gamma);
        offA = sqrtf(alpha * beta);

        if (fabsf(gamma) >= eps * offA)
        {
          exitFlag = 0;
          tmp = (beta - alpha) / (2 * gamma);
          t = sign(tmp) / (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          c = expf(-0.5f * log1pf(t*t));
          s = c * t;
          //ai.data = A->data + M * i; aj.data = A->data + M * j;
          vi.data = V->data + V->n * i; vj.data = V->data + V->n * j;

          //givens_rhs_f32(&ai, &aj, c, s);
          //givens_rhs_f32(&vj, &vj, c, s);
          float32_t *aid, *ajd, *vid, *vjd;
          aid = ai.data; ajd = aj.data; vid = vi.data; vjd = vj.data;
          
          for (uint32_t k = 0; k < N; ++k)
          {
            tmp = *aid;
            *aid = c * tmp - s * *ajd; ++aid;
            *ajd = s * tmp + c * *ajd; ++ajd;
            if (k < V->n)
            {
              tmp = *vid;
              *vid = c * tmp - s * *vjd; ++vid;
              *vjd = s * tmp + c * *vjd; ++vjd;
            }
          }
        }
      }
    }
  }
  for (uint8_t i=0; i < M; i++)
  {
    ai.data = A->data + N * i;
    vv_dot_f32(&ai, &ai, &tmp);
    if (tmp < minSingValSq) {
        minSingValSq = tmp;
        minSingValIdx = i;
    }
  }
  for (uint32_t i=0; i < min_vec->n; i++) min_vec->data[i] = V->data[i + V->n * minSingValIdx];
  return iterations;
}
