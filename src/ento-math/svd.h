#ifndef SVD_H
#define SVD_H

#include <Eigen/Dense>

#include <cmath>
#include "ento-math/core.h"
#include "ento-util/debug.h"

//@TODO: Need to figure out if we need a more extensive
// interface for svd
namespace EntoMath
{

template <typename Scalar, int MaxM, int MaxN, int Order=0>
void osj_svd_bounded(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxN>& A,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxN, MaxN>& V,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxN, 1>& min_v);

template <typename Scalar, int M, int N, int Order>
void osj_svd(Eigen::Matrix<Scalar, M, N, Order, M, N>& A,
             Eigen::Matrix<Scalar, N, N, Order, N, N>& V,
             Eigen::Matrix<Scalar, N, 1, Order, N, 1>& min_v);

template <typename Derived>
void osj_svd(Eigen::DenseBase<Derived>& A,
                     Eigen::DenseBase<Derived>& V,
                     Eigen::DenseBase<Derived>& min_v);


template <typename Scalar, int M, int N, int Order=0>
void osj_svd_c(Eigen::Matrix<Scalar, M, N, Order, M, N>& A,
               Eigen::Matrix<Scalar, N, N, Order, N, N>& V,
               Eigen::Matrix<Scalar, N, 1, Order, N, 1>& min_v);


template <typename Scalar, int MaxM, int MaxN, int Order=0>
void gr_svd(BoundedMatrix<Scalar, MaxM, MaxN>& A,
            BoundedColVector<Scalar, MaxN>& min_v);

// ===================================================================
// Implementations

static float __attribute__ ((noinline)) custom_dot_product_f32(const float *A, const float *B, int32_t blockSize) {
  uint32_t blkCnt;
  float sum = 0.0f;

  const float* pSrcA = A;
  const float* pSrcB = B;
  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = blockSize >> 2U;

  while (blkCnt > 0U) {
    /* Perform 4 multiplications and accumulate the results */
    sum += (*pSrcA++) * (*pSrcB++);
    sum += (*pSrcA++) * (*pSrcB++);
    sum += (*pSrcA++) * (*pSrcB++);
    sum += (*pSrcA++) * (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Process remaining elements */
  blkCnt = blockSize & 0x3U;

  while (blkCnt > 0U) {
    /* Perform a multiplication and accumulate the result */
    sum += (*pSrcA++) * (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

  return sum;
}

template <typename Scalar, int M, int N, int Order>
void osj_svd_c(Eigen::Matrix<Scalar, M, N, Order, M, N>& A,
               Eigen::Matrix<Scalar, N, N, Order, N, N>& V,
               Eigen::Matrix<Scalar, N, 1, Order, N, 1>& min_v)
{
  bool exit = false;
  int iterations = 0;
  int min_idx = 0;

  Scalar alpha = 0;
  Scalar beta = 0;
  Scalar gamma = 0;
  Scalar off_diag = 0;
  Scalar tmp = 0;
  Scalar t = 0;
  Scalar s = 0;
  Scalar c = 0;
  Scalar alpha_sum = 0; 
  Scalar beta_sum = 0;
  Scalar gamma_sum = 0;

  // @TODO: Add a allocator? Or caller is in charge of allocating this?
  constexpr int NUM_PAIRS = (N * (N-1)) / 2;
  static Scalar work_ab[N];
  static Scalar work_gamma[NUM_PAIRS];
  static uint8_t rotated[N] = {0};

  Scalar* aj, *ai, *vi, *vj;
  for (int i = 0; i < N; i++)
  {
    aj = A.data() + M * i;
    work_ab[i] = custom_dot_product_f32(aj, aj, M);
    DPRINTF("Work_ab[%i] = %f\n", i, work_ab[i]); 
  }

  int gamma_idx = 0;
  DPRINTF("Setting up work_gamma!\n");
  for (int j = 1; j < N; ++j)
  {
	  aj = A.data() + M * j;
	  for (int i = 0; i < j; ++i)
    {
		  ai = A.data() + M * i;
		  work_gamma[gamma_idx++] = custom_dot_product_f32(ai, aj, M);
    }
  }
  
  constexpr int max_iters = 100;
  DPRINTF("Entering sweeps loop!\n");
  while (!exit)
  {
    exit = true;
    iterations++;
    if (iterations > max_iters)
    {
      //printf("reached max iters!\n");
      break;
    }
    //DPRINTF("Iterations: %i", iterations);
    gamma_idx = 2;
    for (int j = N - 1; j>=1; --j)
    {
      for (int i = j - 1; i>=0; --i)
      {
        //Scalar* ai = A.row(i).data();
        //Scalar* aj = A.row(j).data();
        //DPRINTF("New sweep for cols i,j = %i, %i\n", i, j);
        alpha = work_ab[i];
        beta = work_ab[j];

        ai = A.data() + M * i;
        aj = A.data() + M * j;
        if (rotated[i] || rotated[j])
        {
          //DPRINTF("Recomputing gamma!\n");
          work_gamma[gamma_idx] = custom_dot_product_f32(ai, aj, M);
        }
        gamma = work_gamma[gamma_idx];

        off_diag = sqrtf(alpha * beta);
        //DPRINTF("alpha, beta: %.30f, %.30f\n", alpha, beta);
        //DPRINTF("alpha * beta: %.30f\n", alpha * beta);
        //DPRINTF("off_diag: %.30f\n", off_diag);
        //DPRINTF("gamma: %.30f, eps*off_diag: %.30f\n", gamma, eps*off_diag);
        if (fabsf(gamma) >= EntoMath::ENTO_EPS * off_diag)
        {
          //DPRINTF("Rotating!\n");
          exit = 0;

          // Givens rotation calculations
          //tmp = (beta - alpha) / (2 * gamma);
          //t = sign(tmp) / (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          //c = 1.0f / sqrtf(1 + t * t);
          //s = c * t;

          tmp = (beta - alpha) / (2 * gamma);
          t = sign(tmp)/ (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          c = expf(-0.5 * std::log1pf(t*t));
          s = c * t;

          //DPRINTF("C, S, T: %.10f, %.10f, %.10f\n", c, s, t);
          //DPRINTF("tmp: %.10f\n", tmp);

          // Apply Givens rotations to both A and V
          ai = A.data() + M * i; aj = A.data() + M * j;
          vi = V.data() + N * i; vj = V.data() + N * j;

          work_ab[i] = 0; work_ab[j] = 0;
          work_gamma[gamma_idx] = 0;
          alpha_sum = 0.0; beta_sum = 0.0; gamma_sum = 0.0;
          for (int k = 0; k < M; ++k)
          {
            tmp = *ai;

            // Compute new Ai's
            *ai = c * tmp - s * *aj;
            alpha_sum += *ai * *ai;

            // Compute new Aj's
            *aj = s * tmp + c * *aj;
            beta_sum += *aj * *aj;

            gamma_sum += *ai * *aj;

            ++ai;
            ++aj;

            if (k < N)
            {
              tmp = *vi;
              *vi = c * tmp - s * *vj; ++vi;
              *vj = s * tmp + c * *vj; ++vj;
            }
          }

          // Update the precomputed values
          //DPRINTF("Alpha sum for work_ab[%i]: %.10f\n", i, alpha_sum);
          //DPRINTF("Beta sum for work_ab[%i]: %.10f\n", j, beta_sum);
          work_ab[i] = alpha_sum;
          work_ab[j] = beta_sum;
          work_gamma[gamma_idx] = gamma_sum;
          //DPRINTF("work_gamma[%i] = %.10f\n", gamma_idx, gamma_sum);
          rotated[i] = 1;
          rotated[j] = 1;
          //DPRINTF("Alpha sum for work_ab[%i]: %.10f\n", i, work_ab[i]);
          //DPRINTF("Beta sum for work_ab[%i]: %.10f\n", j, work_ab[j]);
        }
        else
        {
          // Update rotated flags and track minimum singular value index
          rotated[i] = 0;
          rotated[j] = 0;
          if (work_ab[i] < work_ab[min_idx])
          {
            min_idx = i;
          }
          else if (work_ab[j] < work_ab[min_idx])
          {
            min_idx = j;
          }
        }
        //DPRINTF("Moving onto next sweep!\n\n");
        --gamma_idx;  // Move to the next pair in work_gamma
      }
    }
  }
  DPRINTF("Exited out of sweeps loop!\n");
  min_v = V.row(min_idx);
  return;

}

template <typename Scalar, int M, int N, int Order>
void osj_svd(Eigen::Matrix<Scalar, M, N, Order, M, N>& A,
             Eigen::Matrix<Scalar, N, N, Order, N, N>& V,
             Eigen::Matrix<Scalar, N, 1, Order, N, 1>& min_v)
{
  bool exit = false;
  int iterations = 0;
  int min_idx = 0;

  Scalar alpha = 0;
  Scalar beta = 0;
  Scalar gamma = 0;
  Scalar off_diag = 0;
  Scalar tmp = 0;
  Scalar t = 0;
  Scalar s = 0;
  Scalar c = 0;

  // @TODO: Add a allocator? Or caller is in charge of allocating this?
  constexpr int NUM_PAIRS = (N * (N-1)) / 2;
  DPRINTF("NUM_PAIRS: %i\n", NUM_PAIRS);
  static Scalar work_ab[N];
  static Scalar work_gamma[NUM_PAIRS];
  static uint8_t rotated[N] = {0};

  DPRINTF("Setting up work_ab!\n");
  for (int i = 0; i < N; i++)
  {
    work_ab[i] = A.col(i).dot(A.col(i));
    DPRINTF("Work_ab[%i] = %f\n", i, work_ab[i]); 
  }

  int gamma_idx = 0;
  DPRINTF("Setting up work_gamma!\n");
  for (int j = 1; j < N; ++j)
  {
    for (int i = 0; i < j; ++i)
    {
      work_gamma[gamma_idx++] = A.col(i).dot(A.col(j));
    }
  }
  
  DPRINTF("Entering sweeps loop!\n");
  while (!exit)
  {
    exit = true;
    iterations++;
    //DPRINTF("Iterations: %i", iterations);
    gamma_idx = NUM_PAIRS - 1;
    for (int j = N - 1; j>=1; --j)
    {
      for (int i = j - 1; i>=0; --i)
      {
        //Scalar* ai = A.row(i).data();
        //Scalar* aj = A.row(j).data();
        alpha = work_ab[i];
        beta = work_ab[j];

        if (rotated[i] || rotated[j])
        {
          //DPRINTF("Recomputing gamma!\n");
          work_gamma[gamma_idx] = A.col(i).dot(A.col(j));
        }
        //work_gamma[gamma_idx] = A.col(i).dot(A.col(j));
        gamma = work_gamma[gamma_idx];

        off_diag = sqrtf(alpha * beta);
        //DPRINTF("alpha, beta: %.30f, %.30f\n", alpha, beta);
        //DPRINTF("alpha * beta: %.30f\n", alpha * beta);
        //DPRINTF("off_diag: %.30f\n", off_diag);
        //DPRINTF("gamma: %.30f, eps*off_diag: %.30f\n", gamma, EntoMath::ENTO_EPS*off_diag);
        if (fabsf(gamma) >= EntoMath::ENTO_EPS * off_diag)
        {
          //DPRINTF("Rotating!\n");
          exit = 0;

          // Givens rotation calculations
          tmp = (beta - alpha) / (2 * gamma);
          t = sign(tmp) / (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          c = 1.0f / sqrtf(1 + t * t);
          s = c * t;

          //tmp = (beta - alpha) / (2 * gamma);
          //t = sign(tmp)/ (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          //c = expf(-0.5 * std::log1pf(t*t));
          //s = c * t;

          // Apply Givens rotations to both A and V
          work_ab[i] = 0; work_ab[j] = 0; work_gamma[gamma_idx] = 0;
          Scalar alpha_sum = 0.0f, beta_sum = 0.0f, gamma_sum = 0.0f;
          for (int k = 0; k < M; k++)
          {
            tmp = A(k, i);  // Temporary copy of A(i, k)
            
            // Update A(i, k)
            A(k, i) = c * tmp - s * A(k, j);
            //DPRINTF("Updating A(%i, %i)=%f\n", k, i, A(k, i));
            alpha_sum += (A(k, i) * A(k, i));
            //DPRINTF("Alpha sum intermediary: %f\n", alpha_sum);
            
            // Update A(j, k)
            //DPRINTF("Updating A(%i, %i)=%f\n", k, j, A(k, j));
            A(k, j) = s * tmp + c * A(k, j);
            beta_sum += (A(k, j) * A(k, j));
            //DPRINTF("Beta sum intermediary: %f\n", beta_sum);
            
            // Update dot product
            //DPRINTF("Updating gamma_sum with i,j,k=(%d, %d, %d)\n", i, j, k);
            gamma_sum += (A(k, i) * A(k, j));

            // Apply rotations to V
            if (k < N)
            {
              //DPRINTF("Accumulating Rotation for idx k: %i \n", k);
              tmp = V(k, i);
              V(k, i) = c * tmp - s * V(k, j);
              V(k, j) = s * tmp + c * V(k, j);
              //DPRINTF("Updating V(k,i)=%f\n",V(k,i));
              //DPRINTF("Updating V(k,j)=%f\n",V(k,j));
            }
          }

          // Update the precomputed values
          work_ab[i] = alpha_sum;
          work_ab[j] = beta_sum;
          work_gamma[gamma_idx] = gamma_sum;
          rotated[i] = 1;
          rotated[j] = 1;
        }
        else
        {
          // Update rotated flags and track minimum singular value index
          rotated[i] = 0;
          rotated[j] = 0;
          if (work_ab[i] < work_ab[min_idx])
          {
            min_idx = i;
          }
          else if (work_ab[j] < work_ab[min_idx])
          {
            min_idx = j;
          }
        }
        --gamma_idx;  // Move to the next pair in work_gamma
      }
    }
  }
  ENTO_DEBUG_EIGEN_MATRIX(V);
  min_v = V.row(min_idx).eval();
  ENTO_DEBUG("Finished osj svd. Total iterations: %i", iterations);
  return;
}

template <typename Scalar, int MaxM, int MaxN, int Order>
void osj_svd_bounded(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxN>& A,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxN, MaxN>& V,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxN, 1>& min_v)
{

  const int M = A.rows();
  const int N = A.cols();

  bool exit = false;
  int iterations = 0;
  int min_idx = 0;

  Scalar alpha = 0;
  Scalar beta = 0;
  Scalar gamma = 0;
  Scalar off_diag = 0;
  Scalar tmp = 0;
  Scalar t = 0;
  Scalar s = 0;
  Scalar c = 0;

  // @TODO: Add a allocator? Or caller is in charge of allocating this?
  static Scalar work_ab[12];
  static Scalar work_gamma[66];
  static uint8_t rotated[12];

  for (int i = 0; i < N; i++)
  {
    work_ab[i] = A.col(i).squaredNorm();

  }

  int gamma_idx = 0;
  for (int j = 1; j < N; ++j)
  {
    for (int i = 0; i < j; ++i)
    {
      work_gamma[gamma_idx++] = A.col(i).dot(A.col(j));
    }
  }
  
  while (!exit)
  {
    exit = true;
    iterations++;
    for (int j = M - 1; j>=1; --j)
    {
      for (int i = j - 1; i>=0; --i)
      {
        //Scalar* ai = A.row(i).data();
        //Scalar* aj = A.row(j).data();

        alpha = work_ab[i];
        beta = work_ab[j];

        if (rotated[i] || rotated[j])
        {
          work_gamma[gamma_idx] = A.col(i).dot(A.col(j));
        }
        gamma = work_gamma[gamma_idx];

        off_diag = sqrtf(alpha * beta);
        if (fabs(gamma) >= EntoMath::ENTO_EPS * off_diag)
        {
          exit = 0;

          // Givens rotation calculations
          tmp = (beta - alpha) / (2 * gamma);
          t = copysignf(1.0f, tmp) / (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          c = 1.0f / sqrtf(1 + t * t);
          s = c * t;

          // Apply Givens rotations to both A and V
          Scalar alpha_sum = 0, beta_sum = 0, gamma_sum = 0;
          for (int k = 0; k < M; ++k)
          {
            tmp = A(i, k);  // Temporary copy of A(i, k)
            
            // Update A(i, k)
            A(i, k) = c * tmp - s * A(j, k);
            alpha_sum += A(i, k) * A(i, k);
            
            // Update A(j, k)
            A(j, k) = s * tmp + c * A(j, k);
            beta_sum += A(j, k) * A(j, k);
            
            // Update dot product
            gamma_sum += A(i, k) * A(j, k);

            // Apply rotations to V
            if (k < V.rows())
            {
              tmp = V(i, k);
              V(i, k) = c * tmp - s * V(j, k);
              V(j, k) = s * tmp + c * V(j, k);
            }
          }

          // Update the precomputed values
          work_ab[i] = alpha_sum;
          work_ab[j] = beta_sum;
          work_gamma[gamma_idx] = gamma_sum;
          rotated[i] = 1;
          rotated[j] = 1;
        }
        else
        {
          // Update rotated flags and track minimum singular value index
          rotated[i] = 0;
          rotated[j] = 0;
          if (work_ab[i] < work_ab[min_idx])
          {
            min_idx = i;
          }
          else if (work_ab[j] < work_ab[min_idx])
          {
            min_idx = j;
          }
        }
        --gamma_idx;  // Move to the next pair in work_gamma
      }
    }
  }
  min_v = V.col(min_idx);

}



template <typename Derived>
void osj_svd(Eigen::DenseBase<Derived>& A,
                     Eigen::DenseBase<Derived>& V,
                     Eigen::DenseBase<Derived>& min_v)
{
  using Scalar = typename Derived::Scalar;

  const int M = A.rows();
  const int N = A.cols();

  int min_idx = 0;
  bool exit = false;
  int iterations = 0;

  Scalar alpha = 0;
  Scalar beta = 0;
  Scalar gamma = 0;
  Scalar off_diag = 0;
  Scalar tmp = 0;
  Scalar t = 0;
  Scalar s = 0;
  Scalar c = 0;

  // @TODO: Add a allocator? Or caller is in charge of allocating this?
  static Scalar work_ab[12];
  static Scalar work_gamma[66];
  static uint8_t rotated[12];

  for (int i = 0; i < N; i++)
  {
  }

  int gamma_idx = 0;
  for (int j = 1; j < N; ++j)
  {
    for (int i = 0; i < j; ++i)
    {
      work_gamma[gamma_idx++] = A.col(i).dot(A.col(j));
    }
  }
  
  while (!exit)
  {
    exit = true;
    iterations++;
    for (int j = M - 1; j>=1; --j)
    {
      for (int i = j - 1; i>=0; --i)
      {
        //Scalar* ai = A.row(i).data();
        //Scalar* aj = A.row(j).data();

        alpha = work_ab[i];
        beta = work_ab[j];

        if (rotated[i] || rotated[j])
        {
          work_gamma[gamma_idx] = A.col(i).dot(A.col(j));
        }
        gamma = work_gamma[gamma_idx];

        off_diag = sqrtf(alpha * beta);
        if (fabs(gamma) >= EntoMath::ENTO_EPS * off_diag)
        {
          exit = 0;

          // Givens rotation calculations
          tmp = (beta - alpha) / (2 * gamma);
          t = copysign(1.0f, tmp) / (fabsf(tmp) + sqrtf(1 + tmp * tmp));
          c = 1.0f / sqrtf(1 + t * t);
          s = c * t;

          // Apply Givens rotations to both A and V
          Scalar alpha_sum = 0, beta_sum = 0, gamma_sum = 0;
          for (int k = 0; k < M; ++k)
          {
            tmp = A(i, k);  // Temporary copy of A(i, k)
            
            // Update A(i, k)
            A(i, k) = c * tmp - s * A(j, k);
            alpha_sum += A(i, k) * A(i, k);
            
            // Update A(j, k)
            A(j, k) = s * tmp + c * A(j, k);
            beta_sum += A(j, k) * A(j, k);
            
            // Update dot product
            gamma_sum += A(i, k) * A(j, k);

            // Apply rotations to V
            if (k < V.rows())
            {
              tmp = V(i, k);
              V(i, k) = c * tmp - s * V(j, k);
              V(j, k) = s * tmp + c * V(j, k);
            }
          }

          // Update the precomputed values
          work_ab[i] = alpha_sum;
          work_ab[j] = beta_sum;
          work_gamma[gamma_idx] = gamma_sum;
          rotated[i] = 1;
          rotated[j] = 1;
        }
        else
        {
          // Update rotated flags and track minimum singular value index
          rotated[i] = 0;
          rotated[j] = 0;
          if (work_ab[i] < work_ab[min_idx])
          {
            min_idx = i;
          }
          else if (work_ab[j] < work_ab[min_idx])
          {
            min_idx = j;
          }
        }
        --gamma_idx;  // Move to the next pair in work_gamma
      }
    }
  }
  min_v = V.col(min_idx);
}




}

#endif // SVD_H
