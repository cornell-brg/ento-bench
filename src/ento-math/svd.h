#include <Eigen/Dense>

#include "ento-math/core.h"
#include "ento-util/debug.h"

//@TODO: Need to figure out if we need a more extensive
// interface for svd
namespace EntoMath
{

constexpr float eps = 1e-6;

//template <typename Scalar, int MaxM, int MaxN, int Order=0>
//void osj_svd_bounded(BoundedMatrix<Scalar, MaxM, MaxN>& A,
                     //BoundedMatrix<Scalar, MaxN, MaxN>& V,
                     //BoundedColVector<Scalar, MaxN>& min_v);

template <typename Scalar, int MaxM, int MaxN, int Order=0>
void osj_svd_bounded(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxN>& A,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxN, MaxN>& V,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxN, 1>& min_v);

template <typename Scalar, int M, int N, int Order=0>
void osj_svd(Eigen::Matrix<Scalar, M, N, Order, M, N>& A,
             Eigen::Matrix<Scalar, N, N, Order, N, N>& V,
             Eigen::Matrix<Scalar, N, 1, Order, N, 1>& min_v);

template <typename Derived>
void osj_svd_generic(Eigen::DenseBase<Derived>& A,
                     Eigen::DenseBase<Derived>& V,
                     Eigen::DenseBase<Derived>& min_v);


template <typename Scalar, int MaxM, int MaxN, int Order=0>
void gr_svd(BoundedMatrix<Scalar, MaxM, MaxN>& A,
            BoundedColVector<Scalar, MaxN>& min_v);

// Implementations
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
  static Scalar work_ab[N];
  static Scalar work_gamma[NUM_PAIRS];
  static uint8_t rotated[N];

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
        if (fabs(gamma) >= eps * off_diag)
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
        if (fabs(gamma) >= eps * off_diag)
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



template <typename Derived>
void osj_svd_generic(Eigen::DenseBase<Derived>& A,
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
        if (fabs(gamma) >= eps * off_diag)
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
