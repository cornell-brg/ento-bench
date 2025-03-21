#include <arm_math.h> 
#include <Eigen/Dense>
#include <cstdint>

// Naïve matrix multiplication (contiguous, row-major)
// Computes C = A * B where A is MxN, B is N×P and C is M×P.
template<typename T, int M, int N, int P>
struct NaiveMatMulKernel
{
  using T_ = T;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const T A[M * N], const T B[N * P], T C[M * P])
  {
    for (int i = 0; i < M; i++)
    {
      for (int j = 0; j < P; j++)
      {
        T sum = 0;
        for (int k = 0; k < N; k++)
        {
          sum += A[i * N + k] * B[k * P + j];
        }
        C[i * P + j] = sum;
      }
    }
  }
};

// Symmetric-aware multiplication for computing the Gram matrix (A^T * A)
// Here, A is MxN (row-major) and C is an N×N symmetric matrix.
template<typename T, int M, int N>
struct GramMatMulKernel
{
  using T_ = T;
  static constexpr size_t ScratchSize_ = 0;

  void operator()(const T A[M * N], T C[N * N])
  {
    // Initialize output to zero.
    for (int i = 0; i < N; i++)
    {
      for (int j = 0; j < N; j++)
      {
        C[i * N + j] = 0;
      }
    }
    
    // Compute only the upper-triangular part, then mirror.
    for (int i = 0; i < N; i++)
    {
      for (int j = i; j < N; j++)
      {
        T sum = 0;
        for (int k = 0; k < M; k++)
        {
          sum += A[k * N + i] * A[k * N + j];
        }
        C[i * N + j] = sum;
        if (i != j)
        {
          C[j * N + i] = sum;
        }
      }
    }
  }

};

// Eigen-based multiplication with compile-time dimensions (static)
// Data is mapped as row-major.
template<typename T, int M, int N, int P, int Order=0>
struct EigenStaticMatMulKernel
{
  using T_ = T;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const Eigen::Matrix<T, M, N, Order>& A,
                  const Eigen::Matrix<T, N, P, Order>& B,
                        Eigen::Matrix<T, M, P, Order>& C)
  {
    C.noalias() = A * B;
  }
};

template <typename T, int M, int N, int Order=0>
struct EigenStaticGramMatMulKernel
{
  using T_ = T;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const Eigen::Matrix<T, M, N, Order>& A,
                        Eigen::Matrix<T, N, N, Order>& B)
  {
    B.noalias() = A.transpose() * A;
  }
};

// Eigen-based multiplication with runtime dimensions (dynamic)
// The input matrices are Eigen matrices; no extra copying is done.
template<typename T, int MaxM, int MaxN, int MaxP, int Order=0>
struct EigenDynamicMatMulKernel
{
  using T_ = T;
  static constexpr size_t ScratchSize_ = 0;

  void operator()(
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxN>& A,
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Order, MaxN, MaxP>& B,
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxP>& C)
  {
    C.noalias() = A * B;
  }
};

template <typename T, int MaxM, int MaxN, int Order=0>
struct EigenDynamicGramMatMulKernel
{
  using T_ = T;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxN>& A,
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Order, MaxN, MaxN>& B)
  {
    B.noalias() = A.transpose() * A;
  }
};

template<typename DerivedA, typename DerivedB, typename DerivedC>
struct EigenBaseMatMulKernel
{
  using T_ = DerivedA::Scalar;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const Eigen::MatrixBase<DerivedA>&      A,
                  const Eigen::MatrixBase<DerivedB>&      B,
                        Eigen::PlainObjectBase<DerivedC>& C)
  {
    C.noalias() = A * B;
  }
};

// Eigen-based symmetric multiplication for Gram matrix computation.
// Computes c = a^T * a.
template<typename DerivedA, typename DerivedB>
struct EigenBaseGramMatMulKernel
{
  using T_ = DerivedA::Scalar;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const Eigen::MatrixBase<DerivedA>&       A,
                        Eigen::PlainObjectBase<DerivedB>&  B)
  {
    B.noalias() = A.transpose() * A;
  }
};

// Inline assembly based matrix multiplication for int32_t using MLA.
// Operands are accessed as contiguous row-major arrays.
template<int M, int N, int P>
struct MLAMatMulKernel
{
  void operator()(const int32_t A[M * N], const int32_t B[N * P], int32_t C[M * P])
  {
    for (int i = 0; i < M; i++)
    {
      for (int j = 0; j < P; j++)
      {
        int32_t sum = 0;
        for (int k = 0; k < N; k++)
        {
          int32_t a_val = A[i * N + k];
          int32_t b_val = B[k * P + j];
          asm volatile
          (
            "mla %0, %1, %2, %0\n"
            : "+r" (sum)
            : "r" (a_val), "r" (b_val)
          );
        }
        C[i * P + j] = sum;
      }
    }
  }
};

// Inline assembly using SMLAD for int16_t operands with accumulation into int32_t.
// Assumes N is even.
template<int M, int N, int P>
struct SMLADMatMulKernel
{
  using T_ = int16_t;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const int16_t A[M * N], const int16_t B[N * P], int32_t C[M * P])
  {
    static_assert(N % 2 == 0, "N must be even for SMLAD-based multiplication.");
    for (int i = 0; i < M; i++)
    {
      for (int j = 0; j < P; j++)
      {
        int32_t sum = 0;
        for (int k = 0; k < N; k += 2)
        {
          uint32_t a_val = *reinterpret_cast<const uint32_t*>(&A[i * N + k]);
          // Pack two int16_t values from B: elements (k, j) and (k+1, j).
          uint16_t b0 = B[k * P + j];
          uint16_t b1 = B[(k + 1) * P + j];
          uint32_t b_val = (static_cast<uint32_t>(b1) << 16) | b0;
          asm volatile
          (
            "smlad %0, %1, %2, %0\n"
            : "+r" (sum)
            : "r" (a_val), "r" (b_val)
          );
        }
        C[i * P + j] = sum;
      }
    }
  }
};

// Matrix multiplication with software-managed buffering for one column of B.
// Operates on contiguous row-major arrays.
template<typename T, int M, int N, int P>
struct BufferedMatMulKernel
{
  using T_ = T;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const T A[M * N], const T B[N * P], T C[M * P])
  {
    T col_buffer[N];
    for (int j = 0; j < P; j++)
    {
      for (int k = 0; k < N; k++)
      {
        col_buffer[k] = B[k * P + j];
      }
      for (int i = 0; i < M; i++)
      {
        T sum = 0;
        for (int k = 0; k < N; k++)
        {
          sum += A[i * N + k] * col_buffer[k];
        }
        C[i * P + j] = sum;
      }
    }
  }
};

// SIMD-optimized multiplication for int16_t using the CMSIS __SMLAD intrinsic.
// Operates on contiguous row-major arrays. Assumes N is even.
template<int M, int N, int P>
struct SMLADIntrinsicMatMulKernel 
{
  using T_ = int16_t;
  static constexpr size_t ScratchSize_ = 0;
  void operator()(const int16_t A[M * N], const int16_t B[N * P], int32_t C[M * P])
  {
    static_assert(N % 2 == 0, "N must be even for SMLAD-based multiplication.");
    for (int i = 0; i < M; i++)
    {
      for (int j = 0; j < P; j++)
      {
        int32_t sum = 0;
        for (int k = 0; k < N; k += 2)
        {
          uint32_t a_val = *reinterpret_cast<const uint32_t*>(&A[i * N + k]);
          uint16_t b0 = B[k * P + j];
          uint16_t b1 = B[(k + 1) * P + j];
          uint32_t b_val = (static_cast<uint32_t>(b1) << 16) | b0;
          sum = __SMLAD(a_val, b_val, sum);
        }
        C[i * P + j] = sum;
      }
    }
  }
};

//------------------------------------------------------------------------------
// Floating-Point (f32) Version
//------------------------------------------------------------------------------
struct CmsisF32MatMulKernel
{
  using T_ = int16_t;
  static constexpr size_t ScratchSize_ = 0;
  arm_status operator()(const float *A, const float *B, float *C,
                        uint16_t m, uint16_t n, uint16_t p)
  {
    arm_matrix_instance_f32 matA;
    arm_matrix_instance_f32 matB;
    arm_matrix_instance_f32 matC;
    
    matA.numRows = m;
    matA.numCols = n;
    matA.pData = const_cast<float*>(A);
    
    matB.numRows = n;
    matB.numCols = p;
    matB.pData = const_cast<float*>(B);
    
    matC.numRows = m;
    matC.numCols = p;
    matC.pData = C;
    
    return arm_mat_mult_f32(&matA, &matB, &matC);
  }
};

//------------------------------------------------------------------------------
// Q15 Versions
//------------------------------------------------------------------------------

// Regular Q15 matrix multiplication.
// pState must point to a state buffer of the appropriate size.
// We expose tempalte parameters for the size of C. They are
// only used to setup the size of the scratch space the CMSIS-DSP
// kernels needs.
template <int M, int N>
struct CmsisQ15MatMulKernel
{
  using T_ = q15_t;
  static constexpr size_t ScratchSize_ = 10;
  arm_status operator()(const q15_t *A, const q15_t *B, q15_t *C,
                        uint16_t m, uint16_t n, uint16_t p,
                        q15_t *pState)
  {
    arm_matrix_instance_q15 matA;
    arm_matrix_instance_q15 matB;
    arm_matrix_instance_q15 matC;
    
    matA.numRows = m;
    matA.numCols = n;
    matA.pData = const_cast<q15_t*>(A);
    
    matB.numRows = n;
    matB.numCols = p;
    matB.pData = const_cast<q15_t*>(B);
    
    matC.numRows = m;
    matC.numCols = p;
    matC.pData = C;
    
    return arm_mat_mult_q15(&matA, &matB, &matC, pState);
  }
};

// Fast Q15 matrix multiplication.
// pState must point to a state buffer of the appropriate size.
struct CmsisQ15FastMatMulKernel
{
  arm_status operator(const q15_t *A, const q15_t *B, q15_t *C,
                      uint16_t m, uint16_t n, uint16_t p,
                      q15_t *pState)
  {
    arm_matrix_instance_q15 matA;
    arm_matrix_instance_q15 matB;
    arm_matrix_instance_q15 matC;
    
    matA.numRows = m;
    matA.numCols = n;
    matA.pData = const_cast<q15_t*>(A);
    
    matB.numRows = n;
    matB.numCols = p;
    matB.pData = const_cast<q15_t*>(B);
    
    matC.numRows = m;
    matC.numCols = p;
    matC.pData = C;
    
    return arm_mat_mult_fast_q15(&matA, &matB, &matC, pState);
  }
};

//------------------------------------------------------------------------------
// Q31 Versions
//------------------------------------------------------------------------------

// Regular Q31 matrix multiplication.
arm_status mat_mult_cmsis_q31(const q31_t *A, const q31_t *B, q31_t *C,
                              uint16_t m, uint16_t n, uint16_t p)
{
  arm_matrix_instance_q31 matA;
  arm_matrix_instance_q31 matB;
  arm_matrix_instance_q31 matC;
  
  matA.numRows = m;
  matA.numCols = n;
  matA.pData = const_cast<q31_t*>(A);
  
  matB.numRows = n;
  matB.numCols = p;
  matB.pData = const_cast<q31_t*>(B);
  
  matC.numRows = m;
  matC.numCols = p;
  matC.pData = C;
  
  return arm_mat_mult_q31(&matA, &matB, &matC);
}

// Fast Q31 matrix multiplication.
// Note: The fast variant for Q31 does not require a state buffer.
arm_status mat_mult_cmsis_fast_q31(const q31_t *A, const q31_t *B, q31_t *C,
                                   uint16_t m, uint16_t n, uint16_t p)
{
  arm_matrix_instance_q31 matA;
  arm_matrix_instance_q31 matB;
  arm_matrix_instance_q31 matC;
  
  matA.numRows = m;
  matA.numCols = n;
  matA.pData = const_cast<q31_t*>(A);
  
  matB.numRows = n;
  matB.numCols = p;
  matB.pData = const_cast<q31_t*>(B);
  
  matC.numRows = m;
  matC.numCols = p;
  matC.pData = C;
  
  return arm_mat_mult_fast_q31(&matA, &matB, &matC);
}

//------------------------------------------------------------------------------
// Q7 Version
//------------------------------------------------------------------------------

// Q7 matrix multiplication.
// pState must point to a state buffer of the appropriate size.
arm_status mat_mult_cmsis_q7(const q7_t *A, const q7_t *B, q7_t *C,
                             uint16_t m, uint16_t n, uint16_t p,
                             q7_t *pState)
{
  arm_matrix_instance_q7 matA;
  arm_matrix_instance_q7 matB;
  arm_matrix_instance_q7 matC;
  
  matA.numRows = m;
  matA.numCols = n;
  matA.pData = const_cast<q7_t*>(A);
  
  matB.numRows = n;
  matB.numCols = p;
  matB.pData = const_cast<q7_t*>(B);
  
  matC.numRows = m;
  matC.numCols = p;
  matC.pData = C;
  
  return arm_mat_mult_q7(&matA, &matB, &matC, pState);
}

#ifdef CM7
// Matrix multiplication with explicit prefetching using __builtin_prefetch.
// Operates on contiguous row-major arrays.
template<typename T, int M, int N, int P>
void mat_mul_prefetch_manual(const T A[M * N], const T B[N * P], T C[M * P])
{
  for (int i = 0; i < M; i++)
  {
    for (int j = 0; j < P; j++)
    {
      T sum = 0;
      for (int k = 0; k < N; k++)
      {
        if (k + 1 < N)
        {
          __builtin_prefetch(&A[i * N + k + 1]);
          __builtin_prefetch(&B[(k + 1) * P + j]);
        }
        sum += A[i * N + k] * B[k * P + j];
      }
      C[i * P + j] = sum;
    }
  }
}



// Matrix multiplication with explicit prefetch instructions using inline assembly.
// Operates on contiguous row-major arrays.
template<typename T, int M, int N, int P>
void mat_mul_prefetch_instr(const T A[M * N], const T B[N * P], T C[M * P])
{
  for (int i = 0; i < M; i++)
  {
    for (int j = 0; j < P; j++)
    {
      T sum = 0;
      for (int k = 0; k < N; k++)
      {
        asm volatile
        (
          "pld [%0]\n"
          :
          : "r" (&A[i * N + k])
        );
        asm volatile
        (
          "pld [%0]\n"
          :
          : "r" (&B[k * P + j])
        );
        sum += A[i * N + k] * B[k * P + j];
      }
      C[i * P + j] = sum;
    }
  }
}
#endif
