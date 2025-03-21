#ifndef MAT_MUL_PROBLEM_H
#define MAT_MUL_PROBLEM_H

#include <sstream>
#ifdef NATIVE
#include <string>
#endif

#include <ento-util/containers.h>
#include <ento-bench/problem.h>
#include <Eigen/Dense>
#include <variant>

namespace EntoBench
{

// mat_mul_problem:
//   - Kernel: the kernel kernel.
//   - T: element type.
//   - M, N, P: fixed dimensions (A is M×N, B is N×P, C is M×P).
//   - UseEigen: if true, use Eigen matrices; if false, use EntoUtil::EntoContainer.
//   - RequiresScratch: if true, the kernel expects a state buffer (scratch_).
template <typename Kernel, typename T, size_t M, size_t N, size_t P,
          bool UseEigen = false, bool RequiresScratch = false>
class mat_mul_problem : public EntoProblem<mat_mul_problem<Kernel, T, M, N, P, UseEigen, RequiresScratch>>
{
private:
  Kernel kernel_;

  // Choose container types based on UseEigen.
  using container_a_type = std::conditional_t<UseEigen,
      Eigen::Matrix<T, M, N, Eigen::RowMajor>,
      EntoUtil::EntoContainer<T, M * N>>;
  using container_b_type = std::conditional_t<UseEigen,
      Eigen::Matrix<T, N, P, Eigen::RowMajor>,
      EntoUtil::EntoContainer<T, N * P>>;
  using container_c_type = std::conditional_t<UseEigen,
      Eigen::Matrix<T, M, P, Eigen::RowMajor>,
      EntoUtil::EntoContainer<T, M * P>>;

  container_a_type a_;
  container_b_type b_;
  container_c_type c_;
  container_c_type c_gt_;

  // For kernels that require a state buffer, we use scratch_.
  std::conditional_t<RequiresScratch,
      EntoUtil::EntoContainer<uint8_t, 128>,
      std::monostate> scratch_;

public:
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = true;

#ifdef NATIVE
  std::string serialize_impl() const
  {
    std::ostringstream oss;
    oss << M << "," << N << "," << P << ",";
    if constexpr (!UseEigen)
    {
      for (size_t i = 0; i < M * P; ++i)
      {
        oss << c_[i];
        if (i < M * P - 1)
          oss << ",";
      }
    }
    else
    {
      for (size_t i = 0; i < M; ++i)
      {
        for (size_t j = 0; j < P; ++j)
        {
          oss << c_(i, j);
          if (!(i == M - 1 && j == P - 1))
            oss << ",";
        }
      }
    }
    return oss.str();
  }
#endif

  // Deserialize: read a CSV line and initialize the matrices.
  // Expected format: M,N,P,(M×N values for A),(N×P values for B),(M×P values for ground truth C)
  bool deserialize_impl(const char* line)
  {
    std::istringstream iss(line);
    char comma;
    size_t m, n, p;
    if (!(iss >> m >> comma) || !(iss >> n >> comma) || !(iss >> p >> comma))
      return false;
    if (m != M || n != N || p != P)
      return false;

    if constexpr (!UseEigen)
    {
      a_.clear();
      for (size_t i = 0; i < M * N; ++i)
      {
        T value;
        if (!(iss >> value))
          return false;
        if (i < M * N - 1)
          iss >> comma;
        a_.push_back(value);
      }
      b_.clear();
      for (size_t i = 0; i < N * P; ++i)
      {
        T value;
        if (!(iss >> value))
          return false;
        if (i < N * P - 1)
          iss >> comma;
        b_.push_back(value);
      }
      c_gt_.clear();
      for (size_t i = 0; i < M * P; ++i)
      {
        T value;
        if (!(iss >> value))
          return false;
        if (i < M * P - 1)
          iss >> comma;
        c_gt_.push_back(value);
      }
    }
    else
    {
      // For Eigen matrices, fill element-wise.
      for (size_t i = 0; i < M; ++i)
      {
        for (size_t j = 0; j < N; ++j)
        {
          T value;
          if (!(iss >> value))
            return false;
          if (i * N + j < M * N - 1)
            iss >> comma;
          a_(i, j) = value;
        }
      }
      for (size_t i = 0; i < N; ++i)
      {
        for (size_t j = 0; j < P; ++j)
        {
          T value;
          if (!(iss >> value))
            return false;
          if (i * P + j < N * P - 1)
            iss >> comma;
          b_(i, j) = value;
        }
      }
      for (size_t i = 0; i < M; ++i)
      {
        for (size_t j = 0; j < P; ++j)
        {
          T value;
          if (!(iss >> value))
            return false;
          if (i * P + j < M * P - 1)
            iss >> comma;
          c_gt_(i, j) = value;
        }
      }
    }
    return true;
  }

  void solve_impl()
  {
    if constexpr (RequiresScratch)
    {
      kernel_(a_, b_, c_, scratch_.data(), M, N, P);
    }
    else
    {
      kernel_(a_, b_, c_, M, N, P);
    }
  }

  bool validate_impl()
  {
    T tol = static_cast<T>(1e-5);
    if constexpr (UseEigen)
    {
      return (c_ - c_gt_).norm() <= tol;
    }
    else
    {
      for (size_t i = 0; i < M * P; ++i)
      {
        if (std::abs(c_[i] - c_gt_[i]) > tol)
          return false;
      }
      return true;
    }
  }

  void clear_impl()
  {
    if constexpr (!UseEigen)
    {
      a_.clear();
      b_.clear();
      c_.clear();
      c_gt_.clear();
    }
    else
    {
      a_.setZero();
      b_.setZero();
      c_.setZero();
      c_gt_.setZero();
    }
    if constexpr (RequiresScratch)
    {
      scratch_.clear();
    }
  }

  static constexpr const char* header_impl() { return "M, N, P, Matrix A, Matrix B, Matrix C_gt"; }

  mat_mul_problem(Kernel kernel) : kernel_(std::move(kernel))
  {
    if constexpr (RequiresScratch)
    {
      scratch_.resize(128);
      scratch_.fill(0);
    }
  }
};

} // namespace EntoBench

#endif // MAT_MUL_PROBLEM_H
