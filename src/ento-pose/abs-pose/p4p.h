#ifndef P4P_HH
#define P4P_HH

#include "dlt.h"
#include <ento-util/debug.h>
#include <Eigen/Dense>
#include <cstddef>
#include <ento-math/core.h>
#include <ento-math/svd.h>
#include <ento-pose/pose_util.h>

using namespace EntoMath;

namespace EntoPose
{

template <typename Scalar, std::size_t N, bool CheckCheirality=true, int Method=0, int SVDMethod=0>
int homography(const EntoArray<Vec3<Scalar>, N> &x1,
               const EntoArray<Vec3<Scalar>, N> &x2,
               Matrix3x3<Scalar> *H);

#if defined(NATIVE)
template <typename Scalar, bool CheckCheirality=true, int Method=0, int SVDMethod=0>
int homography(const std::vector<Vec3<Scalar>> &x1,
               const std::vector<Vec3<Scalar>> &x2,
               Matrix3x3<Scalar>* H);
#endif

// Legacy aliases for backward compatibility
template <typename Scalar, std::size_t N, int Order=0, int Method=0, int SVDMethod=0>
int homography_Npt(const EntoArray<Vec3<Scalar>, N> &x1,
                   const EntoArray<Vec3<Scalar>, N> &x2,
                   Matrix3x3<Scalar> *H) {
    return homography<Scalar, N, true, Method, SVDMethod>(x1, x2, H);
}

template <typename Scalar, bool CheckCheirality=true, int Method=0>
int homography_4pt(const EntoArray<Vec3<Scalar>, 4> &x1,
                   const EntoArray<Vec3<Scalar>, 4> &x2,
                   Matrix3x3<Scalar>* H) {
    return homography<Scalar, 4, CheckCheirality, Method, 0>(x1, x2, H);
}

// ================================================================================

template <typename Scalar, std::size_t N, bool CheckCheirality, int Method, int SVDMethod>
int homography(const EntoArray<Vec3<Scalar>, N> &x1,
               const EntoArray<Vec3<Scalar>, N> &x2,
               Matrix3x3<Scalar> *H)
{
  static_assert(N >= 4, "N must be greater or equal to 4 for planar homography!");
  static_assert((Method == 0) || (Method == 1), "Homography Method template must be 0 or 1.");
  
  if constexpr (CheckCheirality)
  {
    Vec3<Scalar> p = x1[0].cross(x1[1]);
    Vec3<Scalar> q = x2[0].cross(x2[1]);

    if (p.dot(x1[2]) * q.dot(x2[2]) < 0)
      return 0;

    if (p.dot(x1[3]) * q.dot(x2[3]) < 0)
      return 0;

    p = x1[2].cross(x1[3]);
    q = x2[2].cross(x2[3]);

    if (p.dot(x1[0]) * q.dot(x2[0]) < 0)
      return 0;

    if (p.dot(x1[1]) * q.dot(x2[1]) < 0)
      return 0;
  }

  // Build constraint matrix using ALL N points (not just first 4!)
  Eigen::Matrix<Scalar, 2*N, 9> M;
  for (size_t i = 0; i < N; ++i)  // Fixed: was hardcoded to 4
  {
    M.template block<1, 3>(2 * i, 0) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i, 3).setZero();
    M.template block<1, 3>(2 * i, 6) = -x2[i].x() * x1[i].transpose();

    M.template block<1, 3>(2 * i + 1, 0).setZero();
    M.template block<1, 3>(2 * i + 1, 3) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i + 1, 6) = -x2[i].y() * x1[i].transpose();
  }

  Eigen::Matrix<Scalar, 9, 1> h;
  
  if constexpr (N == 4 && Method == 1) {
    // For exactly 4 points, we can use LU decomposition (faster but less robust)
    h = M.template block<8, 8>(0, 0).partialPivLu().solve(-M.template block<8, 1>(0, 8)).homogeneous();
  } else if constexpr (N == 4 && Method == 0) {
    // For exactly 4 points, use QR decomposition
    Eigen::Matrix<Scalar, 9, 9> Q = M.transpose().householderQr().householderQ();
    h = Q.col(8);
  } else {
    // For N > 4 points, use SVD (most robust for overdetermined systems)
    if constexpr (SVDMethod == 0) {
      Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2*N, 9>> USV(M, Eigen::ComputeFullV);
      h = USV.matrixV().col(8);
    } else if constexpr (SVDMethod == 1) {
      // OSJ SVD method
      Eigen::Matrix<Scalar, 9, 9> V;
      V.setIdentity();
      osj_svd(M, V, h);
    } else if constexpr (SVDMethod == 2) {
      // Eigenvalue decomposition method
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 9, 9>> solver(M.transpose() * M);
      h = solver.eigenvectors().col(0);  // Smallest eigenvalue
    }
  }
  
  *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
  
  // Normalize and check for degeneracy
  H->normalize();
  Scalar det = H->determinant();
  if (std::abs(det) < 1e-8) {
      return 0;
  }

  return 1;
}

#if defined(NATIVE)
template <typename Scalar, bool CheckCheirality, int Method, int SVDMethod>
int homography(const std::vector<Vec3<Scalar>> &x1,
               const std::vector<Vec3<Scalar>> &x2,
               Matrix3x3<Scalar> *H)
{
  const std::size_t N = x1.size();
  if (N < 4) return 0;
  
  if constexpr (CheckCheirality)
  {
    Vec3<Scalar> p = x1[0].cross(x1[1]);
    Vec3<Scalar> q = x2[0].cross(x2[1]);

    if (p.dot(x1[2]) * q.dot(x2[2]) < 0)
      return 0;

    if (p.dot(x1[3]) * q.dot(x2[3]) < 0)
      return 0;

    p = x1[2].cross(x1[3]);
    q = x2[2].cross(x2[3]);

    if (p.dot(x1[0]) * q.dot(x2[0]) < 0)
      return 0;

    if (p.dot(x1[1]) * q.dot(x2[1]) < 0)
      return 0;
  }

  // Build constraint matrix using ALL N points
  Eigen::Matrix<Scalar, Eigen::Dynamic, 9> M(2*N, 9);
  for (size_t i = 0; i < N; ++i)  // Fixed: was hardcoded to 4
  {
    M.template block<1, 3>(2 * i, 0) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i, 3).setZero();
    M.template block<1, 3>(2 * i, 6) = -x2[i].x() * x1[i].transpose();

    M.template block<1, 3>(2 * i + 1, 0).setZero();
    M.template block<1, 3>(2 * i + 1, 3) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i + 1, 6) = -x2[i].y() * x1[i].transpose();
  }

  Eigen::Matrix<Scalar, 9, 1> h;
  
  if (N == 4 && Method == 1) {
    // For exactly 4 points, we can use LU decomposition (faster but less robust)
    h = M.template block<8, 8>(0, 0).partialPivLu().solve(-M.template block<8, 1>(0, 8)).homogeneous();
  } else if (N == 4 && Method == 0) {
    // For exactly 4 points, use QR decomposition
    Eigen::Matrix<Scalar, 9, 9> Q = M.transpose().householderQr().householderQ();
    h = Q.col(8);
  } else {
    // For N > 4 points, use SVD (most robust for overdetermined systems)
    if constexpr (SVDMethod == 0) {
      Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 9>> USV(M, Eigen::ComputeFullV);
      h = USV.matrixV().col(8);
    } else if constexpr (SVDMethod == 1) {
      // OSJ SVD method
      Eigen::Matrix<Scalar, 9, 9> V;
      V.setIdentity();
      osj_svd(M, V, h);
    } else if constexpr (SVDMethod == 2) {
      // Eigenvalue decomposition method
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 9, 9>> solver(M.transpose() * M);
      h = solver.eigenvectors().col(0);  // Smallest eigenvalue
    }
  }
  
  *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
  
  // Normalize and check for degeneracy
  H->normalize();
  Scalar det = H->determinant();
  if (std::abs(det) < 1e-8) {
      return 0;
  }

  return 1;
}

// Legacy aliases for backward compatibility
template <typename Scalar, int Order, bool CheckCheirality, int Method, int SVDMethod>
int homography_Npt(const std::vector<Vec3<Scalar>> &x1,
                   const std::vector<Vec3<Scalar>> &x2,
                   Matrix3x3<Scalar> *H) {
    return homography<Scalar, CheckCheirality, Method, SVDMethod>(x1, x2, H);
}

template <typename Scalar, bool CheckCheirality, int Method>
int homography_4pt(const std::vector<Vec3<Scalar>> &x1,
                   const std::vector<Vec3<Scalar>> &x2,
                   Matrix3x3<Scalar>* H) {
    return homography<Scalar, CheckCheirality, Method, 0>(x1, x2, H);
}
#endif

} // namespace EntoPose



#endif

