#ifndef P4P_HH
#define P4P_HH

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-pose/pose_util.h>

using namespace EntoMath;

namespace EntoPose
{

template <typename Scalar, std::size_t N, int Order=0, int Method=0>
int homography_Npt(const EntoArray<Vec3<Scalar>, N> &x1,
                   const EntoArray<Vec3<Scalar>, N> &x2,
                   Matrix3x3<Scalar> *H);

#if defined(NATIVE)
template <typename Scalar, std::size_t N, int Order = 0, bool CheckCheirality=true, int Method=0>
int homography_Npt(const std::vector<Vec3<Scalar>> &x1,
                   const std::vector<Vec3<Scalar>> &x2,
                   Matrix3x3<Scalar>* H);

template <typename Scalar, std::size_t N, bool CheckCheirality=true, int Method=0>
int homography_4pt(const EntoArray<Vec3<Scalar>, 4> &x1,
                   const EntoArray<Vec3<Scalar>, 4> &x2,
                   Matrix3x3<Scalar>* H);

template <typename Scalar, bool CheckCheirality=true, int Method=0>
int homography_4pt(const std::vector<Vec3<Scalar>> &x1,
                   const std::vector<Vec3<Scalar>> &x2,
                   Matrix3x3<Scalar>* H);
#endif

// ================================================================================

template <typename Scalar, std::size_t N, int Order, bool CheckCheirality, int Method, int SVDMethod>
int homography_Npt(const EntoArray<Vec3<Scalar>, N> &x1,
                   const EntoArray<Vec3<Scalar>, N> &x2,
                   Matrix3x3<Scalar> *H)
{
  static_assert(N >= 4, "N must be greater or equal to 4 for planar homography!");
  static_assert((Method == 0) || (Method == 1), "Homography N-pt Method template must be 0 or 1.");
  if constexpr(Method == 0)
  {
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
    Eigen::Matrix<Scalar, 2*N, 9> M;
    for (size_t i = 0; i < 4; ++i)
    {
      M.template block<1, 3>(2 * i, 0) = x2[i].z() * x1[i].transpose();
      M.template block<1, 3>(2 * i, 3).setZero();
      M.template block<1, 3>(2 * i, 6) = -x2[i].x() * x1[i].transpose();

      M.template block<1, 3>(2 * i + 1, 0).setZero();
      M.template block<1, 3>(2 * i + 1, 3) = x2[i].z() * x1[i].transpose();
      M.template block<1, 3>(2 * i + 1, 6) = -x2[i].y() * x1[i].transpose();
    }
    //Eigen::Matrix<Scalar, 9, 1> h = M.block<8, 8>(0, 0).partialPivLu().solve(-M.block<8, 1>(0, 8)).homogeneous();
    Eigen::Matrix<Scalar, 9, 1> h;
    if constexpr (SVDMethod == 0)
    {
      Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2*N, 9>> USV(M, Eigen::ComputeFullV);
      h = USV.matrixV.col(8);
      *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
    }
    else 
    {
      // @TODO: Hook up OSJ SVD here.
      
    }
      
  }
  else
  {
    // @TODO Perform DLT-HO here.
    if constexpr (SVDMethod == 0)
    {
      // @TODO Hook up Eigen Jacobi SVD here.
    }
    else
    {
      // @TODO Hook up OSJ SVD Here.
    }
  }
}

template <typename Scalar, std::size_t N, bool CheckCheirality, int Method>
int homography_4pt(const EntoArray<Vec3<Scalar>, 4> &x1,
                   const EntoArray<Vec3<Scalar>, 4> &x2,
                   Matrix3x3<Scalar>* H)
{
  static_assert((Method == 0) || (Method == 1), "Homography 4pt Method template must be 0 or 1.");
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

  Eigen::Matrix<Scalar, 8, 9> M;
  for (size_t i = 0; i < 4; ++i)
  {
    M.template block<1, 3>(2 * i, 0) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i, 3).setZero();
    M.template block<1, 3>(2 * i, 6) = -x2[i].x() * x1[i].transpose();

    M.template block<1, 3>(2 * i + 1, 0).setZero();
    M.template block<1, 3>(2 * i + 1, 3) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i + 1, 6) = -x2[i].y() * x1[i].transpose();
  }

  if constexpr (Method == 0)
  {
    // Find left nullspace to M using QR (slower)
    Eigen::Matrix<Scalar, 9, 9> Q = M.transpose().householderQr().householderQ();
    Eigen::Matrix<Scalar, 9, 1> h = Q.col(8);
  }
  else // if (Method == 1)
  {
    // Find left nullspace using LU (faster but has degeneracies)
    Eigen::Matrix<Scalar, 9, 1> h = M.template block<8, 8>(0, 0).partialPivLu().solve(-M.block<8, 1>(0, 8)).homogeneous();
    *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
  }

  // Check for degenerate homography
  H->normalize();
  Scalar det = H->determinant();
  if (std::abs(det) < 1e-8) {
      return 0;
  }

  return 1;
}



#if defined(NATIVE)
template <typename Scalar, bool CheckCheirality, int Method>
int homography_4pt(const std::vector<Vec3<Scalar>> &x1,
                   const std::vector<Vec3<Scalar>> &x2,
                   Matrix3x3<Scalar>* H)
{
  static_assert((Method == 0) || (Method == 1), "Homography 4pt Method template must be 0 or 1.");
  if (CheckCheirality)
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

  Eigen::Matrix<Scalar, 8, 9> M;
  for (size_t i = 0; i < 4; ++i)
  {
    M.template block<1, 3>(2 * i, 0) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i, 3).setZero();
    M.template block<1, 3>(2 * i, 6) = -x2[i].x() * x1[i].transpose();

    M.template block<1, 3>(2 * i + 1, 0).setZero();
    M.template block<1, 3>(2 * i + 1, 3) = x2[i].z() * x1[i].transpose();
    M.template block<1, 3>(2 * i + 1, 6) = -x2[i].y() * x1[i].transpose();
  }

  if constexpr (Method == 0)
  {
    // Find left nullspace to M using QR (slower)
    Eigen::Matrix<Scalar, 9, 9> Q = M.transpose().householderQr().householderQ();
    Eigen::Matrix<Scalar, 9, 1> h = Q.col(8);
    *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
  }
  else // if (Method == 1)
  {
    // Find left nullspace using LU (faster but has degeneracies)
    Eigen::Matrix<Scalar, 9, 1> h = M.template block<8, 8>(0, 0).partialPivLu().solve(-M.block<8, 1>(0, 8)).homogeneous();
    *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
  }

  // Check for degenerate homography
  H->normalize();
  Scalar det = H->determinant();
  if (std::abs(det) < 1e-8) {
      return 0;
  }

  return 1;
}
#endif


} // namespace EntoPose



#endif

