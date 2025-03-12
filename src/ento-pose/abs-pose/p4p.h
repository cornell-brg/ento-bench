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

template <typename Scalar, std::size_t N, int Order=0, int Method=0, int SVDMethod=0>
int homography_Npt(const EntoArray<Vec3<Scalar>, N> &x1,
                   const EntoArray<Vec3<Scalar>, N> &x2,
                   Matrix3x3<Scalar> *H);

template <typename Scalar, std::size_t N, bool CheckCheirality=true, int Method=0>
int homography_4pt(const EntoArray<Vec3<Scalar>, 4> &x1,
                   const EntoArray<Vec3<Scalar>, 4> &x2,
                   Matrix3x3<Scalar>* H);

#if defined(NATIVE)
template <typename Scalar, int Order = 0, bool CheckCheirality=true, int Method=0, int SVDMethod=0>
int homography_Npt(const std::vector<Vec3<Scalar>> &x1,
                   const std::vector<Vec3<Scalar>> &x2,
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

    Eigen::Matrix<Scalar, N, 1, Order> C1, C2, C3, C4;
    Eigen::Matrix<Scalar, 3, 2*N, Order> Mt;
    Eigen::Matrix<Scalar, 2*N, 3, Order> M;
    
    Eigen::Matrix<Scalar, N, 2, Order> Pt;
    Eigen::Matrix<Scalar, 2, 2, Order> PPt;
    Eigen::Matrix<Scalar, 2, N, Order> Pp;

    Mt.row(4) = x1.col(0) * -1;
    Mt.row(5) = x1.col(1) * -1;
    C1 = Mt.row(4) * x2.col(0);
    C2 = Mt.row(4) * x2.col(1);
    C3 = Mt.row(5) * x2.col(0);
    C4 = Mt.row(5) * x2.col(1);

    Scalar mC1 = C1.mean();
    Scalar mC2 = C2.mean();
    Scalar mC3 = C3.mean();
    Scalar mC4 = C4.mean();

    // Mt is 3x2N, M is 2Nx3
    // C1 is N x 1, mC1 is 1x1
    Mt.row(0) = C1 - mC1;
    Mt.row(1) = C2 - mC2;
    Mt.row(2) = C3 - mC3;
    Mt.row(3) = C4 - mC4;
    M = Mt.transpose();

    // P = A
    // At is Nx2, A is 2xN
    Pt = x1.transpose() * x1;

    Scalar detinv = 1.0f / (PPt(0)(0) * PPt(1)(1) - PPt(0)(1) * PPt(1)(0));

    PPt *= detinv;

    Eigen::Matrix<Scalar, 2*N, 3, Order> E;
    Eigen::Matrix<Scalar, 4, 3, Order> B;
    Eigen::Matrix<Scalar, 2*N, 3, Eigen::ColMajor> A;
    auto Ex = E.block(0, 0, N, 3);
    auto Ey = E.block(N, 0, N, 3);
    auto Bx = B.block(0, 0, 2, 3);
    auto By = B.block(2, 0, 2, 3);
    auto A1 = A.block(0, 0, N, 3);
    auto A2 = A.block(N, 0, N, 3);
    auto Mx = M.block(0, 0, N, 3);
    auto My = M.block(N, 0, N, 3);

    Pp = PPt * x1.tranpose();
    Bx = Pp * Mx;
    By = Pp * My;
    Ex = Pt * Bx;
    Ey = Pt * By;

    A1 = Mx - Ex;
    A2 = My - Ey;

    //Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> V 
    auto V = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>::Zeros(3, 3);

    Eigen::Matrix<Scalar, 9, 1, Eigen::RowMajor> h;
    if constexpr (SVDMethod == 0)
    {
      // @TODO Hook up Eigen Jacobi SVD here.
      Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2*N, 3>> USV(M, Eigen::ComputeFullV);
      h = USV.matrixV.col(3);
      *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
    }
    else
    {
      // @TODO Hook up OSJ SVD Here.
    }
  }
  H->normalize();
  Scalar det = H->determinant();
  if (std::abs(det) < 1e-8) {
      return 0;
  }

  return 1;
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
    *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
  }
  else // if (Method == 1)
  {
    // Find left nullspace using LU (faster but has degeneracies)
    Eigen::Matrix<Scalar, 9, 1> h = M.template block<8, 8>(0, 0).partialPivLu().solve(-M.template block<8, 1>(0, 8)).homogeneous();
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
template <typename Scalar, int Order, bool CheckCheirality, int Method, int SVDMethod>
int homography_Npt(const std::vector<Vec3<Scalar>> &x1,
                   const std::vector<Vec3<Scalar>> &x2,
                   Matrix3x3<Scalar> *H)
{
  static_assert((Method == 0) || (Method == 1), "Homography N-pt Method template must be 0 or 1.");
  const std::size_t N = x1.size();
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
    Eigen::Matrix<Scalar, Eigen::Dynamic, 9> M(2*N, 9);
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
      Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 9>> USV(M, Eigen::ComputeFullV);
      h = USV.matrixV().col(8);
      *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
    }
    else if constexpr (SVDMethod==1) 
    {
      // @TODO: Hook up OSJ SVD here.
      Eigen::Matrix<Scalar, 9, 9> V;
      V.setIdentity();

      osj_svd(M, V, h);
      
      *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
    }
    else if constexpr (SVDMethod==2)
    {
        
      //using MatX9 = Eigen::Matrix<Scalar, Eigen::Dynamic, 9>;
      //using RMat3 = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 9, 9>> solver(M.transpose() *
                                                                        M);
      //Matrix3x3<Scalar> E;
      *H = Eigen::Map<const Matrix3x3<Scalar>>(solver.eigenvectors().col(8).data());
    }
      
  }
  else
  {
    // @TODO Perform DLT-HO here.

    Eigen::Matrix<Scalar, N, 1, Order> C1, C2, C3, C4;
    Eigen::Matrix<Scalar, 3, 2*N, Order> Mt;
    Eigen::Matrix<Scalar, 2*N, 3, Order> M;
    
    Eigen::Matrix<Scalar, N, 2, Order> Pt;
    Eigen::Matrix<Scalar, 2, 2, Order> PPt;
    Eigen::Matrix<Scalar, 2, N, Order> Pp;

    Mt.row(4) = x1.col(0) * -1;
    Mt.row(5) = x1.col(1) * -1;
    C1 = Mt.row(4) * x2.col(0);
    C2 = Mt.row(4) * x2.col(1);
    C3 = Mt.row(5) * x2.col(0);
    C4 = Mt.row(5) * x2.col(1);

    Scalar mC1 = C1.mean();
    Scalar mC2 = C2.mean();
    Scalar mC3 = C3.mean();
    Scalar mC4 = C4.mean();

    // Mt is 3x2N, M is 2Nx3
    // C1 is N x 1, mC1 is 1x1
    Mt.row(0) = C1 - mC1;
    Mt.row(1) = C2 - mC2;
    Mt.row(2) = C3 - mC3;
    Mt.row(3) = C4 - mC4;
    M = Mt.transpose();

    // P = A
    // At is Nx2, A is 2xN
    Pt = x1.transpose() * x1;

    Scalar detinv = 1.0f / (PPt(0)(0) * PPt(1)(1) - PPt(0)(1) * PPt(1)(0));

    PPt *= detinv;

    Eigen::Matrix<Scalar, 2*N, 3, Order> E;
    Eigen::Matrix<Scalar, 4, 3, Order> B;
    Eigen::Matrix<Scalar, 2*N, 3, Eigen::ColMajor> A;
    auto Ex = E.block(0, 0, N, 3);
    auto Ey = E.block(N, 0, N, 3);
    auto Bx = B.block(0, 0, 2, 3);
    auto By = B.block(2, 0, 2, 3);
    auto A1 = A.block(0, 0, N, 3);
    auto A2 = A.block(N, 0, N, 3);
    auto Mx = M.block(0, 0, N, 3);
    auto My = M.block(N, 0, N, 3);

    Pp = PPt * x1.tranpose();
    Bx = Pp * Mx;
    By = Pp * My;
    Ex = Pt * Bx;
    Ey = Pt * By;

    A1 = Mx - Ex;
    A2 = My - Ey;

    //Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> V 
    Eigen::Matrix<Scalar, 3, 3> V;
    V.setZero();

    Eigen::Matrix<Scalar, 9, 1, Eigen::RowMajor> h;
    if constexpr (SVDMethod == 0)
    {
      // @TODO Hook up Eigen Jacobi SVD here.
      Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2*N, 3>> USV(M, Eigen::ComputeFullV);
      h = USV.matrixV.col(3);
      *H = Eigen::Map<const Matrix3x3<Scalar>>(h.data()).transpose();
    }
    else
    {
      // @TODO Hook up OSJ SVD Here.
    }
  }
  H->normalize();
  Scalar det = H->determinant();
  if (std::abs(det) < 1e-8) {
      return 0;
  }

  return 1;
}


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
    Eigen::Matrix<Scalar, 9, 1> h = M.template block<8, 8>(0, 0).partialPivLu().solve(-M.template block<8, 1>(0, 8)).homogeneous();
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

