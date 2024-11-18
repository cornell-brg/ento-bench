#ifndef ENTO_POSE_DLT_H
#define ENTO_POSE_DLT_H

#include <Eigen/Dense>
#include <ento-math/svd.h>
#include <ento-math/core.h>


template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 4> dlt(const Eigen::Matrix<Scalar, N, 2>& points2d,
                                const Eigen::Matrix<Scalar, N, 3>& points3d);


template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, 3, 4>
dlt(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxN, 3>& points3d);

template <typename Derived, typename Scalar>
Eigen::Matrix<Scalar, 3, 4>
dlt(const Eigen::DenseBase<Derived>& points2d,
    const Eigen::DenseBase<Derived>& points3d);

template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 4> dlt_ho(const Eigen::Matrix<Scalar, N, 2>& points2d,
                                   const Eigen::Matrix<Scalar, N, 3>& points3d);


template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, 3, 4>
dlt_ho(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
       const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxN, 3>& points3d);

template <typename Derived, typename Scalar>
Eigen::Matrix<Scalar, 3, 4>
dlt_ho(const Eigen::DenseBase<Derived>& points2d,
       const Eigen::DenseBase<Derived>& points3d);



template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(const Eigen::Matrix<Scalar, N, 2>& points2d, 
           const Eigen::Matrix<Scalar, N, 3>& points3d);


template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
           const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points3d);

template <typename Derived, typename Scalar>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(const Eigen::DenseBase<Derived>& points2d,
           const Eigen::DenseBase<Derived>& points3d);


template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 3> dlt_planar_ho(const Eigen::Matrix<Scalar, N, 2>& points2d,
                                          const Eigen::Matrix<Scalar, N, 2>& points3d);


template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar_ho(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
              const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points3d);

template <typename Derived, typename Scalar>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar_ho(const Eigen::DenseBase<Derived>& points2d,
              const Eigen::DenseBase<Derived>& points3d);


// Nonlinear Optimizations Based on the DLT methods above.
// These LM impls will be hardcoded to the Homography or DLT.
bool levenberg_marquardt_dlt();
bool levenberg_marquardt_dlt_planar();
bool levenberg_marquardt_dlt_planar_ho();

// ===========================================================
// Implementations
// ===========================================================

template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 4> dlt(const Eigen::Matrix<Scalar, N, 2>& points2d,
                                const Eigen::Matrix<Scalar, N, 3>& points3d)
{
  static Eigen::Matrix<Scalar, N, 4> X = Eigen::Matrix<Scalar, N, 4>::Zero();
  X.block(0, 0, N, 3) = points3d;  // Set first 3 columns to points3d
  X.col(3) = Eigen::Matrix<Scalar, N, 1>::Ones();  // Set last column to 1

  // Initialize A (2N x 12) matrix for DLT equation
  Eigen::Matrix<Scalar, 2 * N, 12, 0> A = Eigen::Matrix<Scalar, 2 * N, 12>::Zero();

  // Create XP and YP scaled 3D points matrices
  Eigen::Matrix<Scalar, N, 1> Xdiag = points2d.col(0) * -1;
  Eigen::Matrix<Scalar, N, 1> Ydiag = points2d.col(1) * -1;

  // Xdiag is NxN. X is Nx4
  Eigen::Matrix<Scalar, N, 4> XP = Xdiag.asDiagonal() * X;
  Eigen::Matrix<Scalar, N, 4> YP = Ydiag.asDiagonal() * X;

  // Populate matrix A
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      A(i, j)         = X(i, j);   // First row for each point correspondence
      A(i + N, j + 4) = X(i, j);   // Second row for each point correspondence
      A(i, j + 8)     = XP(i, j);  // X components with negative x scaling
      A(i + N, j + 8) = YP(i, j);  // Y components with negative y scaling
    }
  }

  Eigen::Matrix<Scalar, 12, 12, 0> V;
  V.setIdentity().eval();
  Eigen::Matrix<Scalar, 12, 1, 0> min_v;  // Vector to hold smallest singular vector
  min_v.setZero();

  // Compute SVD of A (A * p = 0), storing result in min_v
  ENTO_DEBUG_EIGEN_MATRIX(A, A.rows(), A.cols(), Scalar);
  EntoMath::osj_svd<Scalar, 2 * N, 12>(A, V, min_v);

  // Reshape min_v into a 3x4 projection matrix P
  Eigen::Matrix<Scalar, 3, 4> P;
  P << min_v(0), min_v(1), min_v(2), min_v(3),
       min_v(4), min_v(5), min_v(6), min_v(7),
       min_v(8), min_v(9), min_v(10), min_v(11);

  //EIGEN_DEBUG_EIGEN_MATRIX(P, 3, 4, float)
  ENTO_DEBUG_EIGEN_MATRIX(P, P.rows(), P.cols(), Scalar);
  if (std::abs(P(2, 3)) > EntoMath::ENTO_EPS) {
    P /= -P(2, 3);
  }
  P /= P(2,3);
  ENTO_DEBUG_EIGEN_MATRIX(P, P.rows(), P.cols(), Scalar);

  return P;
}


template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, 3, 4>
dlt(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxN, 3>& points3d);


template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(Eigen::Matrix<Scalar, N, 2>& points2d, 
           Eigen::Matrix<Scalar, N, 3>& points3d)
{
  Eigen::Matrix<Scalar, 2*N, 9> A = Eigen::Matrix<Scalar, 2*N, 9>::Zero();
  Eigen::Matrix<Scalar, N, 3> XP;
  Eigen::Matrix<Scalar, N, 3> YP;

  Eigen::Matrix<Scalar, N, 1> Xdiag = points2d.col(0) * -1;
  Eigen::Matrix<Scalar, N, 1> Ydiag = points2d.col(1) * -1;

  XP = Xdiag.asDiagonal() * points3d;
  YP = Ydiag.asDiagonal() * points3d;


  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      A(i, j)         =  points3d(i, j);
      A(i + N, j)     =  points3d(i, j);
      A(i, j + 6)     =  XP(i, j);
      A(i + N, j + 6) =  YP(i, j);
    }
  }

  Eigen::Matrix<Scalar, 3, 3> H;
  EntoMath::osj_svd(&A, &H.data());

  if (H(3,3) < 0) H *= -1;
  
  return H;
}

template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, Eigen::Dynamic, 9, 0, 2 * MaxN, 9>
dlt_planar(Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
           Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxN, 3>& points3d,
           const int N)
{
  
  Eigen::Matrix<Scalar, Eigen::Dynamic, 9, 0, 2 * MaxN, 9> A =
              Eigen::Matrix<Scalar, Eigen::Dynamic, 9>::Zero(2 * N, 9);

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxN, 3> XP(N, 3);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxN, 3> YP(N, 3);
  
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxN, 1> Xdiag = points2d.col(0).head(N) * -1;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxN, 1> Ydiag = points2d.col(1).head(N) * -1;

  XP = Xdiag.asDiagonal() * points3d.topRows(N);
  YP = Ydiag.asDiagonal() * points3d.topRows(N);

  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      A(i, j)         =  points3d(i, j);
      A(i + N, j)     =  points3d(i, j);
      A(i, j + 6)     =  XP(i, j);
      A(i + N, j + 6) =  YP(i, j);
    }
  }

  Eigen::Matrix<Scalar, 3, 3> H;
  EntoMath::osj_svd(&A, &H);

  if (H(3,3) < 0) H *= -1;
  
  return H;

}

template <typename Scalar, int N, int Order, int useOSJ=0>
Eigen::Matrix<Scalar, 3, 3> dlt_planar_ho(Eigen::Matrix<Scalar, N, 2, Order>& points2d,
                                          Eigen::Matrix<Scalar, N, 2, Order>& points3d)
{
  Eigen::Matrix<Scalar, N, 1, Order> C1, C2, C3, C4;
  Eigen::Matrix<Scalar, 3, 2*N, Order> Mt;
  Eigen::Matrix<Scalar, 2*N, 3, Order> M;
  
  Eigen::Matrix<Scalar, N, 2, Order> Pt;
  Eigen::Matrix<Scalar, 2, 2, Order> PPt;
  Eigen::Matrix<Scalar, 2, N, Order> Pp;

  Mt.row(4) = points2d.col(0) * -1;
  Mt.row(5) = points2d.col(1) * -1;
  C1 = Mt.row(4) * points3d.col(0);
  C2 = Mt.row(4) * points3d.col(1);
  C3 = Mt.row(5) * points3d.col(0);
  C4 = Mt.row(5) * points3d.col(1);

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
  Pt = points2d.transpose() * points2d;

  float32_t detinv = 1.0f / (PPt(0)(0) * PPt(1)(1) - PPt(0)(1) * PPt(1)(0));

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

  Pp = PPt * points2d.tranpose();
  Bx = Pp * Mx;
  By = Pp * My;
  Ex = Pt * Bx;
  Ey = Pt * By;

  A1 = Mx - Ex;
  A2 = My - Ey;

  //Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> V 
  auto V = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>::Zeros(3, 3);

  Eigen::Matrix<Scalar, 9, 1, Eigen::RowMajor> h;

  EntoMath::osj_svd(&A, &V, &h);
}

#endif // ENTO_POSE_DLT_H
       //
