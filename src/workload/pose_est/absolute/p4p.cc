#include "p4p.hh"
#include <Eigen/Dense>
#include <linalg/svd.hh>

template <typename Scalar, typename N>
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
  osj_svd(&A, &H.data());

  if (H(3,3) < 0) H *= -1;
  
  return H;
}

template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, Eigen::Dynamic, 9, 0, 2 * MaxPoints, 9>
dlt_planar(Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxPoints, 2>& points2d,
           Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxPoints, 3>& points3d,
           const int N)
{
  
  Eigen::Matrix<Scalar, Eigen::Dynamic, 9, 0, 2 * MaxPoints, 9> A =
      Eigen::Matrix<Scalar, Eigen::Dynamic, 9>::Zero(2 * N, 9);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxPoints, 3> XP(N, 3);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxPoints, 3> YP(N, 3);
  
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxPoints, 1> Xdiag = points2d.col(0).head(N) * -1;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxPoints, 1> Ydiag = points2d.col(1).head(N) * -1;

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
  osj_svd(&A, &H);

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

  Mt.row(0) = C1 - mC1;
  Mt.row(1) = C2 - mC2;
  Mt.row(2) = C3 - mC3;
  Mt.row(3) = C4 - mC4;
  M = Mt.transpose();

  Pt = P.transpose();

  float32_t detinv = 1.0f / (PPt(0)(0) * PPt(1)(1) - PPt(0)(1) * PPt(1)(0));

  PPt *= detinv;

  Eigen::Matrix<Scalar, 2*N, 3, Order> E;
  Eigen::Matrix<Scalar, 4, 3, Order> B;
  Eigen::Matrix<Scalar, 2*N, 3, Eigen::ColMajor> A;
  auto Ex = E.block<N, 3>(0, 0);
  auto Ey = E.block<N, 3>(N, 0);
  auto Bx = B.block<2, 3>(0, 0);
  auto By = B.block<2, 3>(2, 0);
  auto A1 = A.block<N, 3>(0, 0);
  auto A2 = A.block<N, 3>(N, 0);

  Pp = PPt * P;
  Bx = Pp * Mx;
  By = Pp * My;
  Ex = Pt * Bx;
  Ey = Pt * By;

  A1 = Mx - Ex;
  A2 = My - Ey;

  //Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> V 
  auto V = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>::Zeros(3, 3);

  Eigen::Matrix<Scalar, 9, 1, Eigen::RowMajor> h;

  EntomotonMath::svd_osj(&At, &V, &h);

}
