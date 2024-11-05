#include <stdlib.h>
#include <cstdio>

#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/pose_est_reader.h>

#include <ento-pose/abs-pose/dlt.h>
#include <feature2d/util.h>


using namespace std;
using namespace Eigen;
using namespace EntoUtil;

const char* dataset_path = DATASET_PATH;

void test_dlt()
{
  constexpr int N = 6;
  Matrix<float, N, 3> points3d;
  points3d << 0.5, 0.25, 2.25,
              1.5, 0.2, 2.5,
              0.5, 1.5, 3,
              1.35, 1.25, 3.5,
              0.1, 0, 1,
              1, 1.2, 1.25;

  // Define a simple ground truth projection matrix (3x4)
  Matrix<float, 3, 4> P_true;
  P_true << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0;

  // Project 3D points to 2D using the ground truth projection matrix
  Matrix<float, N, 2> points2d;
  Matrix<float, 4, 4> Tx;
  Matrix<float, 3, 3> Txp;
  for (int i = 0; i < 6; ++i)
  {
    Matrix<float, 4, 1> homogeneous_point(points3d(i, 0),
                                          points3d(i, 1),
                                          points3d(i, 2),
                                          1.0);
    Matrix<float, 3, 1> projected_point = P_true * homogeneous_point;
    points2d.row(i) = projected_point.hnormalized();  // Convert to 2D (homogeneous normalization)
  }

  // Estimate projection matrix using DLT
  
  ENTO_DEBUG("Running dlt!");
  normalize_2d_points(points2d, Txp);
  normalize_3d_points(points3d, Tx);
  ENTO_DEBUG_EIGEN_MATRIX(points2d, points2d.rows(), points2d.cols(), float);
  ENTO_DEBUG_EIGEN_MATRIX(points3d, points3d.rows(), points3d.cols(), float);
  Matrix<float, 3, 4> P_est = dlt(points2d, points3d);


  P_est = Txp.inverse() * P_est * Tx;

  ENTO_DEBUG("Made it this far!");

  ENTO_DEBUG_EIGEN_MATRIX(P_est, 3, 4, float);
  ENTO_DEBUG_EIGEN_MATRIX(P_true, 3, 4, float);

  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(P_est, P_true, 3, 4);

  ENTO_DEBUG("Made it this far!");

}

void test_dlt_ho()
{

}

void test_dlt_planar()
{

}

void test_dlt_planar_ho()
{

}

void test_ippe()
{

}

int main( int argc, char ** argv)
{
  int __n = ( argc == 1 ) ? 0 : atoi( argv[1] );
  if (__ento_test_num(__n, 1)) test_dlt();

}
