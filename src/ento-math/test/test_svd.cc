#include <cstdio>
#include <Eigen/Dense>
#include "ento-math/svd.h"
#include "ento-math/core.h"
#include "ento-util/matrix_reader.h"

using namespace EntoMath;

template <typename Scalar>
void test_osj_svd_known_size_simple()
{
  Eigen::Matrix<Scalar, 3, 3, 0> A;
  A << 1, 0, 0, 
       0, 2, 0,
       0, 0, 3;

  Eigen::Matrix<Scalar, 3, 1, 0> min_v;
  Eigen::Matrix<Scalar, 3, 3, 0> V;
  V.setIdentity();

  osj_svd<Scalar, 3, 3, 0>(A, V, min_v);
  printf("Min right vec: [ %f, %f, %f ]\n", min_v(0), min_v(1), min_v(2));
  return;
}

template <typename Scalar>
void test_osj_svd_bounded_size_simple()
{
  BoundedMatrix<Scalar, 3, 3, 0> A(3,3);
  A << 1, 0, 0,
       0, 2, 0,
       0, 0, 3;

  BoundedColVector<Scalar, 3> min_v(3,1);
  min_v << 0, 0, 0;
  BoundedMatrix<Scalar, 3, 3, 0> V(3,3);
  V.setIdentity();

  printf("Min right vec: [ %f, %f, %f ]\n", min_v(0), min_v(1), min_v(2));

  osj_svd_bounded<Scalar, 3, 3, 0>(A, V, min_v);
  printf("Min right vec: [ %f, %f, %f ]\n", min_v(0), min_v(1), min_v(2));
  return;
}


template <typename Scalar>
void test_osj_svd_generic_simple()
{
  BoundedMatrix<Scalar, 3, 3> A;
  A.resize(3,3);
  A << 1, 0, 0,
       0, 2, 0,
       0, 0, 3;

  BoundedColVector<Scalar, 3> min_v;
  min_v.resize(3,1);
  BoundedMatrix<Scalar, 3, 3> V;
  V.resize(3,3);
  V.setIdentity();

  osj_svd_generic(A, V, min_v);
  printf("Min right vec: [ %f, %f, %f ]\n", min_v(0), min_v(1), min_v(2));
  return;
}

void test_osj_svd_8x3( char* matrix_file )
{
  Eigen::Matrix<float, 8, 3> A;
  printf("A rows, cols: (%d, %d)\n", A.rows(), A.cols());
  // matrix_from_file("/home/ddo26/workspace/entomoton-bench/datasets/unittest/svd_8x3.txt",
  //                  A);
  matrix_from_file(matrix_file, A);

  // Eigen::Matrix<float, 8, 3> exp_A;
  // exp_A << -0.99687326,  0.00489541,  0.00042951,
  //         0.9968732 , -0.00489541, -0.00042956,
  //        -0.99687326,  0.00489541,  0.00042951,
  //         0.9968733 , -0.00489541, -0.00042956,
  //         0.00111352,  1.0031136,   0.00147715,
  //        -0.00111352, -1.0031135,  -0.001477  ,
  //         0.00111352,  1.0031136,   0.00147715,
  //        -0.00111352, -1.0031137,  -0.001477;  
  // for (int i = 0; i < 8; ++i)
  // {
  //   for (int j = 0; j < 3; ++j)
  //   {
  //     bool withinTol = A.isApprox(exp_A, 1e-5);
  //     DPRINTF("A(%i, %i) = %.8f, Within Tol? %i \n", i, j, A(i,j), withinTol);

  //   }
  // }

  Eigen::Matrix<float, 3, 3> V;
  V.setIdentity();
  DPRINTF("Sanity check diagonal of V: %f, %f, %f", V(0,0), V(1,1), V(2,2));

  Eigen::Matrix<float, 3, 1> min_v;

  printf("Running svd on our 8x3 A matrix!\n");
  osj_svd<float, 8, 3, 0>(A, V, min_v);
  

  for (int i = 0; i < 8; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      DPRINTF("A(%i, %i) = %f\n", i, j, A(i,j));
    }
  }
  DPRINTF("\n");

  Eigen::Matrix<float, 3, 3> S;
  S.setIdentity();
  Eigen::Matrix<float, 8, 3> U;
  for (int i = 0; i < 3; ++i)
  {
    float s = sqrtf(A.col(i).squaredNorm());
    S(i, i) = s;
    for (int j = 0; j < 8; ++j)
    {
      U(j, i) = A(j,i) / s;
      DPRINTF("U(%i, %i) = %.6f\n", j, i, A(j,i) / s);

    }
  }
  DPRINTF("\n");

  DPRINTF("S: %.10f, %.10f, %.10f\n", S(0,0), S(1,1), S(2,2));

  DPRINTF("\n");
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      DPRINTF("V(%i, %i) = %f\n", i, j, V(i,j));
    }
  }
  DPRINTF("\n");
  printf("Min right vec: [%f, %f, %f]\n", min_v(0), min_v(1), min_v(2));

  Eigen::Matrix<float, 8, 3> inter;
  Eigen::Matrix<float, 8, 3> out;
  inter = U * S;
  out = inter * V.transpose();

  for (int i = 0; i < 8; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      DPRINTF("Out(%i, %i) = %f\n", i, j, out(i,j));
    }
  }
  DPRINTF("\n");
  return;

}

int main(int argc, char** argv)
// int main()
{ 

  // Tests:
  // 1. SVD with 12x9
  // 2. SVD with 8x3
  // 3. ...
  // printf("Running OSJ SVD known size simple...\n");
  test_osj_svd_known_size_simple<float>();
  
  // printf("Running OSJ SVD bounded size simple...\n");
  test_osj_svd_bounded_size_simple<float>();

  //int first_dim = atoi( argv[1] );

  //if ( first_dim == 8 ) {
  //  printf( "%s\n", argv[2] );
  //  test_osj_svd_8x3( argv[2] );
  //}
  // if ( strcmp( argv[1], "8" ) == 0 ) {
  //   printf("Running OSJ SVD for an 8x3 matrix from file...\n");
  //   test_osj_svd_8x3( argv[2] );
  // }
  // else if ( strcmp( argv[1], "356" ) == 0 ) {
  //   printf("Running OSJ SVD for an 256x3 matrix from file...\n");
  //   // test_osj_svd_256x3( argv[2] );
  // }
  // else {
  //   printf( "invalid first matrix dimension\n" );
  //   return 1;
  // }

  return 0;
}
