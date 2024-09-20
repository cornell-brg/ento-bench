#include <cstdio>
#include <Eigen/Dense>
#include "ento-math/svd.h"
#include "ento-math/core.h"

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

  osj_svd_generic(A, V, min_v);
  printf("Min right vec: [ %f, %f, %f ]\n", min_v(0), min_v(1), min_v(2));
  return;
}


int main(void)
{

  // Tests:
  // 1. SVD with 12x9
  // 2. SVD with 8x3
  // 3. ...
  volatile int a = 0;
  volatile int b = 1;
  volatile int c = a + b;
  b = c + a + 10;

  printf("A, B, C: %i, %i, %i", a, b, c);
  printf("HI %i\n", a);
  //printf("Running OSJ SVD known size simple...\n");
  //test_osj_svd_known_size_simple<float>();
  
  //printf("Running OSJ SVD bounded size simple...\n");
  //test_osj_svd_bounded_size_simple<float>();

  return 0;
}
