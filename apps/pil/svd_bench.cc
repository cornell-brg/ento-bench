
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>

#include "bench/harness.h"
#include "linalg_kernels.hh"
#include "linalg/blas.h"
#include "linalg/svd.h"

//#ifdef GEM5
//#include "sys/syscalls.c"
//#endif

#define MATRIX_ROWS 8
#define MATRIX_COLS 3
#define V_ROWS      3
#define V_COLS      3
#define CACHE_SIZE  256
#define ITERS       10

constexpr int rows = 8;
constexpr int cols = 3;
using namespace bench;
using namespace linalg_kernels;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::JacobiSVD;
using Matrix8x3 = Matrix<float, rows, cols>;

static volatile char dummy[CACHE_SIZE];

void
__attribute__((noinline))
sequential_flush_cache()
{
  for (size_t i = 0; i < CACHE_SIZE; ++i)
  {
    dummy[i] = i;
  }
}

int main()
{

  // Common raw data backing array
  float raw_data[MATRIX_ROWS * MATRIX_COLS] = {0};
  Matrix<float, MATRIX_ROWS, MATRIX_COLS> eigenA(raw_data);
  JacobiSVD<Matrix<float, rows, cols>, Eigen::ComputeThinU | Eigen::ComputeThinV> eigenJacobiSVD(rows, cols);
  //auto eigenA = Matrix8x3::Zero();
  //JacobiSVD<Matrix8x3, ThinOpts> eigenJacobiSVD(rows, cols);

  /*double V_data[MATRIX_ROWS * MATRIX_COLS] = {0}; // Right Singular Vectors for GSL methods
  double S_data[MATRIX_COLS] = {0}; // Singular values for GSL methods
  double work_data[MATRIX_ROWS] = {0}; // Working space for GSL GR SVD
  double X_data[MATRIX_COLS * MATRIX_COLS] = {0}; // Additional Working Space for GSL GR R-SVD 
                                                 
  // Defining GSL matrix and vector views
  gsl_matrix_view gslA_view = gsl_matrix_view_array(gsl_raw_copy, MATRIX_ROWS, MATRIX_COLS);
  gsl_matrix* gslA = &gslA_view.matrix;

  gsl_vector_view gslS_view = gsl_vector_view_array(S_data, MATRIX_COLS);
  gsl_vector* gslS = &gslS_view.vector;

  gsl_vector_view gslWork_view = gsl_vector_view_array(work_data, MATRIX_COLS);
  gsl_vector* gslWork = &gslWork_view.vector;

  gsl_matrix_view gslV_view = gsl_matrix_view_array(V_data, V_ROWS, V_COLS);
  gsl_matrix* gslV = &gslV_view.matrix;

  gsl_matrix_view gslX_view = gsl_matrix_view_array(X_data, MATRIX_COLS, MATRIX_COLS);
  gsl_matrix* gslX = &gslX_view.matrix;*/
  //auto eigen_tsj_lmbda_no_capture = [&eigenJacobiSVD](Matrix8x3 A) -> void {
  //  eigenJacobiSVD.compute(A);
  //};

  //eigen_tsj_lmbda_no_capture(eigenA);

  /* Lambda Workloads */
  auto eigen_tsj_lmbda = [&eigenJacobiSVD, &eigenA]() -> void {
    eigenJacobiSVD.compute(eigenA);
  };

  /*auto gsl_osj_lmbda = [&gslA, &gslV, &gslS]() -> void {
    gsl_osjacobi_svd(gslA, gslV, gslS); 
  };

  auto gsl_gr_lmbda = [&gslA, &gslV, &gslS, &gslWork]() -> void {
    gsl_golubreinsch_svd(gslA, gslV, gslS, gslWork);
  };

  auto gsl_gr_mod_lmbda = [&gslA, &gslV, &gslS, &gslWork, &gslX]() -> void {
    gsl_golubreinsch_mod_svd(gslA, gslV, gslS, gslWork, gslX);
  };*/

  MultiHarness svd_harness(eigen_tsj_lmbda);
  svd_harness.run();

  return 0;
}
