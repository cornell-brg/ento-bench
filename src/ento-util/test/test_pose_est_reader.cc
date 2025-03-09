#include <stdlib.h>
#include <cstdio>

#include <Eigen/Dense>

#include <ento-util/debug.h>
#include "ento-util/unittest.h"
#include "ento-util/pose_est_reader.h"

using namespace std;
using namespace Eigen;

const char* dataset_path = DATASET_PATH;

void test_case_1_sample_file_open_and_read()
{
  char data_path[256];
  snprintf(data_path,
           sizeof(data_path),
           "%s/%s",
           DATASET_PATH, 
           "abs-pose/test/abs_pose_n1_example.csv");
  PoseEstimationDataLoader loader(data_path);

  Matrix<float, 3, 1> xp;
  Matrix<float, 2, 1> x;
  Matrix<float, 3, 4> T;

  bool finished = false;
  finished = loader.get_next_line(xp, x, T);
  ENTO_DEBUG_EIGEN_MATRIX(xp);
  ENTO_DEBUG_EIGEN_MATRIX(x);
  ENTO_DEBUG_EIGEN_MATRIX(T);
  const char* eof = (finished) ? "Yes" : "No";
  ENTO_DEBUG("Reached end of file? %s", eof);

  ENTO_TEST_CHECK_TRUE(finished)
}

void test_case_2_absolute_pose_file_read()
{
  char data_path[256];
  snprintf(data_path,
           sizeof(data_path),
           "%s/%s",
           DATASET_PATH, 
           "abs-pose/test/abs_pose_n10_example.csv");
  PoseEstimationDataLoader loader(data_path);

  int total_experiments = loader.num_experiments();

  Matrix<float, 3, 10> xp;
  Matrix<float, 2, 10> x;
  Matrix<float, 3, 4> T;

  
  int iters = 0;
  while (loader.get_next_line(xp, x, T))
  {
    iters++;
    if (iters >= 10) break;
  }

  ENTO_DEBUG("Number of experiments read: %i", iters);
  ENTO_TEST_CHECK_INT_EQ(iters, total_experiments);

}

void test_case_3_relative_pose_file_read()
{

}

void test_case_4_n_view_file_read()
{

}

int main( int argc, char ** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_pose_est_reader_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  //int __n = ( argc == 1 ) ? 0 : atoi( argv[1] );
  if (__ento_test_num(__n, 1)) test_case_1_sample_file_open_and_read();
  if (__ento_test_num(__n, 2)) test_case_2_absolute_pose_file_read();
  if (__ento_test_num(__n, 3)) test_case_3_relative_pose_file_read();
}
