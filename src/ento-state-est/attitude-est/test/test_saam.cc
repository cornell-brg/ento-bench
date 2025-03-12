#include <stdlib.h>

#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/dataset_reader.h>
#include <ento-util/file_path_util.h>
#include <ento-util/containers.h>

#include <ento-state-est/attitude-est/saam.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoStateEstimation;

const char* dataset_path = DATASET_PATH;

void test_saam_basic()
{
  printf("Hello saam!\n");
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

  if (__ento_test_num(__n, 1))  test_saam_basic();
}
