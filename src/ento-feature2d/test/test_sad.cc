#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <math/FixedPoint.hh>
#include <ento-feature2d/feat2d_util.h>

#include <ento-feature2d/sad.h>
#include <ento-util/unittest.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoFeature2D;

void test_basic() {
    constexpr int stride = 320;
    uint8_t frame1[stride * 8] = {0};
    uint8_t frame2[stride * 8] = {0};

    // Case 1: both frames are all zeros → SAD should be 0
    ENTO_TEST_CHECK_INT_EQ(sad(frame1, frame2), 0);

    // Case 2: frame1 is all zeros, frame2 is all ones in top-left 8x8
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col)   
            frame2[row*stride + col] = 1;
    }
    ENTO_TEST_CHECK_INT_EQ(sad(frame1, frame2), 8 * 8 * 1);

    // Case 3: frame1 has alternating 0x00 and frame2 has 0xFF → SAD = 8x8x255
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col)   
            frame2[row*stride + col] = 0xFF;
    }
    ENTO_TEST_CHECK_INT_EQ(sad(frame1, frame2), 8 * 8 * 255);
}

int main ( int argc, char ** argv)
{
  using namespace EntoUtil;
  ENTO_TEST_START();
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_sad_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_basic();

  ENTO_TEST_END();
} 