#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <math/FixedPoint.hh>
#include <ento-feature2d/feat2d_util.h>

#include <ento-feature2d/ii_optical_flow.h>
#include <ento-util/unittest.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoFeature2D;

void test_ii_optical_flow_basic() {
    // initialize point arrays
    Keypoint<float> *prevPts = new Keypoint<float>[2];
    Keypoint<float> *nextPts = new Keypoint<float>[2];
    bool *status = new bool[2];

    // initialize point info
    // first point is valid feature
    // second point is invalid feature
    float x_1(197.0f);
    float y_1(106.0f);
    float x_2(207.0f);
    float y_2(110.0f);
    prevPts[0] = Keypoint<float>(x_1, y_1);
    prevPts[1] = Keypoint<float>(x_2, y_2);

    string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
    const char* prev_image_path = top_str.c_str();
    constexpr int DIM = 320;
    Image<DIM, DIM, uint8_t> prevImg;
    ENTO_TEST_CHECK_INT_EQ(prevImg.image_from_pgm(prev_image_path), 1);

    Image<DIM, DIM, uint8_t> nextImg = prevImg;

    constexpr int WIN_DIM = 8; 
    int DET_EPSILON = (int)(1<<20);

    int x_shamt = 7;
    int y_shamt = 7;
    int num_good_points = 2;

    calcOpticalFlowII<DIM, DIM, WIN_DIM, float, uint8_t>(prevImg, nextImg,
        prevPts, nextPts, status, num_good_points, x_shamt, y_shamt, DET_EPSILON);

    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].x, x_1);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].y, y_1);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].x, x_2);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].y, y_2);

    delete [] prevPts;
    delete [] nextPts;
    delete [] status;
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
    __ento_replace_file_suffix(__FILE__, "test_ii_optical_flow_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_ii_optical_flow_basic();

  ENTO_TEST_END();
} 