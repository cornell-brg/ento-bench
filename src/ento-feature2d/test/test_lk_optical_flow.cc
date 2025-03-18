#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <math/FixedPoint.hh>
#include <ento-feature2d/image_pyramid.h>
#include <ento-feature2d/feat2d_util.h>
#include <image_io/Image.h>
#include <ento-feature2d/lk_optical_flow.h>
#include <ento-util/unittest.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoFeature2D;


const int decimal_bits = 20;
using fp_t = FixedPoint<64-decimal_bits, decimal_bits, int64_t>;

void test_lk_optical_flow_pyramidal() {

    constexpr size_t WIN_DIM = 3;

    string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
    const char* prev_image_path = top_str.c_str();
    constexpr size_t DIM = (size_t) 320;
    Image<DIM, DIM, uint8_t> prevImg;
    ENTO_TEST_CHECK_INT_EQ(prevImg.image_from_pgm(prev_image_path), 1);

    string next_str = "/Users/acui21/Documents/brg/pgm_images/image_3.pgm";
    const char* next_image_path = next_str.c_str();
    Image<DIM, DIM, uint8_t> nextImg;
    ENTO_TEST_CHECK_INT_EQ(nextImg.image_from_pgm(next_image_path), 1);

    constexpr size_t NUM_LEVELS = 1;
    ImagePyramid<NUM_LEVELS, 320, 320, uint8_t> prevPyramid;
    prevPyramid.set_top_image(prevImg);
    // Print the types of each level in the pyramid:
    prevPyramid.initialize_pyramid();


    ImagePyramid<NUM_LEVELS, 320, 320, uint8_t> nextPyramid;
    nextPyramid.set_top_image(nextImg);
    // Print the types of each level in the pyramid:
    nextPyramid.initialize_pyramid();

    // initialize point arrays
    Keypoint<fp_t> *prevPts = new Keypoint<fp_t>[2];
    Keypoint<fp_t> *nextPts = new Keypoint<fp_t>[2];
    bool *status = new bool[2];

    // initialize point info
    // first point is valid feature
    // second point is invalid feature
    fp_t x_1(197.0f);
    fp_t y_1(106.0f);
    fp_t x_2(50.0f);
    fp_t y_2(50.0f);
    prevPts[0] = Keypoint<fp_t>(x_1, y_1);
    nextPts[0] = Keypoint<fp_t>(x_1, y_1);
    prevPts[1] = Keypoint<fp_t>(x_2, y_2);
    nextPts[1] = Keypoint<fp_t>(x_2, y_2);

    // Initialize args
    int MAX_COUNT = 1;
    int DET_EPSILON = (int)(1<<20);
    float CRITERIA = 0.01;
    int num_good_points = 2;

    calcOpticalFlowPyrLK<NUM_LEVELS, DIM, DIM, WIN_DIM, fp_t, uint8_t>(prevPyramid, nextPyramid, 
                                                            prevPts, nextPts, status, 
                                                            num_good_points, MAX_COUNT, DET_EPSILON, CRITERIA);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].x.to_float(), 197.154f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].y.to_float(), 106.468f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].x.to_float(), 50.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].y.to_float(), 50.0f);

}
int main ( int argc, char ** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_lk_optical_flow_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_lk_optical_flow_pyramidal();

}
