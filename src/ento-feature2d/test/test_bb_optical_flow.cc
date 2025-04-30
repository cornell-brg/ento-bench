#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <math/FixedPoint.hh>
#include <ento-feature2d/feat2d_util.h>

#include <ento-feature2d/bb_optical_flow.h>
#include <ento-util/unittest.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoFeature2D;

void test_bb_optical_flow_basic_y() {
    // initialize point arrays
    Keypoint<float> *prevPts = new Keypoint<float>[2];

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

    string next_str = "/Users/acui21/Documents/brg/ento-bench/src/ento-feature2d/test/image_2_shifted_y.pgm";
    const char* next_image_path = next_str.c_str();
    Image<DIM, DIM, uint8_t> nextImg;
    ENTO_TEST_CHECK_INT_EQ(nextImg.image_from_pgm(next_image_path), 1);

    constexpr int WIN_DIM = 25; 
    // int DET_EPSILON = (int)(1<<20);
    int DET_EPSILON = 0;

    int ref_motion = 7;
    int num_good_points = 2;
    constexpr int search_area = 1;


    Keypoint<int> flow = calcOpticalFlowBB<DIM, DIM, WIN_DIM, float, uint8_t, search_area, int>(prevImg, nextImg,
        prevPts, num_good_points);

    ENTO_TEST_CHECK_INT_EQ(flow.x, 0);
    ENTO_TEST_CHECK_INT_EQ(flow.y, 1);

    delete [] prevPts;

}

void test_bb_optical_flow_basic_x() {
    // initialize point arrays
    Keypoint<float> *prevPts = new Keypoint<float>[2];

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

    string next_str = "/Users/acui21/Documents/brg/ento-bench/src/ento-feature2d/test/image_2_shifted_x.pgm";
    const char* next_image_path = next_str.c_str();
    Image<DIM, DIM, uint8_t> nextImg;
    ENTO_TEST_CHECK_INT_EQ(nextImg.image_from_pgm(next_image_path), 1);

    constexpr int WIN_DIM = 25; 
    // int DET_EPSILON = (int)(1<<20);
    int DET_EPSILON = 0;

    int ref_motion = 7;
    int num_good_points = 2;
    constexpr int search_area = 1;


    Keypoint<int> flow = calcOpticalFlowBB<DIM, DIM, WIN_DIM, float, uint8_t, search_area, int>(prevImg, nextImg,
        prevPts, num_good_points);

    ENTO_TEST_CHECK_INT_EQ(flow.x, 1);
    ENTO_TEST_CHECK_INT_EQ(flow.y, 0);

    delete [] prevPts;

}

void test_bb_optical_flow_basic_xy() {
    // initialize point arrays
    Keypoint<float> *prevPts = new Keypoint<float>[2];

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

    string next_str = "/Users/acui21/Documents/brg/ento-bench/src/ento-feature2d/test/image_2_shifted_xy.pgm";
    const char* next_image_path = next_str.c_str();
    Image<DIM, DIM, uint8_t> nextImg;
    ENTO_TEST_CHECK_INT_EQ(nextImg.image_from_pgm(next_image_path), 1);

    constexpr int WIN_DIM = 25; 
    // int DET_EPSILON = (int)(1<<20);
    int DET_EPSILON = 0;

    int ref_motion = 7;
    int num_good_points = 2;
    constexpr int search_area = 1;


    Keypoint<int> flow = calcOpticalFlowBB<DIM, DIM, WIN_DIM, float, uint8_t, search_area, int>(prevImg, nextImg,
        prevPts, num_good_points);

    ENTO_TEST_CHECK_INT_EQ(flow.x, 1);
    ENTO_TEST_CHECK_INT_EQ(flow.y, 1);

    delete [] prevPts;

}

void test_bb_optical_flow_subpixel() {
    // initialize point arrays
    Keypoint<float> *prevPts = new Keypoint<float>[2];

    // initialize point info
    // first point is valid feature
    // second point is invalid feature
    float x_1(197.0f);
    float y_1(106.0f);
    // float x_2(207.0f);
    // float y_2(110.0f);
    prevPts[0] = Keypoint<float>(x_1, y_1);
    // prevPts[1] = Keypoint<float>(x_2, y_2);

    string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
    const char* prev_image_path = top_str.c_str();
    constexpr int DIM = 320;
    Image<DIM, DIM, uint8_t> prevImg;
    ENTO_TEST_CHECK_INT_EQ(prevImg.image_from_pgm(prev_image_path), 1);

    string next_str = "/Users/acui21/Documents/brg/ento-bench/src/ento-feature2d/test/image_3.pgm";
    const char* next_image_path = next_str.c_str();
    Image<DIM, DIM, uint8_t> nextImg;
    ENTO_TEST_CHECK_INT_EQ(nextImg.image_from_pgm(next_image_path), 1);

    constexpr int WIN_DIM = 25; 
    // int DET_EPSILON = (int)(1<<20);
    int DET_EPSILON = 0;

    int ref_motion = 7;
    int num_good_points = 1;
    constexpr int search_area = 1;


    Keypoint<int> flow = calcOpticalFlowBB<DIM, DIM, WIN_DIM, float, uint8_t, search_area, int>(prevImg, nextImg,
        prevPts, num_good_points);

    ENTO_TEST_CHECK_INT_EQ(flow.x, 0.0);
    ENTO_TEST_CHECK_INT_EQ(flow.y, 0.5);

    delete [] prevPts;

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
    __ento_replace_file_suffix(__FILE__, "test_lk_optical_flow_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_bb_optical_flow_basic_y();
  if (__ento_test_num(__n, 2)) test_bb_optical_flow_basic_x();
  if (__ento_test_num(__n, 3)) test_bb_optical_flow_basic_xy();
  if (__ento_test_num(__n, 4)) test_bb_optical_flow_subpixel();
  ENTO_TEST_END();
}
