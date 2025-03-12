#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-feature2d/fixed_point.h>
#include <ento-feature2d/feat2d_util.h>

#include <ento-feature2d/lk_optical_flow_iter.h>
#include <ento-util/unittest.h>
#include <iostream>
#include <vector>
#include <png.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoFeature2D;

const int decimal_bits = 20;
using fp_t = FixedPoint<64-decimal_bits, decimal_bits, int64_t>;

void test_calc_gradient_basic()
{
    // initialize window size and point info
    constexpr size_t WIN_DIM = (size_t) 2;
    int halfWin = 0;
    fp_t x(197.0f);
    fp_t y(106.0f);
    Keypoint<fp_t> prevPt(x,y);

    // initialize gradient arrays 
    int Ix_arr[(WIN_DIM+1)*(WIN_DIM+1)];
    int Iy_arr[(WIN_DIM+1)*(WIN_DIM+1)];

    // initialize image 
    string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
    const char* prev_image_path = top_str.c_str();
    constexpr size_t DIM = (size_t) 320;
    Image<DIM, DIM, uint8_t> image;
    ENTO_TEST_CHECK_INT_EQ(image.image_from_pgm(prev_image_path), 1);

    calc_gradient<DIM, DIM, WIN_DIM, fp_t, uint8_t>(prevPt, image, Ix_arr, Iy_arr, halfWin);

    int Ix_arr_golden[4] = {-4, -90,  -5, -86};
    int Iy_arr_golden[4] = {51,  52, -47, -55};

    ENTO_TEST_CHECK_ARRAY_INT_EQ(Ix_arr, Ix_arr_golden, 4);
    ENTO_TEST_CHECK_ARRAY_INT_EQ(Iy_arr, Iy_arr_golden, 4);

}

void test_interpolate_basic() {
    constexpr size_t WIN_DIM = 1;
    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Ix_win_square;
    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Iy_win_square;
    short Ix_arr[4] = {-4, -90,  -5, -86};
    short Iy_arr[4] = {51,  52, -47, -55};
    fp_t x(197.5f);
    fp_t y(106.5f);
    Keypoint<fp_t> prevPt(x,y);
    interpolate<short, WIN_DIM, fp_t>(prevPt, Ix_arr, Ix_win_square, WIN_DIM+1, WIN_DIM+1);
    interpolate<short, WIN_DIM, fp_t>(prevPt, Iy_arr, Iy_win_square, WIN_DIM+1, WIN_DIM+1);

    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Ix_win_square_golden;
    Ix_win_square_golden(0,0) = fp_t(-46.25f);
    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Iy_win_square_golden;
    Iy_win_square_golden(0,0) = fp_t(0.25f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(Ix_win_square(0,0).to_float(), -46.25f);
    ENTO_TEST_CHECK_FLOAT_EQ(Iy_win_square(0,0).to_float(), 0.25f);
}

void test_a_transpose_a() {
    constexpr size_t WIN_DIM = 2;
    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Ix_win_square;
    Ix_win_square.setZero();
    Ix_win_square.block<2,2>(0,0) << fp_t(120), fp_t(100), 
                                     fp_t(100), fp_t(80);
    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Iy_win_square;
    Iy_win_square.setZero();
    Iy_win_square.block<2,2>(0,0) << fp_t(80), fp_t(100), 
                                     fp_t(100), fp_t(120);
    Eigen::Matrix<fp_t, 2, 2> ATA_inv;

    fp_t result = a_transpose_a<WIN_DIM, fp_t>(Ix_win_square, Iy_win_square, ATA_inv, (int)(1<<20));

    int IxIx = 120*120+100*100+100*100+80*80;
    int IyIy = 120*120+100*100+100*100+80*80;
    int IxIy = 120*80+100*100+100*100+80*120;
    int det = IxIx * IyIy - IxIy * IxIy;
    ENTO_TEST_CHECK_FLOAT_EQ(result.to_float(), 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(0,0).to_float(), ((float) IxIx)/((float) det));
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(0,1).to_float(), -((float) IxIy)/((float) det));
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(1,0).to_float(), -((float) IxIy)/((float) det));
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(1,1).to_float(), ((float) IyIy)/((float) det));

}

void test_transpose_a_nonsingular() {
    constexpr size_t WIN_DIM = 2;
    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Ix_win_square;
    Ix_win_square.setZero();
    Ix_win_square.block<2,2>(0,0) << fp_t(1), fp_t(2), 
                                     fp_t(3), fp_t(4);
    Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Iy_win_square;
    Iy_win_square.setZero();
    Iy_win_square.block<2,2>(0,0) << fp_t(5), fp_t(6), 
                                     fp_t(7), fp_t(8);
    Eigen::Matrix<fp_t, 2, 2> ATA_inv;

    fp_t result = a_transpose_a<WIN_DIM, fp_t>(Ix_win_square, Iy_win_square, ATA_inv, (int)(1<<20));

    int IxIx = 1*1+2*2+3*3+4*4;
    int IyIy = 5*5+6*6+7*7+8*8;
    int IxIy = 1*5+2*6+3*7+4*8;
    int det = IxIx * IyIy - IxIy * IxIy;
    ENTO_TEST_CHECK_FLOAT_EQ(result.to_float(), (float) det);
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(0,0).to_float(), 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(0,1).to_float(), 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(1,0).to_float(), 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(ATA_inv(1,1).to_float(), 0.0f);
}

// 1 layer, 1 iter, 1 point
void test_lk_optical_flow_simple() {
    // initialize point arrays
    Keypoint<fp_t> *prevPts = new Keypoint<fp_t>[2];
    Keypoint<fp_t> *nextPts = new Keypoint<fp_t>[2];
    bool *status_simple = new bool[2];

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

    string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
    const char* prev_image_path = top_str.c_str();
    constexpr size_t DIM = (size_t) 320;
    Image<DIM, DIM, uint8_t> prevImg;
    ENTO_TEST_CHECK_INT_EQ(prevImg.image_from_pgm(prev_image_path), 1);

    string next_str = "/Users/acui21/Documents/brg/pgm_images/image_3.pgm";
    const char* next_image_path = next_str.c_str();
    Image<DIM, DIM, uint8_t> nextImg;
    ENTO_TEST_CHECK_INT_EQ(nextImg.image_from_pgm(next_image_path), 1);


    // Initialize args
    constexpr size_t WIN_DIM = 3; 
    int MAX_COUNT = 1;
    int DET_EPSILON = (int)(1<<20);
    float CRITERIA = 0.01;

    calcOpticalFlowIterLK<DIM, DIM, WIN_DIM, fp_t>(prevImg, nextImg, 
                            prevPts, nextPts, status_simple, 2, 
                            MAX_COUNT, 
                            DET_EPSILON, CRITERIA);

    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].x.to_float(), 197.156f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].y.to_float(), 106.464f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].x.to_float(), 50.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].y.to_float(), 50.0f);

    delete [] prevPts;
    delete [] nextPts;
    delete [] status_simple;

}


// 1 layer, 1 iter, 1 point
void test_lk_optical_flow_iter() {
    // initialize point arrays
    Keypoint<fp_t> *prevPts = new Keypoint<fp_t>[2];
    Keypoint<fp_t> *nextPts = new Keypoint<fp_t>[2];
    bool *status_simple = new bool[2];

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

    string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
    const char* prev_image_path = top_str.c_str();
    constexpr size_t DIM = (size_t) 320;
    Image<DIM, DIM, uint8_t> prevImg;
    ENTO_TEST_CHECK_INT_EQ(prevImg.image_from_pgm(prev_image_path), 1);

    string next_str = "/Users/acui21/Documents/brg/pgm_images/image_3.pgm";
    const char* next_image_path = next_str.c_str();
    Image<DIM, DIM, uint8_t> nextImg;
    ENTO_TEST_CHECK_INT_EQ(nextImg.image_from_pgm(next_image_path), 1);

    // Initialize args
    constexpr size_t WIN_DIM = 3; 
    int MAX_COUNT = 5;
    int DET_EPSILON = (int)(1<<20);
    float CRITERIA = 0.01;

    calcOpticalFlowIterLK<DIM, DIM, WIN_DIM, fp_t>(prevImg, nextImg, 
                            prevPts, nextPts, status_simple, 2, 
                            MAX_COUNT, 
                            DET_EPSILON, CRITERIA);

    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].x.to_float(), 197.157f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[0].y.to_float(), 106.466f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].x.to_float(), 50.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(nextPts[1].y.to_float(), 50.0f);

    delete [] prevPts;
    delete [] nextPts;
    delete [] status_simple;

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

  if (__ento_test_num(__n, 1)) test_calc_gradient_basic();
  if (__ento_test_num(__n, 2)) test_interpolate_basic();
  if (__ento_test_num(__n, 3)) test_a_transpose_a();
  if (__ento_test_num(__n, 4)) test_transpose_a_nonsingular();
  if (__ento_test_num(__n, 5)) test_lk_optical_flow_simple();
  if (__ento_test_num(__n, 6)) test_lk_optical_flow_iter();
}
