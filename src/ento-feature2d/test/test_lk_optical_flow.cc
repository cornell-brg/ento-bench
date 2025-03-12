#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-feature2d/fixed_point.h>
#include <ento-feature2d/raw_image.h>
#include <ento-feature2d/image_pyramid_template.h>

#include <ento-feature2d/lk_optical_flow.h>
#include <ento-util/unittest.h>
#include <iostream>
#include <vector>
#include <png.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;

//////////////////
// DEBUG MESSAGES 
//////////////////
void printArray(const short arr[], int size) {
    for (int i = 0; i < size; ++i) {
        cout << "Ix_arr[" << i << "] = " << arr[i] << endl;
    }
}

void printEigenMatrix(const Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM>& matrix) {
    std::cout << "Matrix contents:" << std::endl;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            std::cout << matrix(i, j).to_float() << " ";  // Print each element
        }
        std::cout << std::endl;  // New line after each row
    }
}

void printEigenMatrix_2(const Eigen::Matrix<fp_t, 2, 2>& matrix) {
    std::cout << "Matrix contents:" << std::endl;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            std::cout << matrix(i, j).to_float() << " ";  // Print each element
        }
        std::cout << std::endl;  // New line after each row
    }
}

// void test_lk_optical_flow_pyramidal() {
//     // initialize prevImg 
//     string prev_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
//     const char* prev_image_path = prev_str.c_str();
//     RawImage* prevImg = new_image(320, 320);
//     if (!prevImg) {
//         cerr << "Failed to allocate RawImage!" << endl;
//         return;
//     }
//     read_png_image(prev_image_path, prevImg);

//     string next_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_3.png";
//     const char* next_image_path = next_str.c_str();
//     RawImage* nextImg = new_image(320, 320);
//     if (!nextImg) {
//         cerr << "Failed to allocate RawImage!" << endl;
//         return;
//     }
//     read_png_image(next_image_path, nextImg);

//     // initialize point arrays
//     PointFP *prevPts = new PointFP[2];
//     PointFP *nextPts = new PointFP[2];
//     bool *status = new bool[2];

//     // initialize point info
//     // first point is valid feature
//     // second point is invalid feature
//     fp_t x_1(197.0f);
//     fp_t y_1(106.0f);
//     fp_t x_2(50.0f);
//     fp_t y_2(50.0f);
//     prevPts[0] = PointFP(x_1, y_1);
//     nextPts[0] = PointFP(x_1, y_1);
//     prevPts[1] = PointFP(x_2, y_2);
//     nextPts[1] = PointFP(x_2, y_2);


//     int NUM_LEVELS = 2;
//     ImagePyramid* prevPyramid = new ImagePyramid(prevImg, 320, 320, NUM_LEVELS);
//     ImagePyramid* nextPyramid = new ImagePyramid(nextImg, 320, 320, NUM_LEVELS);
//     prevPyramid->create_pyramids();
//     nextPyramid->create_pyramids();
    
//     // Initialize args
//     int WIN_DIM = 3; 
//     int MAX_COUNT = 1;
//     int DET_EPSILON = (int)(1<<20);
//     float CRITERIA = 0.01;

//     calcOpticalFlowPyrLK(prevPyramid, nextPyramid, prevPts, nextPts, status, 2, 2, WIN_DIM, MAX_COUNT, DET_EPSILON, CRITERIA);
//     ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[0]).to_float(), 197.154f);
//     ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[0]).to_float(), 106.468f);
//     ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[1]).to_float(), 50.0f);
//     ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[1]).to_float(), 50.0f);
// }


void test_lk_optical_flow_pyramidal() {

    constexpr size_t WIN_DIM = 3;
    // initialize prevImg 
    string prev_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
    const char* prev_image_path = prev_str.c_str();
    constexpr size_t DIM = (size_t) 320;
    RawImage<DIM, DIM> prevImg;
    read_png_image<DIM, DIM>(prev_image_path, prevImg);

    constexpr size_t NUM_LEVELS = 1;
    ImagePyramid<NUM_LEVELS, 320, 320> prevPyramid(prevImg);
    // Print the types of each level in the pyramid:
    prevPyramid.initialize_pyramid();

    // initialize prevImg 
    string next_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_3.png";
    const char* next_image_path = next_str.c_str();
    RawImage<DIM, DIM> nextImg;
    read_png_image<DIM, DIM>(next_image_path, nextImg);

    ImagePyramid<NUM_LEVELS, 320, 320> nextPyramid(nextImg);
    // Print the types of each level in the pyramid:
    nextPyramid.initialize_pyramid();

    // initialize point arrays
    PointFP *prevPts = new PointFP[2];
    PointFP *nextPts = new PointFP[2];
    bool *status = new bool[2];

    // initialize point info
    // first point is valid feature
    // second point is invalid feature
    fp_t x_1(197.0f);
    fp_t y_1(106.0f);
    fp_t x_2(50.0f);
    fp_t y_2(50.0f);
    prevPts[0] = PointFP(x_1, y_1);
    nextPts[0] = PointFP(x_1, y_1);
    prevPts[1] = PointFP(x_2, y_2);
    nextPts[1] = PointFP(x_2, y_2);

    // Initialize args
    int MAX_COUNT = 1;
    int DET_EPSILON = (int)(1<<20);
    float CRITERIA = 0.01;
    int num_good_points = 2;

    calcOpticalFlowPyrLK<NUM_LEVELS, DIM, DIM, WIN_DIM>(prevPyramid, nextPyramid, 
                                                            prevPts, nextPts, status, 
                                                            num_good_points, MAX_COUNT, DET_EPSILON, CRITERIA);
    ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[0]).to_float(), 197.154f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[0]).to_float(), 106.468f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[1]).to_float(), 50.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[1]).to_float(), 50.0f);

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
