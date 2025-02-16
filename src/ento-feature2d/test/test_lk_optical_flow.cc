#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-feature2d/fixed_point.h>
#include <ento-feature2d/raw_image.h>
#include <ento-feature2d/image_pyramid.h>

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

void write_png_image(const char* imagepath, RawImage* image) {
    if (!imagepath || !image || !image->data) {
        cerr << "Invalid input to write_png_image!" << endl;
        return;
    }

    FILE *fp = fopen(imagepath, "wb");
    if (!fp) {
        cerr << "Error opening file for writing: " << imagepath << endl;
        return;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        cerr << "Failed to create PNG write structure!" << endl;
        fclose(fp);
        return;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        cerr << "Failed to create PNG info structure!" << endl;
        png_destroy_write_struct(&png, nullptr);
        fclose(fp);
        return;
    }

    if (setjmp(png_jmpbuf(png))) {
        cerr << "Error during PNG creation!" << endl;
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return;
    }

    png_init_io(png, fp);

    // Set PNG metadata
    png_set_IHDR(png, info, image->width, image->height, 8, // 8-bit depth
                 PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png, info);

    // Write image data row-by-row
    png_bytep* row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * image->height);
    for (uint16_t i = 0; i < image->height; ++i) {
        row_pointers[i] = image->data + (i * image->width);
    }

    png_write_image(png, row_pointers);
    png_write_end(png, nullptr);

    // Cleanup
    free(row_pointers);
    png_destroy_write_struct(&png, &info);
    fclose(fp);

    // cout << "PNG saved successfully: " << imagepath << endl;
}

void test_image()
{
    string str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
    const char* image_path = str.c_str();
    string dst_str = "//Users/acui21/Documents/brg/ento-bench/build/native/src/ento-feature2d/bin/image_2.png";
    const char* dst_image_path = dst_str.c_str();
    RawImage* image = new_image(320, 320);
    if (!image) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(image_path, image);
    write_png_image(dst_image_path, image);
    del_raw_image(image);
}

void test_calc_gradient_basic()
{
    // initialize window size and point info
    int WIN_DIM = 1;
    int halfWin = WIN_DIM / 2;
    fp_t x(197.0f);
    fp_t y(106.0f);
    PointFP prevPt = PointFP(x,y);

    // initialize gradient arrays 
    short Ix_arr[(WIN_DIM+1)*(WIN_DIM+1)];
    short Iy_arr[(WIN_DIM+1)*(WIN_DIM+1)];

    // initialize image 
    string str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
    const char* image_path = str.c_str();
    RawImage* image = new_image(320, 320);
    if (!image) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(image_path, image);

    string dst_str = "//Users/acui21/Documents/brg/ento-bench/build/native/src/ento-feature2d/bin/test_gradient.png";
    const char* dst_image_path = dst_str.c_str();
    write_png_image(dst_image_path, image);

    calc_gradient(prevPt, *image, Ix_arr, Iy_arr, WIN_DIM+1, halfWin);

    short Ix_arr_golden[4] = {-4, -90,  -5, -86};
    short Iy_arr_golden[4] = {51,  52, -47, -55};

    ENTO_TEST_CHECK_ARRAY_INT_EQ(Ix_arr, Ix_arr_golden, 4);
    ENTO_TEST_CHECK_ARRAY_INT_EQ(Iy_arr, Iy_arr_golden, 4);

    del_raw_image(image);
}

void test_interpolate_basic() {
    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Ix_win_square;
    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Iy_win_square;
    int WIN_DIM = 1;
    short Ix_arr[4] = {-4, -90,  -5, -86};
    short Iy_arr[4] = {51,  52, -47, -55};
    fp_t x(197.5f);
    fp_t y(106.5f);
    PointFP prevPt = PointFP(x,y);
    interpolate<short>(prevPt, Ix_arr, Ix_win_square, WIN_DIM+1, WIN_DIM+1, WIN_DIM);
    interpolate<short>(prevPt, Iy_arr, Iy_win_square, WIN_DIM+1, WIN_DIM+1, WIN_DIM);

    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Ix_win_square_golden;
    Ix_win_square_golden(0,0) = fp_t(-46.25f);
    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Iy_win_square_golden;
    Iy_win_square_golden(0,0) = fp_t(0.25f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(Ix_win_square(0,0).to_float(), -46.25f);
    ENTO_TEST_CHECK_FLOAT_EQ(Iy_win_square(0,0).to_float(), 0.25f);
}

void test_a_transpose_a() {
    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Ix_win_square;
    Ix_win_square.setZero();
    Ix_win_square.block<2,2>(0,0) << fp_t(120), fp_t(100), 
                                     fp_t(100), fp_t(80);
    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Iy_win_square;
    Iy_win_square.setZero();
    Iy_win_square.block<2,2>(0,0) << fp_t(80), fp_t(100), 
                                     fp_t(100), fp_t(120);
    Eigen::Matrix<fp_t, 2, 2> ATA_inv;

    fp_t result = a_transpose_a(Ix_win_square, Iy_win_square, ATA_inv, 2, (int)(1<<20));

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
    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Ix_win_square;
    Ix_win_square.setZero();
    Ix_win_square.block<2,2>(0,0) << fp_t(1), fp_t(2), 
                                     fp_t(3), fp_t(4);
    Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Iy_win_square;
    Iy_win_square.setZero();
    Iy_win_square.block<2,2>(0,0) << fp_t(5), fp_t(6), 
                                     fp_t(7), fp_t(8);
    Eigen::Matrix<fp_t, 2, 2> ATA_inv;

    fp_t result = a_transpose_a(Ix_win_square, Iy_win_square, ATA_inv, 2, (int)(1<<20));

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
    PointFP *prevPts = new PointFP[2];
    PointFP *nextPts = new PointFP[2];
    bool *status_simple = new bool[2];

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

    // initialize prevImg 
    string prev_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
    const char* prev_image_path = prev_str.c_str();
    RawImage* prevImg = new_image(320, 320);
    if (!prevImg) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(prev_image_path, prevImg);

    string next_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_3.png";
    const char* next_image_path = next_str.c_str();
    RawImage* nextImg = new_image(320, 320);
    if (!nextImg) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(next_image_path, nextImg);

    // Initialize args
    int WIN_DIM = 3; 
    int MAX_COUNT = 1;
    int DET_EPSILON = (int)(1<<20);
    float CRITERIA = 0.01;

    calcOpticalFlowPyrLKSimpleIter(*prevImg, *nextImg, 
                            prevPts, nextPts, status_simple, 2, 
                            WIN_DIM, MAX_COUNT, 
                            DET_EPSILON, CRITERIA);

    ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[0]).to_float(), 197.195f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[0]).to_float(), 106.579f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[1]).to_float(), 50.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[1]).to_float(), 50.0f);

    del_raw_image(prevImg);
    del_raw_image(nextImg);
    delete [] prevPts;
    delete [] nextPts;
    delete [] status_simple;

}


// 1 layer, 1 iter, 1 point
void test_lk_optical_flow_iter() {
    // initialize point arrays
    PointFP *prevPts = new PointFP[2];
    PointFP *nextPts = new PointFP[2];
    bool *status_simple = new bool[2];

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

    // initialize prevImg 
    string prev_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
    const char* prev_image_path = prev_str.c_str();
    RawImage* prevImg = new_image(320, 320);
    if (!prevImg) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(prev_image_path, prevImg);

    string next_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_3.png";
    const char* next_image_path = next_str.c_str();
    RawImage* nextImg = new_image(320, 320);
    if (!nextImg) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(next_image_path, nextImg);

    // Initialize args
    int WIN_DIM = 3; 
    int MAX_COUNT = 5;
    int DET_EPSILON = (int)(1<<20);
    float CRITERIA = 0.01;

    calcOpticalFlowPyrLKSimpleIter(*prevImg, *nextImg, 
                            prevPts, nextPts, status_simple, 2, 
                            WIN_DIM, MAX_COUNT, 
                            DET_EPSILON, CRITERIA);

    ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[0]).to_float(), 197.157f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[0]).to_float(), 106.466f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[1]).to_float(), 50.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[1]).to_float(), 50.0f);

    del_raw_image(prevImg);
    del_raw_image(nextImg);
    delete [] prevPts;
    delete [] nextPts;
    delete [] status_simple;

}

void test_lk_optical_flow_pyramidal() {
    // initialize prevImg 
    string prev_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
    const char* prev_image_path = prev_str.c_str();
    RawImage* prevImg = new_image(320, 320);
    if (!prevImg) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(prev_image_path, prevImg);

    string next_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_3.png";
    const char* next_image_path = next_str.c_str();
    RawImage* nextImg = new_image(320, 320);
    if (!nextImg) {
        cerr << "Failed to allocate RawImage!" << endl;
        return;
    }
    read_png_image(next_image_path, nextImg);

    // initialize point arrays
    PointFP *prevPts = new PointFP[2];
    PointFP *nextPts = new PointFP[2];
    bool *status_simple = new bool[2];

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


    int NUM_LEVELS = 2;
    prevPyramid = new ImagePyramid(prevImg, 320, 320, NUM_LEVELS);
    nextPyramid = new ImagePyramid(nextImg, 320, 320, NUM_LEVELS);

    // Initialize args
    int WIN_DIM = 3; 
    int MAX_COUNT = 1;
    int DET_EPSILON = (int)(1<<20);
    float CRITERIA = 0.01;

    calcOpticalFlowPyrLKPyr(prevPts, nextPts, status_simple, 2, 2, WIN_DIM, MAX_COUNT, DET_EPSILON, CRITERIA);
    ENTO_TEST_CHECK_FLOAT_EQ(get<0>(nextPts[0]).to_float(), 197.176f);
    ENTO_TEST_CHECK_FLOAT_EQ(get<1>(nextPts[0]).to_float(), 106.503f);
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

  if (__ento_test_num(__n, 1)) test_image();
  if (__ento_test_num(__n, 2)) test_calc_gradient_basic();
  if (__ento_test_num(__n, 3)) test_interpolate_basic();
  if (__ento_test_num(__n, 4)) test_a_transpose_a();
  if (__ento_test_num(__n, 5)) test_transpose_a_nonsingular();
  if (__ento_test_num(__n, 6)) test_lk_optical_flow_simple();
  if (__ento_test_num(__n, 7)) test_lk_optical_flow_iter();
  if (__ento_test_num(__n, 8)) test_lk_optical_flow_pyramidal();
}
