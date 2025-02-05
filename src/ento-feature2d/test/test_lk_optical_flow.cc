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

    cout << "PNG saved successfully: " << imagepath << endl;
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

int main(int argc, char **argv)
{
    test_image();
    test_calc_gradient_basic();
    test_interpolate_basic();

}