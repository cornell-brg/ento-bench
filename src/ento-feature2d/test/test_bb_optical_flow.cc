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

template <int InRows, int InCols, typename PixelT>
Image<InRows / 2, InCols / 2, PixelT>
downsample_avg(const Image<InRows, InCols, PixelT>& input)
{
    static_assert(InRows % 2 == 0 && InCols % 2 == 0, "Image dimensions must be even for downsampling");

    Image<InRows / 2, InCols / 2, PixelT> output;

    for (int r = 0; r < InRows; r += 2) {
        for (int c = 0; c < InCols; c += 2) {
            int idx_out = (r / 2) * (InCols / 2) + (c / 2);
            int idx0 = r * InCols + c;
            int idx1 = r * InCols + (c + 1);
            int idx2 = (r + 1) * InCols + c;
            int idx3 = (r + 1) * InCols + (c + 1);

            uint32_t sum = static_cast<uint32_t>(input.data[idx0]) +
                           static_cast<uint32_t>(input.data[idx1]) +
                           static_cast<uint32_t>(input.data[idx2]) +
                           static_cast<uint32_t>(input.data[idx3]);

            output.data[idx_out] = static_cast<PixelT>(sum / 4);
        }
    }

    return output;
}

template <int InRows, int InCols, typename PixelT>
Image<InRows * 2, InCols * 2, PixelT>
upsample_copy(const Image<InRows, InCols, PixelT>& input)
{
    Image<InRows * 2, InCols * 2, PixelT> output;

    for (int r = 0; r < InRows; ++r) {
        for (int c = 0; c < InCols; ++c) {
            PixelT val = input.data[r * InCols + c];

            int r2 = r * 2;
            int c2 = c * 2;
            output.data[(r2) * (InCols * 2) + (c2)] = val;
            output.data[(r2) * (InCols * 2) + (c2 + 1)] = val;
            output.data[(r2 + 1) * (InCols * 2) + (c2)] = val;
            output.data[(r2 + 1) * (InCols * 2) + (c2 + 1)] = val;
        }
    }

    return output;
}

template <int Rows, int Cols, typename PixelT>
void shift_image(Image<Rows, Cols, PixelT>& img, int dir)
{   
    // right
    if (dir == 0) {
        for (int r = 0; r < Rows; ++r) {
            PixelT last = img.data[r * Cols + (Cols - 1)];
            for (int c = Cols - 1; c > 0; --c)
                img.data[r * Cols + c] = img.data[r * Cols + c - 1];
            img.data[r * Cols + 0] = last;
        }
    }
    // down 
    else if (dir == 1) {
        for (int c = 0; c < Cols; ++c) {
            PixelT last = img.data[(Rows - 1) * Cols + c];
            for (int r = Rows - 1; r > 0; --r)
                img.data[r * Cols + c] = img.data[(r - 1) * Cols + c];
            img.data[c] = last;
        }
    }
}

enum class HalfPixelDirectionX { None, PlusHalf, MinusHalf };
enum class HalfPixelDirectionY { None, PlusHalf, MinusHalf };

template <int Rows, int Cols, typename PixelT>
Image<Rows, Cols, PixelT> shift_half_pixel_bilinear(
    const Image<Rows, Cols, PixelT>& input,
    HalfPixelDirectionX dx,
    HalfPixelDirectionY dy)
{
    Image<Rows, Cols, PixelT> output;

    float offset_x = (dx == HalfPixelDirectionX::PlusHalf)  ?  0.5f :
                     (dx == HalfPixelDirectionX::MinusHalf) ? -0.5f : 0.0f;

    float offset_y = (dy == HalfPixelDirectionY::PlusHalf)  ?  0.5f :
                     (dy == HalfPixelDirectionY::MinusHalf) ? -0.5f : 0.0f;

    for (int y = 0; y < Rows; ++y) {
        for (int x = 0; x < Cols; ++x) {
            float src_x = x - offset_x;
            float src_y = y - offset_y;

            int x0 = static_cast<int>(std::floor(src_x));
            int y0 = static_cast<int>(std::floor(src_y));
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            float wx = src_x - x0;
            float wy = src_y - y0;

            float iw00((1.f - wx) * (1.f - wy));
            float iw01(wx*(1.f-wy));
            float iw10((1.f-wx)*wy);
            float iw11(1.f- iw00 - iw01 - iw10);

            auto get_pixel = [&](int r, int c) -> float {
                r = std::max(0, std::min(Rows - 1, r));
                c = std::max(0, std::min(Cols - 1, c));
                return static_cast<float>(input.data[r * Cols + c]);
            };

            float interp = iw00 *  get_pixel(y0, x0) + iw01 * get_pixel(y0, x1)
                            + iw10 * get_pixel(y1, x1)  + iw11 * get_pixel(y1, x1);

            if constexpr (std::is_integral<PixelT>::value)
                output.data[y * Cols + x] = static_cast<PixelT>(std::round(interp));
            else
                output.data[y * Cols + x] = static_cast<PixelT>(interp);
        }
    }

    return output;
}

void test_bb_optical_flow_basic_y_test() {
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

    Image<DIM, DIM, uint8_t> nextImg = shift_half_pixel_bilinear<DIM, DIM, uint8_t>(prevImg, HalfPixelDirectionX::PlusHalf, HalfPixelDirectionY::PlusHalf);
    shift_image<DIM, DIM, uint8_t>(nextImg, 0);
    shift_image<DIM, DIM, uint8_t>(nextImg, 1);

    constexpr int WIN_DIM = 8; 
    int num_good_points = 2;
    constexpr int search_area = 1;

    Keypoint<float> flow = calcOpticalFlowBB<DIM, DIM, WIN_DIM, float, uint8_t, search_area, float>(prevImg, nextImg,
        prevPts, num_good_points);

    ENTO_TEST_CHECK_FLOAT_EQ(flow.x, 1.5);
    ENTO_TEST_CHECK_FLOAT_EQ(flow.y, 1.5);

    delete [] prevPts;
}

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

    Image<DIM, DIM, uint8_t> nextImg = prevImg;
    shift_image<DIM, DIM, uint8_t>(nextImg, 1);

    constexpr int WIN_DIM = 8; 
    int num_good_points = 2;
    constexpr int search_area = 1;

    Keypoint<float> flow = calcOpticalFlowBB<DIM, DIM, WIN_DIM, float, uint8_t, search_area, float>(prevImg, nextImg,
        prevPts, num_good_points);

    ENTO_TEST_CHECK_FLOAT_EQ(flow.x, 0.0);
    ENTO_TEST_CHECK_FLOAT_EQ(flow.y, 1.0);

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
    __ento_replace_file_suffix(__FILE__, "test_bb_optical_flow_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_bb_optical_flow_basic_y_test();
  if (__ento_test_num(__n, 2)) test_bb_optical_flow_basic_y();

  ENTO_TEST_END();
} 