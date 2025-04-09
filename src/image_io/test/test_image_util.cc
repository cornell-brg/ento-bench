#include <cstdio>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <image_io/Image.h>
#include <image_io/image_util.h>

using namespace EntoUtil;

// Globals Variables (file paths)
const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 256;
char dir_path[FILEPATH_SIZE];
char img1_path[FILEPATH_SIZE];
char img2_path[FILEPATH_SIZE];

char* full_paths[] = { 
  img1_path, img2_path
}; 
constexpr size_t num_paths = 2;

template <typename ImageT>
void make_basic_image(ImageT& img)
{
  static constexpr int H = ImageT::rows;
  static constexpr int W = ImageT::cols;
  for (int y = 0; y < H; ++y)
  {
    for (int x = 0; x < W; ++x)
    {
      img(y, x) = 0;
    }
  }
  img(H/2, W/2) = 255;
}

template <typename ImageT>
void copy_into_img(ImageT& img,
                   typename ImageT::pixel_type buff[ImageT::rows][ImageT::cols])
{
  for (int y = 0; y < ImageT::rows; ++y)
    for (int x = 0; x < ImageT::cols; ++x)
      img(y, x) = buff[y][x];
}

template <typename ImgT, int KernelSize, typename KernelT = float>
void run_blur_in_place_test(typename ImgT::pixel_type b[ImgT::rows][ImgT::cols],
                            typename ImgT::pixel_type g[ImgT::rows][ImgT::cols])
{
  ImgT input;
  copy_into_img(input, b);
  gaussian_blur_in_place<ImgT, KernelSize, KernelT>(input);
  ImgT golden;
  copy_into_img(golden, g);
  ENTO_TEST_CHECK_IMAGE_EQ(input, golden);
  ENTO_DEBUG_IMAGE(input);
  ENTO_DEBUG_IMAGE(golden);
}


void test_gaussian_blur_in_place_impulse_7x7_3_uint8()
{
  constexpr int H = 7, W = 7, KSize = 3;
  using PixelT = uint8_t;
  using ImgT = Image<H, W, PixelT>;
  PixelT b[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 255, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  PixelT g[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 16, 32, 16, 0, 0 },
    { 0, 0, 32, 64, 32, 0, 0 },
    { 0, 0, 16, 32, 16, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  run_blur_in_place_test<ImgT, KSize>(b, g);
}

void test_gaussian_blur_in_place_impulse_7x7_3_float()
{
  constexpr int H = 7, W = 7, KSize = 3;
  using PixelT = float;
  using ImgT = Image<H, W, PixelT>;
  PixelT b[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 255, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  PixelT g[H][W] = {
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
    { 0.000000, 0.000000, 15.937500, 31.875000, 15.937500, 0.000000, 0.000000 },
    { 0.000000, 0.000000, 31.875000, 63.750000, 31.875000, 0.000000, 0.000000 },
    { 0.000000, 0.000000, 15.937500, 31.875000, 15.937500, 0.000000, 0.000000 },
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
  };

  run_blur_in_place_test<ImgT, KSize>(b, g);
}

void test_gaussian_blur_in_place_impulse_7x7_5_uint8()
{
  constexpr int H = 7, W = 7, KSize = 5;
  using PixelT = uint8_t;
  using ImgT = Image<H, W, PixelT>;
  PixelT b[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 255, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  PixelT g[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 4, 6, 4, 1, 0 },
    { 0, 4, 16, 24, 16, 4, 0 },
    { 0, 6, 24, 36, 24, 6, 0 },
    { 0, 4, 16, 24, 16, 4, 0 },
    { 0, 1, 4, 6, 4, 1, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  run_blur_in_place_test<ImgT, KSize>(b, g);

}

void test_gaussian_blur_in_place_impulse_7x7_5_float()
{
  constexpr int H = 7, W = 7, KSize = 5;
  using PixelT = float;
  using ImgT = Image<H, W, PixelT>;
  PixelT b[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 255, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  PixelT g[H][W] = {
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
    { 0.000000, 0.996094, 3.984375, 5.976562, 3.984375, 0.996094, 0.000000 },
    { 0.000000, 3.984375, 15.937500, 23.906250, 15.937500, 3.984375, 0.000000 },
    { 0.000000, 5.976562, 23.906250, 35.859375, 23.906250, 5.976562, 0.000000 },
    { 0.000000, 3.984375, 15.937500, 23.906250, 15.937500, 3.984375, 0.000000 },
    { 0.000000, 0.996094, 3.984375, 5.976562, 3.984375, 0.996094, 0.000000 },
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
  };

  run_blur_in_place_test<ImgT, KSize>(b, g);

}

void test_gaussian_blur_in_place_impulse_7x7_7_uint8()
{
  constexpr int H = 7, W = 7, KSize = 7;
  using PixelT = uint8_t;
  using ImgT = Image<H, W, PixelT>;
  PixelT b[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 255, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  PixelT g[H][W] = {
    { 1, 2, 3, 4, 3, 2, 1 },
    { 2, 3, 6, 8, 6, 3, 2 },
    { 3, 6, 12, 16, 12, 6, 3 },
    { 4, 8, 16, 20, 16, 8, 4 },
    { 3, 6, 12, 16, 12, 6, 3 },
    { 2, 3, 6, 8, 6, 3, 2 },
    { 1, 2, 3, 4, 3, 2, 1 },
  };

  run_blur_in_place_test<ImgT, KSize>(b, g);


}
void test_gaussian_blur_in_place_impulse_7x7_7_float()
{
  constexpr int H = 7, W = 7, KSize = 7;
  using PixelT = float;
  using ImgT = Image<H, W, PixelT>;
  PixelT b[H][W] = {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 255, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  };

  PixelT g[H][W] = {
    { 0.996094, 1.743164, 3.486328, 4.482422, 3.486328, 1.743164, 0.996094 },
    { 1.743164, 3.050537, 6.101074, 7.844238, 6.101074, 3.050537, 1.743164 },
    { 3.486328, 6.101074, 12.202148, 15.688477, 12.202148, 6.101074, 3.486328 },
    { 4.482422, 7.844238, 15.688477, 20.170898, 15.688477, 7.844238, 4.482422 },
    { 3.486328, 6.101074, 12.202148, 15.688477, 12.202148, 6.101074, 3.486328 },
    { 1.743164, 3.050537, 6.101074, 7.844238, 6.101074, 3.050537, 1.743164 },
    { 0.996094, 1.743164, 3.486328, 4.482422, 3.486328, 1.743164, 0.996094 },
  };

  run_blur_in_place_test<ImgT, KSize>(b, g);

}


int main ( int argc, char ** argv )
{

  //int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  // Run Tests
  if (__ento_test_num(__n, 1)) test_gaussian_blur_in_place_impulse_7x7_3_uint8();
  if (__ento_test_num(__n, 2)) test_gaussian_blur_in_place_impulse_7x7_3_float();
  if (__ento_test_num(__n, 3)) test_gaussian_blur_in_place_impulse_7x7_5_uint8();
  if (__ento_test_num(__n, 4)) test_gaussian_blur_in_place_impulse_7x7_5_float();
  if (__ento_test_num(__n, 5)) test_gaussian_blur_in_place_impulse_7x7_7_uint8();
  if (__ento_test_num(__n, 6)) test_gaussian_blur_in_place_impulse_7x7_7_float();
}


