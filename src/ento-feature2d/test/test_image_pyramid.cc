#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-feature2d/image_pyramid.h>
#include <image_io/Image.h>
#include <ento-util/unittest.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;

// DEBUG MESSAGES
template <size_t IMG_WIDTH, size_t IMG_HEIGHT, typename PixelT>
void check_pyramid(const Image<IMG_WIDTH, IMG_HEIGHT, PixelT>& img, size_t Is) {
  // initialize oneImg 
  string str = "/Users/acui21/Documents/brg/pgm_images/gem5_pyr_prev_" + to_string(Is) + ".pgm";
  cout << str << endl;
  const char* image_path = str.c_str();
  Image<IMG_WIDTH, IMG_HEIGHT, uint8_t> golden_img;
  golden_img.image_from_pgm(image_path);

  ENTO_TEST_CHECK_ARRAY_INT_EQ(img.data, golden_img.data,  IMG_WIDTH * IMG_HEIGHT);
}

template <typename PyramidType, size_t TOP_WIDTH, size_t TOP_HEIGHT, typename PixelT, std::size_t... Is>
void test_pyramid_helper(PyramidType& pyramid, std::index_sequence<Is...>)
{
    // Use a fold expression to print each level’s type in one go.
    ((check_pyramid<(TOP_WIDTH >> Is), (TOP_HEIGHT >> Is), PixelT>(pyramid.template get_level<Is>(), Is)),
     ...);
}

void test_pyramid_zero_level() {
  // initialize image 
  string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
  const char* prev_image_path = top_str.c_str();
  constexpr size_t DIM = (size_t) 320;
  Image<DIM, DIM, uint8_t> topImg;
  ENTO_TEST_CHECK_INT_EQ(topImg.image_from_pgm(prev_image_path), 1);

  constexpr size_t NUM_LEVELS = 0;
  ImagePyramid<NUM_LEVELS, 320, 320, uint8_t> pyramid(topImg);

  using PyramidType = ImagePyramid<NUM_LEVELS, 320, 320, uint8_t>;
  // Print the types of each level in the pyramid:
  test_pyramid_helper<PyramidType, DIM, DIM, uint8_t>(
      pyramid,
      std::make_index_sequence<NUM_LEVELS+1>{}
  );
}

void test_pyramid_one_level() {
  // initialize image 
  string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
  const char* prev_image_path = top_str.c_str();
  constexpr size_t DIM = (size_t) 320;
  Image<DIM, DIM, uint8_t> topImg;
  ENTO_TEST_CHECK_INT_EQ(topImg.image_from_pgm(prev_image_path), 1);

  constexpr size_t NUM_LEVELS = 1;
  ImagePyramid<NUM_LEVELS, 320, 320, uint8_t> pyramid(topImg);

  using PyramidType = ImagePyramid<NUM_LEVELS, 320, 320, uint8_t>;
  test_pyramid_helper<PyramidType, DIM, DIM, uint8_t>(
      pyramid,
      std::make_index_sequence<NUM_LEVELS+1>{}
  );
}

void test_pyramid_three_level() {
  // initialize image 
  string top_str = "/Users/acui21/Documents/brg/pgm_images/image_2.pgm";
  const char* prev_image_path = top_str.c_str();
  constexpr size_t DIM = (size_t) 320;
  Image<DIM, DIM, uint8_t> topImg;

  constexpr size_t NUM_LEVELS = 2;
  ENTO_TEST_CHECK_INT_EQ(topImg.image_from_pgm(prev_image_path), 1);
  ImagePyramid<NUM_LEVELS, 320, 320, uint8_t> pyramid(topImg);
  using PyramidType = ImagePyramid<2, 320, 320, uint8_t>;
  test_pyramid_helper<PyramidType, DIM, DIM, uint8_t>(
      pyramid,
      std::make_index_sequence<3>{}
  );
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
    __ento_replace_file_suffix(__FILE__, "test_image_pyramid_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_pyramid_zero_level();
  if (__ento_test_num(__n, 2)) test_pyramid_one_level();
  if (__ento_test_num(__n, 3)) test_pyramid_three_level();
  ENTO_TEST_END();
}
