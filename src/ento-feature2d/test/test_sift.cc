#include <cstdio>
#include <ento-feature2d/test/test_sift_blobs.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <image_io/Image.h>
#include <ento-feature2d/sift.h>

using namespace EntoUtil;
using namespace EntoFeature2D;


float golden_dogs_raw_7x7[4][7][7] = {
  {
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
    { 0.000000, 0.996094, 3.984375, 5.976562, 3.984375, 0.996094, 0.000000 },
    { 0.000000, 3.984375, 15.937500, 23.906250, 15.937500, 3.984375, 0.000000 },
    { 0.000000, 5.976562, 23.906250, -219.140625, 23.906250, 5.976562, 0.000000 },
    { 0.000000, 3.984375, 15.937500, 23.906250, 15.937500, 3.984375, 0.000000 },
    { 0.000000, 0.996094, 3.984375, 5.976562, 3.984375, 0.996094, 0.000000 },
    { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 },
  },

  {
    { 0.996094, 1.805420, 3.486328, 4.357910, 3.486328, 1.805420, 0.996094 },
    { 1.805420, 2.276230, 2.334595, 1.922150, 2.334595, 2.276230, 1.805420 },
    { 3.486328, 2.334595, -3.735352, -8.653564, -3.735352, 2.334595, 3.486328 },
    { 4.357910, 1.922150, -8.653564, -16.793518, -8.653564, 1.922150, 4.357910 },
    { 3.486328, 2.334595, -3.735352, -8.653564, -3.735352, 2.334595, 3.486328 },
    { 1.805420, 2.276230, 2.334595, 1.922150, 2.334595, 2.276230, 1.805420 },
    { 0.996094, 1.805420, 3.486328, 4.357910, 3.486328, 1.805420, 0.996094 },
  },

  {
    { 1.946468, 1.946347, 1.890535, 1.834846, 1.890535, 1.946347, 1.946468 },
    { 1.946347, 1.511179, 0.536531, -0.002949, 0.536531, 1.511179, 1.946347 },
    { 1.890535, 0.536531, -2.377152, -3.936832, -2.377152, 0.536531, 1.890535 },
    { 1.834846, -0.002949, -3.936832, -6.032921, -3.936832, -0.002949, 1.834846 },
    { 1.890535, 0.536531, -2.377152, -3.936832, -2.377152, 0.536531, 1.890535 },
    { 1.946347, 1.511179, 0.536531, -0.002949, 0.536531, 1.511179, 1.946347 },
    { 1.946468, 1.946347, 1.890535, 1.834846, 1.890535, 1.946347, 1.946468 },
  },

  {
    { 1.588558, 1.346262, 0.855501, 0.607036, 0.855501, 1.346262, 1.588558 },
    { 1.346262, 0.952363, 0.156623, -0.245219, 0.156623, 0.952363, 1.346262 },
    { 0.855501, 0.156623, -1.252641, -1.963026, -1.252641, 0.156623, 0.855501 },
    { 0.607036, -0.245219, -1.963026, -2.828577, -1.963026, -0.245219, 0.607036 },
    { 0.855501, 0.156623, -1.252641, -1.963026, -1.252641, 0.156623, 0.855501 },
    { 1.346262, 0.952363, 0.156623, -0.245219, 0.156623, 0.952363, 1.346262 },
    { 1.588558, 1.346262, 0.855501, 0.607036, 0.855501, 1.346262, 1.588558 },
  },
};

uint8_t impulse_raw_7x7[7][7] = {
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 255, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
};

template <typename ImageT>
void copy_into_img(ImageT& img,
                   const typename ImageT::pixel_type buff[ImageT::rows][ImageT::cols])
{
  for (int y = 0; y < ImageT::rows; ++y)
    for (int x = 0; x < ImageT::cols; ++x)
      img(y, x) = buff[y][x];
}


void test_dog_octave_float_init_and_step()
{
  constexpr int H = 7, W = 7;
  constexpr int NumDoGLayers = 5;
  using PixelT = uint8_t;
  using DoGPixelT = float;
  using ImgT = Image<H, W, PixelT>;
  using DoGImageT = Image<H, W, DoGPixelT>;

  ImgT input_img;
  copy_into_img(input_img, impulse_raw_7x7);

  DoGImageT golden_dogs[4];
  for (int i = 0; i < 4; i++)
    copy_into_img(golden_dogs[i], golden_dogs_raw_7x7[i]);

  SIFTDoGOctave<ImgT, DoGPixelT, NumDoGLayers> octave(input_img);

  // Initialize by computing DoG layers 0, 1, 2
  octave.initialize();

  auto dog_triplet0 = octave.get_current_DoG_triplet();
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet0.prev_image(), golden_dogs[0], 0.1f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet0.curr_image(), golden_dogs[1], 0.1f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet0.next_image(), golden_dogs[2], 0.1f);

  // Step once, shifting DoG ring and computing the next layer (3)
  bool more = octave.step();
  ENTO_TEST_CHECK_TRUE(more);

  auto dog_triplet1 = octave.get_current_DoG_triplet();
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet1.prev_image(), golden_dogs[1], 0.1f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet1.curr_image(), golden_dogs[2], 0.1f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet1.next_image(), golden_dogs[3], 0.1f);

  // No more octaves to process, more should equal false
  more = octave.step();
  ENTO_TEST_CHECK_FALSE(more);
}

void test_extrema_on_blob()
{
  constexpr int H = 32;
  constexpr int W = 32;
  constexpr int NumDoGLayers = 5;
  using PixelT = uint8_t;
  using DoGPixelT = float;
  using ImgT = Image<H, W, PixelT>;
  using DoGImageT = Image<H, W, DoGPixelT>;
  using KeypointT = SIFTKeypoint<float>;
  

  ImgT input_img;
  copy_into_img(input_img, blur_raw_32x32);

  SIFTDoGOctave<Image<H, W, uint8_t>, DoGPixelT, 5> octave(input_img);
  octave.initialize();

  FeatureArray<KeypointT, 1> feats{};
  auto driver = SIFTDriver<ImgT, 1, KeypointT>(input_img, feats);

  driver.detect_extrema_in_triplet(octave.get_current_DoG_triplet(), 1, 0);

  // Check result vs gt_kp
  ENTO_TEST_CHECK_INT_EQ(feats.size(), 1);
  //ENTO_TEST_CHECK_APPROX_EQ(feats[0].x, static_cast<int>(gt_kp.x), 1.0f);
  //ENTO_TEST_CHECK_APPROX_EQ(feats[0].y, static_cast<int>(gt_kp.y), 1.0f);
  //ENTO_TEST_CHECK_APPROX_EQ(feats[0].response, gt_kp.response, 0.01f);
}


//------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ENTO_TEST_START();

  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_dog_octave_float_init_and_step(); 
  if (__ento_test_num(__n, 2)) test_extrema_on_blob(); 
  
  ENTO_TEST_END();
}
