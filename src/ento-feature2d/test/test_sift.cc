#include <cstdio>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <image_io/Image.h>
#include <ento-feature2d/sift.h>

// Include the new SIFT++ golden reference data
extern "C" {
#include "blob_32x32_test_ascii_test_sift_data.h"
}

using namespace EntoUtil;
using namespace EntoFeature2D;

// uint8_t image data
uint8_t impulse_raw_7x7[1][7][7] = {
  {
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 255, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
  },

};
// float format
float golden_Gaussians_raw_7x7[5][7][7] = {
  {
    { 0.173300, 0.803051, 1.511501, 1.845267, 1.511501, 0.803051, 0.173300 },
    { 0.803051, 3.721239, 7.004106, 8.550736, 7.004106, 3.721239, 0.803051 },
    { 1.511501, 7.004106, 13.183112, 16.094176, 13.183112, 7.004106, 1.511501 },
    { 1.845267, 8.550736, 16.094176, 19.648053, 16.094176, 8.550736, 1.845267 },
    { 1.511501, 7.004106, 13.183112, 16.094176, 13.183112, 7.004106, 1.511501 },
    { 0.803051, 3.721239, 7.004106, 8.550736, 7.004106, 3.721239, 0.803051 },
    { 0.173300, 0.803051, 1.511501, 1.845267, 1.511501, 0.803051, 0.173300 },
  },

  {
    { 2.366861, 3.267854, 4.953280, 5.758116, 4.953280, 3.267854, 2.366861 },
    { 3.267854, 4.511828, 6.838846, 7.950058, 6.838846, 4.511828, 3.267854 },
    { 4.953280, 6.838846, 10.366044, 12.050375, 10.366044, 6.838846, 4.953280 },
    { 5.758116, 7.950058, 12.050375, 14.008385, 12.050375, 7.950058, 5.758116 },
    { 4.953280, 6.838846, 10.366044, 12.050375, 10.366044, 6.838846, 4.953280 },
    { 3.267854, 4.511828, 6.838846, 7.950058, 6.838846, 4.511828, 3.267854 },
    { 2.366861, 3.267854, 4.953280, 5.758116, 4.953280, 3.267854, 2.366861 },
  },

  {
    { 4.499940, 5.073371, 6.218442, 6.790977, 6.218442, 5.073371, 4.499940 },
    { 5.073371, 5.719875, 7.010864, 7.656356, 7.010864, 5.719875, 5.073371 },
    { 6.218442, 7.010864, 8.593231, 9.384413, 8.593231, 7.010864, 6.218442 },
    { 6.790977, 7.656356, 9.384413, 10.248439, 9.384413, 7.656356, 6.790977 },
    { 6.218442, 7.010864, 8.593231, 9.384413, 8.593231, 7.010864, 6.218442 },
    { 5.073371, 5.719875, 7.010864, 7.656356, 7.010864, 5.719875, 5.073371 },
    { 4.499940, 5.073371, 6.218442, 6.790977, 6.218442, 5.073371, 4.499940 },
  },

  {
    { 5.935647, 6.209894, 6.758382, 7.032684, 6.758382, 6.209894, 5.935647 },
    { 6.209894, 6.496812, 7.070642, 7.357618, 7.070642, 6.496812, 6.209894 },
    { 6.758382, 7.070642, 7.695156, 8.007479, 7.695156, 7.070642, 6.758382 },
    { 7.032684, 7.357618, 8.007479, 8.332478, 8.007479, 7.357618, 7.032684 },
    { 6.758382, 7.070642, 7.695156, 8.007479, 7.695156, 7.070642, 6.758382 },
    { 6.209894, 6.496812, 7.070642, 7.357618, 7.070642, 6.496812, 6.209894 },
    { 5.935647, 6.209894, 6.758382, 7.032684, 6.758382, 6.209894, 5.935647 },
  },

  {
    { 6.660326, 6.764453, 6.972697, 7.076818, 6.972697, 6.764453, 6.660326 },
    { 6.764453, 6.870209, 7.081708, 7.187457, 7.081708, 6.870209, 6.764453 },
    { 6.972697, 7.081708, 7.299717, 7.408722, 7.299717, 7.081708, 6.972697 },
    { 7.076818, 7.187457, 7.408722, 7.519355, 7.408722, 7.187457, 7.076818 },
    { 6.972697, 7.081708, 7.299717, 7.408722, 7.299717, 7.081708, 6.972697 },
    { 6.764453, 6.870209, 7.081708, 7.187457, 7.081708, 6.870209, 6.764453 },
    { 6.660326, 6.764453, 6.972697, 7.076818, 6.972697, 6.764453, 6.660326 },
  },

};

// float format
float golden_DoGs_raw_7x7[4][7][7] = {
  {
    { 2.193561, 2.464803, 3.441779, 3.912848, 3.441779, 2.464803, 2.193561 },
    { 2.464803, 0.790590, -0.165261, -0.600678, -0.165261, 0.790590, 2.464803 },
    { 3.441779, -0.165261, -2.817068, -4.043801, -2.817068, -0.165261, 3.441779 },
    { 3.912848, -0.600678, -4.043801, -5.639668, -4.043801, -0.600678, 3.912848 },
    { 3.441779, -0.165261, -2.817068, -4.043801, -2.817068, -0.165261, 3.441779 },
    { 2.464803, 0.790590, -0.165261, -0.600678, -0.165261, 0.790590, 2.464803 },
    { 2.193561, 2.464803, 3.441779, 3.912848, 3.441779, 2.464803, 2.193561 },
  },

  {
    { 2.133079, 1.805517, 1.265162, 1.032861, 1.265162, 1.805517, 2.133079 },
    { 1.805517, 1.208047, 0.172018, -0.293702, 0.172018, 1.208047, 1.805517 },
    { 1.265162, 0.172018, -1.772813, -2.665962, -1.772813, 0.172018, 1.265162 },
    { 1.032861, -0.293702, -2.665962, -3.759946, -2.665962, -0.293702, 1.032861 },
    { 1.265162, 0.172018, -1.772813, -2.665962, -1.772813, 0.172018, 1.265162 },
    { 1.805517, 1.208047, 0.172018, -0.293702, 0.172018, 1.208047, 1.805517 },
    { 2.133079, 1.805517, 1.265162, 1.032861, 1.265162, 1.805517, 2.133079 },
  },

  {
    { 1.435707, 1.136523, 0.539940, 0.241708, 0.539940, 1.136523, 1.435707 },
    { 1.136523, 0.776937, 0.059779, -0.298738, 0.059779, 0.776937, 1.136523 },
    { 0.539940, 0.059779, -0.898075, -1.376934, -0.898075, 0.059779, 0.539940 },
    { 0.241708, -0.298738, -1.376934, -1.915962, -1.376934, -0.298738, 0.241708 },
    { 0.539940, 0.059779, -0.898075, -1.376934, -0.898075, 0.059779, 0.539940 },
    { 1.136523, 0.776937, 0.059779, -0.298738, 0.059779, 0.776937, 1.136523 },
    { 1.435707, 1.136523, 0.539940, 0.241708, 0.539940, 1.136523, 1.435707 },
  },

  {
    { 0.724679, 0.554559, 0.214314, 0.044134, 0.214314, 0.554559, 0.724679 },
    { 0.554559, 0.373396, 0.011065, -0.170161, 0.011065, 0.373396, 0.554559 },
    { 0.214314, 0.011065, -0.395439, -0.598756, -0.395439, 0.011065, 0.214314 },
    { 0.044134, -0.170161, -0.598756, -0.813123, -0.598756, -0.170161, 0.044134 },
    { 0.214314, 0.011065, -0.395439, -0.598756, -0.395439, 0.011065, 0.214314 },
    { 0.554559, 0.373396, 0.011065, -0.170161, 0.011065, 0.373396, 0.554559 },
    { 0.724679, 0.554559, 0.214314, 0.044134, 0.214314, 0.554559, 0.724679 },
  },

};

template <typename ImageT>
void copy_into_img(ImageT& img,
                   const typename ImageT::pixel_type_ buff[ImageT::rows_][ImageT::cols_])
{
  for (int y = 0; y < ImageT::rows_; ++y)
    for (int x = 0; x < ImageT::cols_; ++x)
      img(y, x) = buff[y][x];
}


void test_dog_octave_float_init_and_step()
{
  constexpr int H = 32, W = 32;
  constexpr int NumDoGLayers = 3;
  using PixelT = uint8_t;
  using DoGPixelT = float;
  using ImgT = Image<H, W, PixelT>;
  using DoGImageT = Image<H, W, DoGPixelT>;

  // Use the actual 32x32 blob input image
  ImgT input_img;
  copy_into_img(input_img, img[0]);

  // Load golden DoG values that we should match (from SIFT++ reference)
  DoGImageT golden_dogs[NumDoGLayers + 2];
  for (int i = 0; i < NumDoGLayers + 2; i++)
    copy_into_img(golden_dogs[i], golden_DoGs_octave0[i]);

  // Print input image for verification
  ENTO_DEBUG("\n=== INPUT IMAGE ===");
  
  // Print golden DoG center values for reference
  ENTO_DEBUG("\n=== GOLDEN DOG CENTER VALUES ===");
  ENTO_DEBUG("Golden DoG[0] center (16,16): %f", golden_dogs[0](16, 16));
  ENTO_DEBUG("Golden DoG[1] center (16,16): %f", golden_dogs[1](16, 16));
  ENTO_DEBUG("Golden DoG[2] center (16,16): %f", golden_dogs[2](16, 16));

  SIFTDoGOctave<ImgT, DoGPixelT, NumDoGLayers> octave(input_img);

  // Initialize by computing DoG layers 0, 1, 2
  octave.initialize();

  auto dog_triplet0 = octave.get_current_DoG_triplet();
  
  // Print computed DoG center values for comparison
  ENTO_DEBUG("\n=== COMPUTED DOG CENTER VALUES ===");
  ENTO_DEBUG("Computed DoG[0] center (16,16): %f", dog_triplet0.prev_image()(16, 16));
  ENTO_DEBUG("Computed DoG[1] center (16,16): %f", dog_triplet0.curr_image()(16, 16));
  ENTO_DEBUG("Computed DoG[2] center (16,16): %f", dog_triplet0.next_image()(16, 16));
  
  // Test only the first DoG with a reasonable tolerance
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet0.prev_image(), golden_dogs[0], 0.001f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet0.curr_image(), golden_dogs[1], 0.001f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet0.next_image(), golden_dogs[2], 0.001f);

  // Step once, shifting DoG ring and computing the next layer (3)
  bool more = octave.step();
  ENTO_TEST_CHECK_TRUE(more);  // Should be false since we only have 3 DoG layers
  ENTO_DEBUG("After step: more = %i", more);
  auto dog_triplet1 = octave.get_current_DoG_triplet();

  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet1.prev_image(), golden_dogs[1], 0.001f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet1.curr_image(), golden_dogs[2], 0.001f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet1.next_image(), golden_dogs[3], 0.001f);

  more = octave.step(); 
  ENTO_DEBUG("After step: more = %i", more);
  auto dog_triplet2 = octave.get_current_DoG_triplet();

  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet2.prev_image(), golden_dogs[2], 0.01f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet2.curr_image(), golden_dogs[3], 0.01f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog_triplet2.next_image(), golden_dogs[4], 0.01f);

}

void test_extrema_on_blob()
{
  constexpr int H = 32;
  constexpr int W = 32;
  constexpr int NumDoGLayers = 3;
  using PixelT = uint8_t;
  using DoGPixelT = float;
  using ImgT = Image<H, W, PixelT>;
  using DoGImageT = Image<H, W, DoGPixelT>;
  using DoGTripletT = DoGTriplet<DoGImageT>;
  using KeypointT = SIFTKeypoint<float>;
  FeatureArray<KeypointT, 1> feats{};

  ImgT input_img;
  copy_into_img(input_img, img[0]);
  auto driver = SIFTDriver<ImgT, 1, KeypointT>(input_img, feats);
  
  // Move golden_dogs declaration BEFORE octave to change stack layout
  DoGImageT golden_dogs[3];
  for (int i = 0; i < 3; i++) {
    copy_into_img(golden_dogs[i], golden_DoGs_octave0[i]);
  }

  SIFTDoGOctave<Image<H, W, uint8_t>, DoGPixelT, 3> octave(input_img);
  octave.initialize();
  
  driver.detect_extrema_in_triplet(octave.get_current_DoG_triplet(), 0, 0);
  
  octave.step();

  driver.detect_extrema_in_triplet(octave.get_current_DoG_triplet(), 1, 0);

  octave.step();
  
  driver.detect_extrema_in_triplet(octave.get_current_DoG_triplet(), 2, 0);
  
  
  //// Create a simple kernel and blur the test image
  //auto simple_kernel = make_gaussian_kernel<float, 5>(1.0f);
  //DoGImageT blurred_img;
  //gaussian_blur<DoGImageT, DoGImageT, 5>(test_img, blurred_img, simple_kernel);
  
  //ENTO_DEBUG("After simple blur test: golden_dogs[0](0,0) = %f", golden_dogs[0](0, 0));

  DoGTripletT dog3 = octave.get_current_DoG_triplet();
  
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog3.prev_image(), golden_dogs[0], 0.001f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog3.curr_image(), golden_dogs[1], 0.001f);
  ENTO_TEST_CHECK_IMAGE_EQ_TOL(dog3.next_image(), golden_dogs[2], 0.001f);

  // Run extrema detection and capture results


  driver.detect_extrema_in_triplet(octave.get_current_DoG_triplet(), 1, 0);

  ENTO_DEBUG("\nStepping octave! \n");
  bool more = octave.step();
  ENTO_DEBUG("More? %i", more);
  dog3 = octave.get_current_DoG_triplet();

  ENTO_DEBUG_IMAGE(make_centered_view(dog3.prev_image(), 16, 16, 32, 32));
  ENTO_DEBUG_IMAGE(make_centered_view(dog3.curr_image(), 16, 16, 32, 32));
  ENTO_DEBUG_IMAGE(make_centered_view(dog3.next_image(), 16, 16, 32, 32));

  driver.detect_extrema_in_triplet(octave.get_current_DoG_triplet(), 2, 0);

  // Check result vs gt_kp
  ENTO_TEST_CHECK_INT_EQ(feats.size(), 1);
  ENTO_DEBUG("Feat: %f, %f, %f", feats[0].x, feats[0].y, feats[0].response);
  //ENTO_TEST_CHECK_APPROX_EQ(feats[0].x, static_cast<int>(gt_kp.x), 1.0f);
  //ENTO_TEST_CHECK_APPROX_EQ(feats[0].y, static_cast<int>(gt_kp.y), 1.0f);
  //ENTO_TEST_CHECK_APPROX_EQ(feats[0].response, gt_kp.response, 0.01f);
}

void test_single_blur_comparison()
{
  printf("=== TEST 3: SINGLE BLUR COMPARISON (FRESH SIFT++ DATA) ===\n");
  
  constexpr int H = 32;
  constexpr int W = 32;
  using PixelT = uint8_t;
  using DoGPixelT = float;
  using ImgT = Image<H, W, PixelT>;
  using DoGImageT = Image<H, W, DoGPixelT>;

  // Load the fresh input image (clean 0-255 values from SIFT++)
  ImgT input_img;
  copy_into_img(input_img, img[0]);
  
  printf("Fresh input image center (16,16): %d\n", static_cast<int>(input_img(16, 16)));
  printf("Fresh input image peak value: %d\n", static_cast<int>(input_img(15, 15))); // Peak should be 251
  
  // Convert to float and normalize to 0-1 range (as SIFT++ does internally)
  DoGImageT input_normalized;
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {
      input_normalized(i, j) = static_cast<float>(input_img(i, j)) / 255.0f;
    }
  }
  
  printf("Normalized input center (16,16): %f\n", input_normalized(16, 16));
  printf("Normalized input peak: %f\n", input_normalized(15, 15)); // Should be ~0.984
  
  // Load expected Gaussian results from fresh SIFT++ data
  DoGImageT expected_G_minus1, expected_G0, expected_G1;
  copy_into_img(expected_G_minus1, golden_Gaussians_octave0[0]); // Level -1
  copy_into_img(expected_G0, golden_Gaussians_octave0[1]);       // Level 0  
  copy_into_img(expected_G1, golden_Gaussians_octave0[2]);       // Level 1
  
  printf("Expected G(-1) center: %f\n", expected_G_minus1(16, 16));
  printf("Expected G(0) center: %f\n", expected_G0(16, 16));
  printf("Expected G(1) center: %f\n", expected_G1(16, 16));
  
  // STEP 0: Apply initial pre-blur (SIFT++ assumes input has sigma=0.5, targets sigma=1.6)
  printf("--- Testing initial pre-blur: sigma=%.6f ---\n", sqrtf(1.6f*1.6f - 0.5f*0.5f));
  
  DoGImageT pre_blurred;
  const float initial_sigma = sqrtf(1.6f*1.6f - 0.5f*0.5f); // ≈ 1.519868 or 1.249 depending on SIFT++ version
  
  printf("Pre-blur sigma: %.6f\n", initial_sigma);
  printf("Using SIFT++ exact smooth method\n");
  
  sift_smooth_efficient<DoGImageT, 17>(input_normalized, pre_blurred, initial_sigma);
  
  printf("Pre-blurred center: %f\n", pre_blurred(16, 16));
  
  // STEP 1: Test initial smoothing (sigma = 1.6, level -1) using SIFT++ exact method
  printf("--- Testing first level (G-1): this should match the pre-blurred result ---\n");
  
  // In SIFT++, G(-1) is the result of the initial pre-blur, so it should match our pre_blurred
  DoGImageT our_G_minus1 = pre_blurred; // G(-1) is just the pre-blurred image
  
  printf("Our G(-1) is the pre-blurred image\n");
  
  printf("Our G(-1) center: %f\n", our_G_minus1(16, 16));
  printf("Diff from SIFT++: %f\n", our_G_minus1(16, 16) - expected_G_minus1(16, 16));
  
  // STEP 2: Test first inter-level smoothing (G0 = blur(G-1) with delta sigma)
  printf("--- Testing inter-level smoothing (G0): delta_sigma=1.226273 ---\n");
  
  DoGImageT our_G0;
  const float delta_sigma_0 = 1.226273f; // sqrt((1.6*k)^2 - 1.6^2) where k=2^(1/3)
  
  printf("Our delta sigma: %.6f\n", delta_sigma_0);
  printf("Using SIFT++ exact smooth method\n");
  
  sift_smooth_efficient<DoGImageT, 17>(our_G_minus1, our_G0, delta_sigma_0);
  
  printf("Our G(0) center: %f\n", our_G0(16, 16));
  printf("Diff from SIFT++: %f\n", our_G0(16, 16) - expected_G0(16, 16));
  
  // STEP 3: Test second inter-level smoothing (G1 = blur(G0) with delta sigma)
  printf("--- Testing inter-level smoothing (G1): delta_sigma=1.545008 ---\n");
  
  DoGImageT our_G1;
  const float delta_sigma_1 = 1.545008f; // sqrt((1.6*k^2)^2 - (1.6*k)^2)
  
  printf("Our delta sigma: %.6f\n", delta_sigma_1);
  printf("Using SIFT++ exact smooth method\n");
  
  sift_smooth_efficient<DoGImageT, 17>(our_G0, our_G1, delta_sigma_1);
  
  printf("Our G(1) center: %f\n", our_G1(16, 16));
  printf("Diff from SIFT++: %f\n", our_G1(16, 16) - expected_G1(16, 16));
  
  // STEP 4: Test DoG computation (using SIFT++ correct indexing)
  printf("--- Testing DoG computation (SIFT++ indexing) ---\n");
  
  DoGImageT our_DoG0, our_DoG1;
  
  // SIFT++ DoG indexing: DoG0 = G(1) - G(0), DoG1 = G(2) - G(1)
  // DoG0 = G1 - G0 (SIFT++ level 0)
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {
      our_DoG0(i, j) = our_G1(i, j) - our_G0(i, j);
    }
  }
  
  // We need G2 for DoG1 = G2 - G1, so let's compute G2
  DoGImageT our_G2;
  const float delta_sigma_2 = 1.946588f; // sqrt((1.6*k^3)^2 - (1.6*k^2)^2)
  
  printf("Computing G2 with delta_sigma=%.6f\n", delta_sigma_2);
  sift_smooth_efficient<DoGImageT, 17>(our_G1, our_G2, delta_sigma_2);
  
  // DoG1 = G2 - G1 (SIFT++ level 1)  
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {
      our_DoG1(i, j) = our_G2(i, j) - our_G1(i, j);
    }
  }
  
  // Load expected DoG values
  DoGImageT expected_DoG0, expected_DoG1;
  copy_into_img(expected_DoG0, golden_DoGs_octave0[0]);
  copy_into_img(expected_DoG1, golden_DoGs_octave0[1]);
  
  printf("Our DoG0 center: %f\n", our_DoG0(16, 16));
  printf("Expected DoG0 center: %f\n", expected_DoG0(16, 16));
  printf("DoG0 diff: %f\n", our_DoG0(16, 16) - expected_DoG0(16, 16));
  
  printf("Our DoG1 center: %f\n", our_DoG1(16, 16));
  printf("Expected DoG1 center: %f\n", expected_DoG1(16, 16));
  printf("DoG1 diff: %f\n", our_DoG1(16, 16) - expected_DoG1(16, 16));
  
  // Summary
  printf("--- SUMMARY ---\n");
  printf("G(-1) error: %.6f\n", fabs(our_G_minus1(16, 16) - expected_G_minus1(16, 16)));
  printf("G(0) error: %.6f\n", fabs(our_G0(16, 16) - expected_G0(16, 16)));
  printf("G(1) error: %.6f\n", fabs(our_G1(16, 16) - expected_G1(16, 16)));
  printf("DoG0 error: %.6f\n", fabs(our_DoG0(16, 16) - expected_DoG0(16, 16)));
  printf("DoG1 error: %.6f\n", fabs(our_DoG1(16, 16) - expected_DoG1(16, 16)));
}

void test_python_gaussian_comparison()
{
  printf("=== TEST 4: PYTHON GAUSSIAN COMPARISON ===\n");
  
  constexpr int H = 32;
  constexpr int W = 32;
  using PixelT = uint8_t;
  using DoGPixelT = float;
  using ImgT = Image<H, W, PixelT>;
  using DoGImageT = Image<H, W, DoGPixelT>;

  // Load the input image
  ImgT input_img;
  copy_into_img(input_img, img[0]);
  
  // Convert to float - NO NORMALIZATION since input is already normalized
  DoGImageT input_float;
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {
      input_float(i, j) = static_cast<float>(input_img(i, j));
    }
  }
  
  // Apply gaussian blur with sigma=1.519868
  DoGImageT result_gaussian_blur;
  const float sigma = 1.519868f;
  auto kernel = make_gaussian_kernel<float, 15>(sigma);
  
  gaussian_blur<DoGImageT, DoGImageT, 15>(input_float, result_gaussian_blur, kernel);
  
  // Output input image for Python comparison
  printf("INPUT_IMAGE = [\n");
  for (int i = 0; i < H; i++) {
    printf("  [");
    for (int j = 0; j < W; j++) {
      printf("%.6f", input_float(i, j));
      if (j < W-1) printf(", ");
    }
    printf("]");
    if (i < H-1) printf(",");
    printf("\n");
  }
  printf("]\n\n");
  
  // Output our gaussian blur result for Python comparison  
  printf("OUR_RESULT = [\n");
  for (int i = 0; i < H; i++) {
    printf("  [");
    for (int j = 0; j < W; j++) {
      printf("%.6f", result_gaussian_blur(i, j));
      if (j < W-1) printf(", ");
    }
    printf("]");
    if (i < H-1) printf(",");
    printf("\n");
  }
  printf("]\n\n");
  
  printf("SIGMA = %.6f\n", sigma);
  printf("Center result: %.6f\n", result_gaussian_blur(16, 16));
  printf("Expected (SIFT++): 0.958898\n");
  printf("Diff: %.6f\n", result_gaussian_blur(16, 16) - 0.958898f);
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
  if (__ento_test_num(__n, 3)) test_single_blur_comparison();
  if (__ento_test_num(__n, 4)) test_python_gaussian_comparison();
  
  ENTO_TEST_END();
}
