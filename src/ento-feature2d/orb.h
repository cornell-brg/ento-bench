#ifndef FEATURE2D_ORB_H
#define FEATURE2D_ORB_H

#include <math.h>
#include <stdint.h>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/fast.h>
#include <ento-feature2d/brief.h>
#include <ento-feature2d/image_pyramid.h>
#include <image_io/image_util.h>

namespace EntoFeature2D
{

template <typename ImageT,
          int Threshold,
          size_t MaxFeatures,
          size_t NumLevels = 1,  // Number of pyramid levels
          typename KeypointT = ORBKeypoint<int16_t, float>>
void orb(const ImageT& img, 
         FeatureArray<KeypointT, MaxFeatures>& fdo,
         std::array<BRIEFDescriptor, MaxFeatures>& descriptors);

template <typename Image, typename KeypointType, size_t PatchSize = 31>
BRIEFDescriptor compute_rotated_brief_descriptor(const Image& img, const KeypointType& kp, float angle);


template <typename ImageT, size_t MaxFeatures, int PatchSize = 31, typename KeypointT = ORBKeypoint<int16_t, float>>
void compute_keypoint_orientation(const ImageT& img,
                                  FeatureArray<KeypointT, MaxFeatures>& feats);

template <typename Image, typename KeypointType, size_t PatchSize = 31, size_t MaxFeatures = 100>
void compute_orb_descriptors(const Image& img,
                             const FeatureArray<KeypointType, MaxFeatures>& feats,
                             std::array<BRIEFDescriptor, MaxFeatures>& descriptors);



// ========================================
// Implementations
//
template <typename Image, typename KeypointType, size_t PatchSize>
BRIEFDescriptor compute_rotated_brief_descriptor(const Image& img,
                                                 const KeypointType& kp,
                                                 float angle)
{
  constexpr int DESC_SIZE = 256;
  BRIEFDescriptor descriptor;
  descriptor.data.fill(0);  // Initialize with zeros

  float cos_a = std::cos(angle);
  float sin_a = std::sin(angle);

  for (size_t i = 0; i < DESC_SIZE; ++i)
  {
    // Original offsets from the keypoint center
    float dx1 = bit_pattern_31[i].x1;
    float dy1 = bit_pattern_31[i].y1;
    float dx2 = bit_pattern_31[i].x2;
    float dy2 = bit_pattern_31[i].y2;

    // Rotated offsets
    float rx1 = dx1 * cos_a - dy1 * sin_a;
    float ry1 = dx1 * sin_a + dy1 * cos_a;
    float rx2 = dx2 * cos_a - dy2 * sin_a;
    float ry2 = dx2 * sin_a + dy2 * cos_a;

    // Rotated absolute positions
    int x1 = static_cast<int>(std::round(kp.x + rx1));
    int y1 = static_cast<int>(std::round(kp.y + ry1));
    int x2 = static_cast<int>(std::round(kp.x + rx2));
    int y2 = static_cast<int>(std::round(kp.y + ry2));

    // Ensure points are within bounds
    if (x1 < 0 || x1 >= img.cols || y1 < 0 || y1 >= img.rows ||
        x2 < 0 || x2 >= img.cols || y2 < 0 || y2 >= img.rows)
    {
      continue;
    }

    // Compute intensity values
    uint8_t orig_val1 = img(static_cast<int>(kp.y + dy1), static_cast<int>(kp.x + dx1));
    uint8_t orig_val2 = img(static_cast<int>(kp.y + dy2), static_cast<int>(kp.x + dx2));
    uint8_t rot_val1 = img(y1, x1);
    uint8_t rot_val2 = img(y2, x2);

    bool orig_bit = orig_val1 < orig_val2;
    bool rot_bit  = rot_val1 < rot_val2;

    ENTO_DEBUG("Keypoint (%d, %d) | Angle: %.5f | i=%zu", kp.x, kp.y, angle, i);
    ENTO_DEBUG(" - Original pattern: (%.1f, %.1f) = %u vs (%.1f, %.1f) = %u | Bit: %d",
      kp.y + dy1, kp.x + dx1, orig_val1, kp.y + dy2, kp.x + dx2, orig_val2, static_cast<int>(orig_bit));
    ENTO_DEBUG(" - Rotated pattern:  (%d, %d) = %u vs (%d, %d) = %u | Bit: %d",
      y1, x1, rot_val1, y2, x2, rot_val2, static_cast<int>(rot_bit));


    descriptor.set_bit(i, rot_bit);
  }

  return descriptor;
}

template <typename Image, typename KeypointType, size_t PatchSize, size_t MaxFeatures>
void compute_orb_descriptors(const Image& img,
                             const FeatureArray<KeypointType, MaxFeatures>& feats,
                             std::array<BRIEFDescriptor, MaxFeatures>& descriptors)
{
  for (size_t i = 0; i < feats.size(); ++i)
  {
    const auto& kp = feats[i];

    if (kp.x < PatchSize / 2 || kp.y < PatchSize / 2 ||
        kp.x >= img.cols - PatchSize / 2 || kp.y >= img.rows - PatchSize / 2)
    {
      descriptors[i].data.fill(0);  // Zero out if out of bounds
      continue;
    }

    // Use already computed keypoint orientation
    float angle = kp.orientation;

    // Compute rotated BRIEF descriptor
    descriptors[i] = compute_rotated_brief_descriptor(img, kp, angle);
  }
}

template <typename ImageT, typename KeypointT, int PatchSize = 31>
float compute_keypoint_orientation(const ImageT& img, const KeypointT& kp)
{
  using PixelType = typename ImageT::pixel_type;
  constexpr int PatchHalf = PatchSize / 2;

  if (kp.x < PatchHalf || kp.y < PatchHalf ||
      kp.x >= img.cols - PatchHalf || kp.y >= img.rows - PatchHalf)
  {
    return 0;
  }

  int32_t m00 = 0, m10 = 0, m01 = 0;

  for (int dy = -PatchHalf; dy <= PatchHalf; dy++)
  {
    for (int dx = -PatchHalf; dx <= PatchHalf; dx++)
    {
      int x = kp.x + dx;
      int y = kp.y + dy;

      if (x >= 0 && x < img.cols && y >= 0 && y < img.rows)
      {
        PixelType pixel_value = img(y, x); // Image() operatore is row, col
        m00 += pixel_value;
        m10 += dx * pixel_value;
        m01 += dy * pixel_value;
      }
    }
  }
  ENTO_DEBUG("m00=%d, m10=%d, m01=%d", m00, m10, m01);


  if (m00 != 0)
  {
    float cx = static_cast<float>(m10) / static_cast<float>(m00);
    float cy = static_cast<float>(m01) / static_cast<float>(m00);
    return std::atan2(cy, cx);
  }
  else
  {
    return 0;
  }
}

template <typename ImageT, size_t MaxFeatures, int PatchSize, typename KeypointT>
void compute_keypoint_orientation(const ImageT& img,
                                  FeatureArray<KeypointT, MaxFeatures>& feats)
{
  ENTO_DEBUG("Computing Keypoint orientations on found features...");
  ENTO_DEBUG("feats size: %i", feats.size());
  
  for (size_t i = 0; i < feats.size(); i++)  
  {
    feats[i].orientation = compute_keypoint_orientation<ImageT, KeypointT, PatchSize>(img, feats[i]);
  }
}

template <typename ImageT, typename KeypointT, int MaxFeatures, int Threshold, size_t... Is>
void run_fast_on_pyramid_levels(const ImagePyramid<sizeof...(Is)-1, ImageT::cols, ImageT::rows, typename ImageT::pixel_type>& pyramid,
                                FeatureArray<KeypointT, /*MaxFeatures*/ 100>& feature_array,
                                std::index_sequence<Is...>)
{
  (fast<ImageT, KeypointT, 16, Threshold, 9, MaxFeatures, true, true>(
     std::get<Is>(pyramid.pyramid), feature_array), ...);
}


template <typename ImageT,
          int Threshold,
          size_t MaxFeatures,
          size_t NumLevels,  // Number of pyramid levels
          typename KeypointT>
void orb(ImageT& img, 
         FeatureArray<KeypointT, MaxFeatures>& feature_array,
         std::array<BRIEFDescriptor, MaxFeatures>& descriptors)
{
  // 1. Construct Image Pyramid if necessary
  if constexpr (NumLevels > 1)
  {
    ImagePyramid<NumLevels - 1, ImageT::rows, ImageT::cols, typename ImageT::pixel_type> pyramid(img);
    pyramid.set_top_image(img);
    pyramid.initialize_pyramid();

    run_fast_on_pyramid_levels<ImageT, KeypointT>(
        pyramid, feature_array, std::make_index_sequence<NumLevels>{}
    );
  }
  else
  {
     fast<ImageT,
          KeypointT,
          16,
          Threshold,
          9,
          MaxFeatures,
          true,
          true>(img, feature_array);   

  }

  // 3. Compute Keypoint Orientations
  compute_keypoint_orientation(img, feature_array);

  // 4. Compute Rotated BRIEF Descriptors
  using KernelScalarT = float;
  constexpr int KernelSize = 7;
  gaussian_blur_in_place<ImageT, KernelSize, KernelScalarT>(img);
  compute_orb_descriptors(img, feature_array, descriptors);
}

template <size_t MaxFeatures,
          int Threshold = 10,
          size_t NumLevels = 1>
struct ORBKernel
{
  using KeypointType = ORBKeypoint<int16_t, float>;
  using DescriptorType = BRIEFDescriptor;

  template <typename ImageT,
            typename KeypointT,
            typename DescriptorArray>
  void operator()(ImageT& img,
                  FeatureArray<KeypointT, MaxFeatures>& feats,
                  DescriptorArray& descs) const
  {
    orb<ImageT, Threshold, MaxFeatures, NumLevels, KeypointT>(img, feats, descs);
  }

  static constexpr const char* name() { return "ORB Kernel"; }
};

} // namespace EntoFeature2D



#endif // FEATURE2D_ORB_H
