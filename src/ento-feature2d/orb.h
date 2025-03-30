#ifndef FEATURE2D_ORB_H
#define FEATURE2D_ORB_H

#include <math.h>
#include <stdint.h>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/brief.h>
#include <ento-feature2d/image_pyramid.h>

namespace EntoFeature2D
{

template <typename ImageType,
          int Threshold,
          size_t MaxFeatures,
          size_t NumLevels = 1,  // Number of pyramid levels
          typename KeypointT = ORBKeypoint<int16_t, float>>
void orb(const ImageType& img, 
         FeatureArray<KeypointT, MaxFeatures>& fdo,
         std::array<BRIEFDescriptor, MaxFeatures>& descriptors);

template <typename Image, typename KeypointType, size_t PatchSize = 31>
BRIEFDescriptor compute_rotated_brief_descriptor(const Image& img, const KeypointType& kp, float angle);


template <typename ImageType, size_t MaxFeatures, int PatchSize = 31, typename KeypointT = ORBKeypoint<int16_t, float>>
void compute_keypoint_orientation(const ImageType& img,
                                  FeatureArray<KeypointT, MaxFeatures>& feats);

template <typename Image, typename KeypointType, size_t PatchSize = 31, size_t MaxFeatures = 100>
void compute_orb_descriptors(const Image& img,
                             const FeatureArray<KeypointType, MaxFeatures>& feats,
                             std::array<BRIEFDescriptor, MaxFeatures>& descriptors);



// ========================================
// Implementations
//
template <typename Image, typename KeypointType, size_t PatchSize>
BRIEFDescriptor compute_rotated_brief_descriptor(const Image& img, const KeypointType& kp, float angle)
{
  constexpr int DESC_SIZE = 256;
  BRIEFDescriptor descriptor;
  descriptor.data.fill(0);  // Initialize with zeros

  float cos_a = std::cos(angle);
  float sin_a = std::sin(angle);

  for (size_t i = 0; i < DESC_SIZE; ++i)
  {
    // Rotate sample points
    int x1 = kp.x + (bit_pattern_31[i].x1 * cos_a - bit_pattern_31[i].y1 * sin_a);
    int y1 = kp.y + (bit_pattern_31[i].x1 * sin_a + bit_pattern_31[i].y1 * cos_a);
    int x2 = kp.x + (bit_pattern_31[i].x2 * cos_a - bit_pattern_31[i].y2 * sin_a);
    int y2 = kp.y + (bit_pattern_31[i].x2 * sin_a + bit_pattern_31[i].y2 * cos_a);

    // Ensure points are within bounds
    if (x1 < 0 || x1 >= img.cols() || y1 < 0 || y1 >= img.rows() ||
        x2 < 0 || x2 >= img.cols() || y2 < 0 || y2 >= img.rows())
    {
      continue;
    }

    // Compare pixel intensities and set bit
    descriptor.set_bit(i, img(y1, x1) < img(y2, x2));
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
        kp.x >= img.cols() - PatchSize / 2 || kp.y >= img.rows() - PatchSize / 2)
    {
      descriptors[i].data.fill(0);  // Zero out if out of bounds
      continue;
    }

    // Compute keypoint orientation
    float angle = compute_keypoint_orientation(img, kp);

    // Compute rotated BRIEF descriptor
    descriptors[i] = compute_rotated_brief_descriptor(img, kp, angle);
  }
}

template <typename ImageType, size_t MaxFeatures, int PatchSize, typename KeypointT>
void compute_keypoint_orientation(const ImageType& img,
                                  FeatureArray<KeypointT, MaxFeatures>& feats)
{
  using PixelType = typename ImageType::pixel_type;
  constexpr int PatchHalf = PatchSize / 2;

  using Scalar = typename KeypointT::Scalar_;
  
  for (size_t i = 0; i < feats.size(); i++)  
  {
    KeypointT& kp = feats[i];  // Modify directly feature array in place

    // Ensure the patch is within bounds
    if (kp.x - PatchHalf < 0 || kp.x + PatchHalf >= ImageType::cols ||
        kp.y - PatchHalf < 0 || kp.y + PatchHalf >= ImageType::rows) 
    {
      kp.orientation = 0;  // Default orientation for out-of-bounds cases
      continue;
    }
    
    uint32_t m00 = 0, m10 = 0, m01 = 0;  // Reset moments for each keypoint

    for (int dy = -PatchHalf; dy <= PatchHalf; dy++)
    {
      for (int dx = -PatchHalf; dx <= PatchHalf; dx++)
      {
        int x = kp.x + dx;
        int y = kp.y + dy;

        if (x >= 0 && x < img.cols() && y >= 0 && y < img.rows())  
        {
          PixelType pixel_value = img(y, x);
          m00 += pixel_value;
          m10 += dx * pixel_value;
          m01 += dy * pixel_value;
        }
      }
    }

    if (m00 != 0)
    {
      Scalar cx = static_cast<Scalar>(m10) / m00;
      Scalar cy = static_cast<Scalar>(m01) / m00;
      kp.orientation = std::atan2(cy, cx);
    }
    else
    {
      kp.orientation = 0;
    }
  }
}


template <typename ImageType,
          int Threshold,
          size_t MaxFeatures,
          size_t NumLevels,  // Number of pyramid levels
          typename KeypointT>
void orb(const ImageType& img, 
         FeatureArray<KeypointT, MaxFeatures>& fdo,
         std::array<BRIEFDescriptor, MaxFeatures>& descriptors)
{
  // 1. Construct Image Pyramid
  ImagePyramid<NumLevels - 1, ImageType::rows, ImageType::cols, typename ImageType::PixelType> pyramid(img);

  // 2. Feature Detection (FAST on each pyramid level)
  FeatureArray<KeypointT, MaxFeatures> all_keypoints;

  for (size_t level = 0; level < NumLevels; ++level)
  {
    const auto& img_level = std::get<level>(pyramid.pyramid);
    
    // Run FAST feature detection
    fast<ImageType, KeypointT, 16, Threshold, 9, MaxFeatures, true, true>(img_level, all_keypoints);
  }

  // 3. Compute Keypoint Orientations
  compute_keypoint_orientation(img, all_keypoints);

  // 4. Compute Rotated BRIEF Descriptors
  compute_orb_descriptors(img, all_keypoints, descriptors);

  // Copy detected features to output
  fdo = all_keypoints;}


} // namespace EntoFeature2D


#endif // FEATURE2D_ORB_H
