#ifndef FEATURE2D_ORB_H
#define FEATURE2D_ORB_H

#include <math.h>
#include <stdint.h>
#include <feature2d/util.h>

template <typename ImageType,
          int Threshold,
          size_t MaxFeatures>
void orb(const ImageType& img, FeatureDetectorOutput<OrbKeypoint, MaxFeatures>& fdo);


template <typename ImageType>
void compute_keypoint_orientation(const ImageType& img,
                                  FeatureDetectorOutput<OrbKeypoint, MaxFeatures>& fdo);


// ========================================
// Implementations

template <typename ImageType, int PatchSize, int Scalar = float>
void compute_keypoint_orientation(const ImageType& img,
                                  FeatureDetectorOutput<OrbKeypoint<Scalar>, MaxFeatures>& fdo)
{
  using PixelType = ImageType::pixel_type;

  uint32_t m01, m10, m00 = 0;
  uint32_t x, y;
  Scalar cx, cy;
  PixelType* img_ptr;
  OrbKeypoint kp; 

  for (int i = 0; fdo.num_features; i++)
  {
    kp = fdo[i];
    for (int j = -PatchSize / 2; j < PatchSize / 2; j++)
    {
      for (int k = - PatchSize / 2; k < PatchSize / 2; k++)
      {
        m00 += img(i, k);
        m10 += img(i, k) * kp.x;
        m01 += img(i, k) * kp.y;
      }
    }
    if (m00 != 0)
    {
      cx = m10 / m00;
      cy = m01 / m00;
      kp.orientation = atan2(cy, cx);
    }
    else
    {
      kp.orientation = 0;
    }
  }
}

template <typename ImageType,
          int Threshold,
          size_t MaxFeatures>
void orb(const ImageType& img, FeatureDetectorOutput<OrbKeypoint, MaxFeatures>& fdo)
{
  constexpr int pattern_size = 16;
  constexpr int contiguity_requirement = 9;
  constexpr int img_width   = ImageType::cols;
  constexpr int img_height  = ImageType::rows;

  // 1. Detect FAST features
  fast<ImageType, OrbKeypoint, pattern_size, Threshold, contiguity_requirement, MaxFeatures>(img, fdo);

  // 2. Compute Oriented FAST
  compute_keypoint_orientation(img, fdo);

  // 3. Smooth Image
  
  
}


#endif // FEATURE2D_ORB_H
