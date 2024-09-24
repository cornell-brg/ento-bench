#ifndef FEATURE2D_ORB_H
#define FEATURE2D_ORB_H



template <typename ImageType,
          int Threshold,
          size_t MaxFeatures>
void orb(const ImageType& img, FeatureDetectorOutput<OrbKeypoint, MaxFeatures>& fdo);


template <>
void compute_keypoint_orientation(const ImageType& img,
                                  FeatureDetectorOutput<OrbKeypoint, MaxFeatures>& fdo,
                                  );


// ========================================
// Implementations

template <typename ImageType,
          int Threshold,
          size_t MaxFeatures>
void orb(const ImageType& img, FeatureDetectorOutput<OrbKeypoint, MaxFeatures>& fdo)
{
  constexpr int pattern_size = 16;
  constexpr int contiguiuty_requirement = 9;
  constexpr int img_width   = ImageType::cols;
  constexpr int img_height  = ImageType::rows;

  
}

#endif // FEATURE2D_ORB_H
