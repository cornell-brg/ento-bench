#ifndef FAST_H
#define FAST_H

#include <cstdint>

#include <numbers>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/brief.h>
#include <ento-util/debug.h>
#include <variant>

namespace EntoFeature2D
{

constexpr float PI = std::numbers::pi_v<float>;

template <int BitDepth, int Threshold>
struct ThresholdTable {
  static constexpr int tab_size = (1 << (BitDepth + 1)) + 1;

  static constexpr auto create()
  {
    std::array<uint8_t, tab_size> table = {};
    constexpr int half = tab_size / 2;
    for (int i = -half; i <= half; ++i)
    {
      table[i + half] = (i < -Threshold ? 1 : i > Threshold ? 2 : 0);
    }
    return table;
  }

  static constexpr std::array<uint8_t, tab_size> table = create();
};

template <typename PixelType, int PatternSize, int RowStride, int ContiguityRequirement = 9>
constexpr std::array<PixelType, PatternSize + ContiguityRequirement>
generate_bresenham_circle();

template <typename PixelType, typename CircleType, int PatternSize, int ContiguityRequirement>
int corner_score(const PixelType* ptr,
                 const std::array<CircleType, PatternSize + ContiguityRequirement>& circle,
                 int threshold);

template <typename KeypointType, size_t MaxFeatures>
void apply_nms(FeatureArray<KeypointType, MaxFeatures>& feats);



template <typename Image,
          typename KeypointType,
          int PatternSize,
          int Threshold,
          int ContiguityRequirement = 9,
          size_t MaxFeatures,
          bool PerformNMS = false,
          bool Orb = false>
void fast(const Image& img,
          FeatureArray<KeypointType, MaxFeatures>& feats );


// ===========================================================
// Implementations
// ===========================================================

template <typename PixelType, int PatternSize, int RowStride, int ContiguityRequirement>
constexpr std::array<PixelType, PatternSize + ContiguityRequirement>
generate_bresenham_circle() {
  static_assert(PatternSize == 16 || PatternSize == 12 || PatternSize == 8, "Unsupported diameter size.");

  constexpr int offsets_size = PatternSize + ContiguityRequirement;  // Circle circumference

  std::array<PixelType, offsets_size> circle_offsets{};
  int k = 0;

  if constexpr (PatternSize == 16) {
    const int offsets16[][2] = {
      {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
      {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    for ( ; k < PatternSize; k++) {
      circle_offsets[k] = static_cast<PixelType>(offsets16[k][0] + offsets16[k][1] * RowStride);
    }
  }
  else if constexpr (PatternSize == 12) {
    const int offsets12[][2] = {
      {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
      {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    for ( ; k < PatternSize; k++) {
      circle_offsets[k] = static_cast<PixelType>(offsets12[k][0] + offsets12[k][1] * RowStride);
    }
  }
  else if constexpr (PatternSize == 8) {
    const int offsets8[][2] = {
      {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
      {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    for ( ; k < PatternSize; k++) {
      circle_offsets[k] = static_cast<PixelType>(offsets8[k][0] + offsets8[k][1] * RowStride);
    }
  }
  for ( ; k < PatternSize + ContiguityRequirement; ++k)
  {
    circle_offsets[k] = circle_offsets[k - PatternSize];
  }

  return circle_offsets;
}

//@TODO: Make this conform to the template and be bit depth agnostic
// We will only support 4-bit, 8-bit, 10-bit images.
// For 10-bit Need to decide how it is stored, e.g., contiguously or non-contiguously...
// So probably hold off on that for now!
template <typename ImageType,
          typename KeypointType,
          int PatternSize,
          int Threshold,
          int ContiguityRequirement,
          size_t MaxFeatures,
          bool PerformNMS,
          bool Orb>
void fast(const ImageType& img,
          FeatureArray<KeypointType, MaxFeatures>& feats)
{
  // Compile-time access to number of columns
  constexpr int img_width   = ImageType::cols;
  constexpr int img_height  = ImageType::rows_;
  constexpr int max_dim     = (img_width > img_height) ? img_width : img_height;

  // Extract pixel type info
  using PixelType = ImageType::pixel_type;
  constexpr int bit_depth = ImageType::bit_depth;
  constexpr int middle_value = (1 << (bit_depth + 1)) / 2;

  // Calculate fast attention region
  constexpr int BORDER = Orb ? 15 : 3;
  constexpr int START_X = BORDER;
  constexpr int END_X = img_width - BORDER;
  constexpr int START_Y = BORDER;
  constexpr int END_Y = img_height - BORDER;


  // Coordinate type
  using CoordType = typename std::conditional<
      (max_dim <= 255), uint8_t, typename std::conditional<
        (max_dim <= 65535), uint16_t, uint32_t
      >::type
    >::type;

  using CircleType = typename std::conditional<
      (max_dim <= 255), int16_t, int32_t>::type;

  // Create a circle of offsets
  constexpr int circle_buff_sz = PatternSize + ContiguityRequirement;
  //static PixelType circle[circle_buff_sz];
  //bressenham_circle<PixelType, CircleDiameter, img_width>(circle);
  
  static constexpr auto circle = generate_bresenham_circle<CircleType, PatternSize, img_width, ContiguityRequirement>();
  // const PixelType* ptemp = &img.data[3*img_width] + 3;

  CoordType i, j, k;
  const PixelType* ptr;

  // Important: These may be "16 bit" depth images but the assumption
  // is that this is to hold the raw 10 bit pixels from the NaneyeC.
  // I also made this choice because a tab array of 2^17 is no bueno.
  // I might have to rethink the tradeoffs for this implementation that
  // OpenCV.
  constexpr auto threshold_tab = ThresholdTable<bit_depth, Threshold>::table;
  // constexpr int tab_size = (1 << (bit_depth + 1));
  
  for (i = START_Y; i < END_Y; ++i)
  {
    ptr = &img.data[i*img_width] + START_Y;

    if (i < img_height - BORDER)
    {
      j = START_X;
      for (; j < END_X; j++, ptr++)
      {
        int v = ptr[0];

        const PixelType* tab = &threshold_tab[0] - v + middle_value;
        int d = tab[ptr[circle[0]]] | tab[ptr[circle[8]]];

        if ( d==0 ) continue;

        d &= tab[ptr[circle[2]]] | tab[ptr[circle[10]]];
        d &= tab[ptr[circle[4]]] | tab[ptr[circle[12]]];
        d &= tab[ptr[circle[6]]] | tab[ptr[circle[14]]];

        if( d == 0 )
            continue;

        d &= tab[ptr[circle[1]]] | tab[ptr[circle[9]]];
        d &= tab[ptr[circle[3]]] | tab[ptr[circle[11]]];
        d &= tab[ptr[circle[5]]] | tab[ptr[circle[13]]];
        d &= tab[ptr[circle[7]]] | tab[ptr[circle[15]]];

        if( d & 1 )
        {
          int vt = v - Threshold;
          int count = 0;

          for( k = 0; k < circle_buff_sz; k++ )
          {
            int x = ptr[circle[k]];
            if(x < vt)
            {
              if( ++count > ContiguityRequirement )
              {
                int score = corner_score<
                  PixelType, CircleType, PatternSize, ContiguityRequirement
                                        >(ptr, circle, Threshold);
                feats.add_keypoint(KeypointType(j, i, score));
                ENTO_DEBUG("Found feature: %i, %i", j, i);
                break;
              }
            }
            else
            {
              count = 0;
            }
          }
        }

        if( d & 2 )
        {
          int vt = v + Threshold;
          int count = 0;

          for( k = 0; k < circle_buff_sz; k++ )
          {
            int x = ptr[circle[k]];
            if(x > vt)
            {
              if( ++count > ContiguityRequirement )
              {
                int score = corner_score<
                  PixelType, CircleType, PatternSize, ContiguityRequirement
                                        >(ptr, circle, Threshold);
                feats.add_keypoint(KeypointType(j, i, score));
                ENTO_DEBUG("Found feature: %i, %i", j, i);
                break;
              }
            }
            else
            {
              count = 0;
            }
          }
        }

      }
    }

  }
  if constexpr (PerformNMS)
  {
    apply_nms(feats);
  }
}

template <typename PixelType, typename CircleType, int PatternSize, int ContiguityRequirement>
int corner_score(const PixelType* kp,
                 const std::array<CircleType, PatternSize+ContiguityRequirement>& circle,
                 int threshold)
{
  // Code inspired by fast_score.cpp found in OpenCV repo.
  constexpr int ExtendedCircleSize = PatternSize + ContiguityRequirement;
  int v = static_cast<int>(kp[0]);  // Keypoint intensity
  int min_diff = std::numeric_limits<PixelType>::max();
  int max_diff = std::numeric_limits<PixelType>::min();

  // 1. Compute intensity differences
  //for (int i = 0; i < PatternSize; i++)
  //{
  //  int diff = v - static_cast<int>(kp[circle[i]]);  // Compute difference
  //  min_diff = std::min(min_diff, diff);
  //  max_diff = std::max(max_diff, diff);
  //}

  // 2. Best thresholded arc
  int best_dark = threshold;
  int best_bright = -threshold;

  for (int i = 0; i < PatternSize; i += 2)
  {
    int d1 = v - kp[circle[i]];
    int d2 = v - kp[circle[i + 1]];
    int arc_min = std::min(d1, d2);
    int arc_max = std::max(d1, d2);

    best_dark = std::max(best_dark, arc_min);
    best_bright = std::min(best_bright, arc_max);
  }

  return std::max(best_dark, -best_bright) - 1;  // Final score
}

template <typename KeypointType, size_t MaxFeatures>
void apply_nms(FeatureArray<KeypointType, MaxFeatures>& feats)
{
  if (feats.size() == 0)
    return;

  FeatureArray<KeypointType, MaxFeatures> kept;
  
  for (size_t i = 0; i < feats.size(); i++)
  {
    const auto& kp = feats[i];
    bool suppressed = false;

    for (size_t j = 0; j < feats.size(); j++)
    {
      if (i == j)
        continue;

      const auto& neighbor = feats[j];

      if (std::abs(neighbor.x - kp.x) <= 1 &&
          std::abs(neighbor.y - kp.y) <= 1 &&
          neighbor.score >= kp.score)
      {
        suppressed = true;
        break;
      }
    }

    if (!suppressed)
      kept.add_keypoint(kp);
  }

  // Copy back
  for (size_t i = 0; i < kept.size(); i++)
    feats[i] = kept[i];
  feats.num_features = kept.size();
}

template <int MaxFeatures,
          int PatternSize = 16,
          int Threshold = 10,
          bool PerformNMS = false >
struct FastKernel
{
  using KeypointType   = FastKeypoint<uint16_t>;
  using DescriptorType = std::monostate; // FAST has no descriptor

  static constexpr int PatternSize_ = PatternSize;
  static constexpr int Threshold_ = Threshold;  // Customize as needed
  static constexpr size_t MaxFeatures_ = MaxFeatures;
  static constexpr bool PerformNMS_ = PerformNMS;
  static constexpr bool Orb_ = false;
  static constexpr bool ContiguityRequirement_ = 9;

  template <typename ImageT, typename KeypointT>
  void operator()(const ImageT& img, FeatureArray<KeypointT, MaxFeatures_>& feats)
  {
    EntoFeature2D::fast<ImageT,
                        KeypointT,
                        PatternSize_,
                        Threshold_,
                        ContiguityRequirement_,
                        MaxFeatures_,
                        PerformNMS_,
                        Orb_>(img, feats);
  }

  static constexpr const char* name() { return "FAST Kernel"; }
};

template <int MaxFeatures,
          int PatchSize = 31,
          int DescriptorSize = 256>
struct FastBriefKernel
{
  using KeypointType   = FastKeypoint<uint16_t>;
  using DescriptorType = BRIEFDescriptor;

  static constexpr int PatchSize_ = PatchSize;
  static constexpr int DescriptorSize_ = DescriptorSize;
  static constexpr size_t MaxFeatures_ = MaxFeatures;

  template <typename ImageT,
            typename KeypointT,
            typename DescriptorArray>
  void operator()(      ImageT& img,
                  FeatureArray<KeypointT, MaxFeatures_>& feats,
                  DescriptorArray& descs) const
  {
    using KernelScalarT = float;
    constexpr int KernelSize = 5;

    gaussian_blur_in_place<ImageT, KernelSize, KernelScalarT>(img);

    // Run FAST
    fast<ImageT,
         KeypointT,
         16,
         10,
         9,
         MaxFeatures_,
         true,
         true>(img, feats);

    // Then BRIEF
    compute_brief_descriptors<ImageT,
                              KeypointT,
                              PatchSize_,
                              MaxFeatures_>(img, feats, descs);
  }

  static constexpr const char* name() { return "FAST + BRIEF Kernel"; }
};

} // namespace EntoFeature2D

#endif // FAST_H
