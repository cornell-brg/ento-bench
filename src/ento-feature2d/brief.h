#ifndef BRIEF_H
#define BRIEF_H

#include "feat2d_util.h"
#include <array>
#include <cstdint>

namespace EntoFeature2D
{

struct BRIEFDescriptor
{
  std::array<uint8_t, 32> data{};
  BRIEFDescriptor() = default;
  BRIEFDescriptor(const BRIEFDescriptor&) = default;
  BRIEFDescriptor& operator=(const BRIEFDescriptor&) = default;

  // Set bit needed for rBRIEF (in ORB)
  inline void set_bit(size_t index, bool value)
  {
    if (value)
      data[index / 8] |= (1 << (index % 8));  // Set bit to 1
    else
      data[index / 8] &= ~(1 << (index % 8)); // Clear bit to 0
  }
};

constexpr size_t BRIEF_NUM_PAIRS = 256; // 256 pairs for BRIEF

struct PatternPoint
{
  int8_t x1, y1, x2, y2;
};

constexpr std::array<PatternPoint, BRIEF_NUM_PAIRS> bit_pattern_31 = {{
    {8, -3, 9, 5},    {4, 2, 7, -12},   {-11, 9, -8, 2},   {7, -12, 12, -13},
    {2, -13, 2, 12},  {1, -7, 1, 6},    {-2, -10, -2, -4}, {-13, -13, -11, -8},
    {-13, -3, -12, -9}, {10, 4, 11, 9},  {-13, -8, -8, -9}, {-11, 7, -9, 12},
    {7, 7, 12, 6},    {-4, -5, -3, 0},  {-13, 2, -12, -3}, {-9, 0, -7, 5},
    {12, -6, 12, -1}, {-3, 6, -2, 12},  {-6, -13, -4, -8}, {11, -13, 12, -8},
    {4, 7, 5, 1},     {5, -3, 10, -3},  {3, -7, 6, 12},    {-8, -7, -6, -2},
    {-2, 11, -1, -10}, {-13, 12, -8, 10}, {-7, 3, -5, -3}, {-4, 2, -3, 7},
    {-10, -12, -6, 11}, {5, -12, 6, -7}, {5, -6, 7, -1},   {1, 0, 4, -5},
    {9, 11, 11, -13}, {4, 7, 4, 12},    {2, -1, 4, 4},     {-4, -12, -2, 7},
    {-8, -5, -7, -10}, {4, 11, 9, 12},  {0, -8, 1, -13},   {-13, -2, -8, 2},
    {-3, -2, -2, 3},  {-6, 9, -4, -9},  {8, 12, 10, 7},    {0, 9, 1, 3},
    {7, -5, 11, -10}, {-13, -6, -11, 0}, {10, 7, 12, 1},   {-6, -3, -6, 12},
    {10, -9, 12, -4}, {-13, 8, -8, -12}, {-13, 0, -8, -4}, {3, 3, 7, 8},
    {5, 7, 10, -7},   {-1, 7, 1, -12},  {3, -10, 5, 6},    {2, -4, 3, -10},
    {-13, 0, -13, 5}, {-13, -7, -12, 12}, {-13, 3, -11, 8}, {-7, 12, -4, 7},
    {6, -10, 12, 8},  {-9, -1, -7, -6}, {-2, -5, 0, 12},   {-12, 5, -7, 5},
    {3, -10, 8, -13}, {-7, -7, -4, 5},  {-3, -2, -1, -7},  {2, 9, 5, -11},
    {-11, -13, -5, -13}, {-1, 6, 0, -1}, {5, -3, 5, 2},    {-4, -13, -4, 12},
    {-9, -6, -9, 6},  {-12, -10, -8, -4}, {10, 2, 12, -3}, {7, 12, 12, 12},
    {-7, -13, -6, 5}, {-4, 9, -3, 4},   {7, -1, 12, 2},    {-7, 6, -5, 1},
    {-13, 11, -12, 5}, {-3, 7, -2, -6}, {7, -8, 12, -7},   {-13, -7, -11, -12},
    {1, -3, 12, 12},  {2, -6, 3, 0},    {-4, 3, -2, -13},  {-1, -13, 1, 9},
    {7, 1, 8, -6},    {1, -1, 3, 12},   {9, 1, 12, 6},     {-1, -9, -1, 3},
    {-13, -13, -10, 5}, {7, 7, 10, 12},  {12, -5, 12, 9},   {6, 3, 7, 11},
    {5, -13, 6, 10},  {2, -12, 2, 3},   {3, 8, 4, -6},     {2, 6, 12, -13},
    {9, -12, 10, 3},  {-8, 4, -7, 9},   {-11, 12, -4, -6}, {1, 12, 2, -8},
    {6, -9, 7, -4},   {2, 3, 3, -2},    {6, 3, 11, 0},     {3, -3, 8, -8},
    {7, 8, 9, 3},     {-11, -5, -6, -4}, {-10, 11, -5, 10}, {-5, -8, -3, 12},
    {-10, 5, -9, 0},  {8, -1, 12, -6},  {4, -6, 6, -11},   {-10, 12, -8, 7},
    {4, -2, 6, 7},    {-2, 0, -2, 12},  {-5, -8, -5, 2},   {7, -6, 10, 12},
    {-9, -13, -8, -8}, {-5, -13, -5, -2}, {8, -8, 9, -13},  {-9, -11, -9, 0},
    {1, -8, 1, -2},   {7, -4, 9, 1},    {-2, 1, -1, -4},   {11, -6, 12, -11},
    {-12, -9, -6, 4}, {3, 7, 7, 12},    {5, 5, 10, 8},     {0, -4, 2, 8},
    {-9, 12, -5, -13}, {0, 7, 2, 12},   {-1, 2, 1, 7},     {5, 11, 7, -9}
}};

template <typename PixelType, size_t PatchSize = 31, typename KeypointType = FastKeypoint<int16_t>>
BRIEFDescriptor compute_brief_descriptor(const PixelType& img,
                                         const KeypointType& kp)
{
  constexpr int PATCH_HALF = 16;  // 31x31 patch, half is 16
  constexpr int DESC_SIZE = 256;  // 256 bits

  BRIEFDescriptor descriptor;
  
  // Initialize descriptor
  std::fill(descriptor.data.begin(), descriptor.data.end(), 0);

  // Iterate over each pair in the BRIEF pattern
  for (size_t i = 0; i < DESC_SIZE; ++i)
  {
    const auto& p = bit_pattern_31[i];

    // Compute pixel positions in the image buffer
    int x1 = kp.x + p.x1, y1 = kp.y + p.y1;
    int x2 = kp.x + p.x2, y2 = kp.y + p.y2;

    // Compare intensities and store result as a bit
    uint8_t bit = (img(y1, x1) < img(y2, x2)) ? 1 : 0;
    descriptor.data[i / 8] |= (bit << (i % 8));
  }

  return descriptor;
}

template <typename Image, typename KeypointType, size_t PatchSize = 31, size_t MaxFeatures = 100>
void compute_brief_descriptors(const Image& img,
                               const FeatureArray<KeypointType, MaxFeatures>& feats,
                               std::array<BRIEFDescriptor, MaxFeatures>& descriptors)
{
  constexpr int PATCH_HALF = PatchSize / 2;
  constexpr int DESC_SIZE = 256;  // 256-bit descriptor (32 bytes)

  for (size_t i = 0; i < feats.size(); ++i)
  {
    const auto& kp = feats[i];

    // Ensure keypoint is within valid bounds
    if (kp.x < PATCH_HALF || kp.y < PATCH_HALF ||
        kp.x >= img.cols() - PATCH_HALF || kp.y >= img.rows() - PATCH_HALF)
    {
      // Zero out descriptor for invalid keypoints
      std::fill(descriptors[i].data.begin(), descriptors[i].data.end(), 0);
      continue;
    }

    // Compute BRIEF descriptor
    descriptors[i] = compute_brief_descriptor<Image, KeypointType, PatchSize>(img, kp);
  }
}


} // namespace EntoFeature2D


#endif // BRIEF_H
