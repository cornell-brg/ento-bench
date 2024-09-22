#ifndef FEAT_UTIL_H
#define FEAT_UTIL_H

#include <array>

struct Keypoint
{
  int16_t x;
  int16_t y;

  Keypoint() : x(0), y(0) {}
  Keypoint(int16_t _x, int16_t _y)
    : x(_x), y(_y) {}
  Keypoint(const Keypoint&) = default;
  Keypoint(Keypoint&&) = default;
  Keypoint& operator=(const Keypoint&) = default;
  Keypoint& operator=(Keypoint&&) = default;
};

struct FastKeypoint : public Keypoint
{
  int score;
  FastKeypoint() : Keypoint(0, 0), score(0) {}
  FastKeypoint(int16_t _x, int16_t _y, int _score)
    : Keypoint(_x, _y), score(_score) {}
  FastKeypoint(const FastKeypoint&) = default;
  FastKeypoint(FastKeypoint&&) = default;
  FastKeypoint& operator=(const FastKeypoint&) = default;
  FastKeypoint& operator=(FastKeypoint&&) = default;
};

struct ORBKeypoint : public FastKeypoint
{

};

template <typename KeypointType, size_t MaxFeatures = 100>
struct FeatureDetectorOutput
{
  std::array<KeypointType, MaxFeatures> keypoints;
  size_t num_features = 0;
  static constexpr size_t max_features = MaxFeatures;

  FeatureDetectorOutput() = default;
  
  bool add_keypoint(const KeypointType& kp)
  {
    if (num_features < max_features)
    {
      keypoints[num_features++] = kp;
      return true;
    }
    return false;
  }

  const KeypointType& operator[](size_t idx) const
  {
    return keypoints[idx];
  }

  size_t size() const
  {
    return num_features;
  }
};

#endif // FEAT_UTIL_H
