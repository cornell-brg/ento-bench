#ifndef FEAT_UTIL_H
#define FEAT_UTIL_H

#include <array>

struct Keypoint
{
  int16_t x;
  int16_t y;

  Keypoint(int16_t x, int16_t y)
    : x(_x), y(_y) {}
};

struct FastKeypoint : public Keypoint
{
  int score;
  FastKeypoint(int16_t _x, int16_t _y, int _score)
    : x(_x), y(_y), score(_score) {}
};

struct ORBKeypoint : public FastKeypoint
{

};

template <size_t MaxFeatures = 100>
struct FeatureDetectorOutput
{
  std::array<Keypoint, MaxFeatures> keypoints;
  size_t num_features = 0;
  size_t max_features = 0;

  
  bool add_keypoint(const Keypoint& kp)
  {
    if (feature_count < MaxFeatures)
    {
      keypoints[num_features++] = kp;
      return true;
    }
    return false;
  }

  const Keypoint& operator[](size_t idx) const
  {
    return keypoints[idx];
  }

  size_t size() const
  {
    return num_features;
  }


}

#endif // FEAT_UTIL_H
