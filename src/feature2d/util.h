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

// ===========================================================
// Utility Functions

template<typename Scalar, typename KeypointType, int MaxFeatures>
void undistort_points(const FeatureDetectorOutput<KeypointType, MaxFeatures>& fdo,
                      const Eigen::Matrix<Scalar, 3, 3>& K,
                      const Eigen::Matrix<Scalar, 1, 5>& dist,
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxFeatures, 3>& points)
{
  // Intrinsic parameters
  Scalar fxinv = 1.0 / K(0, 0);  // Inverse of fx
  Scalar fyinv = 1.0 / K(1, 1);  // Inverse of fy
  Scalar cx = K(0, 2);           // Principal point x
  Scalar cy = K(1, 2);           // Principal point y

  // Distortion coefficients
  Scalar k1 = dist(0, 0);
  Scalar k2 = dist(0, 1);
  Scalar p1 = dist(0, 2);
  Scalar p2 = dist(0, 3);
  Scalar k3 = dist(0, 4);

  uint32_t N = fdo.num_features();  // Number of features
  for (uint32_t i = 0; i < N; ++i) {
    // Original point coordinates from fdo
    Scalar u = fdo.keypoints(i, 0);  // x-coordinate
    Scalar v = fdo.keypoints(i, 1);  // y-coordinate

    // 1. Normalize points
    Scalar x = (u - cx) * fxinv;
    Scalar y = (v - cy) * fyinv;

    // 2. Iterative undistortion using distortion coefficients
    Scalar x0 = x, y0 = y;
    for (int j = 0; j < 5; ++j)
    {
      Scalar r2 = x * x + y * y;  // Radial distance squared
      Scalar radial_dist = 1 + ((k3 * r2 + k2) * r2 + k1) * r2;

      // Tangential distortion
      Scalar deltaX = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
      Scalar deltaY = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

      // Update point positions
      x = (x0 - deltaX) / radial_dist;
      y = (y0 - deltaY) / radial_dist;
    }

    // 3. Store undistorted points in the output matrix
    points(i, 0) = x * K(0, 0) + cx;
    points(i, 1) = y * K(1, 1) + cy;
    points(i, 2) = 1.0;  // Homogeneous coordinate
  }
}

template <typename Scalar, int MaxFeatures>
void iso_normalize_points(Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxFeatures, 3>& points,
                          Eigen::Matrix<Scalar, 3, 3>& T)
{
  int num_points = points.rows();

  // 1. Compute centroi of points
  Eigen::Matrix<Scalar, 1, 2> = points.leftCols(2).colwise().mean();

  // 2. Center points around origin
  for (int i = 0; i < num_points; ++i)
  {
    points(i, 0) -= centroid(0);
    points(i, 1) -= centroid(1);
  }

  // 3. Compute average distance of points from origin
  //    so we can scale them properly.
  Scalar mean_distance = 0;
  for (int i = 0; i < num_points; ++i)
  {
    mean_distance += std::sqrt(
      points(i, 0)) * points(i, 0) + points(i, 1) * points(i, 1)
    );
  }

  mean_distance /= num_points;

  // 4. Scale points such that mean distance from origin
  //    is sqrt(2) (unit square)
  Scalar scale = std::sqrt(2) / mean_distance;
  for (int i = 0; i < num_points; ++i)
  {
    points(i, 0) *= scale;
    points(i, 1) *= scale;
  }

  T.setIdentity();
  T(0, 0) = scale;
  T(0, 1) = 0;
  T(0, 2) = -scale * centroid(0);

  T(1, 0) = 0;
  T(1, 1) = scale;
  T(1, 2) = -scale * centroid(1);

  T(2, 0) = 0;
  T(2, 1) = 0;
  T(2, 2) = 1;

}

template <typename Scalar>
void unnormalize_homography(const Eigen::Matrix<Scalar, 3, 3>& H,
                            const Eigen::Matrix<Scalar, 3, 3>& T1,
                            const Eigen::Matrix<Scalar, 3, 3>& T2,
                            Eigen::Matrix<Scalar, 3, 3>& result)
{
  result = T1.inverse() * H * T2;
}


#endif // FEAT_UTIL_H
