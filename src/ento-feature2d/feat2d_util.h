#ifndef FEAT_UTIL_H
#define FEAT_UTIL_H


#include <array>
#include <Eigen/Dense>

namespace EntoFeature2D
{

template <typename CoordT = int16_t>
struct Keypoint
{
  using CoordT_ = CoordT;

  CoordT x;
  CoordT y;

  Keypoint() : x(0), y(0) {}
  Keypoint(CoordT _x, CoordT _y)
    : x(_x), y(_y) {}
  Keypoint(const Keypoint&) = default;
  Keypoint(Keypoint&&) = default;
  Keypoint& operator=(const Keypoint&) = default;
  Keypoint& operator=(Keypoint&&) = default;
};

template <typename CoordT = int16_t>
struct FastKeypoint : public Keypoint<CoordT>
{
  int score;
  FastKeypoint() : Keypoint<CoordT>(0, 0), score(0) {}
  FastKeypoint(CoordT _x, CoordT _y, int _score)
    : Keypoint<CoordT>(_x, _y), score(_score) {}
  FastKeypoint(const FastKeypoint<CoordT>&) = default;
  FastKeypoint(FastKeypoint&&) = default;
  FastKeypoint& operator=(const FastKeypoint&) = default;
  FastKeypoint& operator=(FastKeypoint&&) = default;
};

template <typename CoordT = int16_t, typename Scalar = float>
struct ORBKeypoint : public FastKeypoint<CoordT>
{
  Scalar orientation;
  ORBKeypoint() : FastKeypoint<CoordT>(), orientation(0) {}
  ORBKeypoint(int16_t _x, int16_t _y, int _score, Scalar _ori)
    : FastKeypoint<CoordT>(_x, _y, _score), orientation(_ori) {} 
  ORBKeypoint(const ORBKeypoint&) = default;
  ORBKeypoint(ORBKeypoint&&) = default;
  ORBKeypoint& operator=(const ORBKeypoint&) = default;
  ORBKeypoint& operator=(ORBKeypoint&&) = default;

};

template <typename KeypointType, size_t MaxFeatures = 100>
struct FeatureArray
{
  std::array<KeypointType, MaxFeatures> keypoints;
  size_t num_features = 0;
  static constexpr size_t max_features = MaxFeatures;

  FeatureArray() = default;
  
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

  KeypointType& operator[](size_t idx)
  {
    return keypoints[idx];
  }

  void clear()
  {
    keypoints.fill(KeypointType{});
    num_features = 0;
  }
  

  size_t size() const
  {
    return num_features;
  }
};

// ===========================================================
// Utility Functions
template<typename Scalar, typename KeypointType, int N>
void undistort_points(const std::array<KeypointType, N>& keypoints,
                      const Eigen::Matrix<Scalar, 3, 3>& K,
                      const Eigen::Matrix<Scalar, 1, 5>& dist,
                      Eigen::Matrix<Scalar, N, 3>& points)
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

  for (int i = 0; i < N; ++i) {
    Scalar u = keypoints[i].x;  // x-coordinate
    Scalar v = keypoints[i].y;  // y-coordinate

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

template<typename Scalar, typename KeypointType, int MaxFeatures>
void undistort_points(const std::array<KeypointType, MaxFeatures>& keypoints,
                      const Eigen::Matrix<Scalar, 3, 3>& K,
                      const Eigen::Matrix<Scalar, 1, 5>& dist,
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxFeatures, 3>& points,
                      const int num_features)
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

  for (int i = 0; i < num_features; ++i) {
    Scalar u = keypoints[i].x;  // x-coordinate
    Scalar v = keypoints[i].y;  // y-coordinate

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

template<typename Scalar, typename KeypointType, int MaxFeatures>
void undistort_points(const FeatureArray<KeypointType, MaxFeatures>& fdo,
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

  int N = fdo.num_features();  // Number of features
  for (int i = 0; i < N; ++i) {
    // Original point coordinates from fdo
    Scalar u = fdo.keypoints[i].x;  // x-coordinate
    Scalar v = fdo.keypoints[i].y;  // y-coordinate

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

template <typename Scalar, int N>
void iso_normalize_points(Eigen::Matrix<Scalar, N, 3>& points,
                          Eigen::Matrix<Scalar, 3, 3>& T)
{
  // 1. Compute centroi of points
  Eigen::Matrix<Scalar, 1, 2> centroid = points.leftCols(2).colwise().mean();

  // 2. Center points around origin
  for (int i = 0; i < N; ++i)
  {
    points(i, 0) -= centroid(0);
    points(i, 1) -= centroid(1);
  }

  // 3. Compute average distance of points from origin
  //    so we can scale them properly.
  Scalar mean_distance = 0;
  for (int i = 0; i < N; ++i)
  {
    mean_distance += std::sqrt(
      points(i, 0) * points(i, 0) + points(i, 1) * points(i, 1)
    );
    
  }

  mean_distance /= (Scalar)N;

  // 4. Scale points such that mean distance from origin
  //    is sqrt(2) (unit square)
  Scalar scale = std::sqrt(2) / mean_distance;
  for (int i = 0; i < N; ++i)
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

template <typename Scalar, int MaxFeatures>
void iso_normalize_points(Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxFeatures, 3>& points,
                          Eigen::Matrix<Scalar, 3, 3>& T)
{
  int num_points = points.rows();

  // 1. Compute centroi of points
  Eigen::Matrix<Scalar, 1, 2> centroid = points.leftCols(2).colwise().mean();

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
      points(i, 0) * points(i, 0) + points(i, 1) * points(i, 1)
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

// Normalize 2D points as per Hartley and Zisserman
template <typename Scalar, int N>
void normalize_2d_points(Eigen::Matrix<Scalar, N, 2>& points, Eigen::Matrix<Scalar, 3, 3>& T)
{
  // 1. Compute the centroid of the points
  Eigen::Matrix<Scalar, 1, 2> centroid = points.colwise().mean();

  // 2. Center the points around the origin
  for (int i = 0; i < N; ++i) {
    points(i, 0) -= centroid(0);
    points(i, 1) -= centroid(1);
  }

  // 3. Compute the mean distance from the origin
  Scalar mean_distance = (points.rowwise().norm()).mean();

  // 4. Scale points such that mean distance from origin is sqrt(2)
  Scalar scale = (mean_distance > std::numeric_limits<Scalar>::epsilon()) ? std::sqrt(2) / mean_distance : 1.0;

  points *= scale;  // Apply scaling

  // 5. Set up the transformation matrix T
  T.setIdentity();
  T(0, 0) = scale;
  T(1, 1) = scale;
  T(0, 2) = -scale * centroid(0);
  T(1, 2) = -scale * centroid(1);
}

// Normalize 3D points as per Hartley and Zisserman
template <typename Scalar, int N>
void normalize_3d_points(Eigen::Matrix<Scalar, N, 3>& points, Eigen::Matrix<Scalar, 4, 4>& T)
{
  // 1. Compute the centroid of the points
  Eigen::Matrix<Scalar, 1, 3> centroid = points.colwise().mean();

  // 2. Center the points around the origin
  for (int i = 0; i < N; ++i) {
    points(i, 0) -= centroid(0);
    points(i, 1) -= centroid(1);
    points(i, 2) -= centroid(2);
  }

  // 3. Compute the mean distance from the origin
  Scalar mean_distance = (points.rowwise().norm()).mean();

  // 4. Scale points such that mean distance from origin is sqrt(3)
  Scalar scale = (mean_distance > std::numeric_limits<Scalar>::epsilon()) ? std::sqrt(3) / mean_distance : 1.0;

  points *= scale;  // Apply scaling

  // 5. Set up the transformation matrix T
  T.setIdentity();
  T(0, 0) = scale;
  T(1, 1) = scale;
  T(2, 2) = scale;
  T(0, 3) = -scale * centroid(0);
  T(1, 3) = -scale * centroid(1);
  T(2, 3) = -scale * centroid(2);
}

template <typename Scalar>
void unnormalize_homography(const Eigen::Matrix<Scalar, 3, 3>& H,
                            const Eigen::Matrix<Scalar, 3, 3>& T1,
                            const Eigen::Matrix<Scalar, 3, 3>& T2,
                            Eigen::Matrix<Scalar, 3, 3>& result)
{
  result = T1.inverse() * H * T2;
}

template <size_t Rows, size_t Cols, typename PixelType>
struct PGMHeader
{
  static constexpr size_t max_pixel_value = std::numeric_limits<PixelType>::max();

  static constexpr auto generate() {
    return std::array<char, 32>{'P', '5', '\n', 
                                 digit(Cols / 100), digit((Cols / 10) % 10), digit(Cols % 10), ' ',
                                 digit(Rows / 100), digit((Rows / 10) % 10), digit(Rows % 10), '\n',
                                 digit(max_pixel_value / 100), digit((max_pixel_value / 10) % 10), digit(max_pixel_value % 10), '\n', '\0'};
  }

private:
  static constexpr char digit(size_t value) {
    return static_cast<char>('0' + value);
  }
};


} // namespace EntoFeature2D

#endif // FEAT_UTIL_H
