#include <stdlib.h>

#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-feature2d/feat2d_util.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;

const char* dataset_path = DATASET_PATH;

using namespace EntoFeature2D;

void test_keypoint_default_constructor()
{
  Keypoint kp_default;
  ENTO_TEST_CHECK_INT_EQ(kp_default.x, 0);
  ENTO_TEST_CHECK_INT_EQ(kp_default.y, 0);
}

void test_keypoint_parameterized_constructor()
{
  Keypoint kp_param(10, 20);
  ENTO_TEST_CHECK_INT_EQ(kp_param.x, 10);
  ENTO_TEST_CHECK_INT_EQ(kp_param.y, 20);
}

void test_keypoint_copy_constructor()
{
  Keypoint kp_original(10, 20);
  Keypoint kp_copy = kp_original;
  ENTO_TEST_CHECK_INT_EQ(kp_copy.x, 10);
  ENTO_TEST_CHECK_INT_EQ(kp_copy.y, 20);
}

void test_fast_keypoint_default_constructor()
{
  FastKeypoint fk_default;
  ENTO_TEST_CHECK_INT_EQ(fk_default.x, 0);
  ENTO_TEST_CHECK_INT_EQ(fk_default.y, 0);
  ENTO_TEST_CHECK_INT_EQ(fk_default.score, 0);
}

void test_fast_keypoint_parameterized_constructor()
{
  FastKeypoint fk_param(15, 25, 85);
  ENTO_TEST_CHECK_INT_EQ(fk_param.x, 15);
  ENTO_TEST_CHECK_INT_EQ(fk_param.y, 25);
  ENTO_TEST_CHECK_INT_EQ(fk_param.score, 85);
}

void test_fast_keypoint_copy_constructor()
{
  FastKeypoint fk_original(15, 25, 85);
  FastKeypoint fk_copy = fk_original;
  ENTO_TEST_CHECK_INT_EQ(fk_copy.x, 15);
  ENTO_TEST_CHECK_INT_EQ(fk_copy.y, 25);
  ENTO_TEST_CHECK_INT_EQ(fk_copy.score, 85);
}

void test_orb_keypoint_default_constructor()
{
  ORBKeypoint<> orb_default;
  ENTO_TEST_CHECK_INT_EQ(orb_default.x, 0);
  ENTO_TEST_CHECK_INT_EQ(orb_default.y, 0);
  ENTO_TEST_CHECK_INT_EQ(orb_default.score, 0);
  ENTO_TEST_CHECK_FLOAT_EQ(orb_default.orientation, 0.0f);
}

void test_orb_keypoint_parameterized_constructor()
{
  ORBKeypoint<float> orb_param(30, 40, 100, 45.5f);
  ENTO_TEST_CHECK_INT_EQ(orb_param.x, 30);
  ENTO_TEST_CHECK_INT_EQ(orb_param.y, 40);
  ENTO_TEST_CHECK_INT_EQ(orb_param.score, 100);
  ENTO_TEST_CHECK_FLOAT_EQ(orb_param.orientation, 45.5f);
}

void test_orb_keypoint_copy_constructor()
{
  ORBKeypoint<float> orb_original(30, 40, 100, 45.5f);
  ORBKeypoint<float> orb_copy = orb_original;
  ENTO_TEST_CHECK_INT_EQ(orb_copy.x, 30);
  ENTO_TEST_CHECK_INT_EQ(orb_copy.y, 40);
  ENTO_TEST_CHECK_INT_EQ(orb_copy.score, 100);
  ENTO_TEST_CHECK_FLOAT_EQ(orb_copy.orientation, 45.5f);
}

void test_feature_detector_output_add_keypoint_within_limit()
{
  FeatureArray<Keypoint> detector;
  for (int i = 0; i < 100; ++i)
  {
    Keypoint kp(i, i);
    bool added = detector.add_keypoint(kp);
    ENTO_TEST_CHECK_TRUE(added);
  }
  ENTO_TEST_CHECK_INT_EQ(detector.size(), 100);
}

void test_feature_detector_output_add_keypoint_exceeding_limit()
{
  FeatureArray<Keypoint> detector;
  for (int i = 0; i < 100; ++i)
  {
    Keypoint kp(i, i);
    detector.add_keypoint(kp);
  }
  Keypoint extra_kp(101, 101);
  bool added = detector.add_keypoint(extra_kp);
  ENTO_TEST_CHECK_FALSE(added);
  ENTO_TEST_CHECK_INT_EQ(detector.size(), 100);
}

void test_feature_detector_output_access_keypoints()
{
  FeatureArray<Keypoint> detector;
  for (int i = 0; i < 100; ++i)
  {
    Keypoint kp(i, i);
    detector.add_keypoint(kp);
  }

  for (int i = 0; i < 100; ++i)
  {
    const Keypoint& kp = detector[i];
    ENTO_TEST_CHECK_INT_EQ(kp.x, i);
    ENTO_TEST_CHECK_INT_EQ(kp.y, i);
  }
}

void test_undistort_points_fixed_size()
{
  Eigen::Matrix<float, 3, 3> K;
  K << 500, 0, 320,
       0, 500, 240,
       0, 0, 1;

  Eigen::Matrix<float, 1, 5> dist;
  dist << 0.1, -0.15, 0.001, 0.001, 0.05;

  std::array<Keypoint, 2> keypoints = {Keypoint(330, 250), Keypoint(315, 235)};
  Eigen::Matrix<float, 2, 3> undistorted_points;

  undistort_points<float, Keypoint, 2>(keypoints, 
                                       K,
                                       dist,
                                       undistorted_points);

  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(0, 0), 330, 0.1);
  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(0, 1), 250, 0.1);
  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(1, 0), 315, 0.1);
  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(1, 1), 235, 0.1);
}

void test_undistort_points_dynamic_size()
{
  Eigen::Matrix<float, 3, 3> K;
  K << 500, 0, 320,
       0, 500, 240,
       0, 0, 1;

  Eigen::Matrix<float, 1, 5> dist;
  dist << 0.1, -0.15, 0.001, 0.001, 0.05;

  std::array<Keypoint, 5> keypoints = {Keypoint(330, 250), Keypoint(315, 235), Keypoint(300, 220), Keypoint(285, 205), Keypoint(270, 190)};
  Eigen::Matrix<float, Eigen::Dynamic, 3, 0, 5, 3> undistorted_points(5, 3);
  int num_features = 5;

  undistort_points<float, Keypoint, 5>(keypoints,
                                       K,
                                       dist,
                                       undistorted_points,
                                       num_features);

  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(0, 0), 330, 0.1);
  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(1, 0), 315, 0.1);
  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(2, 0), 300, 0.1);
  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(3, 0), 285, 0.1);
  ENTO_TEST_CHECK_APPROX_EQ(undistorted_points(4, 0), 270, 0.1);
}

void test_iso_normalize_points_fixed_size()
{
  Eigen::Matrix<float, 5, 3> points;
  points << 100, 50, 1,
            200, 100, 1,
            300, 150, 1,
            400, 200, 1,
            500, 250, 1;

  Eigen::Matrix<float, 3, 3> T;
  iso_normalize_points(points, T);

  Eigen::Matrix<float, 1, 2> centroid = points.leftCols(2).colwise().mean();
  ENTO_TEST_CHECK_APPROX_EQ(centroid(0, 0), 0, 0.01);
  ENTO_TEST_CHECK_APPROX_EQ(centroid(0, 1), 0, 0.01);

  float mean_distance = 0;
  for (int i = 0; i < points.rows(); ++i)
  {
    mean_distance += std::sqrt(points(i, 0) * points(i, 0) + points(i, 1) * points(i, 1));
  }
  mean_distance /= points.rows();
  ENTO_TEST_CHECK_APPROX_EQ(mean_distance, std::sqrt(2), 0.01);
}

void test_iso_normalize_points_dynamic_size()
{
  Eigen::Matrix<float, Eigen::Dynamic, 3, 0, 5, 3> points(5, 3);
  points << 100, 50, 1,
            200, 100, 1,
            300, 150, 1,
            400, 200, 1,
            500, 250, 1;

  Eigen::Matrix<float, 3, 3> T;
  iso_normalize_points(points, T);

  Eigen::Matrix<float, 1, 2> centroid = points.leftCols(2).colwise().mean();
  ENTO_TEST_CHECK_APPROX_EQ(centroid(0, 0), 0, 0.01);
  ENTO_TEST_CHECK_APPROX_EQ(centroid(0, 1), 0, 0.01);

  float mean_distance = 0;
  for (int i = 0; i < points.rows(); ++i)
  {
    mean_distance += std::sqrt(points(i, 0) * points(i, 0) + points(i, 1) * points(i, 1));
  }
  mean_distance /= points.rows();
  ENTO_TEST_CHECK_APPROX_EQ(mean_distance, std::sqrt(2), 0.01);
}

void test_unnormalize_homography()
{
  // Set up a homography matrix H
  Eigen::Matrix<float, 3, 3> H;
  H << 1, 0.5, 100,
       0.3, 1, 200,
       0.001, 0.002, 1;

  // Set up normalization transformations T1 and T2
  Eigen::Matrix<float, 3, 3> T1, T2;
  T1 << 0.01, 0, -1,
        0, 0.01, -2,
        0, 0, 1;

  T2 << 0.02, 0, -2,
        0, 0.02, -3,
        0, 0, 1;

  // Calculate the unnormalized homography
  Eigen::Matrix<float, 3, 3> result;
  unnormalize_homography(H, T1, T2, result);

  // Compute expected result manually
  Eigen::Matrix<float, 3, 3> expected_result = T1.inverse() * H * T2;

  // Verify each element of the resulting homography matrix
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      ENTO_TEST_CHECK_APPROX_EQ(result(i, j), expected_result(i, j), 0.01);
    }
  }
}

int main( int argc, char** argv )
{
  int __n = ( argc == 1 ) ? 0 : atoi( argv[1] );
  if (__ento_test_num(__n, 1))  test_keypoint_default_constructor();
  if (__ento_test_num(__n, 2))  test_keypoint_parameterized_constructor();
  if (__ento_test_num(__n, 3))  test_keypoint_copy_constructor();
  if (__ento_test_num(__n, 4))  test_fast_keypoint_default_constructor();
  if (__ento_test_num(__n, 5))  test_fast_keypoint_default_constructor();
  if (__ento_test_num(__n, 6))  test_fast_keypoint_parameterized_constructor();
  if (__ento_test_num(__n, 7))  test_orb_keypoint_default_constructor();
  if (__ento_test_num(__n, 8))  test_fast_keypoint_default_constructor();
  if (__ento_test_num(__n, 9))  test_undistort_points_fixed_size();
  if (__ento_test_num(__n, 10)) test_undistort_points_dynamic_size();
  if (__ento_test_num(__n, 11)) test_iso_normalize_points_fixed_size();
  if (__ento_test_num(__n, 12)) test_iso_normalize_points_dynamic_size();
  if (__ento_test_num(__n, 13)) test_unnormalize_homography();
  
}
