#include <cstdio>
#include <limits>
#include <Eigen/Dense>
#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/containers.h>
#include <ento-pose/rel-pose/upright_planar_two_pt.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/synthetic_relpose.h>

using namespace std;
using namespace Eigen;
using namespace EntoPose;
using namespace EntoUtil;

void test_upright_planar_two_pt_single_float()
{
  using Scalar = float;
  constexpr size_t N = 2;
  constexpr Scalar tol = 0.5;

  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_upright_planar_two_pt_single_float...");

  EntoContainer<Vec2<Scalar>, N> x1_2d, x2_2d;
  CameraPose<Scalar> true_pose;
  generate_synthetic_relpose_upright_planar<Scalar, N>(x1_2d, x2_2d, true_pose, N, 0.0f);

  EntoContainer<Vec3<Scalar>, N> x1, x2;
  for (size_t i = 0; i < N; ++i) {
    x1[i] = Vec3<Scalar>(x1_2d[i](0), x1_2d[i](1), Scalar(1));
    x1[i].normalize();
    x2[i] = Vec3<Scalar>(x2_2d[i](0), x2_2d[i](1), Scalar(1));
    x2[i].normalize();
  }

  EntoContainer<CameraPose<Scalar>, 4> solutions;
  int num_solns = relpose_upright_planar_2pt<Scalar, N>(x1, x2, &solutions);
  ENTO_DEBUG("Num solutions: %d", num_solns);

  Scalar min_rot_error = std::numeric_limits<Scalar>::max();
  Scalar min_angle_error_deg = std::numeric_limits<Scalar>::max();
  for (int i = 0; i < num_solns; ++i) {
    const CameraPose<Scalar> &pose = solutions[i];
    Scalar rot_error = (pose.R() - true_pose.R()).norm();
    Scalar angle_rad = std::abs(Eigen::AngleAxis<Scalar>(pose.R().transpose() * true_pose.R()).angle());
    Scalar angle_deg = angle_rad * Scalar(180.0 / M_PI);
    ENTO_DEBUG("Solution %d: Rotation error (mat norm) = %f, Angular error = %f deg", i, rot_error, angle_deg);
    ENTO_DEBUG("  Estimated R:\n%f %f %f\n%f %f %f\n%f %f %f", pose.R()(0,0), pose.R()(0,1), pose.R()(0,2), pose.R()(1,0), pose.R()(1,1), pose.R()(1,2), pose.R()(2,0), pose.R()(2,1), pose.R()(2,2));
    ENTO_DEBUG("  True R:\n%f %f %f\n%f %f %f\n%f %f %f", true_pose.R()(0,0), true_pose.R()(0,1), true_pose.R()(0,2), true_pose.R()(1,0), true_pose.R()(1,1), true_pose.R()(1,2), true_pose.R()(2,0), true_pose.R()(2,1), true_pose.R()(2,2));
    ENTO_DEBUG("  Estimated t: %f %f %f", pose.t.x(), pose.t.y(), pose.t.z());
    ENTO_DEBUG("  True t: %f %f %f", true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
    min_rot_error = std::min(min_rot_error, rot_error);
    min_angle_error_deg = std::min(min_angle_error_deg, angle_deg);
  }

  bool found_gt_pose = (min_angle_error_deg < tol);
  ENTO_TEST_CHECK_TRUE(found_gt_pose);
  ENTO_DEBUG("================\n");
}

void test_upright_planar_two_pt_single_double()
{
  using Scalar = double;
  constexpr size_t N = 2;
  constexpr Scalar tol = 1.0;

  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_upright_planar_two_pt_single_double...");

  EntoContainer<Vec2<Scalar>, N> x1_2d, x2_2d;
  CameraPose<Scalar> true_pose;
  generate_synthetic_relpose_upright_planar<Scalar, N>(x1_2d, x2_2d, true_pose, N, 0.0);

  EntoContainer<Vec3<Scalar>, N> x1, x2;
  for (size_t i = 0; i < N; ++i) {
    x1[i] = Vec3<Scalar>(x1_2d[i](0), x1_2d[i](1), Scalar(1));
    x1[i].normalize();
    x2[i] = Vec3<Scalar>(x2_2d[i](0), x2_2d[i](1), Scalar(1));
    x2[i].normalize();
  }

  EntoContainer<CameraPose<Scalar>, 4> solutions;
  int num_solns = relpose_upright_planar_2pt<Scalar, N>(x1, x2, &solutions);
  ENTO_DEBUG("Num solutions: %d", num_solns);

  Scalar min_rot_error = std::numeric_limits<Scalar>::max();
  Scalar min_angle_error_deg = std::numeric_limits<Scalar>::max();
  for (int i = 0; i < num_solns; ++i) {
    const CameraPose<Scalar> &pose = solutions[i];
    Scalar rot_error = (pose.R() - true_pose.R()).norm();
    Scalar angle_rad = std::abs(Eigen::AngleAxis<Scalar>(pose.R().transpose() * true_pose.R()).angle());
    Scalar angle_deg = angle_rad * Scalar(180.0 / M_PI);
    ENTO_DEBUG("Solution %d: Rotation error (mat norm) = %f, Angular error = %f deg", i, rot_error, angle_deg);
    ENTO_DEBUG("  Estimated R:\n%f %f %f\n%f %f %f\n%f %f %f", pose.R()(0,0), pose.R()(0,1), pose.R()(0,2), pose.R()(1,0), pose.R()(1,1), pose.R()(1,2), pose.R()(2,0), pose.R()(2,1), pose.R()(2,2));
    ENTO_DEBUG("  True R:\n%f %f %f\n%f %f %f\n%f %f %f", true_pose.R()(0,0), true_pose.R()(0,1), true_pose.R()(0,2), true_pose.R()(1,0), true_pose.R()(1,1), true_pose.R()(1,2), true_pose.R()(2,0), true_pose.R()(2,1), true_pose.R()(2,2));
    ENTO_DEBUG("  Estimated t: %f %f %f", pose.t.x(), pose.t.y(), pose.t.z());
    ENTO_DEBUG("  True t: %f %f %f", true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
    min_rot_error = std::min(min_rot_error, rot_error);
    min_angle_error_deg = std::min(min_angle_error_deg, angle_deg);
  }

  bool found_gt_pose = (min_angle_error_deg < tol);
  ENTO_TEST_CHECK_TRUE(found_gt_pose);
  ENTO_DEBUG("================\n");
}

int main(int argc, char **argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1) {
    __n = atoi(argv[1]);
  } else {
    __ento_replace_file_suffix(__FILE__, "test_pose_est_reader_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }
  if (__ento_test_num(__n, 1)) test_upright_planar_two_pt_single_float();
  if (__ento_test_num(__n, 2)) test_upright_planar_two_pt_single_double();
} 