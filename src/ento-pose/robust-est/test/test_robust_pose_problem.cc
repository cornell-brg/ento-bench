#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-pose/problem-types/robust_pose_problem.h>
#include <ento-pose/synthetic_relpose.h>
#include <ento-pose/data_gen.h>
#include <ento-pose/robust-est/robust_pose_solver.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/pose_util.h>
#include <sstream>

#ifndef ENTO_DEBUG_ARRAY
#define ENTO_DEBUG_ARRAY(...)
#endif

using namespace EntoPose;
using namespace EntoUtil;

// Dummy solver for now; replace with a real solver when ready
struct DummySolver {};

#include <sstream>
#include <string>
#include <cstdint>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

template <typename Scalar, size_t N>
std::string
make_csv_line_with_inliers(
  uint32_t                                          problem_type,
  const CameraPose<Scalar>                         &pose,
  const EntoContainer<Eigen::Matrix<Scalar,2,1>, N> &x1,
  const EntoContainer<Eigen::Matrix<Scalar,2,1>, N> &x2,
  const EntoContainer<bool, N>                     &inliers,
  Scalar                                            scale = Scalar{1},
  Scalar                                            focal = Scalar{1})
{
  using Vec2 = Eigen::Matrix<Scalar,2,1>;
  std::ostringstream oss;

  // 1) problem_type, N
  oss << problem_type << ',' << N << ',';

  // 2) quaternion (w, x, y, z)
  oss << pose.q.w() << ','
      << pose.q.x() << ','
      << pose.q.y() << ','
      << pose.q.z() << ',';

  // 3) translation (tx, ty, tz)
  oss << pose.t.x() << ','
      << pose.t.y() << ','
      << pose.t.z() << ',';

  // 4) camera parameters (scale, focal)
  oss << scale << ',' << focal << ',';

  // 5) x1: dump N points (x, y)
  for (size_t i = 0; i < N; ++i) {
    const Vec2 &pt = x1[i];
    oss << pt(0) << ',' << pt(1) << ',';
  }

  // 6) x2: dump N points (x, y)
  for (size_t i = 0; i < N; ++i) {
    const Vec2 &pt = x2[i];
    oss << pt(0) << ',' << pt(1) << ',';
  }

  // 7) pack “inliers” (bool flags) into bytes
  constexpr size_t kNumBytes = (N + 7) / 8;
  std::array<uint8_t, kNumBytes> mask_bytes{};
  for (size_t i = 0; i < N; ++i) {
    if (inliers[i]) {
      size_t byte_idx = i / 8;
      size_t bit_idx  = i % 8; // LSB = point‐0, next bit = point‐1, etc.
      mask_bytes[byte_idx] |= static_cast<uint8_t>(1u << bit_idx);
    }
  }

  // 8) append each byte in decimal.  If N<=8, that's one byte; otherwise multiple.
  for (size_t b = 0; b < kNumBytes; ++b) {
    oss << static_cast<uint32_t>(mask_bytes[b]);
    if (b + 1 < kNumBytes) {
      oss << ',';  // comma‐separate multiple inlier‐bytes
    }
  }

  return oss.str();
}

void test_robust_relative_pose_problem_basic()
{
    ENTO_DEBUG("Running test_robust_relative_pose_problem_basic...");
    using Scalar = float;
    constexpr size_t N = 8;

    // Generate synthetic data
    EntoContainer<Vec2<Scalar>, N> x1, x2;
    CameraPose<Scalar> true_pose;
    generate_synthetic_relpose_general<Scalar, N>(x1, x2, true_pose, N, 0.0);

    // Instantiate the robust problem (replace DummySolver with a real solver when ready)
    RobustRelativePoseProblem<Scalar, DummySolver, N> problem(DummySolver{});
    problem.initialize_inliers();

    // Set/check inlier mask
    problem.set_inlier(3, true);
    ENTO_TEST_CHECK_TRUE(problem.is_inlier(3));
    problem.set_inlier(3, false);
    ENTO_TEST_CHECK_TRUE(!problem.is_inlier(3));

    ENTO_DEBUG("test_robust_relative_pose_problem_basic passed.");
}

void test_robust_relative_pose_problem_deserialize_inlier_mask()
{
    ENTO_DEBUG("Running test_robust_relative_pose_problem_deserialize_inlier_mask...");
    using Scalar = float;
    constexpr size_t N = 8;
    RobustRelativePoseProblem<Scalar, DummySolver, N> problem(DummySolver{});
    // Compose a CSV line for inlier mask only (no trailing comma, as would be in a real file)
    // Only 1 byte for N=8
    char csv_line[64] = "1";
    char* pos = csv_line;
    bool ok = problem.deserialize_inlier_mask(pos);

    // Print the inlier mask bytes and bits (debugging already fixed by user)
    ENTO_DEBUG_ARRAY(problem.inlier_flags());
    for (size_t i = 0; i < problem.inlier_flags().size(); ++i) {
        std::string bits;
        for (int b = 7; b >= 0; --b) {
            bits += ((problem.inlier_flags()[i] >> b) & 1) ? '1' : '0';
        }
        ENTO_DEBUG("inlier_flags_[%zu] bits: %s", i, bits.c_str());
    }

    ENTO_TEST_CHECK_TRUE(ok);
    ENTO_TEST_CHECK_TRUE(problem.is_inlier(0)); // 1st bit of 1 is set
    ENTO_TEST_CHECK_TRUE(!problem.is_inlier(1)); // 2nd bit of 1 is not set
    // No check for is_inlier(16) since N=8 (static-size)
    ENTO_DEBUG("test_robust_relative_pose_problem_deserialize_inlier_mask passed.");
}

void test_robust_relative_pose_problem_static_large_inlier_mask()
{
    ENTO_DEBUG("Running test_robust_relative_pose_problem_static_large_inlier_mask...");
    using Scalar = float;
    constexpr size_t N = 16; // static-size, 2 bytes
    RobustRelativePoseProblem<Scalar, DummySolver, N> problem(DummySolver{});
    problem.initialize_inliers();
    // Inlier mask: 0b10101010, 0b01010101
    // CSV: "170,85"
    char csv_line[64] = "170,85";
    ENTO_DEBUG("CSV input: %s", csv_line);
    char* pos = csv_line;
    bool ok = problem.deserialize_inlier_mask(pos);
    ENTO_DEBUG_ARRAY(problem.inlier_flags());
    for (size_t i = 0; i < problem.inlier_flags().size(); ++i) {
        std::string bits;
        for (int b = 0; b < 8; ++b) { // LSB first
            bits += ((problem.inlier_flags()[i] >> b) & 1) ? '1' : '0';
        }
        ENTO_DEBUG("inlier_flags_[%zu] bits (LSB first): %s", i, bits.c_str());
    }
    ENTO_TEST_CHECK_TRUE(ok);
    // Expected bits: [0]=170: 0 1 0 1 0 1 0 1, [1]=85: 1 0 1 0 1 0 1 0
    int expected[16] = {0,1,0,1,0,1,0,1, 1,0,1,0,1,0,1,0};
    for (int i = 0; i < 16; ++i) {
        ENTO_TEST_CHECK_TRUE(problem.is_inlier(i) == (expected[i] != 0));
    }
    ENTO_DEBUG("test_robust_relative_pose_problem_static_large_inlier_mask passed.");
}

void test_robust_relative_pose_problem_full_line()
{
    ENTO_DEBUG("Running test_robust_relative_pose_problem_full_line...");
    using Scalar = float;
    constexpr size_t N = 4; // static-size, 4 points
    std::string csv_line =
        "3,4," // problem_type, num_pts
        "1,0,0,0," // q
        "0,0,0," // t
        "1,1," // scale, focal
        "1,1,2,2,3,3,4,4," // x1
        "5,5,6,6,7,7,8,8," // x2
        "15"; // inlier mask (0b00001111)
    ENTO_DEBUG("CSV line: %s", csv_line.c_str());
    RobustRelativePoseProblem<Scalar, DummySolver, N> problem(DummySolver{});
    bool ok = problem.deserialize_impl(csv_line.c_str());
    ENTO_TEST_CHECK_TRUE(ok);
    ENTO_DEBUG_ARRAY(problem.inlier_flags());
    ENTO_TEST_CHECK_TRUE(problem.inlier_flags().size() == 1); // Only 1 byte needed for 4 points
    for (size_t i = 0; i < problem.inlier_flags().size(); ++i) {
        std::string bits;
        for (int b = 0; b < 8; ++b) { // LSB first
            bits += ((problem.inlier_flags()[i] >> b) & 1) ? '1' : '0';
        }
        ENTO_DEBUG("inlier_flags_[%zu] bits (LSB first): %s", i, bits.c_str());
    }
    // Check inlier mask
    int expected[4] = {1,1,1,1};
    for (int i = 0; i < 4; ++i) {
        ENTO_TEST_CHECK_TRUE(problem.is_inlier(i) == (expected[i] != 0));
    }
    ENTO_DEBUG("test_robust_relative_pose_problem_full_line passed.");
}

void test_robust_relative_pose_problem_with_solver()
{
    ENTO_DEBUG("Running test_robust_relative_pose_problem_with_solver...");
    using Scalar = float;
    constexpr size_t N = 20;  // We'll use 8 points but only need 5 for the solver
    constexpr size_t num_outliers = 0;

    // Generate synthetic data with some noise
    EntoContainer<Vec2<Scalar>, N> x1, x2;
    CameraPose<Scalar> true_pose;
    EntoContainer<Vec3<Scalar>, N> x1_bear;
    EntoContainer<Vec3<Scalar>, N> x2_bear;
    EntoPose::generate_synthetic_relpose_bearing_vectors<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.01);
    std::default_random_engine rng(123);
    std::normal_distribution<Scalar> outlier_gen(0.0, 1.0);
    for (size_t i = 0; i < num_outliers; ++i) {
      Vec3<Scalar> x1_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
      Vec3<Scalar> x2_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
      x1_outlier.normalize();
      x2_outlier.normalize();
      x1_bear.push_back(x1_outlier);
      x2_bear.push_back(x2_outlier);
    }
    EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x1_bear, x1);
    EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x2_bear, x2);

    EntoContainer<bool, N> inlier_flags;
    for (size_t i = 0; i < N; ++i) {
      inlier_flags.push_back(true);
    }

    std::string csv_line = make_csv_line_with_inliers<Scalar, N>(
      3,
      true_pose,
      x1,
      x2,
      inlier_flags);

    // Create a solver that uses the 5-point algorithm
    using MinimalSolver = SolverRel5pt<Scalar>;
    using RansacSolver = RobustRelativePoseSolver<MinimalSolver, N>;

    RansacOptions<Scalar> ransac_opt;
    ransac_opt.max_iters = 1000;
    ransac_opt.max_epipolar_error = 0.1;
    ransac_opt.success_prob = 0.99;

    using CameraModel = IdentityCameraModel<Scalar>;
    using Params = std::array<Scalar, 0>;
    Params params;
    Camera<Scalar, CameraModel> camera(1.0, 1.0, params);

    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.loss_scale = 0.1;
    bundle_opt.max_iterations = 25;

    RansacSolver robust_solver(ransac_opt, bundle_opt, camera, camera);
    // Instantiate the robust problem with the 5-point solver
    RobustRelativePoseProblem<Scalar, RansacSolver, N> problem(robust_solver);

    ENTO_DEBUG("CSV line: %s", csv_line.c_str());

    problem.deserialize_impl(csv_line.c_str());

    // Solve the problem
    ENTO_DEBUG("Running problem...");
    problem.solve_impl();

    CameraPose<Scalar> estimated_pose = problem.best_model_;
    EntoContainer<uint8_t, N> inliers = problem.inliers_; 
    RansacStats<Scalar> stats = problem.ransac_stats_;
    ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 5);
    size_t clean_inliers = 0;
    for (size_t i = 0; i < N; ++i) if (inliers[i]) clean_inliers++;
    ENTO_TEST_CHECK_TRUE(clean_inliers >= stats.num_inliers * 0.8);
    ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
    ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
    double trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
    double angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
    double angle_deg = angle_rad * 180.0 / M_PI;
    //ENTO_TEST_CHECK_TRUE(angle_deg < 10.0);
    ENTO_DEBUG("Upright 3pt (double) test stats:");
    ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
    ENTO_DEBUG("  Clean inliers: %zu/%zu", clean_inliers, N);
    ENTO_DEBUG("  Score: %f", stats.model_score);
    ENTO_DEBUG("  Iterations: %zu", stats.iters);
    double rot_error = (estimated_pose.R() - true_pose.R()).norm();
    ENTO_DEBUG("  Rotation error (matrix norm): %f", rot_error);
    ENTO_DEBUG("  Angular rotation error: %f degrees", angle_deg);
    ENTO_DEBUG("  True pose - R: %f %f %f %f, t: %f %f %f", true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(), true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
    ENTO_DEBUG("  Estimated pose - R: %f %f %f %f, t: %f %f %f", estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(), estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
    ENTO_DEBUG("================");

    // Check that we got some solutions
    ENTO_DEBUG("test_robust_relative_pose_problem_with_solver passed.");
}

int main(int argc, char** argv)
{
    int __n = (argc == 1) ? 0 : atoi(argv[1]);
    if (__ento_test_num(__n, 1)) test_robust_relative_pose_problem_basic();
    if (__ento_test_num(__n, 2)) test_robust_relative_pose_problem_deserialize_inlier_mask();
    if (__ento_test_num(__n, 3)) test_robust_relative_pose_problem_static_large_inlier_mask();
    if (__ento_test_num(__n, 4)) test_robust_relative_pose_problem_full_line();
    if (__ento_test_num(__n, 5)) test_robust_relative_pose_problem_with_solver();
    return 0;
}
