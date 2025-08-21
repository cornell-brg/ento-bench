#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-pose/data_gen.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/ransac.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoPose;

void bench_gold_standard_bundle_adjustment()
{
  using Scalar = float;
  
  // Test data
  EntoContainer<Vec2<Scalar>, 100> points2D;
  EntoContainer<Vec3<Scalar>, 100> points3D;
  
  // Generate synthetic data with outliers
  ProblemOptions options;
  options.n_point_point_ = 100;
  options.noise_2d_ = 1.0;
  options.noise_3d_ = 0.01;
  options.outlier_ratio_ = 0.3;  // 30% outliers
  
  CameraPose<Scalar> true_pose;
  generate_absolute_pose_problem(options, &points2D, &points3D, &true_pose);
  
  // Setup RANSAC options with bundle adjustment
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.max_reproj_error = 2.0;
  ransac_opt.refinement_type = RefinementType::BUNDLE_ADJUSTMENT;
  
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, 100> inliers;
  
  printf("Testing LO-RANSAC with Bundle Adjustment refinement...\n");
  auto start_time = get_time_us();
  
  RansacStats<Scalar> stats = lo_ransac_pnp<Scalar, 100, 3>(
    points2D, points3D, ransac_opt, &estimated_pose, &inliers);
  
  auto end_time = get_time_us();
  Scalar pose_error = (estimated_pose.t - true_pose.t).norm();
  
  printf("Bundle Adjustment Results:\n");
  printf("  Time: %lu us\n", end_time - start_time);
  printf("  Iterations: %zu\n", stats.iterations);
  printf("  Refinements: %zu\n", stats.refinements);
  printf("  Inliers: %zu/%zu (%.1f%%)\n", stats.num_inliers, points2D.size(), 
         100.0 * stats.inlier_ratio);
  printf("  Pose Error: %.6f\n", pose_error);
  printf("  MSAC Score: %.6f\n", stats.model_score);
}

void bench_gold_standard_linear_refinement()
{
  using Scalar = float;
  
  // Test data
  EntoContainer<Vec2<Scalar>, 100> points2D;
  EntoContainer<Vec3<Scalar>, 100> points3D;
  
  // Generate synthetic data with outliers
  ProblemOptions options;
  options.n_point_point_ = 100;
  options.noise_2d_ = 1.0;
  options.noise_3d_ = 0.01;
  options.outlier_ratio_ = 0.3;  // 30% outliers
  
  CameraPose<Scalar> true_pose;
  generate_absolute_pose_problem(options, &points2D, &points3D, &true_pose);
  
  // Setup RANSAC options with linear refinement
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.max_reproj_error = 2.0;
  ransac_opt.refinement_type = RefinementType::LINEAR_SOLVE;
  
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, 100> inliers;
  
  printf("Testing LO-RANSAC with Linear refinement...\n");
  auto start_time = get_time_us();
  
  RansacStats<Scalar> stats = lo_ransac_pnp<Scalar, 100, 3>(
    points2D, points3D, ransac_opt, &estimated_pose, &inliers);
  
  auto end_time = get_time_us();
  Scalar pose_error = (estimated_pose.t - true_pose.t).norm();
  
  printf("Linear Refinement Results:\n");
  printf("  Time: %lu us\n", end_time - start_time);
  printf("  Iterations: %zu\n", stats.iterations);
  printf("  Refinements: %zu\n", stats.refinements);
  printf("  Inliers: %zu/%zu (%.1f%%)\n", stats.num_inliers, points2D.size(), 
         100.0 * stats.inlier_ratio);
  printf("  Pose Error: %.6f\n", pose_error);
  printf("  MSAC Score: %.6f\n", stats.model_score);
}

int main()
{
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();
  software_delay_cycles(10000);

  printf("================\n");
  printf("Gold Standard LO-RANSAC Benchmark\n");
  printf("================\n");
  software_delay_cycles(10000);

  bench_gold_standard_bundle_adjustment();
  printf("\n");
  bench_gold_standard_linear_refinement();

  printf("\n================\n");
  printf("Benchmark Complete\n");

  exit(1);
  return 0;
} 