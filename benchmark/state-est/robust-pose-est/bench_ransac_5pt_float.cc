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
#include <ento-pose/robust-est/robust_pose_solver.h>
#include <ento-pose/problem-types/robust_pose_problem.h>

// Include benchmark configuration
#include <ento-bench/bench_config.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;

int main()
{
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // NEW IDIOM: Generic cache setup using configuration
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  using Scalar = float;
  using MinimalSolver = EntoPose::SolverRel5pt<Scalar>;
  constexpr size_t N = 64;
  using CameraModel = EntoPose::SimplePinholeCameraModel<Scalar>;
  using RobustSolver = EntoPose::RobustRelativePoseSolver<MinimalSolver, CameraModel, N>;
  using Problem = EntoPose::RobustRelativePoseProblem<Scalar, RobustSolver, N>;

  const char* base_path = DATASET_PATH;
  const char* rel_path = "robust-pose/5pt_golden.csv";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build dataset path for 5pt pose estimation.");
    exit(1);
  }

  ENTO_DEBUG("RANSAC 5pt: %s", dataset_path);

  // Configure RANSAC options (MCU-optimized parameters)
  EntoPose::RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 10000;
  ransac_opt.min_iters = 20;                    // MCU-optimized minimum
  ransac_opt.max_epipolar_error = 3.0;          // MCU-optimized threshold for 5pt
  ransac_opt.success_prob = 0.99;
  ransac_opt.final_refinement = true;           // Enable bundle adjustment
  
  // Camera configuration (SimplePinholeCameraModel with focal=500.0 to match dataset)  
  using CameraModel = EntoPose::SimplePinholeCameraModel<Scalar>;
  using Params = std::array<Scalar, 3>;
  Params params = {500.0, 250.0, 250.0};  // Principal point at image center (assuming 500x500 image)
  EntoPose::Camera<Scalar, CameraModel> camera(500.0, 500.0, params);  // focal_x=500, focal_y=500

  // Bundle adjustment options (PoseLib-inspired configuration)
  EntoPose::BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = EntoPose::BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.5 * ransac_opt.max_epipolar_error;  // PoseLib heuristic
  bundle_opt.max_iterations = 100;              // PoseLib default
  bundle_opt.verbose = false;

  RobustSolver robust_solver(ransac_opt, bundle_opt, camera, camera);
  static Problem problem(robust_solver);
  
  // NEW IDIOM: Configuration-driven harness type
  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "RANSAC 5pt", dataset_path, output_path);
  
  harness.run();

  exit(1);
  return 0;
} 