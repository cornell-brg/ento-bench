#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-pose/data_gen.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/abs-pose/dlt.h>
#include <ento-pose/robust-est/robust_pose_solver.h>
#include <ento-pose/problem-types/robust_pose_problem.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoPose;

int main()
{
  using Scalar = float;
  using MinimalSolver = SolverDLT<Scalar>;
  constexpr size_t N = 100;  // Support up to 100 points for robust estimation
  using RansacSolver = RobustAbsolutePoseSolver<MinimalSolver, N>;
  using Problem = RobustAbsolutePoseProblem<Scalar, RansacSolver, N>;
  
#if defined(SEMIHOSTING)
  initialise_monitor_handles();
#endif

  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "abs-pose/dlt_noise_0.010_outliers_0.250.csv";  // LO-RANSAC dataset with outliers
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_lo_ransac_dlt_float!");
  }

  // Configure RANSAC options for LO-RANSAC
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 1000;
  ransac_opt.max_reproj_error = 3.0;
  ransac_opt.success_prob = 0.99;
  ransac_opt.final_refinement = false;
  
  // Enable Linear Local Optimization (LO-RANSAC)
  // For DLT, we can use the same DLT method for refinement
  ransac_opt.lo_type = LocalRefinementType::Linear;
  ransac_opt.linear_method = LinearRefinementMethod::DLT;
  ransac_opt.use_irls = false;

  // Camera configuration
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);

  // Bundle adjustment options (not used for linear refinement)
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.5 * ransac_opt.max_reproj_error;  // PoseLib heuristic
  bundle_opt.max_iterations = 100;  // PoseLib default
  bundle_opt.verbose = false;

  RansacSolver robust_solver(ransac_opt, bundle_opt, camera);
  Problem problem(robust_solver);

  printf("File path: %s\n", dataset_path);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench LO-RANSAC DLT Linear [float]",
                       dataset_path, output_path);

  harness.run();

  exit(1);
  return 0;
} 