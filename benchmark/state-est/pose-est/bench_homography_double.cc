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
#include <ento-pose/problem-types/homography_problem.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;

int main()
{
  using Scalar  = double;
  using Solver  = EntoPose::SolverHomography4pt<Scalar>;
  using Problem = EntoPose::HomographyProblem<Scalar, Solver, 8>;
  constexpr Scalar tol = 1e-8;
  initialise_monitor_handles();

  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  // Build input dataset filepath
  const char* base_path = DATASET_PATH;
  const char* rel_path = "rel-pose/homography_double_noise1.0.csv";  // Use homography dataset
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_homography_double!");
  }

  ENTO_DEBUG("File path: %s", dataset_path);

  // Construct problem with solver
  Problem problem(Solver{});

  printf("File path: %s\n", dataset_path);

  // Construct harness and run
  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench Homography [double]",
                       dataset_path, output_path);
  harness.run();

  exit(1);
  return 0;
} 