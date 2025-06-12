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
#include <ento-pose/rel-pose/upright_planar_three_pt.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;

int main()
{
  using Scalar  = double;
  using Solver  = EntoPose::SolverRelUprightPlanar3pt<Scalar>;
  using Problem = EntoPose::RelativePoseProblem<Scalar, Solver, 3>;
  constexpr Scalar tol = 1e-8;
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  // Build input dataset filepath
  const char* base_path = DATASET_PATH;
  const char* rel_path = "rel-pose/upright_planar_3pt3_double_noise1.0.csv";  // Updated to use 3-point with 1.0 noise dataset
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_upright_planar_3pt_double!");
  }

  ENTO_DEBUG("File path: %s", dataset_path);

  // Construct problem with solver
  Problem problem(Solver{});

  // Construct harness and run
  EntoBench::Harness<Problem, false, 5> harness(problem, "Bench Relative Upright Planar 3pt [double]",
                                                 dataset_path,
                                                 output_path);
  harness.run();

  exit(1);
  return 0;
} 