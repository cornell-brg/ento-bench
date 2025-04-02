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
#include <ento-pose/rel-pose/eight_pt.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;

int main()
{
  using Scalar  = float;
  using Solver  = EntoPose::SolverRel8pt<Scalar>;
  using Problem = EntoPose::RelativePoseProblem<Scalar, Solver, 8>;
  constexpr Scalar tol = 1e-4;
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
  const char* rel_path = "rel-pose/rel_linear8pt_float_100.csv";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for rel_linear8pt_float_100.csv!");
  }

  ENTO_DEBUG("File path: %s", dataset_path);

  // Construct problem with solver
  Problem problem(Solver{});

  // Construct harness and run.
  //EntoBench::Harness harness(problem, "Bench Relative 8pt [float]",
  //                           dataset_path,
  //                           output_path);
  // Or run each experiment in the datasets for 10 reps
  EntoBench::Harness<Problem, false, 10> harness(problem, "Bench Relative 8pt [float]",
                                                 dataset_path,
                                                 output_path);
  harness.run();

  exit(1);
  return 0;
}

