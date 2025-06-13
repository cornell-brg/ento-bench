#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

#include <ento-pose/data_gen.h>
#include <ento-pose/problem-types/relative_pose_problem.h>

using namespace EntoBench;
using namespace EntoPose;
using namespace EntoUtil;

using Problem = RelativePoseProblem<float, SolverGoldStandardRel<float>, 0>;

extern "C" void initialise_monitor_handles(void);

int main()
{
  initialise_monitor_handles();

  ENTO_BENCH_SETUP();
  SysTick_Setup();
  __enable_irq();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "rel-pose/8pt_float_noise1.0.csv";
  char dataset_path[512];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_gold_standard_rel_float!");
    return -1;
  }

  SolverGoldStandardRel<float> solver;
  Problem problem(solver);

  using BenchHarness = ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, dataset_path);

  harness.run();

  return 0;
} 