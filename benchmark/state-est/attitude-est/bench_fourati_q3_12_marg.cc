#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/fourati_fixed.h>

#include <ento-bench/bench_config.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoAttitude;

int main()
{
  using Scalar = Q3_12;
  using Filter = FilterFouratiFixed<Scalar>; // MARG only (Fourati requires magnetometer)
  using Problem = AttitudeProblem<Scalar, Filter, true>;
  
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "state-est/tuned_icm42688_1khz_marg_dataset.txt";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_fourati_q3_12_marg!");
  }

  Filter filter;
  Problem problem(filter, Scalar(0.1f));

  printf("File path: %s\n", dataset_path);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench Fourati Q3_12 MARG",
                             dataset_path,
                             output_path);

  harness.run();

  exit(1);
  return 0;
} 