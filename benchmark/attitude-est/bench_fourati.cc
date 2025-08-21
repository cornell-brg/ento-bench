// Fourati Nonlinear Filter Benchmark
// Part of ento-bench attitude estimation benchmarks

#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/fourati_nonlinear.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoAttitude;

int main()
{
  using Scalar = float;
  using Filter = FilterFourati<Scalar>;
  using Problem = AttitudeProblem<Scalar, Filter, true>; // true = MARG (with magnetometer)
  
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "state-est/benchmark_marg_dataset.txt";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_fourati!");
  }

  // Create filter with default constructor
  Filter filter;
  // Create problem with filter (Fourati uses default constructor without gains)
  Problem problem(filter);

  printf("File path: %s", dataset_path);
  EntoBench::Harness harness(problem, "Bench Fourati Nonlinear Filter",
                             dataset_path,
                             output_path);

  harness.run();

  exit(1);
  return 0;
}
