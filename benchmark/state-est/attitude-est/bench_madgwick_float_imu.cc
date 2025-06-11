#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/madgwick.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoAttitude;

int main()
{
  using Scalar = float;
  using Filter = FilterMadgwick<Scalar, false>; // IMU only (no magnetometer)
  using Problem = AttitudeProblem<Scalar, Filter, false>;
  
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "state-est/tuned_icm42688_1khz_imu_dataset.txt";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_madgwick_float_imu!");
  }

  // Create filter and problem with tuned gain (0.001)
  Filter filter;  // Default constructor - no internal state
  Problem problem(filter, 0.001f);  // Pass tuned gain to AttitudeProblem

  printf("File path: %s", dataset_path);
  EntoBench::Harness harness(problem, "Bench Madgwick Float IMU",
                             dataset_path,
                             output_path);

  harness.run();

  exit(1);
  return 0;
} 