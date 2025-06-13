// Upright Planar 3pt (Linear Method) Benchmark
// Part of ento-bench pose estimation benchmarks

#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
// TODO: Add pose estimation specific includes
// #include <ento-pose/upright_planar.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;

int main()
{
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "pose-est/upright_planar_3pt3_dataset.txt";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_upright_planar_3pt!");
  }

  // TODO: Initialize upright planar 3pt linear solver
  // TODO: Load test correspondences for upright planar motion
  // TODO: Run benchmark with timing measurements
  // TODO: Validate pose estimation accuracy

  printf("File path: %s", dataset_path);
  ENTO_DEBUG("Upright Planar 3pt Linear Method benchmark not yet implemented - placeholder");

  exit(1);
  return 0;
} 
