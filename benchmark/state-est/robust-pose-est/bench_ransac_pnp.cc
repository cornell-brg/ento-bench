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

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;

int main()
{
  using Scalar  = float;
  using Solver  = EntoPose::SolverRANSACPnP<Scalar>;
  using Problem = EntoPose::RobustAbsolutePoseProblem<Scalar, Solver, 100>;
  constexpr Scalar tol = 1e-2;  // More relaxed tolerance for robust estimation
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();
  software_delay_cycles(10000);

  const char* base_path = DATASET_PATH;
  const char* rel_path = "robust-pose/ransac_pnp_float_1000.csv";
  char dataset_path[512];
  char output_path[256];

  printf("================\n");
  printf("Running bench_ransac_pnp...\n");
  software_delay_cycles(10000);

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    software_delay_cycles(10000);
    printf("ERROR! Could not build file path for bench_ransac_pnp!");
  }
  printf("...\n");
  printf("%s\n", dataset_path);
  software_delay_cycles(10000);
  Problem problem(Solver{});

  printf("File path: %s", dataset_path);
  EntoBench::Harness harness(problem, "Bench RANSAC PnP [float]",
                             dataset_path,
                             output_path);

  harness.run();

  exit(1);
  return 0;
} 