#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>
#include <ento-control/lqr_traits_robofly.h>
#include <ento-control/lqr_base.h>
#include <ento-control/opt_control_problem.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;

int main()
{
  using Scalar = float;
  using Solver = LQRStep<RoboFlyLQRTraits>;
  constexpr int num_states = RoboFlyLQRTraits::N;  // 10 states
  constexpr int num_inputs = RoboFlyLQRTraits::M;  // 3 inputs
  constexpr int len_horizon = 1;  // LQR is single-step
  constexpr int path_len = 100;
  
  using Problem = OptControlProblem<Scalar, Solver, num_states, num_inputs, len_horizon, path_len>;
  
#if defined(SEMIHOSTING)
  initialise_monitor_handles();
#endif

  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "control/robofly_lqr_linear.csv";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_robofly_lqr!");
  }

  Solver solver;
  Problem problem(solver);

  printf("File path: %s\n", dataset_path);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench RoboFly LQR [float]",
                       dataset_path, output_path);

  harness.run();

  exit(1);
  return 0;
} 