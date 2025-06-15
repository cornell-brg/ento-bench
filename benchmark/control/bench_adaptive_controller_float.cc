#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>

#include <ento-control/control_problem.h>
#include <ento-control/adaptive_controller.h>

#include <Eigen/Dense>
#include <vector>
#include <array>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoControl;

int main()
{
  using Scalar = float;
  using Solver = EntoControl::AdaptiveController;
  constexpr int num_states = 6;    // AdaptiveController now has 6 states: [x,y,z,roll,pitch,yaw]
  constexpr int num_inputs = 3;    // AdaptiveController has 3 control inputs: [thrust, roll_torque, pitch_torque]
  constexpr int len_horizon = 10;  // Horizon length for the problem
  constexpr int path_len = 100;   // Maximum trajectory length (we have 955 points)
  
  using Problem = AdaptiveControlProblem<Scalar, num_states, num_inputs, len_horizon, path_len>;
  constexpr Scalar tol = 1e-4;
  
  initialise_monitor_handles();

  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "control/adaptive_controller_trajectory.csv";  // Use existing hover trajectory
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_adaptive_controller_float!");
  }

  // Create solver with 10ms time step (matching RoboBee dynamics)
  AdaptiveController solver(0.01f);
  Problem problem(solver);

  printf("File path: %s\n", dataset_path);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench Adaptive Controller [float]",
                       dataset_path, output_path);

  harness.run();

  exit(1);
  return 0;
} 