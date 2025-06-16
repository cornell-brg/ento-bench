#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>
#include <ento-control/tinympc_solver.h>
#include <ento-control/opt_control_problem.h>
#include <ento-control/lqr_traits_robofly.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;

// Paper specifications: 10 states, 3 inputs, 5-step horizon
constexpr int path_len = 100;
constexpr int num_states = 10;  // Paper specification
constexpr int num_inputs = 3;   // Paper specification  
constexpr int len_horizon = 5;  // Paper specification (vs our 10)

int main()
{
  using Scalar = float;
  using Solver = TinyMPCSolver<Scalar, num_states, num_inputs, len_horizon>;
  using Problem = OptControlProblem<Scalar, Solver, num_states, num_inputs, len_horizon, path_len>;
  
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
  const char* rel_path = "control/robofly_tinympc_linear.csv";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_tinympc_paper_spec!");
  }

  printf("=== TinyMPC Paper Specification Benchmark ===\n");
  printf("States: %d, Inputs: %d, Horizon: %d\n", num_states, num_inputs, len_horizon);
  printf("Expected complexity: O(N*n^2) = O(%d*%d^2) = %d FLOPs\n", 
         len_horizon, num_states, len_horizon * num_states * num_states);

  // Use RoboFly dynamics but truncate to 10 states, 3 inputs to match paper
  float dt = 0.01f;  // 100 Hz control rate
  Eigen::Matrix<Scalar, num_states, num_states> Ac = RoboFlyLQRTraits::Adyn;
  Eigen::Matrix<Scalar, num_states, num_inputs> Bc = RoboFlyLQRTraits::Bdyn;
  
  Eigen::Matrix<Scalar, num_states, num_states> Adyn = 
      Eigen::Matrix<Scalar, num_states, num_states>::Identity() + Ac * dt;
  Eigen::Matrix<Scalar, num_states, num_inputs> Bdyn = Bc * dt;

  // Cost matrices - paper-like values
  Eigen::Matrix<Scalar, num_states, 1> Q;
  Q << 20.0f, 20.0f, 20.0f,   // position (x,y,z)
       5.0f, 5.0f, 5.0f,       // velocity (vx,vy,vz)
       50.0f, 50.0f,           // orientation (roll, pitch)
       10.0f, 10.0f;           // angular velocity (ωx, ωy)
  
  Eigen::Matrix<Scalar, num_inputs, 1> R;
  R << 1.0f, 1.0f, 1.0f;       // 3 control inputs

  // Constraints (large enough to be non-binding)
  Eigen::Matrix<Scalar, num_states, len_horizon> x_min = 
      Eigen::Matrix<Scalar, num_states, len_horizon>::Constant(-1000.0f);
  Eigen::Matrix<Scalar, num_states, len_horizon> x_max = 
      Eigen::Matrix<Scalar, num_states, len_horizon>::Constant(1000.0f);
  Eigen::Matrix<Scalar, num_inputs, len_horizon-1> u_min = 
      Eigen::Matrix<Scalar, num_inputs, len_horizon-1>::Constant(-100.0f);
  Eigen::Matrix<Scalar, num_inputs, len_horizon-1> u_max = 
      Eigen::Matrix<Scalar, num_inputs, len_horizon-1>::Constant(100.0f);

  float rho = 1.0f;

  // Create solver with paper specifications
  Solver solver(Adyn, Bdyn, Q, R, rho, x_min, x_max, u_min, u_max, false);
  
  // Configure solver settings to match paper (tighter tolerances for faster convergence)
  auto tiny_settings = solver.get_settings();
  solver.update_settings(1e-4f,    // abs_pri_tol (tighter)
                        1e-4f,     // abs_dua_tol (tighter)
                        50,        // max_iter (lower for faster convergence)
                        tiny_settings.check_termination,
                        tiny_settings.en_state_bound,
                        tiny_settings.en_input_bound);

  Problem problem(solver);

  printf("File path: %s\n", dataset_path);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "TinyMPC Paper Spec [10 states, 3 inputs, 5 horizon]",
                       dataset_path, output_path);

  harness.run();

  return 0;
} 