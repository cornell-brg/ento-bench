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

constexpr int path_len = 500;  // Longer path for square trajectory
constexpr int num_states = RoboFlyLQRTraits::N;  // 10 states
constexpr int num_inputs = RoboFlyLQRTraits::M;  // 3 inputs
constexpr int len_horizon = 10;

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
  const char* rel_path = "control/robofly_square_robognat.csv";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_robofly_tinympc_square!");
  }

  // Discretize RoboFly dynamics for TinyMPC
  float dt = 0.01f;  // 100 Hz control rate
  Eigen::Matrix<Scalar, num_states, num_states> Ac = RoboFlyLQRTraits::Adyn;
  Eigen::Matrix<Scalar, num_states, num_inputs> Bc = RoboFlyLQRTraits::Bdyn;
  
  Eigen::Matrix<Scalar, num_states, num_states> Adyn = 
      Eigen::Matrix<Scalar, num_states, num_states>::Identity() + Ac * dt;
  Eigen::Matrix<Scalar, num_states, num_inputs> Bdyn = Bc * dt;

  // Higher cost matrices for aggressive tracking of square trajectory
  Eigen::Matrix<Scalar, num_states, 1> Q;
  Q << 50.0f, 50.0f, 50.0f,   // position (x,y,z) - higher for tight tracking
       10.0f, 10.0f, 10.0f,   // velocity (vx,vy,vz) - higher for smooth motion
       100.0f, 100.0f,        // orientation (roll, pitch) - higher for stability
       20.0f, 20.0f;          // angular velocity (ωx, ωy) - higher for damping
  
  Eigen::Matrix<Scalar, num_inputs, 1> R;
  R << 2.0f, 2.0f, 2.0f;     // 3 control inputs - slightly higher for smoother control

  // Constraints (large enough to be non-binding)
  Eigen::Matrix<Scalar, num_states, len_horizon> x_min = 
      Eigen::Matrix<Scalar, num_states, len_horizon>::Constant(-1000.0f);
  Eigen::Matrix<Scalar, num_states, len_horizon> x_max = 
      Eigen::Matrix<Scalar, num_states, len_horizon>::Constant(1000.0f);
  Eigen::Matrix<Scalar, num_inputs, len_horizon-1> u_min = 
      Eigen::Matrix<Scalar, num_inputs, len_horizon-1>::Constant(-100.0f);
  Eigen::Matrix<Scalar, num_inputs, len_horizon-1> u_max = 
      Eigen::Matrix<Scalar, num_inputs, len_horizon-1>::Constant(100.0f);

  float rho = 2.0f;  // Higher rho for faster convergence on dynamic trajectory

  // Create solver with RoboFly dynamics
  Solver solver(Adyn, Bdyn, Q, R, rho, x_min, x_max, u_min, u_max, false);
  
  // Configure solver settings - tighter tolerances for better tracking
  auto tiny_settings = solver.get_settings();
  solver.update_settings(1e-4f,  // tighter primal tolerance
                        1e-4f,   // tighter dual tolerance
                        150,     // more iterations for complex trajectory
                        tiny_settings.check_termination,
                        tiny_settings.en_state_bound,
                        tiny_settings.en_input_bound);

  Problem problem(solver);

  printf("File path: %s\n", dataset_path);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench RoboFly TinyMPC Square [float]",
                       dataset_path, output_path);

  harness.run();

  exit(1);
  return 0;
} 