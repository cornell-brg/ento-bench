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

constexpr int path_len = 100;
constexpr int num_states = 10;
constexpr int num_inputs = 3;

template<int HORIZON>
void benchmark_tinympc_horizon() {
  using Scalar = float;
  using Solver = TinyMPCSolver<Scalar, num_states, num_inputs, HORIZON>;
  using Problem = OptControlProblem<Scalar, Solver, num_states, num_inputs, HORIZON, path_len>;
  
  printf("\n=== TinyMPC Horizon %d Benchmark ===\n", HORIZON);
  printf("States: %d, Inputs: %d, Horizon: %d\n", num_states, num_inputs, HORIZON);
  printf("Expected complexity: O(N*n^2) = O(%d*%d^2) = %d FLOPs\n", 
         HORIZON, num_states, HORIZON * num_states * num_states);

  const char* base_path = DATASET_PATH;
  const char* rel_path = "control/robofly_tinympc_linear.csv";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path!");
    return;
  }

  // Use RoboFly dynamics
  float dt = 0.01f;
  Eigen::Matrix<Scalar, num_states, num_states> Ac = RoboFlyLQRTraits::Adyn;
  Eigen::Matrix<Scalar, num_states, num_inputs> Bc = RoboFlyLQRTraits::Bdyn;
  
  Eigen::Matrix<Scalar, num_states, num_states> Adyn = 
      Eigen::Matrix<Scalar, num_states, num_states>::Identity() + Ac * dt;
  Eigen::Matrix<Scalar, num_states, num_inputs> Bdyn = Bc * dt;

  // Cost matrices
  Eigen::Matrix<Scalar, num_states, 1> Q;
  Q << 20.0f, 20.0f, 20.0f, 5.0f, 5.0f, 5.0f, 50.0f, 50.0f, 10.0f, 10.0f;
  
  Eigen::Matrix<Scalar, num_inputs, 1> R;
  R << 1.0f, 1.0f, 1.0f;

  // Constraints
  Eigen::Matrix<Scalar, num_states, HORIZON> x_min = 
      Eigen::Matrix<Scalar, num_states, HORIZON>::Constant(-1000.0f);
  Eigen::Matrix<Scalar, num_states, HORIZON> x_max = 
      Eigen::Matrix<Scalar, num_states, HORIZON>::Constant(1000.0f);
  Eigen::Matrix<Scalar, num_inputs, HORIZON-1> u_min = 
      Eigen::Matrix<Scalar, num_inputs, HORIZON-1>::Constant(-100.0f);
  Eigen::Matrix<Scalar, num_inputs, HORIZON-1> u_max = 
      Eigen::Matrix<Scalar, num_inputs, HORIZON-1>::Constant(100.0f);

  float rho = 1.0f;

  Solver solver(Adyn, Bdyn, Q, R, rho, x_min, x_max, u_min, u_max, false);
  
  // Configure solver settings
  auto tiny_settings = solver.get_settings();
  solver.update_settings(1e-4f, 1e-4f, 50,
                        tiny_settings.check_termination,
                        tiny_settings.en_state_bound,
                        tiny_settings.en_input_bound);

  Problem problem(solver);

  char benchmark_name[256];
  snprintf(benchmark_name, sizeof(benchmark_name), 
           "TinyMPC Horizon %d [%d states, %d inputs]", HORIZON, num_states, num_inputs);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, benchmark_name, dataset_path, output_path);

  harness.run();
}

int main()
{
  initialise_monitor_handles();

  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  printf("=== TinyMPC Horizon Length Comparison ===\n");
  printf("This benchmark compares computational complexity for different horizon lengths\n");
  printf("Paper claims O(N*n^2) complexity where N=horizon, n=states\n\n");

  // Benchmark different horizon lengths
  benchmark_tinympc_horizon<5>();   // Paper specification: 500 FLOPs expected
  benchmark_tinympc_horizon<10>();  // Our current setup: 1000 FLOPs expected
  benchmark_tinympc_horizon<15>();  // Longer horizon: 1500 FLOPs expected

  printf("\n=== Horizon Comparison Complete ===\n");
  printf("Expected cycle ratios (5:10:15) should be approximately (1:2:3)\n");

  return 0;
} 