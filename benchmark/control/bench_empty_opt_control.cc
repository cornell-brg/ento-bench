#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <Eigen/Dense>
#include <ento-control/opt_control_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

// Dummy solver emulating Optimal Control solver (MPC/LQR) interface

template<int StateSize, int CtrlSize, int HorizonSize>
struct DummyOptSolver {
  using State   = Eigen::Matrix<float, StateSize, 1>;
  using Control = Eigen::Matrix<float, CtrlSize, 1>;

  bool set_x0(const State&) { return true; }
  void set_x_ref(const Eigen::Matrix<float, StateSize, HorizonSize>&) {}
  void reset_duals() {}
  bool solve() { return true; }
  Control get_u0() const { return Control::Zero(); }
  int get_status() const { return 0; }
  int get_iterations() const { return 0; }
};

int main() {
  constexpr int num_states = 12;
  constexpr int num_inputs = 3;
  constexpr int len_horizon = 3;
  constexpr int path_len = 100;

  using Solver  = DummyOptSolver<num_states, num_inputs, len_horizon>;
  using Problem = OptControlProblem<float, Solver, num_states, num_inputs, len_horizon, path_len>;

  // Configure clock and caches
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Solver solver;
  Problem problem(solver);
  BenchHarness harness(problem, "Empty Optimal Control Baseline", "", "");
  harness.run();
  return 0;
} 