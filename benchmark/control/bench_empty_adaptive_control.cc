#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <Eigen/Dense>
#include <ento-control/control_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

// Dummy solver emulating Adaptive Controller interface

template<typename Scalar, int StateSize, int CtrlSize>
struct DummyAdaptiveSolver {
  using StateVec = Eigen::Matrix<Scalar, StateSize, 1>;
  using CtrlVec  = Eigen::Matrix<Scalar, CtrlSize, 1>;

  DummyAdaptiveSolver(Scalar dt = Scalar(0.01)) {}

  void set_x0(const StateVec&) {}
  void set_x_ref(const StateVec&) {}
  void solve() {}
  CtrlVec get_u0() const { return CtrlVec::Zero(); }
  StateVec get_next_state() const { return StateVec::Zero(); }
};

int main() {
  using Scalar = float;
  constexpr int num_states = 6;
  constexpr int num_inputs = 3;
  constexpr int len_horizon = 10;
  constexpr int path_len = 100;

  using Solver = DummyAdaptiveSolver<Scalar, num_states, num_inputs>;
  using Problem = ControlProblem<Scalar, Solver, num_states, num_inputs, len_horizon, path_len>;

  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Solver solver(0.01f);
  Problem problem(solver);
  BenchHarness harness(problem, "Empty Adaptive Control Baseline", "", "");
  harness.run();
  return 0;
} 