#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <Eigen/Dense>
#include <ento-control/control_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

// Dummy solver to emulate Geometric Controller interface

template<typename Scalar, int StateSize, int CtrlSize>
struct DummyGeometricSolver {
  using StateVec  = Eigen::Matrix<Scalar, StateSize, 1>;
  using CtrlVec   = Eigen::Matrix<Scalar, CtrlSize, 1>;

  DummyGeometricSolver(Scalar dt = Scalar(0.01)) {}

  void set_x0(const StateVec&) {}
  void set_x_ref(const StateVec&) {}
  void solve() {}
  CtrlVec get_u0() const { return CtrlVec::Zero(); }
  StateVec get_next_state() const { return StateVec::Zero(); }
};

int main() {
  using Scalar = float;
  constexpr int num_states = 13;
  constexpr int num_inputs = 4;
  constexpr int len_horizon = 1;
  constexpr int path_len = 100;

  using Solver  = DummyGeometricSolver<Scalar, num_states, num_inputs>;
  using Problem = ReferenceControlProblem<Scalar, Solver,
                                          num_states, num_inputs,
                                          len_horizon, path_len>;

  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  Solver solver(0.01f);
  Problem problem(solver);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Empty Geometric Control Baseline", "", "");
  harness.run();
  return 0;
} 