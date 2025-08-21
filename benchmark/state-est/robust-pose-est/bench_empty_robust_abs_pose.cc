#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <ento-pose/problem-types/robust_pose_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

namespace EntoPose {

template <typename Scalar>
struct DummyRobustAbsSolver {
  static constexpr size_t MaxSolns = 1;

  template <typename... Args>
  static RansacStats<Scalar> solve(const Args&...) {
    asm volatile("" ::: "memory");
    return {};
  }
};

} // namespace EntoPose

int main()
{
  using Scalar = float;
  using Solver = EntoPose::DummyRobustAbsSolver<Scalar>;
  using Problem = EntoPose::RobustAbsolutePoseProblem<Scalar, Solver, 64>;

  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(Solver{});
  BenchHarness harness(problem, "Empty Robust Abs Pose Baseline",
                       "dummy_input.txt", "dummy_output.txt");
  harness.run();
  return 0;
} 