#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <ento-pose/problem-types/relative_pose_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

namespace EntoPose {
// Dummy solver for Relative Pose
template<typename Scalar>
struct DummyRelSolver {
  static constexpr size_t MaxSolns = 1;
  template<size_t N, typename... Args>
  size_t solve(const Args&...) { return 0; }
};
}

int main()
{
  using Scalar = float;
  using Solver = EntoPose::DummyRelSolver<Scalar>;
  using Problem = EntoPose::RelativePoseProblem<Scalar, Solver, 5>; // 0 = variable pts

  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(Solver{});
  BenchHarness harness(problem, "Empty Relative Pose Baseline",
                       "dummy_input.txt", "dummy_output.txt");
  harness.run();
  return 0;
} 