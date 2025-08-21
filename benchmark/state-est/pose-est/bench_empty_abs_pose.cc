#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <ento-pose/problem-types/absolute_pose_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

namespace EntoPose {
 template<typename Scalar>
 struct DummyAbsSolver {
   static constexpr size_t MaxSolns = 1;
   template<size_t N, typename... Args>
   size_t solve(const Args&...) { return 0; }
 };
}

int main()
{
  using Scalar = float;
  using Solver = EntoPose::DummyAbsSolver<Scalar>;
  using Problem = EntoPose::AbsolutePoseProblem<Scalar, Solver, 3>;

  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(Solver{});
  BenchHarness harness(problem, "Empty Absolute Pose Baseline",
                       "dummy_input.txt", "dummy_output.txt");
  harness.run();
  return 0;
} 