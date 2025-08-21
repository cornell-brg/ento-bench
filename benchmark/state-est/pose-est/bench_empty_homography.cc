#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <ento-pose/problem-types/homography_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

namespace EntoPose {
 template<typename Scalar>
 struct DummyHomoSolver {
   static constexpr size_t MaxSolns = 1;
   template<size_t N, typename... Args>
   static size_t solve(const Args&...) { return 0; }
 };
}

int main()
{
  using Scalar = float;
  using Solver = EntoPose::DummyHomoSolver<Scalar>;
  using Problem = EntoPose::HomographyProblem<Scalar, Solver, 4>;

  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(Solver{});
  BenchHarness harness(problem, "Empty Homography Baseline",
                       "dummy_input.txt", "dummy_output.txt");
  harness.run();
  return 0;
} 