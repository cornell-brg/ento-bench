#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <Eigen/Dense>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

namespace EntoAttitude {

template <typename Scalar>
struct DummyAttitudeKernel {
  using Scalar_ = Scalar;
  template <typename... Args>
  Eigen::Quaternion<Scalar> operator()(Args&&...) const {
    asm volatile("" ::: "memory");
    return Eigen::Quaternion<Scalar>::Identity();
  }
};

} // namespace EntoAttitude

int main()
{
  using Scalar = float;
  using Kernel = EntoAttitude::DummyAttitudeKernel<Scalar>;
  constexpr bool UseMag = false;
  using Problem = EntoAttitude::AttitudeProblem<Scalar, Kernel, UseMag>;

  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(Kernel{});
  BenchHarness harness(problem, "Empty Attitude Estimation Baseline", "", "");
  harness.run();
  return 0;
} 