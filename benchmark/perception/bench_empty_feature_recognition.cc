#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <ento-feature2d/feature_recognition_problem.h>
#include <Eigen/Dense>
#include <ento-feature2d/feat2d_util.h>
#include <variant>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

using namespace EntoFeature2D;

// Dummy kernel that does nothing but matches the operator() signature
struct DummyFRKernel {
  using KeypointType   = Keypoint<int16_t>;
  using DescriptorType = std::monostate;
  template<typename... Args>
  void operator()(Args&&...) const { asm volatile("" ::: "memory"); }
};

int main()
{
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  ENTO_BENCH_SETUP();
  ENTO_BENCH_PRINT_CONFIG();

  constexpr int Rows = 160;
  constexpr int Cols = 160;
  constexpr int MaxFeats = 50;
  using PixelT = uint8_t;
  using Kernel = DummyFRKernel;
  using Problem = FeatureRecognitionProblem<Kernel, MaxFeats, Rows, Cols, PixelT, true, false>;

  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(Kernel{});
  BenchHarness harness(problem, "Empty Feature Recognition Baseline",
                       "dummy_input.txt", "dummy_output.txt");
  harness.run();
  return 0;
} 