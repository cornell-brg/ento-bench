#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <ento-feature2d/sparse_optical_flow_problem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

using namespace EntoFeature2D;

// Dummy kernel for optical flow
template<size_t ImgW, size_t ImgH, typename PixelT, size_t NumF>
struct DummyOFKernel {
  using CoordT_ = float;
  using PixelT_ = PixelT;
  static constexpr size_t NumFeats_ = NumF;

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

  constexpr size_t IMG_WIDTH  = 80;
  constexpr size_t IMG_HEIGHT = 80;
  constexpr size_t NumFeats   = 10;
  using PixelT = uint8_t;
  using Kernel = DummyOFKernel<IMG_WIDTH, IMG_HEIGHT, PixelT, NumFeats>;
  using KeypointT = Keypoint<float>;
  using Problem = SparseOpticalFlowProblem<Kernel, IMG_HEIGHT, IMG_WIDTH, NumFeats, KeypointT, PixelT>;

  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(Kernel{});
  BenchHarness harness(problem, "Empty Optical Flow Baseline",
                       "dummy_input.txt", "dummy_output.txt");
  harness.run();
  return 0;
} 