#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

#include <ento-feature2d/lk_optical_flow.h>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/sparse_optical_flow_problem.h>
#include <ento-feature2d/image_pyramid.h>
#include <image_io/Image.h>

// Include benchmark configuration
#include <ento-bench/bench_config.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoFeature2D;

constexpr int decimal_bits = 10;
using fp_t = FixedPoint<32 - decimal_bits, decimal_bits, int32_t>;

// Configuration for LARGE
constexpr size_t NUM_LEVELS = 3;
constexpr size_t IMG_WIDTH  = 320;
constexpr size_t IMG_HEIGHT = 320;
constexpr size_t WIN_DIM    = 15;
constexpr size_t NumFeats   = 10;
using PixelT = uint8_t;
using CoordT = float;

using LK = LucasKanadeOFKernel<
  NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT,
  WIN_DIM, CoordT, PixelT, NumFeats>;

using Prob = SparseOpticalFlowProblem<
  LK, IMG_HEIGHT, IMG_WIDTH, NumFeats,
  Keypoint<CoordT>, PixelT>;

int main()
{
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // NEW IDIOM: Generic cache setup using configuration
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  // Dataset path
  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/sparse_of_large.txt";
  char dataset_path[512];
  char output_path[256];

  printf("Hello world");
  if (!build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
    ENTO_DEBUG("ERROR! Failed to construct dataset path.");

  // Params
  int max_count     = 20;
  int det_epsilon   = 1 << 20;
  float criteria    = 0.01f;
  int num_good_pts  = NumFeats;

  // Problem construction
  LK adapter(num_good_pts, max_count, det_epsilon, criteria);
  Prob problem(adapter);

  // NEW IDIOM: Configuration-driven harness type
  ENTO_BENCH_HARNESS_TYPE(Prob);
  BenchHarness harness(problem, "Bench LK Optical Flow [large]", dataset_path, output_path);

  harness.run();
  exit(1);
  return 0;
}
