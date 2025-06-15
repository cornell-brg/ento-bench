// Block-Based Optical Flow Benchmark - Small Dataset
// Part of ento-bench perception benchmarks

#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

#include <ento-feature2d/bb_optical_flow.h>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/sparse_optical_flow_problem.h>
#include <image_io/Image.h>

// Include benchmark configuration
#include <ento-bench/bench_config.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoFeature2D;

// Configuration for SMALL
constexpr size_t IMG_WIDTH  = 80;
constexpr size_t IMG_HEIGHT = 80;
constexpr size_t WIN_DIM = 8;  
constexpr size_t NumFeats = 10;
constexpr size_t SEARCH_AREA = 1;
using PixelT = uint8_t;
using CoordT = float;

using BBOF = BlockBasedOFKernel<IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT, NumFeats, SEARCH_AREA>;

using Prob = SparseOpticalFlowProblem<BBOF, IMG_HEIGHT, IMG_WIDTH, NumFeats, Keypoint<CoordT>, PixelT>;

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
  const char* rel_path  = "feat2d/sparse_of_small.txt";
  char dataset_path[512];
  char output_path[256];

  if (!build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
    ENTO_DEBUG("ERROR! Failed to construct dataset path.");

  ENTO_DEBUG("Dataset file path: %s\n", dataset_path);

  // Problem construction
  BBOF adapter;
  Prob problem(adapter);

  // NEW IDIOM: Configuration-driven harness type
  ENTO_BENCH_HARNESS_TYPE(Prob);
  BenchHarness harness(problem, "Bench Block-Based Optical Flow [small]", dataset_path, output_path);

  harness.run();
  exit(1);
  return 0;
} 