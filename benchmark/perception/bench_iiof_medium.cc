#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

#include <ento-feature2d/ii_optical_flow.h>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/sparse_optical_flow_problem.h>
#include <ento-feature2d/image_pyramid.h>
#include <image_io/Image.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoFeature2D;

// Configuration for SMALL
constexpr size_t IMG_WIDTH  = 160;
constexpr size_t IMG_HEIGHT = 160;
constexpr size_t WIN_DIM = 25;  
constexpr size_t NumFeats = 10;
using PixelT = uint8_t;
using CoordT = float;

using K = ImageInterpolationOFKernel<IMG_WIDTH, IMG_HEIGHT, WIN_DIM, float, PixelT, NumFeats>;

using P = SparseOpticalFlowProblem<K, IMG_HEIGHT, IMG_WIDTH, NumFeats, Keypoint<float>, PixelT>;

int main()
{
  initialise_monitor_handles();
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  // Dataset path
  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/sparse_of_medium.txt";
  char dataset_path[512];
  char output_path[256];

  if (!build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
    ENTO_DEBUG("ERROR! Failed to construct dataset path.");

  // Params
  //int max_count     = 20;
  //int det_epsilon   = 1 << 20;
  //float criteria    = 0.01f;
  //int num_good_pts  = NumFeats;

  // Problem construction
  //int num_good_points = 2;
  int DET_RADIUS = 7;
  int DET_EPSILON = 0;  // or 1 << 20 depending on your expectations

  K adapter(DET_RADIUS, DET_EPSILON);
  P problem(adapter);

  EntoBench::Harness<P, false, 10> harness(problem,
    "Bench II Optical Flow [medium]",
    dataset_path,
    output_path);

  harness.run();
  exit(1);
  return 0;
}


