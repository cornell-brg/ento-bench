#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-feature2d/feature_recognition_problem.h>
#include <ento-feature2d/fast.h>
#include <ento-feature2d/brief.h>

// Include benchmark configuration
#include <ento-bench/bench_config.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoFeature2D;

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

  constexpr int MaxFeatures = 250;
  using Kernel  = FastBriefKernel<MaxFeatures>;
  using PixT    = uint8_t;
  constexpr int Rows = 320;
  constexpr int Cols = 320;
  using Problem = FeatureRecognitionProblem<Kernel, MaxFeatures, Rows, Cols, PixT, true, true>;

  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/fastbrief_large_books_data.txt";

  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build dataset path for large image.");
    exit(1);
  }

  ENTO_DEBUG("FAST+BRIEF Large: %s", dataset_path);

  static Problem problem(Kernel{});
  
  // NEW IDIOM: Configuration-driven harness type
  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "FAST+BRIEF Large", dataset_path, output_path);
  
  harness.run();

  exit(1);
  return 0;
}


