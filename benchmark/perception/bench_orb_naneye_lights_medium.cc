#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-feature2d/feature_recognition_problem.h>
#include <ento-feature2d/orb.h>

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

  constexpr int MaxFeats = 100;
  constexpr int Threshold = 20;
  using Kernel  = ORBKernel<MaxFeats, Threshold>;
  using PixT    = uint8_t;
  constexpr int Rows = 160;
  constexpr int Cols = 160;
  using Problem = FeatureRecognitionProblem<Kernel, MaxFeats, Rows, Cols, PixT, true, true>;

  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/orb_medium_naneye-lights_data.txt";

  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build dataset path for naneye lights medium.");
    exit(1);
  }

  ENTO_DEBUG("ORB Naneye Lights Medium: %s", dataset_path);

  static Problem problem(Kernel{});
  
  // NEW IDIOM: Configuration-driven harness type
  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "ORB Naneye Lights Medium", dataset_path, output_path);
  
  harness.run();

  exit(1);
  return 0;
} 
