#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-feature2d/feature_recognition_problem.h>
#include <ento-feature2d/orb.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoFeature2D;

int main()
{
  initialise_monitor_handles();
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  // -------- Kernel and Problem Setup --------
  constexpr int MaxFeats = 100;
  constexpr int Threshold = 20;
  using Kernel  = ORBKernel<MaxFeats, Threshold>;
  using PixT    = uint8_t;
  constexpr int Rows = 160;
  constexpr int Cols = 160;
  using Problem = FeatureRecognitionProblem<Kernel, MaxFeats, Rows, Cols, PixT, true, true>;

  // -------- Dataset Path (TXT file with one line of CSV input) --------
  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/orb_medium_books_data.txt";  // This file contains the 3-path line

  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build ORB [medium] dataset path.");
    exit(1);
  }

  ENTO_DEBUG("Dataset file path: %s", dataset_path);

  // -------- Run Harness --------
  static Problem problem(Kernel{});
  EntoBench::Harness<Problem, false, 10> harness(problem, "Test ORB [medium] Image",
                                                dataset_path, output_path);
  harness.run();

  exit(1);
  return 0;
}


