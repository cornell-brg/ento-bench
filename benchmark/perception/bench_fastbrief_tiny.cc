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
  using Kernel  = FastBriefKernel<5>;
  using PixT    = uint8_t;
  constexpr int Rows = 31;
  constexpr int Cols = 31;
  using Problem = FeatureRecognitionProblem<Kernel, 5, Rows, Cols, PixT, true, true>;

  // -------- Dataset Path (TXT file with one line of CSV input) --------
  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/test_fastbrief_tiny_dataset.txt";  // This file contains the 3-path line

  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build FAST+BRIEF dataset path.");
    exit(1);
  }

  ENTO_DEBUG("Dataset file path: %s", dataset_path);

  // -------- Run Harness --------
  static Problem problem(Kernel{});
  EntoBench::Harness<Problem, false, 1> harness(problem, "Test FAST+BRIEF Tiny Image",
                                                dataset_path, output_path);
  harness.run();

  exit(1);
  return 0;
}
