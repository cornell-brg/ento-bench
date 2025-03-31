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

  constexpr int MaxFeats = 50;
  using Kernel  = FastBriefKernel<MaxFeats>;
  using PixT    = uint8_t;
  constexpr int Rows = 80;
  constexpr int Cols = 80;
  using Problem = FeatureRecognitionProblem<Kernel, MaxFeats, Rows, Cols, PixT, true, true>;

  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/fastbrief_small_books_data.txt";

  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build dataset path for small image.");
    exit(1);
  }

  ENTO_DEBUG("FAST+BRIEF Small: %s", dataset_path);

  static Problem problem(Kernel{});
  EntoBench::Harness<Problem, false, 10> harness(problem, "FAST+BRIEF Small", dataset_path, output_path);
  harness.run();

  exit(1);
  return 0;
}
