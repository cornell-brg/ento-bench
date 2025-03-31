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

template <int MaxFeats = 50>
struct FastBriefKernelSmall
{
  using KeypointType   = FastKeypoint<uint16_t>;
  using DescriptorType = BRIEFDescriptor;

  template <typename ImageT,
            typename KeypointT,
            typename DescriptorArray>
  void operator()(ImageT& img,
                  FeatureArray<KeypointT, MaxFeats>& feats,
                  DescriptorArray& descs) const
  {
    FastKernel<MaxFeats>{}(img, feats);
    BriefKernel<MaxFeats>{}(img, feats, descs);
  }

  static constexpr const char* name() { return "FAST+BRIEF Medium Kernel"; }
};

int main()
{
  initialise_monitor_handles();
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  using Kernel  = FastBriefKernelSmall<50>;
  using PixT    = uint8_t;
  constexpr int Rows = 160;
  constexpr int Cols = 160;
  using Problem = FeatureRecognitionProblem<Kernel, 50, Rows, Cols, PixT, true, true>;

  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feature2d/test_fast_brief_medium.txt";

  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build dataset path for small image.");
    exit(1);
  }

  ENTO_DEBUG("FAST+BRIEF Small: %s", dataset_path);

  static Problem problem(Kernel{});
  EntoBench::Harness<Problem, false, 1> harness(problem, "FAST+BRIEF Medium", dataset_path, output_path);
  harness.run();

  exit(1);
  return 0;
}

