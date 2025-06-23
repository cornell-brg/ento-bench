// SIFT Feature Detection and Description Benchmark - Large Dataset
// Part of ento-bench perception benchmarks

#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-feature2d/feature_recognition_problem.h>
#include <ento-feature2d/multi_octave_sift_v2.h>

// Include benchmark configuration
#include <ento-bench/bench_config.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoFeature2D;

// TODO: Add SIFT-specific includes
// #include <ento-feature2d/sift.h>

// TODO: Implement SIFT feature detection and description
// This benchmark should test SIFT keypoint detection and descriptor computation
// on a large dataset for performance evaluation

// SIFT Kernel for benchmarking (detection-only)
template <int MaxFeatures = 250>
struct MultiOctaveSIFTKernel
{
  using KeypointType   = SIFTKeypoint<>;
  using DescriptorType = std::monostate; // Detection-only for now

  static constexpr size_t MaxFeatures_ = MaxFeatures;

  template <typename ImageT>
  bool solve(const ImageT& img, FeatureArray<KeypointType, MaxFeatures>& feats) const {
    // Create SIFT driver using the image as working buffer
    MultiOctaveSIFTDriverV2<ImageT, MaxFeatures, KeypointType, 4> sift_driver(
      const_cast<ImageT&>(img)
    );
    
    // Run SIFT detection
    return sift_driver.run(img, feats);
  }

  // Function call operator for FeatureRecognitionProblem compatibility
  template <typename ImageT>
  bool operator()(const ImageT& img, FeatureArray<KeypointType, MaxFeatures>& feats) const {
    return solve(img, feats);
  }
};

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

  constexpr int MaxFeats = 250;
  using Kernel  = MultiOctaveSIFTKernel<MaxFeats>;
  using PixT    = uint8_t;
  constexpr int Rows = 320;
  constexpr int Cols = 320;
  using Problem = FeatureRecognitionProblem<Kernel, MaxFeats, Rows, Cols, PixT, true, false>;

  const char* base_path = DATASET_PATH;
  const char* rel_path  = "feat2d/sift_large_data.txt";  // TODO: Create dataset

  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build dataset path for large SIFT image.");
    exit(1);
  }

  ENTO_DEBUG("SIFT Large: %s", dataset_path);

  static Problem problem(Kernel{});
  
  // NEW IDIOM: Configuration-driven harness type
  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "SIFT Large", dataset_path, output_path);
  
  harness.run();

  exit(1);
  return 0;
} 