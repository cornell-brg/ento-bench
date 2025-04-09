#include <cstdio>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-feature2d/feature_recognition_problem.h>
#include <image_io/Image.h>
#include <image_io/image_util.h>
#include <ento-bench/harness.h>

#include <ento-feature2d/orb.h>
#include <ento-feature2d/fast.h>
#include <ento-feature2d/brief.h>
#include <ento-feature2d/feat2d_util.h>


using namespace EntoUtil;
using namespace EntoFeature2D;

// Globals Variables (file paths)
const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 256;
char dir_path[FILEPATH_SIZE];

char img1_tiny_path[FILEPATH_SIZE];
char img2_tiny_path[FILEPATH_SIZE];
char img3_tiny_path[FILEPATH_SIZE];

char img1_tiny_features_path[FILEPATH_SIZE];
char img2_tiny_features_path[FILEPATH_SIZE];
char img3_tiny_features_path[FILEPATH_SIZE];

char img1_tiny_descriptors_path[FILEPATH_SIZE];
char img2_tiny_descriptors_path[FILEPATH_SIZE];
char img3_tiny_descriptors_path[FILEPATH_SIZE];

char img2_tiny_features_nms_path[FILEPATH_SIZE];

char* full_paths[] = { 
  img1_tiny_path, img1_tiny_features_path, img1_tiny_descriptors_path,
  img2_tiny_path, img2_tiny_features_path, img2_tiny_descriptors_path,
  img2_tiny_features_nms_path,
  img3_tiny_path, img3_tiny_features_path, img3_tiny_descriptors_path
}; 
constexpr size_t num_paths = 10;

struct NullKernel
{
  using KeypointType = Keypoint<uint16_t>;
  using DescriptorType = std::monostate;

  template <typename ImageT, typename KeypointT, size_t MaxFeats>
  void operator()([[maybe_unused]] const ImageT& img,
                  FeatureArray<KeypointT, MaxFeats>& feats)
  {
    for (int i = 0; i < MaxFeats; i++)
    {
      feats[i].x = 1;
      feats[i].y = 1;
    }
  }

  static constexpr const char* name() { return "Null Kernel"; }
};

struct NullDescriptionKernel
{
  using KeypointType = FastKeypoint<uint16_t>;
  using DescriptorType = BRIEFDescriptor;

  template <typename ImageT, typename KeypointT, size_t MaxFeats>
  void operator()([[maybe_unused]] const ImageT& img,
                  FeatureArray<KeypointT, MaxFeats>& feats,
                  std::array<DescriptorType, MaxFeats>& descriptors)
  {
    for (int i = 0; i < feats.size(); i++)
    {
      for (int j = 0; j < 8; j++)
        descriptors[i].set_bit(j, 1);
    }
  }
  static constexpr const char* name() { return "Null Kernel"; }
};

void test_feature_recognition_prob_detection_deserialize_string()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT>;

  ProbT problem(K{});

  std::ostringstream oss;
  oss << img1_tiny_path << "," << img1_tiny_features_path;

  std::string line = oss.str(); 

  ENTO_DEBUG("Deserialize test (std::string): %s", line.c_str());

  bool success = problem.deserialize(line);

  ENTO_TEST_CHECK_TRUE(success);

  ENTO_DEBUG("Finished deserialization test (std::string)");


}

void test_feature_recognition_prob_detection_deserialize_char()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s",
           img1_tiny_path, img1_tiny_features_path);

  bool success = problem.deserialize(line);

  ENTO_TEST_CHECK_TRUE(success);
  ENTO_DEBUG("Finished deserialization test (char*)");
}

void test_feature_recognition_prob_description_deserialize_string()
{
  using K = NullDescriptionKernel;
  using Kp = Keypoint<int16_t>;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT, false, true>;

  ProbT problem(K{});

  std::ostringstream oss;
  oss << img1_tiny_path << "," << img1_tiny_features_path
      << "," << img1_tiny_descriptors_path;

  std::string line = oss.str(); 

  ENTO_DEBUG("Deserialize test (std::string): %s", line.c_str());

  bool success = problem.deserialize(line);

  ENTO_TEST_CHECK_TRUE(success);

  ENTO_DEBUG("Finished deserialization test (std::string)");

}

void test_feature_recognition_prob_description_deserialize_char()
{
  using K = NullDescriptionKernel;
  using Kp = Keypoint<int16_t>;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT, false, true>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s",
           img1_tiny_path,
           img1_tiny_features_path,
           img1_tiny_descriptors_path);

  bool success = problem.deserialize(line);

  ENTO_TEST_CHECK_TRUE(success);
  ENTO_DEBUG("Finished deserialization test (char*)");
  
}


void test_feature_recognition_prob_detsec_deserialize_string()
{
  using K = NullDescriptionKernel;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT, true, true>;

  ProbT problem(K{});

  std::ostringstream oss;
  oss << img1_tiny_path << "," << img1_tiny_features_path << "," << img1_tiny_descriptors_path;

  bool success = problem.deserialize(oss.str());

  ENTO_TEST_CHECK_TRUE(success);
  ENTO_DEBUG("Finished detdesc deserialize test (std::string)");
}

void test_feature_recognition_prob_detsec_deserialize_char()
{
  using K = NullDescriptionKernel;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT, true, true>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s",
           img1_tiny_path, img1_tiny_features_path, img1_tiny_descriptors_path);

  bool success = problem.deserialize(line);

  ENTO_TEST_CHECK_TRUE(success);
  ENTO_DEBUG("Finished detdesc deserialize test (char*)");
}

void test_feature_recognition_prob_detection_run()
{
  using K = NullKernel;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT, true, false>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s", img1_tiny_path, img1_tiny_features_path);
  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));

  problem.solve();

  ENTO_TEST_CHECK_INT_EQ(problem.feats_[0].x, 1);
  ENTO_TEST_CHECK_INT_EQ(problem.feats_[0].y, 1);

  ENTO_DEBUG("Finished detection run test");
}

void test_feature_recognition_prob_description_run()
{
  using K = NullDescriptionKernel;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT, false, true>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s",
           img1_tiny_path, img1_tiny_features_path, img1_tiny_descriptors_path);
  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));

  problem.solve();

  // Descriptor bits [0:7] set to 1, rest should be zero
  for (int i = 0; i < 8; i++)
    ENTO_TEST_CHECK_TRUE(problem.descs_[0].data[0] & (1 << i));

  ENTO_DEBUG("Finished description run test");
}

void test_feature_recognition_prob_detdesc_run()
{
  using K = NullDescriptionKernel;
  using PixT = uint8_t;
  using ProbT = FeatureRecognitionProblem<K, 1, 7, 7, PixT, true, true>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s",
           img1_tiny_path, img1_tiny_features_path, img1_tiny_descriptors_path);
  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));

  problem.solve();

  ENTO_TEST_CHECK_INT_EQ(problem.feats_[0].x, 0); // Default-initialized in this kernel
  ENTO_TEST_CHECK_INT_EQ(problem.feats_[0].y, 0);

  for (int i = 0; i < 8; i++)
    ENTO_TEST_CHECK_TRUE(problem.descs_[0].data[0] & (1 << i));

  ENTO_DEBUG("Finished detdesc run test");
}


void test_feature_recognition_prob_fast_kernel()
{
  constexpr int NumFeats = 1;
  using K = FastKernel<NumFeats>;
  using PixT = uint8_t;
  constexpr size_t Rows = 7, Cols = 7;
  using ProbT = FeatureRecognitionProblem<K, NumFeats, Rows, Cols, PixT, true, false>;

  ProbT problem(K{});

  // Assuming global variables defined previously
  char line[1024];
  snprintf(line, sizeof(line), "%s,%s",
           img1_tiny_path, img1_tiny_features_path);

  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));
  
  problem.solve();

  ENTO_TEST_CHECK_TRUE(problem.feats_.size() > 0);
  
  for (size_t i = 0; i < problem.feats_.size(); i++)
  {
    ENTO_DEBUG("Detected Feature #%zu: (%u, %u), score=%d",
               i, problem.feats_[i].x, problem.feats_[i].y, problem.feats_[i].score);
  }

  ENTO_DEBUG("FAST Kernel test complete.");
}

void test_feature_recognition_prob_fast_kernel_14x14()
{
  constexpr int NumFeats = 10;
  using K = FastKernel<NumFeats>;
  using PixT = uint8_t;
  constexpr size_t Rows = 14, Cols = 14;
  using ProbT = FeatureRecognitionProblem<K, NumFeats, Rows, Cols, PixT, true, false>;

  ProbT problem(K{});

  // Assuming global variables defined previously
  char line[1024];
  snprintf(line, sizeof(line), "%s,%s",
           img2_tiny_path, img2_tiny_features_path);

  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));
  
  problem.solve();

  ENTO_TEST_CHECK_TRUE(problem.feats_.size() > 0);
  
  for (size_t i = 0; i < problem.feats_.size(); i++)
  {
    ENTO_DEBUG("Detected Feature #%zu: (%u, %u), score=%d",
               i, problem.feats_[i].x, problem.feats_[i].y, problem.feats_[i].score);
  }

  ENTO_DEBUG("FAST Kernel test complete.");
}


void test_feature_recognition_prob_fast_kernel_14x14_nms()
{
  constexpr int NumFeats = 10;
  using K = FastKernel<NumFeats, 16, 10, true>;
  using PixT = uint8_t;
  constexpr size_t Rows = 14, Cols = 14;
  using ProbT = FeatureRecognitionProblem<K, NumFeats, Rows, Cols, PixT, true, false>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s",
           img2_tiny_path, img2_tiny_features_nms_path);

  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));
  
  problem.solve();

  ENTO_TEST_CHECK_TRUE(problem.feats_.size() > 0);
  
  for (size_t i = 0; i < problem.feats_.size(); i++)
  {
    ENTO_DEBUG("Detected Feature #%zu: (%u, %u), score=%d",
               i, problem.feats_[i].x, problem.feats_[i].y, problem.feats_[i].score);
  }

  ENTO_DEBUG("FAST Kernel test complete.");
}

void test_feature_recognition_prob_brief_kernel_31x31()
{
  constexpr int NumFeats = 1;
  using K = BriefKernel<NumFeats>;
  using PixT = uint8_t;
  constexpr size_t Rows = 31, Cols = 31;
  using ProbT = FeatureRecognitionProblem<K, NumFeats, Rows, Cols, PixT, false, true>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s",
           img3_tiny_path, img3_tiny_features_path, img3_tiny_descriptors_path);

  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));
  
  problem.solve();

  for (size_t i = 0; i < problem.descs_.size(); i++)
  {
    ENTO_DEBUG("Computed descriptor for feature[%i] at (%u, %u)", i,
              problem.feats_gt_[i].x, problem.feats_gt_[i].y);
    BRIEFDescriptor desc = problem.descs_[i];
    char desc_str[256];  // 32 bytes × ~4 chars each + commas = safe
    size_t pos = 0;

    for (int j = 0; j < 32; j++)
    {
      // write byte, optionally followed by comma
      pos += snprintf(&desc_str[pos], sizeof(desc_str) - pos,
                      (j < 31) ? "%u," : "%u", static_cast<uint8_t>(desc.data[j]));
    }

    ENTO_DEBUG("Descriptor: %s", desc_str);
  }

}

void test_feature_recognition_prob_orb_kernel_31x31()
{
  constexpr int NumFeats = 5;
  using K = ORBKernel<NumFeats, 20>; 
  using PixT = uint8_t;
  constexpr size_t Rows = 31, Cols = 31;
  using ProbT = FeatureRecognitionProblem<K, NumFeats, Rows, Cols, PixT, true, true>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s",
           img3_tiny_path, img3_tiny_features_path, img3_tiny_descriptors_path);

  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));
  problem.solve();

  ENTO_DEBUG("Num features: %i", problem.feats_.size());
  for (size_t i = 0; i < problem.feats_.size(); ++i)
  {
    const auto& kp = problem.feats_[i];
    ENTO_DEBUG("ORB Feature #%zu: (%u, %u), score=%d, orientation (deg)=%.3f",
               i, kp.x, kp.y, kp.score, kp.orientation * (180.0f / M_PI));
  }

  ENTO_DEBUG("Descriptors size: %i", problem.descs_.size());
  for (size_t i = 0; i < problem.descs_.size(); ++i)
  {
    const auto& desc = problem.descs_[i];
    char desc_str[256];
    size_t pos = 0;

    for (int j = 0; j < 32; ++j)
    {
      pos += snprintf(&desc_str[pos], sizeof(desc_str) - pos,
                      (j < 31) ? "%u," : "%u", static_cast<uint8_t>(desc.data[j]));
    }

    ENTO_DEBUG("Descriptor[%zu]: %s", i, desc_str);
  }

  ENTO_DEBUG("ORB Kernel test complete.");

}

void test_feature_recognition_prob_fast_brief_kernel_31x31()
{
  constexpr int NumFeats = 5;
  using K = FastBriefKernel<NumFeats>;
  using PixT = uint8_t;
  constexpr size_t Rows = 31, Cols = 31;
  using ProbT = FeatureRecognitionProblem<K, NumFeats, Rows, Cols, PixT, true, true>;

  ProbT problem(K{});

  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s",
           img3_tiny_path, img3_tiny_features_path, img3_tiny_descriptors_path);

  ENTO_TEST_CHECK_TRUE(problem.deserialize(line));
  problem.solve();

  ENTO_DEBUG("FAST+BRIEF: Num features: %i", problem.feats_.size());
  for (size_t i = 0; i < problem.feats_.size(); ++i)
  {
    const auto& kp = problem.feats_[i];
    ENTO_DEBUG("FAST+BRIEF Feature #%zu: (%u, %u), score=%d",
               i, kp.x, kp.y, kp.score);
  }

  for (size_t i = 0; i < problem.descs_.size(); ++i)
  {
    const auto& desc = problem.descs_[i];
    char desc_str[256];
    size_t pos = 0;
    for (int j = 0; j < 32; ++j)
    {
      pos += snprintf(&desc_str[pos], sizeof(desc_str) - pos,
                      (j < 31) ? "%u," : "%u", static_cast<uint8_t>(desc.data[j]));
    }
    ENTO_DEBUG("Descriptor[%zu]: %s", i, desc_str);
  }
}

int main ( int argc, char ** argv )
{

  //int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  // Setup Directory Path and Test Data Paths
  get_file_directory(file_path, sizeof(dir_path), dir_path);
  const char* file_names[] = {
    "test_img1_tiny.pgm" , "test_img1_tiny_feats.txt", "test_img1_tiny_descs.txt",
    "test_img2_tiny.pgm" , "test_img2_tiny_feats.txt", "test_img2_tiny_descs.txt",
    "test_img2_tiny_feats_nms.txt",
    "test_img3_tiny.pgm" , "test_img3_tiny_feats.txt", "test_img3_tiny_descs.txt"
  };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);

  // Run Tests
  if (__ento_test_num(__n, 1))  test_feature_recognition_prob_detection_deserialize_string();
  if (__ento_test_num(__n, 2))  test_feature_recognition_prob_detection_deserialize_char();
  if (__ento_test_num(__n, 3))  test_feature_recognition_prob_description_deserialize_string();
  if (__ento_test_num(__n, 4))  test_feature_recognition_prob_description_deserialize_char();
  if (__ento_test_num(__n, 5))  test_feature_recognition_prob_detsec_deserialize_string();
  if (__ento_test_num(__n, 6))  test_feature_recognition_prob_detsec_deserialize_char();
  if (__ento_test_num(__n, 7))  test_feature_recognition_prob_detection_run();
  if (__ento_test_num(__n, 8))  test_feature_recognition_prob_description_run();
  if (__ento_test_num(__n, 9))  test_feature_recognition_prob_detdesc_run();
  if (__ento_test_num(__n, 10)) test_feature_recognition_prob_fast_kernel(); 
  if (__ento_test_num(__n, 11)) test_feature_recognition_prob_fast_kernel_14x14(); 
  if (__ento_test_num(__n, 12)) test_feature_recognition_prob_fast_kernel_14x14_nms(); 
  if (__ento_test_num(__n, 13)) test_feature_recognition_prob_brief_kernel_31x31(); 
  if (__ento_test_num(__n, 14)) test_feature_recognition_prob_orb_kernel_31x31();
  if (__ento_test_num(__n, 15)) test_feature_recognition_prob_fast_brief_kernel_31x31();



}
