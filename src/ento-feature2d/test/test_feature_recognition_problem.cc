#include <cstdio>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-feature2d/feature_recognition_problem.h>
#include <image_io/Image.h>
#include <ento-bench/harness.h>

#include <ento-feature2d/brief.h>
#include <ento-feature2d/feat2d_util.h>

using namespace EntoUtil;
using namespace EntoFeature2D;

// Globals Variables (file paths)
const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 256;
char dir_path[FILEPATH_SIZE];

char img1_tiny_path[FILEPATH_SIZE];
char img1_tiny_features_path[FILEPATH_SIZE];
char img1_tiny_descriptors_path[FILEPATH_SIZE];
char* full_paths[] = { img1_tiny_path, img1_tiny_features_path, img1_tiny_descriptors_path }; 
constexpr size_t num_paths = 3;

struct NullKernel
{
  using KeypointType = Keypoint<uint16_t>;
  using DescriptorType = std::monostate;

  template <typename ImageT, typename KeypointT, size_t MaxFeats>
  void operator()([[maybe_unused]] const ImageT& img,
                  FeatureArray<KeypointT, MaxFeats>& feats)
  {
    uint16_t xval = 1, yval = 1;
    for (int i = 0; i < MaxFeats; i++)
    {
      feats[i].x = 1; xval++;
      feats[i].y = 1; yval++;
    }
  }

  static constexpr char* name() { return "Null Kernel"; }
};

struct NullDescriptionKernel
{
  using KeypointType = Keypoint<uint16_t>;
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
  static constexpr char* name() { return "Null Kernel"; }
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
  const char* file_names[] =
    { "test_img1_tiny.pgm" , "test_img1_tiny_feats.txt", "test_img1_tiny_descs.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
  ENTO_DEBUG("Generated Paths:\n");
  for (size_t i = 0; i < num_paths; ++i) {
    ENTO_DEBUG("  %s\n", full_paths[i]);
  }

  // Run Tests
  if (__ento_test_num(__n, 1)) test_feature_recognition_prob_detection_deserialize_string();
  if (__ento_test_num(__n, 2)) test_feature_recognition_prob_detection_deserialize_char();
  if (__ento_test_num(__n, 3)) test_feature_recognition_prob_description_deserialize_string();
  if (__ento_test_num(__n, 4)) test_feature_recognition_prob_description_deserialize_char();
  if (__ento_test_num(__n, 5)) test_feature_recognition_prob_detsec_deserialize_string();
  if (__ento_test_num(__n, 6)) test_feature_recognition_prob_detsec_deserialize_char();
  if (__ento_test_num(__n, 7)) test_feature_recognition_prob_detection_run();
  if (__ento_test_num(__n, 8)) test_feature_recognition_prob_description_run();
  if (__ento_test_num(__n, 9)) test_feature_recognition_prob_detdesc_run();


}
