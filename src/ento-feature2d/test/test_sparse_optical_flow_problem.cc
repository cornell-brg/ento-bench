#include <cstdio>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-feature2d/sparse_optical_flow_problem.h>
#include <ento-feature2d/optical_flow_problem.h>
#include <image_io/Image.h>
//#include <string>


const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 128;

char dir_path[FILEPATH_SIZE];
char img1_tiny_path[FILEPATH_SIZE];
char img1_tiny_features_path[FILEPATH_SIZE];
char img2_tiny_path[FILEPATH_SIZE];
char img2_tiny_features_path[FILEPATH_SIZE];
char test1_flows_gt_path[FILEPATH_SIZE];
char test1_output_path[FILEPATH_SIZE];

char* full_paths[] = { img1_tiny_path, img1_tiny_features_path,
                       img2_tiny_path, img2_tiny_features_path,
                       test1_flows_gt_path, test1_output_path };
constexpr size_t num_paths = 6;

using namespace EntoFeature2D;

class NullKernel
{
  template <typename IM, typename FE, typename FL>
  void operator()([[maybe_unused]] const IM& img1,
                  [[maybe_unused]] const IM& img2,
                  [[maybe_unused]] const FE& feats1,
                  [[maybe_unused]] const FE& feats2,
                  FL* flows)
  {
    for (int i = 0; i < feats1.size(); i++)
    {
      flows[i][0] = 1;
      flows[i][1] = -1;
    }
  }

  static constexpr const char* name() { return "Null Kernel"; }
};


void test_sparse_of_prob_deserialize_char()
{
  using Ke = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<Ke, 7, 7, 1, Kp, Pt>;

  P problem(Ke{}); // Instantiate with NullKernel
  
  // Construct the dataset line manually using the paths
  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s,%s,%s",
           img1_tiny_path, img2_tiny_path,
           img1_tiny_features_path, img2_tiny_features_path,
           test1_flows_gt_path);

  ENTO_DEBUG("Deserialize test (char*): %s", line);

  // Deserialize the constructed dataset line
  bool success = problem.deserialize(std::string(line));
  
  // Assert deserialization worked
  ENTO_TEST_CHECK_TRUE(success);

  ENTO_DEBUG("Finished deserialization test (char*)");
}

void test_sparse_of_prob_deserialize_string()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<K, 7, 7, 1, Kp, Pt>;

  P problem(K{}); // Instantiate with NullKernel

  // Construct the dataset line manually using the paths
  std::ostringstream oss;
  oss << img1_tiny_path << "," << img2_tiny_path << ","
      << img1_tiny_features_path << "," << img2_tiny_features_path << ","
      << test1_flows_gt_path;

  std::string line = oss.str();

  ENTO_DEBUG("Deserialize test (std::string): %s", line.c_str());

  // Deserialize the constructed dataset line
  bool success = problem.deserialize(line);

  // Assert deserialization worked
  // ENTO_ASSERT(success, "Deserialization failed for std::string input");
  ENTO_TEST_CHECK_TRUE(success);

  ENTO_DEBUG("Finished deserialization test (std::string)");
}
void test_sparse_of_prob_serialize_char()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<K, 7, 7, 1, Kp, Pt>;

  char line[1024]; // line holds the data for flows

}

void test_sparse_of_prob_serialize_string()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<K, 7, 7, 1, Kp, Pt>;

  char line[1024]; // line holds the data for flows
}

void test_sparse_prob_run()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<K, 7, 7, 1, Kp, Pt>;

  char line[1024];
  //@TODO: Concatenate paths for img1_tiny, img2_tiny, feats1, feats2, flows_gt
}

void test_sparse_of_prob_validate()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<K, 7, 7, 1, Kp, Pt>;
}

void test_sparse_of_prob_clear()
{

}

int main ( int argc, char ** argv )
{

  using namespace EntoUtil;
  int __n;
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
    { "test_img1_tiny.pgm" , "test_img1_tiny_feats.txt", "test_img2_tiny.pgm", 
      "test_img2_tiny_feats.txt", "test_simple_flows_gt.txt", "test1_output_path.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
  printf("Generated Paths:\n");
  for (size_t i = 0; i < num_paths; ++i) {
    printf("  %s\n", full_paths[i]);
  }

  // Run Tests
  if (__ento_test_num(__n, 1)) test_sparse_of_prob_deserialize_char();
  if (__ento_test_num(__n, 2)) test_sparse_of_prob_deserialize_string();
  if (__ento_test_num(__n, 3)) test_sparse_of_prob_serialize_char();
  if (__ento_test_num(__n, 4)) test_sparse_of_prob_serialize_string();
}


