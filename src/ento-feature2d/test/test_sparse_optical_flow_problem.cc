#include <cstdio>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-feature2d/sparse_optical_flow_problem.h>
#include <ento-feature2d/optical_flow_problem.h>
#include <image_io/Image.h>
#include <ento-bench/harness.h>

#include <math/FixedPoint.hh>
#include <ento-feature2d/image_pyramid.h>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/lk_optical_flow.h>

//#include <string>


const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 256;

char dir_path[FILEPATH_SIZE];
char img1_tiny_path[FILEPATH_SIZE];
char img1_tiny_features_path[FILEPATH_SIZE];
char img2_tiny_path[FILEPATH_SIZE];
char img2_tiny_features_path[FILEPATH_SIZE];
char test1_feats_next_gt_path[FILEPATH_SIZE];
char test1_harness_dataset_path[FILEPATH_SIZE];
char test1_output_path[FILEPATH_SIZE];


// for test_lk_optical_flow_validate
char image_2_path[FILEPATH_SIZE];
char image_3_path[FILEPATH_SIZE];
char image_2_feats_path[FILEPATH_SIZE];
char image_2_feats_next_gt_path[FILEPATH_SIZE];

char* full_paths[] = { img1_tiny_path, img1_tiny_features_path,
                       img2_tiny_path, test1_feats_next_gt_path,
                       test1_output_path, test1_harness_dataset_path,
                       image_2_path, image_3_path,
                       image_2_feats_path, image_2_feats_next_gt_path };
constexpr size_t num_paths = 10;

using namespace EntoFeature2D;

struct NullKernel
{
  using CoordT_ = int16_t; // hardcoded coordt
  template <typename Img, typename KeypointT, size_t NumFeats>
  void operator()([[maybe_unused]] const Img& img1,
                  [[maybe_unused]] const Img& img2,
                  [[maybe_unused]] const FeatureArray<KeypointT, NumFeats> feats,
                  FeatureArray<KeypointT, NumFeats>* feats_next)
  {
    for (size_t i = 0; i < feats.size(); i++)
    {
      (*feats_next)[i].x = feats[i].x;
      (*feats_next)[i].y = feats[i].y;
    }
  }

  static constexpr const char* name() { return "Null Kernel"; }
};

const int decimal_bits = 20;
using fp_t = FixedPoint<64-decimal_bits, decimal_bits, int64_t>;

template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT, size_t NumFeats>
class LKOpticalFlowAdapter
{
private:
  ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT> prevPyramid_;
  ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT> nextPyramid_;
  bool status_[NumFeats];
  int num_good_points_;
  int MAX_COUNT_;
  int DET_EPSILON_;
  float CRITERIA_;

public:
  using CoordT_ = CoordT;
  using PixelT_ = PixelT;
  static constexpr size_t NumLevels_ = NUM_LEVELS;
  static constexpr size_t Width_ = IMG_WIDTH;
  static constexpr size_t Height_ = IMG_HEIGHT;
  LKOpticalFlowAdapter(int num_good_points, 
                       int MAX_COUNT,
                       int DET_EPSILON,
                       float CRITERIA)
      : num_good_points_(num_good_points), MAX_COUNT_(MAX_COUNT), 
        DET_EPSILON_(DET_EPSILON), CRITERIA_(CRITERIA) {}
  
  void operator()(const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img1,
                  const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img2,
                 FeatureArray<Keypoint<CoordT>, 2> feats,
                 FeatureArray<Keypoint<CoordT>, 2>* feats_next)
  {
    prevPyramid_.set_top_image(img1);
    prevPyramid_.initialize_pyramid();

    nextPyramid_.set_top_image(img2);
    nextPyramid_.initialize_pyramid();

    // TODO: have function take in Feature Array 
    feats_next->num_features = num_good_points_;

    calcOpticalFlowPyrLK<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT>(prevPyramid_, nextPyramid_, 
                                                          (Keypoint<CoordT> *)feats.keypoints.data(), (Keypoint<CoordT> *)feats_next->keypoints.data(), status_, 
                                                          num_good_points_, MAX_COUNT_, DET_EPSILON_, CRITERIA_);

  }
  
  // Name method for identification
  static constexpr const char* name()
  {
    return "Lukas Kanade Sparse Optical Flow";
  }
};

void test_lk_optical_flow_validate()
{
  const size_t NUM_LEVELS = 2;
  const size_t IMG_WIDTH = 320;
  const size_t IMG_HEIGHT = 320;
  const size_t WIN_DIM = 3;  
  const size_t NumFeats = 2;
  using PixelT = uint8_t;
// template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT, size_t NumFeats>

  using K = LKOpticalFlowAdapter<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, WIN_DIM, float, PixelT, NumFeats>;

  // template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType = Keypoint<int16_t>, typename PixelType = int16_t>

  using P = SparseOpticalFlowProblem<K, IMG_HEIGHT, IMG_WIDTH, NumFeats, Keypoint<float>, PixelT>;


  int MAX_COUNT = 1;
  int DET_EPSILON = (int)(1<<20);
  float CRITERIA = 0.01;
  int num_good_points = 2;

  K adapter(num_good_points, MAX_COUNT, DET_EPSILON, CRITERIA);

  P problem(adapter);

  // Construct the dataset line manually using the paths
  // TODO: make the problem able to take in points of different types
  std::ostringstream oss;
  oss << image_2_path << "," << image_3_path << ","
      << image_2_feats_path << ","
      << image_2_feats_next_gt_path;

  std::string line = oss.str();

  ENTO_DEBUG("Deserialize test (std::string): %s", line.c_str());

  // Deserialize the constructed dataset line
  bool success = problem.deserialize(line.c_str());
  
  // Assert deserialization worked
  ENTO_TEST_CHECK_TRUE(success);


  problem.solve();

  ENTO_TEST_CHECK_TRUE(problem.validate());


}

void test_sparse_of_prob_deserialize_char()
{
  using Ke = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<Ke, 7, 7, 1, Kp, Pt>;

  P problem(Ke{}); // Instantiate with NullKernel
  
  // Construct the dataset line manually using the paths
  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s,%s",
           img1_tiny_path, img2_tiny_path,
           img1_tiny_features_path,
           test1_feats_next_gt_path);

  ENTO_DEBUG("Deserialize test (char*): %s", line);

  // Deserialize the constructed dataset line
  bool success = problem.deserialize(line);
  
  // Assert deserialization worked
  ENTO_TEST_CHECK_TRUE(success);

  ENTO_DEBUG("Finished deserialization test (char*)");

  // TODO: Add checks to make sure loaded data is correct.
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
      << img1_tiny_features_path << ","
      << test1_feats_next_gt_path;

  std::string line = oss.str();

  ENTO_DEBUG("Deserialize test (std::string): %s", line.c_str());

  // Deserialize the constructed dataset line
  bool success = problem.deserialize(line);

  // Assert deserialization worked
  // ENTO_ASSERT(success, "Deserialization failed for std::string input");
  ENTO_TEST_CHECK_TRUE(success);

  ENTO_DEBUG("Finished deserialization test (std::string)");

  // TODO: Add checks to make sure loaded data is correct.
}

void test_sparse_of_prob_serialize_char()
{
  using Ke = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<Ke, 7, 7, 1, Kp, Pt>;

  P problem(Ke{}); // Instantiate with NullKernel
  
  // Construct the dataset line manually using the paths
  char line[1024];
  snprintf(line, sizeof(line), "%s,%s,%s,%s",
           img1_tiny_path, img2_tiny_path,
           img1_tiny_features_path,
           test1_feats_next_gt_path);

  ENTO_DEBUG("Deserialize test (char*): %s", line);

  // Deserialize the constructed dataset line
  bool success = problem.deserialize(line);
}

void test_sparse_of_prob_serialize_string()
{
  using Ke = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<Ke, 7, 7, 1, Kp, Pt>;

  P problem(Ke{}); // Instantiate with NullKernel
  
  problem.feats_next_.add_keypoint(Kp(1,1));
  problem.feats_next_.add_keypoint(Kp(2,2));
  problem.feats_next_.add_keypoint(Kp(3,3));

  std::string ser = problem.serialize();

  std::string expected("3,1,1,2,2,3,3");

  bool success = ser != expected;
  ENTO_TEST_CHECK_TRUE(success);
}

void test_sparse_of_prob_run()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<K, 7, 7, 2, Kp, Pt>;

  P problem(K{});

  problem.feats_.add_keypoint(Kp(1,1));
  problem.feats_.add_keypoint(Kp(2,2));
  problem.feats_.add_keypoint(Kp(3,3));
  problem.feats_next_gt_.add_keypoint(Kp(1,1));
  problem.feats_next_gt_.add_keypoint(Kp(2,2));
  problem.feats_next_gt_.add_keypoint(Kp(3,3));

  problem.solve();

  for (size_t i = 0; i < problem.NumFeats_; i++)
  {
    ENTO_TEST_CHECK_TRUE(problem.feats_next_[i].x == problem.feats_next_gt_[i].x);
    ENTO_TEST_CHECK_TRUE(problem.feats_next_[i].y == problem.feats_next_gt_[i].y);
  }
}

void test_sparse_of_prob_validate()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t;
  using P = SparseOpticalFlowProblem<K, 7, 7, 1, Kp, Pt>;

  P problem(K{});

  problem.img1_.data[0] = 1;
  problem.img2_.data[0] = 1;
  problem.feats_.add_keypoint(Kp(1,1));
  problem.feats_next_.add_keypoint(Kp(1,1));
  problem.feats_next_gt_.add_keypoint(Kp(1,1));

  problem.solve();

  ENTO_TEST_CHECK_TRUE(problem.validate());
}

void test_sparse_of_prob_clear()
{
  using K = NullKernel;
  using Kp = Keypoint<int16_t>;
  using Pt = uint8_t; // PixelType
  using P = SparseOpticalFlowProblem<K, 7, 7, 1, Kp, Pt>;

  P problem(K{});

  problem.img1_.data[0] = 1;
  problem.img2_.data[0] = 1;
  problem.feats_.add_keypoint(Kp(1,1));
  problem.feats_next_.add_keypoint(Kp(1,1));
  problem.feats_next_gt_.add_keypoint(Kp(1,1));

  problem.clear();

  ENTO_TEST_CHECK_TRUE(problem.img1_.data[0] == 0);
  ENTO_TEST_CHECK_TRUE(problem.img2_.data[0] == 0);
  ENTO_TEST_CHECK_TRUE(problem.feats_next_.size() == 0);
  ENTO_TEST_CHECK_TRUE(problem.feats_next_gt_.size() == 0);
}


void test_sparse_of_prob_harness_native()
{
  using Kernel = NullKernel;
  using Kp = Keypoint<int16_t>;
  using PixelType = uint8_t;
  using P = SparseOpticalFlowProblem<Kernel, 7, 7, 1, Kp, PixelType>;
  P problem(Kernel{});

  ENTO_DEBUG("header: %s", P::header_impl());

  std::ofstream dataset_file(test1_harness_dataset_path, std::ios::out | std::ios::trunc);
  if (!dataset_file.is_open())
  {
    ENTO_ERROR("Failed to create dataset file: %s", test1_harness_dataset_path);
    return;
  }

  ENTO_DEBUG("Opened path: %s", test1_harness_dataset_path);

  // Write header row
  dataset_file << "Sparse Optical Flow Dataset" << std::endl;

  // Write dataset paths (comma-separated)
  dataset_file << img1_tiny_path << ","
               << img2_tiny_path << ","
               << img1_tiny_features_path << ","
               << test1_feats_next_gt_path << std::endl;

  ENTO_DEBUG("Wrote %s, %s, %s, %s to %s", img1_tiny_path, img2_tiny_path, img1_tiny_features_path, test1_feats_next_gt_path);

  dataset_file.close();

  EntoBench::Harness harness(problem, "Sparse OF Prob Harnes Test",
                             test1_harness_dataset_path,
                             test1_output_path);
  harness.run();

  for (size_t i = 0; i < problem.NumFeats_; ++i)
  {
    ENTO_DEBUG("Problem feats_i: %i, %i", problem.feats_[i].x, problem.feats_[i].y);
    ENTO_DEBUG("Problem feats_next_i: %i, %i", problem.feats_next_[i].x, problem.feats_next_[i].y);
    ENTO_DEBUG("Problem feats_next_gt: %i, %i", problem.feats_next_gt_[i].x, problem.feats_next_gt_[i].y);

  }
  ENTO_TEST_CHECK_TRUE(problem.validate());
}

void test_sparse_of_prob_harness()
{
  using Kernel = NullKernel;
  using Kp = Keypoint<int16_t>;
  using PixelType = uint8_t;
  using P = SparseOpticalFlowProblem<Kernel, 7, 7, 1, Kp, PixelType>;
  P problem(Kernel{});

  ENTO_DEBUG("header: %s", P::header_impl());

  FILE* dataset_file = fopen(test1_harness_dataset_path, "w");
  if (!dataset_file)
  {
    ENTO_ERROR("Failed to create dataset file: %s", test1_harness_dataset_path);
    return;
  }

  // Write header row
  fprintf(dataset_file, "Sparse Optical Flow Problem\n");

  // Write dataset paths (comma-separated)
  fprintf(dataset_file, "%s,%s,%s,%s\n",
          img1_tiny_path,
          img2_tiny_path,
          img1_tiny_features_path,
          test1_feats_next_gt_path);

  fclose(dataset_file);

  EntoBench::Harness harness(problem, "Sparse OF Prob Harnes Test",
                             test1_harness_dataset_path,
                             test1_output_path);
  harness.run();
}


int main ( int argc, char ** argv )
{

  using namespace EntoUtil;
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
    { "test_img1_tiny.pgm" , "test_img1_tiny_feats.txt", "test_img2_tiny.pgm",
      "test_feats_next_gt.txt", "test1_output_path.txt", "test_sparse_of_prob_harness_dataset.txt",
      "image_2.pgm", "image_3.pgm", "image_2_feats.txt", "image_2_feats_next_gt.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
  ENTO_DEBUG("Generated Paths:\n");
  for (size_t i = 0; i < num_paths; ++i) {
    ENTO_DEBUG("  %s\n", full_paths[i]);
  }

  // Run Tests
  if (__ento_test_num(__n, 1)) test_sparse_of_prob_deserialize_char();
  if (__ento_test_num(__n, 2)) test_sparse_of_prob_deserialize_string();
  if (__ento_test_num(__n, 3)) test_sparse_of_prob_serialize_char();
  if (__ento_test_num(__n, 4)) test_sparse_of_prob_serialize_string();
  if (__ento_test_num(__n, 5)) test_sparse_of_prob_run();
  if (__ento_test_num(__n, 6)) test_sparse_of_prob_validate();
  if (__ento_test_num(__n, 7)) test_sparse_of_prob_clear();
  if (__ento_test_num(__n, 8)) test_sparse_of_prob_harness_native();
  if (__ento_test_num(__n, 9)) test_lk_optical_flow_validate();

}
