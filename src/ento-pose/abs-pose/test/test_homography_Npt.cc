#include <stdlib.h>

#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/dataset_reader.h>
#include <ento-util/file_path_util.h>
#include <ento-util/containers.h>

#include <ento-pose/abs-pose/p4p.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/prob_gen.h>


using namespace std;
using namespace Eigen;
using namespace EntoPose;
using namespace EntoUtil;

const char* dataset_path = DATASET_PATH;

void test_homography_4pt_single()
{
  using Scalar = float32_t;
  using Problem = HomographyProblemInstance<Scalar>;
  typedef HomographyValidator<Scalar> validator;

  constexpr Scalar tol = 1e-4;
  const char* base_path = DATASET_PATH;
  const char* rel_path = "homography/test/homography4pt_float_1.csv";

  char full_path[256];
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_p4p_svd_single...");
  if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for test_p4p_svd_single!");
  }
  Problem problem;
  problem.x1_.reserve(4);
  problem.x2_.reserve(4);

  ENTO_DEBUG("File path: %s", full_path);
  DatasetReader<Problem> reader(full_path);

  int num_experiments = 0;
  bool found_gt_pose = false;
  Matrix3x3<Scalar> solution;
  while (reader.read_next(problem))
  {
    num_experiments++;
    int success = homography_Npt<Scalar>(problem.x1_, problem.x2_, &solution);
    if (success)
    {
      ENTO_DEBUG_EIGEN_MATRIX(solution, 3, 3, Scalar);

      Scalar H_error = std::numeric_limits<Scalar>::max();
      const Matrix3x3<Scalar> &H_gt = solution;
      H_error = std::min(H_error, validator::compute_pose_error(problem, H_gt));
      ENTO_DEBUG("Pose error for homography: %f", H_error);
      found_gt_pose = (H_error < tol);
    }
    else
    {
      ENTO_DEBUG("Did not find homography!");
    }
  }
  ENTO_TEST_CHECK_INT_EQ(num_experiments, 1);
  ENTO_TEST_CHECK_TRUE(found_gt_pose);
  ENTO_DEBUG("================\n");
}

void test_homography_4pt_multi()
{
  using Scalar = float32_t;
  using Problem = HomographyProblemInstance<Scalar>;
  typedef HomographyValidator<Scalar> validator;

  constexpr Scalar tol = 1e-4;
  const char* base_path = DATASET_PATH;
  const char* rel_path = "homography/test/homography4pt_float_5.csv";

  char full_path[256];
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_p4p_tsj_svd_multi...");
  if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for test_p4p_tsj_svd_multi!");
  }
  Problem problem;

  ENTO_DEBUG("File path: %s", full_path);
  DatasetReader<Problem> reader(full_path);

  int num_experiments = 0;
  Matrix3x3<Scalar> solution;
  bool found_gt_pose = true;
  bool found_all_poses = true;
  while (reader.read_next(problem))
  {
    num_experiments++;
    ENTO_DEBUG("Test %i/10", num_experiments);
    int success = homography_Npt<Scalar, 0, 1>(problem.x1_, problem.x2_, &solution);
    if (success)
    {
      ENTO_DEBUG_EIGEN_MATRIX(solution, 3, 3, Scalar);

      Scalar H_error = std::numeric_limits<Scalar>::max();
      const Matrix3x3<Scalar> &H_gt = solution;
      H_error = std::min(H_error, validator::compute_pose_error(problem, H_gt));
      ENTO_DEBUG("Pose error for homography: %f", H_error);
      found_gt_pose = (H_error < tol);
    }
    else
    {
      ENTO_DEBUG("Did not find homography!");
      found_gt_pose = false;
    }
    found_all_poses &= found_gt_pose;
    ENTO_TEST_CHECK_TRUE(found_gt_pose);
  }
  ENTO_TEST_CHECK_INT_EQ(num_experiments, 5);
  ENTO_TEST_CHECK_TRUE(found_all_poses);


  
  ENTO_DEBUG("================\n");
 
}

void test_homography_Npt_single()
{
  
}

void test_homography_Npt_multi()
{
  using Scalar = float32_t;
  using Problem = HomographyProblemInstance<Scalar>;
  typedef HomographyValidator<Scalar> validator;

  constexpr Scalar tol = 1e-4;
  const char* base_path = DATASET_PATH;
  const char* rel_path = "homography/test/homography4pt_float_5.csv";
  for (int N = 8; N < 512; N*=2)
  {
    char full_path[256];
    ENTO_DEBUG("================\n");
    ENTO_DEBUG("Running test_p4p_tsj_svd_multi...");
    if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
    {
      ENTO_DEBUG("ERROR! Could not build file path for test_p4p_tsj_svd_multi!");
    }
    Problem problem;

    ENTO_DEBUG("File path: %s", full_path);
    DatasetReader<Problem> reader(full_path);

    int num_experiments = 0;
    Matrix3x3<Scalar> solution;
    bool found_gt_pose = true;
    bool found_all_poses = true;
    while (reader.read_next(problem))
    {
      num_experiments++;
      ENTO_DEBUG("Test %i/10", num_experiments);
      int success = homography_Npt<Scalar, 0, 1>(problem.x1_, problem.x2_, &solution);
      if (success)
      {
        ENTO_DEBUG_EIGEN_MATRIX(solution, 3, 3, Scalar);

        Scalar H_error = std::numeric_limits<Scalar>::max();
        const Matrix3x3<Scalar> &H_gt = solution;
        H_error = std::min(H_error, validator::compute_pose_error(problem, H_gt));
        ENTO_DEBUG("Pose error for homography: %f", H_error);
        found_gt_pose = (H_error < tol);
      }
      else
      {
        ENTO_DEBUG("Did not find homography!");
        found_gt_pose = false;
      }
      found_all_poses &= found_gt_pose;
      ENTO_TEST_CHECK_TRUE(found_gt_pose);
    }
    ENTO_TEST_CHECK_INT_EQ(num_experiments, 5);
    ENTO_TEST_CHECK_TRUE(found_all_poses);
    ENTO_DEBUG("================\n");

  }


}
