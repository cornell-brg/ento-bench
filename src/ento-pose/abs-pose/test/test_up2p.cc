#include <stdlib.h>
#include <cstdio>
#include <limits>
#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/dataset_reader.h>
#include <ento-util/file_path_util.h>
#include <ento-util/containers.h>

#include <ento-pose/abs-pose/up2p.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/data_gen.h>

using namespace std;
using namespace Eigen;
using namespace EntoPose;
using namespace EntoUtil;

void test_up2p_single()
{
  using Scalar = float;
  using Solver = EntoPose::SolverUP2P<Scalar>;
  using Problem = AbsolutePoseProblem<Scalar, Solver, 0>;
  constexpr Scalar tol = 1e-4;

  const char* base_path = DATASET_PATH;
  const char* rel_path = "abs-pose/test/up2p_float_n1.csv";
  char full_path[256];
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_up2p_single...");
  if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for test_up2p_single!");
  }
  Problem problem(Solver{});

  ENTO_DEBUG("File path: %s", full_path);
  DatasetReader reader(full_path);

  int num_experiments = 0;
  while (reader.read_next(problem))
  {
    num_experiments++;
  }
  ENTO_TEST_CHECK_INT_EQ(num_experiments, 1);
  
  //EntoContainer<CameraPose<Scalar>> solutions;
  int num_solns = up2p(problem.x_point_, problem.X_point_, &problem.solns_);
  for (size_t i = 0; i < static_cast<size_t>(num_solns); i++)
  {
    ENTO_DEBUG_EIGEN_MATRIX(problem.solns_[i].q, 4, 1, Scalar);
    ENTO_DEBUG_EIGEN_MATRIX(problem.solns_[i].t, 3, 1, Scalar);
  }

  Scalar pose_error = std::numeric_limits<Scalar>::max();
  for (size_t i = 0; i < static_cast<size_t>(num_solns); i++)
  {
    const CameraPose<Scalar> &pose = problem.solns_[i];
    pose_error = std::min(pose_error, Problem::validator_::compute_pose_error(pose,
                                                                              problem.pose_gt_,
                                                                              1.0,
                                                                              problem.scale_gt_));
    ENTO_DEBUG("Pose error for soln %li/%i: %f", i+1, num_solns, pose_error);
  }

  bool found_gt_pose = (pose_error < tol);
  
  ENTO_TEST_CHECK_TRUE(found_gt_pose);
  ENTO_DEBUG("================\n");
}

void test_up2p_multi()
{
  using Scalar = float;
  using Solver = EntoPose::SolverUP2P<Scalar>;
  using Problem = AbsolutePoseProblem<Scalar, Solver, 0>;
  constexpr Scalar tol = 1e-4;

  const char* base_path = DATASET_PATH;
  const char* rel_path = "abs-pose/test/up2p_float_n10.csv";
  char full_path[256];

  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_up2p_multi...");

  if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for test_up2p_single!");
  }
  Problem problem(Solver{});

  ENTO_DEBUG("File path: %s", full_path);
  DatasetReader reader(full_path);

  int num_experiments = 0;
  bool found_gt_pose, found_all_poses = true;
  Scalar pose_error;
  EntoContainer<CameraPose<Scalar>> solutions;
  while (reader.read_next(problem))
  {
    num_experiments++;
    ENTO_DEBUG("Test %i/10", num_experiments);

    int num_solns = up2p(problem.x_point_, problem.X_point_, &problem.solns_);
    pose_error = std::numeric_limits<Scalar>::max();
    ENTO_DEBUG("Num solutions: %i", num_solns);

  

    for (size_t i = 0; i < static_cast<size_t>(num_solns); i++)
    {
      const CameraPose<Scalar> &pose = solutions[i];
      pose_error = std::min(pose_error, Problem::validator_::compute_pose_error(pose,
                                                  problem.pose_gt_, 1.0, problem.scale_gt_));
      ENTO_DEBUG("Pose error for soln %li/%i: %f", i+1, num_solns, pose_error);
    }

    found_gt_pose = (pose_error < tol);
    found_all_poses &= found_gt_pose;
    ENTO_TEST_CHECK_TRUE(found_gt_pose);
    problem.clear();
    ENTO_DEBUG("================");
  }
  ENTO_TEST_CHECK_TRUE(found_all_poses);
  ENTO_TEST_CHECK_INT_EQ(num_experiments, 10);
  ENTO_DEBUG("End of test_up2p_multi.");
  ENTO_DEBUG("================\n");
}

void test_up2p_ento_array_single()
{
  using Scalar = float;
  using Solver = EntoPose::SolverUP2P<Scalar>;
  using Problem = AbsolutePoseProblem<Scalar, Solver, 2>;
  //typedef CalibPoseValidator<Scalar, 4> validator;
  constexpr Scalar tol = 1e-4;

  const char* base_path = DATASET_PATH;
  const char* rel_path = "abs-pose/test/up2p_float_n1.csv";
  char full_path[256];
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_up2p_single...");
  if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for test_up2p_single!");
  }
  Problem problem(Solver{});

  ENTO_DEBUG("File path: %s", full_path);
  DatasetReader reader(full_path);

  int num_experiments = 0;
  while (reader.read_next(problem))
  {
    num_experiments++;
  }
  ENTO_TEST_CHECK_INT_EQ(num_experiments, 1);
  
  EntoContainer<CameraPose<Scalar>, 4> solutions;
  int num_solns = up2p(problem.x_point_, problem.X_point_, &problem.solns_);

  Scalar pose_error = std::numeric_limits<Scalar>::max();
  for (size_t i = 0; i < static_cast<size_t>(num_solns); i++)
  {
    const CameraPose<Scalar> &pose = solutions[i];
    pose_error = std::min(pose_error, Problem::validator_::compute_pose_error(pose,
                                                problem.pose_gt_, 1.0, problem.scale_gt_));
    ENTO_DEBUG("Pose error for soln %li/%i: %f", i+1, num_solns, pose_error);
  }

  bool found_gt_pose = (pose_error < tol);
  
  ENTO_TEST_CHECK_TRUE(found_gt_pose);
  ENTO_DEBUG("================\n");
}

void test_up2p_ento_array_multi()
{
  using Scalar = float;
  using Solver = EntoPose::SolverUP2P<Scalar>;
  using Problem = AbsolutePoseProblem<Scalar, Solver, 2>;
  typedef CalibPoseValidator<Scalar, 4> validator;
  constexpr Scalar tol = 1e-4;

  const char* base_path = DATASET_PATH;
  const char* rel_path = "abs-pose/test/up2p_float_n10.csv";
  char full_path[256];

  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_up2p_multi...");

  if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for test_up2p_single!");
  }
  Problem problem;

  ENTO_DEBUG("File path: %s", full_path);
  DatasetReader reader(full_path);

  int num_experiments = 0;
  bool found_gt_pose, found_all_poses = true;
  Scalar pose_error;
  EntoContainer<CameraPose<Scalar>, 4> solutions;
  while (reader.read_next(problem))
  {
    num_experiments++;
    ENTO_DEBUG("Test %i/10", num_experiments);

    ENTO_DEBUG("Solutions size: %li", solutions.size());
    int num_solns = up2p(problem.x_point_, problem.X_point_, &solutions);
    pose_error = std::numeric_limits<Scalar>::max();
    ENTO_DEBUG("Num solutions: %i", num_solns);

  

    for (size_t i = 0; i < static_cast<size_t>(num_solns); i++)
    {
      const CameraPose<Scalar> &pose = solutions[i];
      pose_error = std::min(pose_error, validator::compute_pose_error(problem, pose, 1.0));
      ENTO_DEBUG("Pose error for soln %li/%i: %f", i+1, num_solns, pose_error);
    }

    found_gt_pose = (pose_error < tol);
    found_all_poses &= found_gt_pose;
    ENTO_TEST_CHECK_TRUE(found_gt_pose);
    solutions.clear();
    ENTO_DEBUG("================");
  }
  ENTO_TEST_CHECK_TRUE(found_all_poses);
  ENTO_TEST_CHECK_INT_EQ(num_experiments, 10);
  ENTO_DEBUG("End of test_up2p_multi.");
  ENTO_DEBUG("================\n");
}




int main ( int argc, char ** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_pose_est_reader_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_up2p_single();
  if (__ento_test_num(__n, 2)) test_up2p_multi();
  if (__ento_test_num(__n, 3)) test_up2p_ento_array_single();
  if (__ento_test_num(__n, 4)) test_up2p_ento_array_multi();
}

