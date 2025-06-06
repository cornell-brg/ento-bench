#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/EKFProblem.h>
#include <ento-state-est/ekf_kernels.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using Scalar = double;
constexpr Scalar TOLERANCE = 1e-6;

using namespace EntoUtil;
using namespace EntoStateEst;

//------------------------------------------------------------------------------
// Test RoboBee EKF with Real Golden Data - Validation Test
//------------------------------------------------------------------------------
void test_robobee_real_data_validation()
{
  ENTO_DEBUG("Running test_robobee_real_data_validation...");

  using RoboBeeKernel = RoboBeeEKFKernel<Scalar>;
  using DynamicsModel = RobobeeDynamicsModel<Scalar>;
  using MeasurementModel = RobobeeMeasurementModel<Scalar>;
  
  // Use more realistic noise matrices
  Eigen::Matrix<Scalar, 10, 10> Q = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.001;  // Small process noise
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0, 0) = 0.01;  // Gyro x noise
  R(1, 1) = 0.01;  // Gyro y noise
  R(2, 2) = 0.01;  // Gyro z noise
  R(3, 3) = 0.001; // ToF sensor noise (more accurate)
  
  RoboBeeKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboBeeKernel, DynamicsModel, MeasurementModel, 0> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Load real data from file
  std::ifstream file("../../datasets/state-est/robobee_ekf_data_fixed.csv");
  ENTO_TEST_CHECK_TRUE(file.is_open());
  
  std::string line;
  std::getline(file, line); // Skip header
  
  int line_count = 0;
  int max_lines = 100; // Test with first 100 lines for validation
  
  while (std::getline(file, line) && line_count < max_lines) {
    if (!line.empty()) {
      bool result = problem.deserialize_impl(line.c_str());
      if (!result) {
        ENTO_DEBUG("Failed to parse line %d: %s", line_count, line.c_str());
      }
      ENTO_TEST_CHECK_TRUE(result);
      line_count++;
    }
  }
  file.close();
  
  ENTO_DEBUG("Processed %d lines of real data", line_count);
  
  // Validate trajectory exists and has reasonable length
  auto trajectory = problem.getStateTrajectory();
  ENTO_TEST_CHECK_TRUE(trajectory.size() > 0);
  ENTO_TEST_CHECK_INT_EQ(trajectory.size(), line_count);
  
  // Basic sanity checks on final state
  auto final_state = trajectory.back();
  ENTO_DEBUG("Final state:");
  for (int i = 0; i < 10; ++i) {
    ENTO_DEBUG("  x[%d] = %f", i, final_state(i));
  }
  
  // Check that state values are in reasonable ranges for real bee flight
  // States: [phi, theta, psi, p, q, r, z, vx, vy, vz]
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(0)) < 10.0);  // phi < 10 rad (multiple loops)
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(1)) < 10.0);  // theta < 10 rad  
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(2)) < 10.0);  // psi (can wrap around)
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(3)) < 1000.0); // angular rates can be very high for bees
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(4)) < 1000.0);
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(5)) < 1000.0);
  ENTO_TEST_CHECK_TRUE(final_state(6) > 0 && final_state(6) < 5.0);  // z altitude positive, reasonable
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(7)) < 50.0);  // velocities can be high for bees
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(8)) < 50.0);
  ENTO_TEST_CHECK_TRUE(std::abs(final_state(9)) < 50.0);

  ENTO_DEBUG("test_robobee_real_data_validation PASSED!");
}

//------------------------------------------------------------------------------
// Test Comparison of Three Update Methods with Real Data
//------------------------------------------------------------------------------
void test_robobee_update_methods_comparison()
{
  ENTO_DEBUG("Running test_robobee_update_methods_comparison...");

  using RoboBeeKernel = RoboBeeEKFKernel<Scalar>;
  using DynamicsModel = RobobeeDynamicsModel<Scalar>;
  using MeasurementModel = RobobeeMeasurementModel<Scalar>;
  using EKFProblemType = EKFProblem<RoboBeeKernel, DynamicsModel, MeasurementModel, 0>;
  using UpdateMethod = typename EKFProblemType::UpdateMethod;
  
  // Same initial conditions for all three methods
  Eigen::Matrix<Scalar, 10, 10> Q = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.001;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  
  // Create three EKF problems with different update methods
  RoboBeeKernel kernel1(Q, R), kernel2(Q, R), kernel3(Q, R);
  DynamicsModel dynamics1, dynamics2, dynamics3;
  MeasurementModel measurement1, measurement2, measurement3;
  
  EKFProblemType problem_sync(
    std::move(kernel1), std::move(dynamics1), std::move(measurement1), 
    UpdateMethod::SYNCHRONOUS);
    
  EKFProblemType problem_seq(
    std::move(kernel2), std::move(dynamics2), std::move(measurement2),
    UpdateMethod::SEQUENTIAL);
    
  EKFProblemType problem_trunc(
    std::move(kernel3), std::move(dynamics3), std::move(measurement3),
    UpdateMethod::TRUNCATED);
  
  // Process same data with all three methods
  std::ifstream file("../../datasets/state-est/robobee_ekf_data_fixed.csv");
  ENTO_TEST_CHECK_TRUE(file.is_open());
  
  std::string line;
  std::getline(file, line); // Skip header
  
  std::vector<std::string> test_lines;
  int max_lines = 50; // Smaller set for comparison test
  
  while (std::getline(file, line) && test_lines.size() < max_lines) {
    if (!line.empty()) {
      test_lines.push_back(line);
    }
  }
  file.close();
  
  // Process all lines with each method
  for (const auto& test_line : test_lines) {
    bool r1 = problem_sync.deserialize_impl(test_line.c_str());
    bool r2 = problem_seq.deserialize_impl(test_line.c_str());
    bool r3 = problem_trunc.deserialize_impl(test_line.c_str());
    
    ENTO_TEST_CHECK_TRUE(r1 && r2 && r3);
  }
  
  // Get final states from all three methods
  auto traj_sync = problem_sync.getStateTrajectory();
  auto traj_seq = problem_seq.getStateTrajectory();
  auto traj_trunc = problem_trunc.getStateTrajectory();
  
  ENTO_TEST_CHECK_INT_EQ(traj_sync.size(), test_lines.size());
  ENTO_TEST_CHECK_INT_EQ(traj_seq.size(), test_lines.size());
  ENTO_TEST_CHECK_INT_EQ(traj_trunc.size(), test_lines.size());
  
  auto final_sync = traj_sync.back();
  auto final_seq = traj_seq.back();
  auto final_trunc = traj_trunc.back();
  
  ENTO_DEBUG("Final states comparison:");
  ENTO_DEBUG("Synchronous:  [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    final_sync(0), final_sync(1), final_sync(2), final_sync(3), final_sync(4),
    final_sync(5), final_sync(6), final_sync(7), final_sync(8), final_sync(9));
  ENTO_DEBUG("Sequential:   [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    final_seq(0), final_seq(1), final_seq(2), final_seq(3), final_seq(4),
    final_seq(5), final_seq(6), final_seq(7), final_seq(8), final_seq(9));
  ENTO_DEBUG("Truncated:    [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    final_trunc(0), final_trunc(1), final_trunc(2), final_trunc(3), final_trunc(4),
    final_trunc(5), final_trunc(6), final_trunc(7), final_trunc(8), final_trunc(9));
  
  // The methods should give different results (otherwise they're not really different)
  Scalar diff_sync_seq = (final_sync - final_seq).norm();
  Scalar diff_sync_trunc = (final_sync - final_trunc).norm();
  Scalar diff_seq_trunc = (final_seq - final_trunc).norm();
  
  ENTO_DEBUG("Differences: sync-seq=%.6f, sync-trunc=%.6f, seq-trunc=%.6f",
    diff_sync_seq, diff_sync_trunc, diff_seq_trunc);
  
  // Methods should produce different results (but not wildly different)
  ENTO_TEST_CHECK_TRUE(diff_sync_seq > 1e-6);  // Not identical
  ENTO_TEST_CHECK_TRUE(diff_sync_seq < 10.0);  // Not wildly different
  ENTO_TEST_CHECK_TRUE(diff_sync_trunc > 1e-6);
  ENTO_TEST_CHECK_TRUE(diff_sync_trunc < 10.0);

  ENTO_DEBUG("test_robobee_update_methods_comparison PASSED!");
}

//------------------------------------------------------------------------------
// Test Covariance Evolution Makes Sense
//------------------------------------------------------------------------------
void test_robobee_covariance_evolution()
{
  ENTO_DEBUG("Running test_robobee_covariance_evolution...");

  using RoboBeeKernel = RoboBeeEKFKernel<Scalar>;
  using DynamicsModel = RobobeeDynamicsModel<Scalar>;
  using MeasurementModel = RobobeeMeasurementModel<Scalar>;
  
  Eigen::Matrix<Scalar, 10, 10> Q = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.001;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  
  RoboBeeKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboBeeKernel, DynamicsModel, MeasurementModel, 0> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Load some real data
  std::ifstream file("../../datasets/state-est/robobee_ekf_data_fixed.csv");
  ENTO_TEST_CHECK_TRUE(file.is_open());
  
  std::string line;
  std::getline(file, line); // Skip header
  
  std::vector<Scalar> covariance_traces;
  int line_count = 0;
  int max_lines = 20;
  
  while (std::getline(file, line) && line_count < max_lines) {
    if (!line.empty()) {
      bool result = problem.deserialize_impl(line.c_str());
      ENTO_TEST_CHECK_TRUE(result);
      
      // Get current covariance trace (sum of diagonal elements)
      auto cov_traj = problem.getCovarianceTrajectory();
      if (!cov_traj.empty()) {
        Scalar trace = cov_traj.back().trace();
        covariance_traces.push_back(trace);
        ENTO_DEBUG("Step %d: Covariance trace = %.6f", line_count, trace);
      }
      line_count++;
    }
  }
  file.close();
  
  // Covariance should generally decrease over time (measurements reduce uncertainty)
  // But it's not monotonic due to process noise, so just check it's bounded
  ENTO_TEST_CHECK_TRUE(covariance_traces.size() > 5);
  
  for (Scalar trace : covariance_traces) {
    ENTO_TEST_CHECK_TRUE(trace > 0);      // Positive definite
    ENTO_TEST_CHECK_TRUE(trace < 1000.0); // Not exploding
  }
  
  // Final covariance should be reasonable
  Scalar final_trace = covariance_traces.back();
  ENTO_TEST_CHECK_TRUE(final_trace > 0.1 && final_trace < 100.0);

  ENTO_DEBUG("test_robobee_covariance_evolution PASSED!");
}

//------------------------------------------------------------------------------
// Main Test Runner  
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  using namespace EntoUtil;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_DEBUG("N: %i", __n);
  ENTO_TEST_START();

  if (__ento_test_num(__n, 1)) test_robobee_real_data_validation();
  if (__ento_test_num(__n, 2)) test_robobee_update_methods_comparison(); 
  if (__ento_test_num(__n, 3)) test_robobee_covariance_evolution();

  ENTO_TEST_END();
} 