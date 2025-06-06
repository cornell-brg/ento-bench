#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/EKFProblem.h>
#include <ento-state-est/ekf_kernels.h>
#include <cstring>

using Scalar = double;
constexpr Scalar TOLERANCE = 1e-6;

using namespace EntoUtil;
using namespace EntoStateEst;

//------------------------------------------------------------------------------
// Test RoboFly EKF Problem Basic CSV Line Deserialization
//------------------------------------------------------------------------------
void test_robofly_csv_line_deserialization()
{
  ENTO_DEBUG("Running test_robofly_csv_line_deserialization...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  // Initialize EKF with proper noise matrices
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 100> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Test valid CSV line: timestamp,dt,range,flow_x,flow_y,accel_z,theta,mask0,mask1,mask2,mask3  
  const char* valid_line = "1.2345,0.004,0.075,-0.1,0.05,9.81,0.1,1,1,1,1";
  
  bool result = problem.deserialize_impl(valid_line);
  ENTO_TEST_CHECK_TRUE(result);

  ENTO_DEBUG("test_robofly_csv_line_deserialization PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF Problem CSV Line with Sensor Dropout
//------------------------------------------------------------------------------
void test_robofly_csv_line_sensor_dropout()
{
  ENTO_DEBUG("Running test_robofly_csv_line_sensor_dropout...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 100> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Test CSV line with optical flow sensors offline (mask1=0, mask2=0)
  const char* dropout_line = "2.5,0.004,0.080,0.0,0.0,9.75,0.05,1,0,0,1";
  
  bool result = problem.deserialize_impl(dropout_line);
  ENTO_TEST_CHECK_TRUE(result);

  ENTO_DEBUG("test_robofly_csv_line_sensor_dropout PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF Problem Invalid CSV Line Handling
//------------------------------------------------------------------------------
void test_robofly_csv_line_invalid()
{
  ENTO_DEBUG("Running test_robofly_csv_line_invalid...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 100> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Test various invalid cases
  const char* invalid_lines[] = {
    "",                                              // empty
    "1.0",                                          // too few fields
    "1.0,0.004,0.1",                               // missing fields
    "abc,0.004,0.1,0.2,0.3,9.8,0.1,1,1,1,1",      // non-numeric timestamp
    "1.0,xyz,0.1,0.2,0.3,9.8,0.1,1,1,1,1",        // non-numeric dt
    "1.0,0.004,0.1,0.2,0.3,9.8,0.1,1,1,1,2",      // invalid sensor mask
    "1.0,0.004,0.1,0.2,0.3,9.8,0.1,1,1,1,-1"      // invalid sensor mask
  };
  
  for (const char* invalid_line : invalid_lines) {
    bool result = problem.deserialize_impl(invalid_line);
    ENTO_TEST_CHECK_FALSE(result);
    ENTO_DEBUG("Correctly rejected invalid line: '%s'", invalid_line);
  }

  ENTO_DEBUG("test_robofly_csv_line_invalid PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF Problem Edge Cases
//------------------------------------------------------------------------------
void test_robofly_csv_line_edge_cases()
{
  ENTO_DEBUG("Running test_robofly_csv_line_edge_cases...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 100> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Test with very small values
  const char* small_values = "0.0001,1e-6,1e-10,1e-10,1e-10,9.81,1e-10,1,1,1,1";
  bool result = problem.deserialize_impl(small_values);
  ENTO_TEST_CHECK_TRUE(result);
  
  // Test with negative optical flow (valid)
  const char* negative_flow = "5.0,0.004,0.08,-0.5,-0.3,9.8,-0.2,1,1,1,1";
  result = problem.deserialize_impl(negative_flow);
  ENTO_TEST_CHECK_TRUE(result);
  
  // Test all sensors masked (complete dropout)
  const char* all_masked = "3.0,0.004,0.0,0.0,0.0,0.0,0.0,0,0,0,0";
  result = problem.deserialize_impl(all_masked);
  ENTO_TEST_CHECK_TRUE(result);

  ENTO_DEBUG("test_robofly_csv_line_edge_cases PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF Problem Multiline Deserialization
//------------------------------------------------------------------------------
void test_robofly_multiline_deserialization()
{
  ENTO_DEBUG("Running test_robofly_multiline_deserialization...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 100> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Test multiple single-line deserializations (simulating stream processing)
  const char* lines[] = {
    "1.0,0.004,0.075,-0.02,0.01,9.81,0.1,1,1,1,1",
    "1.004,0.004,0.078,-0.01,0.02,9.80,0.0,1,1,1,1",
    "1.008,0.004,0.080,0.00,0.01,9.82,0.05,1,1,1,0"
  };
  
  for (const char* line : lines) {
    bool result = problem.deserialize_impl(line);
    ENTO_TEST_CHECK_TRUE(result);
  }
  
  // Basic validation
  bool validation_result = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(validation_result);

  ENTO_DEBUG("test_robofly_multiline_deserialization PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF Problem C-String Parse Count
//------------------------------------------------------------------------------
void test_robofly_parse_count()
{
  ENTO_DEBUG("Running test_robofly_parse_count...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 100> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Parse multiple lines and check count
  const char* lines[] = {
    "1.0,0.004,0.075,-0.02,0.01,9.81,0.1,1,1,1,1",
    "1.004,0.004,0.078,-0.01,0.02,9.80,0.0,1,1,1,1",
    "1.008,0.004,0.080,0.00,0.01,9.82,0.05,1,1,1,0"
  };
  
  for (const char* line : lines) {
    bool result = problem.deserialize_impl(line);
    ENTO_TEST_CHECK_TRUE(result);
  }
  
  // Validate we have trajectory data
  bool validation_result = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(validation_result);
  
  // Check trajectory size
  auto trajectory = problem.getStateTrajectory();
  ENTO_TEST_CHECK_INT_EQ(trajectory.size(), 3);

  ENTO_DEBUG("test_robofly_parse_count PASSED!");
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

  if (__ento_test_num(__n, 1)) test_robofly_csv_line_deserialization();
  if (__ento_test_num(__n, 2)) test_robofly_csv_line_sensor_dropout();
  if (__ento_test_num(__n, 3)) test_robofly_csv_line_invalid();
  if (__ento_test_num(__n, 4)) test_robofly_csv_line_edge_cases();
  if (__ento_test_num(__n, 5)) test_robofly_multiline_deserialization();
  if (__ento_test_num(__n, 6)) test_robofly_parse_count();

  ENTO_TEST_END();
} 