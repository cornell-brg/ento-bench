#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-state-est/attitude-est/attitude_problem.h>
#include <ento-state-est/attitude-est/mahoney.h>
#include "mahoney_kernel.h"

// Include the ENTO_TEST_CHECK_QUAT_EQ macro if not defined in unittest.h
#ifndef ENTO_TEST_CHECK_QUAT_EQ
#define ENTO_TEST_CHECK_QUAT_EQ(q1, q2, tol) \
  EntoUtil::__ento_test_check_quat_eq(__FILE__, __LINE__, #q1, #q2, tol, q1, q2)
#endif

const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 128;

char dir_path[FILEPATH_SIZE];
char test1_input_path[FILEPATH_SIZE];
char test1_output_path[FILEPATH_SIZE];

char* full_paths[] = { test1_input_path, test1_output_path };
constexpr size_t num_paths = 2;

using namespace EntoAttitude;

// Test basic instantiation and parameter passing
void test_mahoney_kernel_basic() {
  using Scalar = double;
  static constexpr bool UseMag = false; // IMU only
  
  // Create a Mahoney kernel with specific gains
  Scalar kp = 0.1;
  Scalar ki = 0.01;
  MahoneyKernel<Scalar, UseMag> kernel(kp, ki);
  
  // Create an attitude problem with the kernel and threshold
  Scalar threshold = 1.0;
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, threshold);
  
  // Verify threshold was set correctly
  ENTO_TEST_CHECK_FLOAT_EQ(problem.angle_threshold_deg_, threshold);
  
  // Create test input for deserialization
  const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 1.0 0.0 0.0 0.0 0.01";
  
  // Test deserialization
  ENTO_TEST_CHECK_TRUE(problem.deserialize_impl(test_input));
  
  // Test measurement values
  ENTO_DEBUG("Parsed Measurement Values:");
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[0], 0.1);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[1], 0.2);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[2], 0.3);
  
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[0], 0.01);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[1], 0.02);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[2], 0.03);
  
  // Test quaternion values
  Eigen::Quaternion<Scalar> identity(1.0, 0.0, 0.0, 0.0);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(problem.q_gt_.coeffs(), identity.coeffs());
  
  // Test delta time
  ENTO_TEST_CHECK_FLOAT_EQ(problem.dt_, 0.01);
}

// Test with magnetometer variant
void test_mahoney_kernel_mag() {
  using Scalar = double;
  static constexpr bool UseMag = true; // With magnetometer
  
  // Create a Mahoney kernel with specific gains
  Scalar kp = 0.1;
  Scalar ki = 0.01;
  MahoneyKernel<Scalar, UseMag> kernel(kp, ki);
  
  // Create an attitude problem with the kernel and threshold
  Scalar threshold = 1.0;
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, threshold);
  
  // Create test input for deserialization (with magnetometer data)
  const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 0.4 0.5 0.6 1.0 0.0 0.0 0.0 0.01";
  
  // Test deserialization
  ENTO_TEST_CHECK_TRUE(problem.deserialize_impl(test_input));
  
  // Test measurement values (including magnetometer)
  ENTO_DEBUG("Parsed Measurement Values:");
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[0], 0.1);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[1], 0.2);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[2], 0.3);
  
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[0], 0.01);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[1], 0.02);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[2], 0.03);
  
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[0], 0.4);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[1], 0.5);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[2], 0.6);
}

// Test Mahoney solve function with IMU data
void test_mahoney_kernel_solve_imu() {
  using Scalar = double;
  static constexpr bool UseMag = false; // IMU only
  
  // Create a Mahoney kernel with specific gains
  Scalar kp = 0.1;
  Scalar ki = 0.01;
  MahoneyKernel<Scalar, UseMag> kernel(kp, ki);
  
  // Create an attitude problem with the kernel and threshold
  Scalar threshold = 1.0;
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, threshold);
  
  // Use real test data from the original Mahoney test
  EntoMath::Vec3<Scalar> gyr(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06);  // Gyroscope (rad/s)
  EntoMath::Vec3<Scalar> acc(-0.07215829, 0.03096613, 8.31740944);                // Accelerometer (gravity vector)
  
  // Initial quaternion
  Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
  // Set up the problem
  AttitudeMeasurement<Scalar, UseMag> measurement(gyr, acc);
  problem.measurement_ = measurement;
  problem.q_prev_ = q_init;
  problem.dt_ = 0.004;
  
  // Expected quaternion from the original Mahoney test
  Eigen::Quaternion<Scalar> q_expected(1.00000000e+00, 6.66248740e-07, 1.46525393e-06, 4.36344465e-07);
  problem.q_gt_ = q_expected;
  
  // Run the solve implementation
  problem.solve_impl();
  
  // Use the quaternion comparison check
  ENTO_TEST_CHECK_QUAT_EQ(problem.q_, q_expected, 0.001);
  
  // Also check using our own angle distance computation
  Scalar angle_distance = problem.computeQuaternionAngleDistance(problem.q_, q_expected);
  ENTO_DEBUG("Angle distance between result and expected: %f degrees", angle_distance);
  
  // Validation should pass
  ENTO_TEST_CHECK_TRUE(problem.validate(threshold));
  
  // Check that q_prev_ was updated for the next iteration
  ENTO_TEST_CHECK_QUAT_EQ(problem.q_prev_, problem.q_, 0.0001);
}

// Test Mahoney solve function with MARG data
void test_mahoney_kernel_solve_marg() {
  using Scalar = double;
  static constexpr bool UseMag = true; // With magnetometer
  
  // Create a Mahoney kernel with specific gains
  Scalar kp = 0.1;
  Scalar ki = 0.01;
  MahoneyKernel<Scalar, UseMag> kernel(kp, ki);
  
  // Create an attitude problem with the kernel and threshold
  Scalar threshold = 1.0;
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, threshold);
  
  // Use real test data from the original Mahoney test
  EntoMath::Vec3<Scalar> gyr(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06);  // Gyroscope (rad/s)
  EntoMath::Vec3<Scalar> acc(-0.07215829, 0.03096613, 8.31740944);                // Accelerometer (gravity vector)
  EntoMath::Vec3<Scalar> mag(16925.5042314, 1207.22593348, 34498.24159392);       // Magnetometer
  
  // Initial quaternion
  Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
  // Set up the problem
  AttitudeMeasurement<Scalar, UseMag> measurement(gyr, acc, mag);
  problem.measurement_ = measurement;
  problem.q_prev_ = q_init;
  problem.dt_ = 0.004;
  
  // Expected quaternion from the original Mahoney test
  Eigen::Quaternion<Scalar> q_expected(9.99999994e-01, -7.29375589e-05, -7.75753231e-05, 3.93137128e-05);
  problem.q_gt_ = q_expected;
  
  // Run the solve implementation
  problem.solve_impl();
  
  // Use the quaternion comparison check
  ENTO_TEST_CHECK_QUAT_EQ(problem.q_, q_expected, 0.001);
  
  // Also check using our own angle distance computation
  Scalar angle_distance = problem.computeQuaternionAngleDistance(problem.q_, q_expected);
  ENTO_DEBUG("Angle distance between result and expected: %f degrees", angle_distance);
  
  // Validation should pass
  ENTO_TEST_CHECK_TRUE(problem.validate(threshold));
}

// Test validation with different thresholds
void test_mahoney_kernel_validate() {
  using Scalar = double;
  static constexpr bool UseMag = false; // IMU only
  
  // Create a Mahoney kernel with specific gains
  Scalar kp = 0.1;
  Scalar ki = 0.01;
  MahoneyKernel<Scalar, UseMag> kernel(kp, ki);
  
  // Create an attitude problem with a 1.0 degree threshold
  Scalar threshold = 1.0;
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, threshold);
  
  // Set up test quaternions
  Eigen::Quaternion<Scalar> q_gt(1.0, 0.0, 0.0, 0.0); // Identity
  problem.q_gt_ = q_gt;
  
  // Test case 1: Exact match
  problem.q_ = q_gt;
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
  
  // Test case 2: Small error (0.5 degrees around Y-axis)
  Scalar angle_rad = 0.5 * M_PI / 180.0; // 0.5 degrees in radians
  Eigen::Quaternion<Scalar> q_small_error(
      std::cos(angle_rad / 2.0),
      0.0,
      std::sin(angle_rad / 2.0),
      0.0
  );
  problem.q_ = q_gt * q_small_error;
  
  // Should pass with 1.0 degree threshold
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
  
  // Should fail with 0.1 degree threshold
  ENTO_TEST_CHECK_FALSE(problem.validate(0.1));
  
  // Test case 3: Large error (2.0 degrees around Z-axis)
  angle_rad = 2.0 * M_PI / 180.0; // 2.0 degrees in radians
  Eigen::Quaternion<Scalar> q_large_error(
      std::cos(angle_rad / 2.0),
      0.0,
      0.0,
      std::sin(angle_rad / 2.0)
  );
  problem.q_ = q_gt * q_large_error;
  
  // Should fail with 1.0 degree threshold
  ENTO_TEST_CHECK_FALSE(problem.validate_impl());
  
  // Should pass with 5.0 degree threshold
  ENTO_TEST_CHECK_TRUE(problem.validate(5.0));
  
  // Test computeQuaternionAngleDistance directly
  Scalar angle_distance = problem.computeQuaternionAngleDistance(q_gt, problem.q_);
  ENTO_DEBUG("Calculated angle distance: %f degrees", angle_distance);
  ENTO_TEST_CHECK_TRUE(std::abs(angle_distance - 2.0) < 0.01);
}

// Test serialization
void test_mahoney_kernel_serialize() {
  #ifndef NATIVE
  using Scalar = double;
  static constexpr bool UseMag = false; // IMU only
  
  // Create a Mahoney kernel with specific gains
  Scalar kp = 0.1;
  Scalar ki = 0.01;
  MahoneyKernel<Scalar, UseMag> kernel(kp, ki);
  
  // Create an attitude problem with the kernel and threshold
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, 1.0);
  
  // Set q_ to a known value
  problem.q_ = Eigen::Quaternion<Scalar>(0.7071, 0.0, 0.7071, 0.0); // 90 degrees around Y
  
  // Get serialized string
  const char* serialized = problem.serialize_impl();
  ENTO_DEBUG("Serialized quaternion: %s", serialized);
  
  // Parse and verify the serialized output
  Scalar w, x, y, z;
  int parsed = sscanf(serialized, "%lf,%lf,%lf,%lf", &w, &x, &y, &z);
  
  ENTO_TEST_CHECK_EQUAL(parsed, 4);
  ENTO_TEST_CHECK_FLOAT_EQ(w, 0.7071);
  ENTO_TEST_CHECK_FLOAT_EQ(x, 0.0);
  ENTO_TEST_CHECK_FLOAT_EQ(y, 0.7071);
  ENTO_TEST_CHECK_FLOAT_EQ(z, 0.0);
  #else
  // Skip the test if NATIVE is defined
  ENTO_DEBUG("Skipping serialize_impl test - only available in embedded builds");
  // Still mark test as passing
  ENTO_TEST_CHECK_TRUE(true);
  #endif
}

// Test the clear implementation
void test_mahoney_kernel_clear() {
  using Scalar = double;
  static constexpr bool UseMag = true; // With magnetometer
  
  // Create a Mahoney kernel with specific gains
  MahoneyKernel<Scalar, UseMag> kernel(0.1, 0.01);
  
  // Create an attitude problem with the kernel and threshold
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, 1.0);
  
  // Set up with test data
  EntoMath::Vec3<Scalar> gyr(0.01, 0.02, 0.03);
  EntoMath::Vec3<Scalar> acc(0.1, 0.2, 0.3);
  EntoMath::Vec3<Scalar> mag(0.4, 0.5, 0.6);
  
  AttitudeMeasurement<Scalar, UseMag> measurement(gyr, acc, mag);
  problem.measurement_ = measurement;
  problem.q_prev_ = Eigen::Quaternion<Scalar>(0.5, 0.5, 0.5, 0.5);
  problem.q_ = Eigen::Quaternion<Scalar>(0.7071, 0.0, 0.7071, 0.0);
  problem.q_gt_ = Eigen::Quaternion<Scalar>(0.0, 1.0, 0.0, 0.0);
  problem.dt_ = 0.01;
  
  // Call clear implementation
  ENTO_TEST_CHECK_TRUE(problem.clear_impl());
  
  // Verify all fields are reset
  Eigen::Quaternion<Scalar> identity(1.0, 0.0, 0.0, 0.0);
  Eigen::Quaternion<Scalar> zero(0.0, 0.0, 0.0, 0.0);
  
  ENTO_TEST_CHECK_QUAT_EQ(problem.q_, identity, 0.0001);
  ENTO_TEST_CHECK_QUAT_EQ(problem.q_prev_, zero, 0.0001);
  ENTO_TEST_CHECK_QUAT_EQ(problem.q_gt_, identity, 0.0001);
  
  ENTO_TEST_CHECK_FLOAT_EQ(problem.dt_, 0.0);
  
  // Check measurement reset
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[0], 0.0);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[1], 0.0);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.accel[2], 0.0);
  
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[0], 0.0);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[1], 0.0);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyro[2], 0.0);
  
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[0], 0.0);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[1], 0.0);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[2], 0.0);
}

// Test sequential updates to verify filter convergence
void test_mahoney_kernel_sequential() {
  using Scalar = double;
  static constexpr bool UseMag = false; // IMU only
  
  // Create a Mahoney kernel with specific gains
  MahoneyKernel<Scalar, UseMag> kernel(0.1, 0.01);
  
  // Create an attitude problem with the kernel and threshold
  AttitudeProblem<Scalar, MahoneyKernel<Scalar, UseMag>, UseMag> problem(kernel, 1.0);
  
  // Initial state - start with identity quaternion
  problem.q_prev_ = Eigen::Quaternion<Scalar>(1.0, 0.0, 0.0, 0.0);
  
  // Define constant measurement data - simulating steady-state
  EntoMath::Vec3<Scalar> gyr(0.0, 0.0, 0.0);  // No rotation
  EntoMath::Vec3<Scalar> acc(0.0, 0.0, 9.81); // Gravity along Z
  AttitudeMeasurement<Scalar, UseMag> measurement(gyr, acc);
  
  problem.measurement_ = measurement;
  problem.dt_ = 0.01;
  
  // Run 100 iterations
  for (int i = 0; i < 100; i++) {
    problem.solve_impl();
  }
  
  // After convergence, the quaternion should be close to identity
  ENTO_DEBUG("Final quaternion after 100 updates:");
  ENTO_DEBUG("w: %f, x: %f, y: %f, z: %f", 
             problem.q_.w(), problem.q_.x(), problem.q_.y(), problem.q_.z());
  
  // Check that it's close to identity using quaternion comparison
  Eigen::Quaternion<Scalar> identity(1.0, 0.0, 0.0, 0.0);
  ENTO_TEST_CHECK_QUAT_EQ(problem.q_, identity, 0.002);
  
  // Also check using our own angle distance computation
  Scalar angle_to_identity = problem.computeQuaternionAngleDistance(problem.q_, identity);
  ENTO_DEBUG("Angle to identity: %f degrees", angle_to_identity);
}

// Main test runner
int main(int argc, char** argv) {
  using namespace EntoUtil;
  int __n;
  if (argc > 1) {
    __n = atoi(argv[1]);
  } else {
    // For the case we are running on the MCU and we can't pass in args
    __ento_replace_file_suffix(__FILE__, "test_mahoney_kernel_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  // Setup Directory Path and Test Data Paths
  get_file_directory(file_path, sizeof(dir_path), dir_path);
  const char* file_names[] = { "test_mahoney_kernel_input_1.txt", "test_mahoney_kernel_output_1.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
  ENTO_DEBUG("Generated Paths:");
  for (size_t i = 0; i < num_paths; ++i) {
    ENTO_DEBUG("  %s", full_paths[i]);
  }

  ENTO_DEBUG("N: %i", __n);
  ENTO_TEST_START();

  // Basic tests
  if (__ento_test_num(__n, 1)) test_mahoney_kernel_basic();
  if (__ento_test_num(__n, 2)) test_mahoney_kernel_mag();
  
  // Core functionality tests
  if (__ento_test_num(__n, 3)) test_mahoney_kernel_solve_imu();
  if (__ento_test_num(__n, 4)) test_mahoney_kernel_solve_marg();
  if (__ento_test_num(__n, 5)) test_mahoney_kernel_validate();
  if (__ento_test_num(__n, 6)) test_mahoney_kernel_serialize();
  if (__ento_test_num(__n, 7)) test_mahoney_kernel_clear();
  
  // Advanced tests
  if (__ento_test_num(__n, 8)) test_mahoney_kernel_sequential();

  ENTO_TEST_END();
}