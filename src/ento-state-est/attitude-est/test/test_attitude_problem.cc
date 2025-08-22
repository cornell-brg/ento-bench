#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>

const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 128;

char dir_path[FILEPATH_SIZE];
char test1_input_path[FILEPATH_SIZE];
char test1_output_path[FILEPATH_SIZE];
// Add more char arrays for other input/output paths.

char* full_paths[] = { test1_input_path, test1_output_path };
constexpr size_t num_paths = 2;

using namespace EntoAttitude;

template <typename Scalar>
class TestAttitudeFilter {
public:
  inline void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                         [[maybe_unused]] const MARGMeasurement<Scalar>& meas,
                         [[maybe_unused]] Scalar dt,
                         Eigen::Quaternion<Scalar>* q)
  {
    *q = q_prev; // always returns q_prev;
  }

  inline void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                         [[maybe_unused]] const IMUMeasurement<Scalar>& meas,
                         [[maybe_unused]] Scalar dt,
                         Eigen::Quaternion<Scalar>* q)
  {
    *q = q_prev; // always returns q_prev;
  }

  static constexpr const char* name()
  {
    return "Test Attitude Filter";
  }
};

void test_attitude_problem_basic() {
  using Scalar = float;
  using Kernel = TestAttitudeFilter<Scalar>;
  static constexpr bool UseMag = true;
  using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
  
  Kernel k;
  Problem problem(k);

  // Create test input for deserialization
  const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 0.4 0.5 0.6 1.0 0.0 0.0 0.0 0.01";
  
  // Test deserialization
  ENTO_TEST_CHECK_TRUE(problem.deserialize_impl(test_input));
  
  // Test measurement values
  // Using the proper vector-based field access:
  ENTO_DEBUG("Parsed Measurement Values:");
  if constexpr (UseMag) {
    // MARGMeasurement fields
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[0], 0.1f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[1], 0.2f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[2], 0.3f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[0], 0.01f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[1], 0.02f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[2], 0.03f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[0], 0.4f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[1], 0.5f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[2], 0.6f);
  } else {
    // IMUMeasurement fields
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[0], 0.1f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[1], 0.2f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[2], 0.3f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[0], 0.01f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[1], 0.02f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[2], 0.03f);
  }
  
  // Test quaternion values
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.w(), 1.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.x(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.y(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.z(), 0.0f);
  
  // Test delta time
  ENTO_TEST_CHECK_FLOAT_EQ(problem.dt_, 0.01f);
}




// New test for solve_impl
void test_attitude_problem_solve() {
  using Scalar = float;
  using Kernel = TestAttitudeFilter<Scalar>;
  static constexpr bool UseMag = true;
  using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
  
  Kernel k;
  Problem problem(k);

  // Set up test data
  const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 0.4 0.5 0.6 1.0 0.0 0.0 0.0 0.01";
  ENTO_TEST_CHECK_TRUE(problem.deserialize_impl(test_input));
  
  // Ensure q_prev is zero initially (this will test the initialization from q_gt)
  problem.q_prev_ = Eigen::Quaternion<Scalar>(0, 0, 0, 0);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.w(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.x(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.y(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.z(), 0.0f);
  
  // Now call solve_impl - it should initialize q_prev with q_gt and then set q to the same value
  problem.solve_impl();

  // Since our TestAttitudeFilter just returns q_prev, and q_prev was initialized with q_gt,
  // q should now be equal to q_gt
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.w(), problem.q_gt_.w());
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.x(), problem.q_gt_.x());
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.y(), problem.q_gt_.y());
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.z(), problem.q_gt_.z());
  
  // And q_prev should be updated to the new q value
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.w(), problem.q_.w());
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.x(), problem.q_.x());
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.y(), problem.q_.y());
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.z(), problem.q_.z());
  
  // Call solve_impl a second time to verify that q_prev is used
  // Create a custom ground truth (which should be ignored in this call)
  problem.q_gt_ = Eigen::Quaternion<Scalar>(0.5f, 0.5f, 0.5f, 0.5f);
  
  // Solve again
  problem.solve_impl();
  
  // Check that q still matches the original q_gt because our filter just returns q_prev
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.w(), 1.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.x(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.y(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.z(), 0.0f);
}

// Test for validate_impl
void test_attitude_problem_validate() {
  using Scalar = float;
  using Kernel = TestAttitudeFilter<Scalar>;
  static constexpr bool UseMag = true;
  using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
  
  Kernel k;
  Problem problem(k);

  // Initialize the problem with default test data
  const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 0.4 0.5 0.6 1.0 0.0 0.0 0.0 0.01";
  ENTO_TEST_CHECK_TRUE(problem.deserialize_impl(test_input));
  
  // Test case 1: Exact match - q_ exactly matches q_gt_
  // Set q_ to be the same as q_gt_
  problem.q_ = problem.q_gt_;
  // Validation should pass (angle difference is 0 degrees)
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
  
  // Test case 2: Small angle difference (2 degrees rotation around Y-axis)
  // Create a quaternion with a small rotation around Y
  Scalar angle_rad = static_cast<Scalar>(2.0 * M_PI / 180.0); // 2 degrees in radians
  Eigen::Quaternion<Scalar> q_small_diff(
      std::cos(angle_rad / 2.0f),
      0.0f,
      std::sin(angle_rad / 2.0f),
      0.0f
  );
  // Apply this to the ground truth
  problem.q_ = problem.q_gt_ * q_small_diff;
  
  // Validation should still pass (angle difference is less than 5 degrees)
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
  
  // Test case 3: Angle difference exactly at the threshold (5 degrees)
  angle_rad = static_cast<Scalar>(5.0 * M_PI / 180.0); // 5 degrees in radians
  Eigen::Quaternion<Scalar> q_threshold_diff(
      std::cos(angle_rad / 2.0f),
      0.0f,
      std::sin(angle_rad / 2.0f),
      0.0f
  );
  
  // Apply this to the ground truth
  problem.q_ = problem.q_gt_ * q_threshold_diff;
  
  // Validation should fail (angle difference is exactly at the threshold)
  // The validate_impl returns true if angle < threshold, so equality fails
  ENTO_TEST_CHECK_FALSE(problem.validate_impl());
  
  // Test case 4: Large angle difference (10 degrees rotation around Z-axis)
  angle_rad = static_cast<Scalar>(10.0 * M_PI / 180.0); // 10 degrees in radians
  Eigen::Quaternion<Scalar> q_large_diff(
      std::cos(angle_rad / 2.0f),
      0.0f,
      0.0f,
      std::sin(angle_rad / 2.0f)
  );
  
  // Apply this to the ground truth
  problem.q_ = problem.q_gt_ * q_large_diff;
  
  // Validation should fail (angle difference is greater than 5 degrees)
  ENTO_TEST_CHECK_FALSE(problem.validate_impl());
  
  // Test case 5: Opposite quaternions (180 degrees difference)
  // Set q_ to be the negative of q_gt_ (represents the same rotation in 3D space)
  problem.q_ = Eigen::Quaternion<Scalar>(-problem.q_gt_.w(), -problem.q_gt_.x(), -problem.q_gt_.y(), -problem.q_gt_.z());
  
  // Validation should pass because dot product is absolute
  // |q1·q2| will be 1.0 even though the quaternions have opposite signs
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
}

void test_attitude_problem_serialize() {
  #ifndef NATIVE
    using Scalar = float;
    using Kernel = TestAttitudeFilter<Scalar>;
    static constexpr bool UseMag = true;
    using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
    
    Kernel k;
    Problem problem(k);
    
    // Test case 1: Identity quaternion (1,0,0,0)
    problem.q_ = Eigen::Quaternion<Scalar>(1.0f, 0.0f, 0.0f, 0.0f);
    const char* serialized = problem.serialize_impl();
    ENTO_DEBUG("Serialized quaternion (identity): %s", serialized);
    
    // Check format - should be "1.000000,0.000000,0.000000,0.000000" or similar
    // We can't directly compare strings because of float rounding, so we'll parse back
    float w, x, y, z;
    int parsed = sscanf(serialized, "%f,%f,%f,%f", &w, &x, &y, &z);
    
    ENTO_TEST_CHECK_EQUAL(parsed, 4); // Should parse 4 values
    ENTO_TEST_CHECK_FLOAT_EQ(w, 1.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(x, 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(y, 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(z, 0.0f);

    // Test case 2: Custom quaternion
    problem.q_ = Eigen::Quaternion<Scalar>(0.5f, 0.5f, 0.5f, 0.5f);
    
    serialized = problem.serialize_impl();
    ENTO_DEBUG("Serialized quaternion (custom): %s", serialized);
    
    parsed = sscanf(serialized, "%f,%f,%f,%f", &w, &x, &y, &z);
    
    ENTO_TEST_CHECK_EQUAL(parsed, 4); // Should parse 4 values
    ENTO_TEST_CHECK_FLOAT_EQ(w, 0.5f);
    ENTO_TEST_CHECK_FLOAT_EQ(x, 0.5f);
    ENTO_TEST_CHECK_FLOAT_EQ(y, 0.5f);
    ENTO_TEST_CHECK_FLOAT_EQ(z, 0.5f);
    // Test case 3: Verify buffer size is sufficient
  // Create a quaternion with long decimal places
  problem.q_ = Eigen::Quaternion<Scalar>(1.234567f, 2.345678f, 3.456789f, 4.567890f);
  
  serialized = problem.serialize_impl();
  ENTO_DEBUG("Serialized quaternion (long decimal): %s", serialized);
  
  // Buffer should be large enough (64 chars)
  ENTO_TEST_CHECK_TRUE(strlen(serialized) < 64);
  
  parsed = sscanf(serialized, "%f,%f,%f,%f", &w, &x, &y, &z);
  
  ENTO_TEST_CHECK_EQUAL(parsed, 4); // Should parse 4 values
  ENTO_TEST_CHECK_FLOAT_EQ(w, 1.234567f);
  ENTO_TEST_CHECK_FLOAT_EQ(x, 2.345678f);
  ENTO_TEST_CHECK_FLOAT_EQ(y, 3.456789f);
  ENTO_TEST_CHECK_FLOAT_EQ(z, 4.567890f);
#else
  // Skip the test if NATIVE is defined
  ENTO_DEBUG("Skipping serialize_impl test - only available in embedded builds");
  // Still mark test as passing
  ENTO_TEST_CHECK_TRUE(true);
  #endif
}

// Test for clear_impl
void test_attitude_problem_clear() {
  using Scalar = float;
  using Kernel = TestAttitudeFilter<Scalar>;
  static constexpr bool UseMag = true;
  using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
  
  Kernel k;
  Problem problem(k);

  // First, set up the problem with non-default values
  // Initialize with test data
  const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 0.4 0.5 0.6 1.0 0.0 0.0 0.0 0.01";
  problem.deserialize_impl(test_input);
  
  // Set q_ to a custom value
  problem.q_ = Eigen::Quaternion<Scalar>(0.5f, 0.5f, 0.5f, 0.5f);
  
  // Run solve_impl to ensure q_prev_ has a value
  problem.solve_impl();
  
  // Verify pre-conditions before clearing
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.w(), 0.5f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.x(), 0.5f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.y(), 0.5f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.z(), 0.5f);
  
  // q_prev_ should be the same as q_ after solve_impl
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.w(), 0.5f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.x(), 0.5f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.y(), 0.5f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.z(), 0.5f);
  
  // Measurement should have non-zero values
  if constexpr (UseMag) {
    // Verify MARGMeasurement has values
    ENTO_TEST_CHECK_TRUE(std::abs(problem.measurement_.acc[0] - 0.1f) < 1e-6f);
    ENTO_TEST_CHECK_TRUE(std::abs(problem.measurement_.gyr[0] - 0.01f) < 1e-6f);
    ENTO_TEST_CHECK_TRUE(std::abs(problem.measurement_.mag[0] - 0.4f) < 1e-6f);
  } else {
    // Verify IMUMeasurement has values
    ENTO_TEST_CHECK_TRUE(std::abs(problem.measurement_.acc[0] - 0.1f) < 1e-6f);
    ENTO_TEST_CHECK_TRUE(std::abs(problem.measurement_.gyr[0] - 0.01f) < 1e-6f);
  }
  
  // Delta time should be 0.01
  ENTO_TEST_CHECK_FLOAT_EQ(problem.dt_, 0.01f);
  
  // Now call clear_impl
  problem.clear_impl();
  
  // Check that variables are reset to their defaults
  
  // q_ should be Identity (1,0,0,0)
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.w(), 1.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.x(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.y(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_.z(), 0.0f);
  
  // q_prev_ should be Zero (0,0,0,0)
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.w(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.x(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.y(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_prev_.z(), 0.0f);
  
  // q_gt_ should be Identity (1,0,0,0)
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.w(), 1.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.x(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.y(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.z(), 0.0f);
  
  // dt_ should be 0.0
  ENTO_TEST_CHECK_FLOAT_EQ(problem.dt_, 0.0f);
  
  // Measurement values should be reset to zero/default
  if constexpr (UseMag) {
    // Check MARGMeasurement is reset
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[0], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[1], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[2], 0.0f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[0], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[1], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[2], 0.0f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[0], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[1], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[2], 0.0f);
  } else {
    // Check IMUMeasurement is reset
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[0], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[1], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[2], 0.0f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[0], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[1], 0.0f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[2], 0.0f);
  }
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
  const char* file_names[] = { "test_attitude_problem_input_1.txt" , "test_attitude_problem_output_1.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
  printf("Generated Paths:\n");
  for (size_t i = 0; i < num_paths; ++i) {
    printf("  %s\n", full_paths[i]);
  }

  // Run Tests

  // Base cases
  if (__ento_test_num(__n, 1)) test_attitude_problem_basic();
  if (__ento_test_num(__n, 2)) test_attitude_problem_solve();
  if (__ento_test_num(__n, 3)) test_attitude_problem_validate();
  if (__ento_test_num(__n, 4)) test_attitude_problem_serialize();
  if (__ento_test_num(__n, 5)) test_attitude_problem_clear();



}
