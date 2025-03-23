#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/madgwick.h>

using Scalar = double;
using namespace EntoMath;

constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

// Adapter class to make Madgwick filter work with AttitudeProblem
template <typename Scalar, bool UseMag>
class MadgwickAdapter
{
private:
    // Madgwick filter parameter
    Scalar gain_;

public:
   
    MadgwickAdapter(Scalar gain = static_cast<Scalar>(0.001))
        : gain_(gain) {}
    
    // Operator that matches the interface expected by AttitudeProblem
    void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                   const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& measurement,
                   Scalar dt,
                   Eigen::Quaternion<Scalar>* q_out)
    {
        // Call the appropriate Madgwick update function based on UseMag
        if constexpr (UseMag) {
            // For MARG (with magnetometer)
            *q_out = EntoAttitude::madgwick_update_marg(
                q_prev,
                measurement.gyr,  // Gyroscope vector
                measurement.acc,  // Accelerometer vector
                measurement.mag,  // Magnetometer vector
                dt,
                gain_
            );
        } else {
            // For IMU (no magnetometer)
            *q_out = EntoAttitude::madgwick_update_imu(
                q_prev,
                measurement.gyr,  // Gyroscope vector
                measurement.acc,  // Accelerometer vector
                dt,
                gain_
            );
        }
    }
    
    // Name method for identification
    static constexpr const char* name()
    {
        if constexpr (UseMag) {
            return "Madgwick MARG Filter";
        } else {
            return "Madgwick IMU Filter";
        }
    }
};


//------------------------------------------------------------------------------
// Test for Madgwick IMU Update
//------------------------------------------------------------------------------
void test_madgwick_imu_update()
{
  ENTO_DEBUG("Running test_madgwick_imu_update...");

  // Initial quaternion (identity)
  Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
  // Sample IMU sensor readings (from RoboBee dataset)
  EntoMath::Vec3<Scalar> gyr(-1.52713681e-04,-6.10919329e-05,-4.35697544e-06);  // Gyroscope (rad/s)
  EntoMath::Vec3<Scalar> acc(-0.07215829,0.03096613,8.31740944);   // Accelerometer (gravity vector)

  Scalar dt = 0.004;  // Time step (s)
  Scalar gain = 0.001; // Madgwick filter gain

  // Print Out Matrices
  ENTO_DEBUG_EIGEN_MATRIX(q_init.coeffs());
  ENTO_DEBUG_EIGEN_MATRIX(gyr);
  ENTO_DEBUG_EIGEN_MATRIX(acc);

  // Run the Madgwick IMU update
  Eigen::Quaternion<Scalar> q_updated = EntoAttitude::madgwick_update_imu(q_init, gyr, acc, dt, gain);

  // Hardcoded expected quaternion (replace with actual values from Python)
  Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,1.49870419e-06,3.40537366e-06,4.36343901e-07); // Placeholder

  // Validate each quaternion component
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
  ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
  ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

  ENTO_DEBUG("test_madgwick_imu_update PASSED!");
}

//------------------------------------------------------------------------------
// Test for Madgwick MARG Update
//------------------------------------------------------------------------------
void test_madgwick_marg_update()
{
  ENTO_DEBUG("Running test_madgwick_marg_update...");

  // Initial quaternion (identity)
  Eigen::Quaternion<Scalar> q_init(1.0, 0.0, 0.0, 0.0);

  // Sample IMU sensor readings
  EntoMath::Vec3<Scalar> gyr(1.31868102828397E-06,-2.71143525493424E-06,3.17779011484456E-06);  // Gyroscope (rad/s)
  EntoMath::Vec3<Scalar> acc(1.13147340585962E-01,1.5124603405802E-01,8.4355704699742E+00);   // Accelerometer (gravity vector)
  EntoMath::Vec3<Scalar> mag(-1.69710456595215E+04,-9.92469996952225E+02,3.46472685717592E+04);

  Scalar dt = 0.04f;  // Time step (s)
  Scalar gain = 0.001f; // Madgwick filter gain

  // Run the Madgwick MARG update
  Eigen::Quaternion<Scalar> q_updated = EntoAttitude::madgwick_update_marg(q_init, gyr, acc, mag, dt, gain);

  // Hardcoded expected quaternion (replace with actual values from Python)
  Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,1.49870419e-06,3.40537366e-06,4.36343901e-07); // Placeholder

  //ENTO_DEBUG("Expected Quaternion: (%f, %f, %f, %f)", q_expected.w(), q_expected.x(), q_expected.y(), q_expected.z());
  //ENTO_DEBUG("Computed Quaternion: (%f, %f, %f, %f)", q_updated.w(), q_updated.x(), q_updated.y(), q_updated.z());

  // Validate each quaternion component
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());

  ENTO_DEBUG("test_madgwick_marg_update PASSED!");
}

//------------------------------------------------------------------------------
// Test for Madgwick IMU Problem
//------------------------------------------------------------------------------
void test_madgwick_imu_problem()
{
    ENTO_DEBUG("Running test_madgwick_imu_problem...");
  
    MadgwickAdapter<Scalar, false> adapter(0.001);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, false>, 
                                false> problem(adapter);
   
    EntoAttitude::AttitudeMeasurement<Scalar, false> measurement(
        Vec3<Scalar>(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06),  // Gyroscope
        Vec3<Scalar>(-0.07215829, 0.03096613, 8.31740944)                 // Accelerometer
    );
    
    problem.measurement_ = measurement;
    problem.dt_ = 0.004;
    problem.q_gt_ = Eigen::Quaternion<Scalar>(1.0, 2.26892869e-07, -1.48352885e-07, 4.45058993e-07);
    problem.q_prev_ = problem.q_gt_;
    
    // Solve the problem
    problem.solve_impl();
    
    // Expected quaternion from previous test
    Eigen::Quaternion<Scalar> q_expected(1.00000000e+00, 1.49870419e-06, 3.40537366e-06, 4.36343901e-07);
    
    // Check results
    ENTO_DEBUG_EIGEN_QUATERNION(problem.q_);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(problem.q_.coeffs(), q_expected.coeffs());
    
    ENTO_DEBUG("test_madgwick_imu_problem PASSED!");
}

//------------------------------------------------------------------------------
// Test for Madgwick MARG Problem
//------------------------------------------------------------------------------
void test_madgwick_marg_problem()
{
    ENTO_DEBUG("Running test_madgwick_marg_problem...");
    
    MadgwickAdapter<Scalar, true> adapter(0.001);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, true>, 
                                true> problem(adapter);
    
    // Create a test measurement using the same data from your original test
    EntoAttitude::AttitudeMeasurement<Scalar, true> measurement(
        Vec3<Scalar>(1.31868102828397E-06, -2.71143525493424E-06, 3.17779011484456E-06),  // Gyroscope
        Vec3<Scalar>(1.13147340585962E-01, 1.5124603405802E-01, 8.4355704699742E+00),     // Accelerometer
        Vec3<Scalar>(-1.69710456595215E+04, -9.92469996952225E+02, 3.46472685717592E+04)  // Magnetometer
    );
    
    // Set up the problem
    problem.measurement_ = measurement;
    problem.dt_ = 0.04;
    problem.q_gt_ = Eigen::Quaternion<Scalar>(1.0, 0.0, 0.0, 0.0);
    problem.q_prev_ = problem.q_gt_;
    
    // Solve the problem
    problem.solve_impl();
    
    // Expected quaternion from previous test
    Eigen::Quaternion<Scalar> q_expected(1.00000000e+00, 1.49870419e-06, 3.40537366e-06, 4.36343901e-07);
    
    // Check results
    ENTO_DEBUG_EIGEN_QUATERNION(problem.q_);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(problem.q_.coeffs(), q_expected.coeffs());
    
    ENTO_DEBUG("test_madgwick_marg_problem PASSED!");
}



//------------------------------------------------------------------------------
// Test for Madgwick IMU Serialization + Deserialization
//------------------------------------------------------------------------------
void test_madgwick_imu_serialization()
{
    ENTO_DEBUG("Running test_madgwick_imu_serialization...");
    
    // Create the adapter with optimized parameters
    MadgwickAdapter<Scalar, false> adapter(0.001);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, false>, 
                                false> problem(adapter);
    
    // Create a sample line that matches the expected format for deserialize_impl
    // Format for IMU (no mag): ax ay az gx gy gz qw qx qy qz dt
    std::string input_line = "-0.07215829 0.03096613 8.31740944 -1.52713681e-04 -6.10919329e-05 -4.35697544e-06 1.0 2.26892869e-07 -1.48352885e-07 4.45058993e-07 0.004";
    
    // Deserialize the input line
    bool deserialize_success = problem.deserialize_impl(input_line.c_str());
    ENTO_TEST_CHECK_TRUE(deserialize_success);
    
    // Solve the problem to update q_
    problem.solve_impl();
    
    // Now serialize the result
    std::string serialized = problem.serialize_impl();
    ENTO_DEBUG("Serialized quaternion: %s", serialized.c_str());
    
    // Get the expected quaternion for comparison
    Eigen::Quaternion<Scalar> expected_q = problem.q_;
    
    // Create a new problem instance for testing deserialization of the quaternion
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, false>, 
                                false> new_problem(adapter);
    
    // For testing the quaternion deserialization, we need to:
    // 1. First set up the new problem with the same input data
    new_problem.deserialize_impl(input_line.c_str());
    
    // 2. Manually set the q_ value from our serialized output
    // Parse the serialized string (format is "w,x,y,z")
    std::istringstream iss(serialized);
    std::string w_str, x_str, y_str, z_str;
    
    // Parse using ',' as delimiter
    std::getline(iss, w_str, ',');
    std::getline(iss, x_str, ',');
    std::getline(iss, y_str, ',');
    std::getline(iss, z_str, ',');
    
    double w = std::stod(w_str);
    double x = std::stod(x_str);
    double y = std::stod(y_str);
    double z = std::stod(z_str);
    
    new_problem.q_ = Eigen::Quaternion<Scalar>(w, x, y, z);
    
    // Check that the result
    ENTO_DEBUG_EIGEN_QUATERNION(new_problem.q_);
    ENTO_DEBUG_EIGEN_QUATERNION(expected_q);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(new_problem.q_.coeffs(), expected_q.coeffs());
    
    ENTO_DEBUG("test_madgwick_imu_serialization PASSED!");
}

//------------------------------------------------------------------------------
// Test for Madgwick MARG Serialization + Deserialization
//------------------------------------------------------------------------------
void test_madgwick_marg_serialization()
{
    ENTO_DEBUG("Running test_madgwick_marg_serialization...");
    
    // Create the adapter with optimized parameters
    MadgwickAdapter<Scalar, true> adapter(0.001);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, true>, 
                                true> problem(adapter);
    
    // Create a sample line that matches the expected format for deserialize_impl
    // Format for MARG (with mag): ax ay az gx gy gz mx my mz qw qx qy qz dt
    std::string input_line = "1.13147340585962E-01 1.5124603405802E-01 8.4355704699742E+00 1.31868102828397E-06 -2.71143525493424E-06 3.17779011484456E-06 -1.69710456595215E+04 -9.92469996952225E+02 3.46472685717592E+04 1.0 0.0 0.0 0.0 0.04";
    
    // Deserialize the input line
    bool deserialize_success = problem.deserialize_impl(input_line.c_str());
    ENTO_TEST_CHECK_TRUE(deserialize_success);
    
    // Solve the problem to update q_
    problem.solve_impl();
    
    // Now serialize the result
    std::string serialized = problem.serialize_impl();
    ENTO_DEBUG("Serialized quaternion: %s", serialized.c_str());
    
    // Get the expected quaternion for comparison
    Eigen::Quaternion<Scalar> expected_q = problem.q_;
    
    // Create a new problem instance for testing deserialization of the quaternion
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, true>, 
                                true> new_problem(adapter);
    
    // For testing the quaternion deserialization, we need to:
    // 1. First set up the new problem with the same input data
    new_problem.deserialize_impl(input_line.c_str());
    
    // 2. Manually set the q_ value from our serialized output
    // Parse the serialized string (format is "w,x,y,z")
    std::istringstream iss(serialized);
    std::string w_str, x_str, y_str, z_str;
    
    // Parse using ',' as delimiter
    std::getline(iss, w_str, ',');
    std::getline(iss, x_str, ',');
    std::getline(iss, y_str, ',');
    std::getline(iss, z_str, ',');
    
    double w = std::stod(w_str);
    double x = std::stod(x_str);
    double y = std::stod(y_str);
    double z = std::stod(z_str);
    
    new_problem.q_ = Eigen::Quaternion<Scalar>(w, x, y, z);
    
    ENTO_DEBUG_EIGEN_QUATERNION(new_problem.q_);
    ENTO_DEBUG_EIGEN_QUATERNION(expected_q);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(new_problem.q_.coeffs(), expected_q.coeffs());
    
    ENTO_DEBUG("test_madgwick_marg_serialization PASSED!");
}

//------------------------------------------------------------------------------
// Test for Madgwick IMU Validate
//------------------------------------------------------------------------------
void test_madgwick_imu_validation()
{
    ENTO_DEBUG("Running test_madgwick_imu_validation...");
    
    // Create the adapter with optimized parameters
    MadgwickAdapter<Scalar, false> adapter(0.001);
    
    // Create the problem with this adapter - use a 5 degree threshold
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, false>, 
                                false> problem(adapter, 5.0);
    
    // Create a sample line that matches the expected format for deserialize_impl
    // Format for IMU (no mag): ax ay az gx gy gz qw qx qy qz dt
    std::string input_line = "-0.07215829 0.03096613 8.31740944 -1.52713681e-04 -6.10919329e-05 -4.35697544e-06 1.0 2.26892869e-07 -1.48352885e-07 4.45058993e-07 0.004";
    
    // Deserialize the input line
    bool deserialize_success = problem.deserialize_impl(input_line.c_str());
    ENTO_TEST_CHECK_TRUE(deserialize_success);
    
    // Solve the problem to update q_
    problem.solve_impl();
    
    // Test validate_impl (should use the default threshold of 5 degrees)
    bool validation_result = problem.validate_impl();
    ENTO_DEBUG("Default validation result (5 deg threshold): %s", validation_result ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(validation_result);
    
    // Test validate with a tight threshold (0.1 degrees)
    bool tight_validation = problem.validate(0.1);
    ENTO_DEBUG("Tight validation result (0.1 deg threshold): %s", tight_validation ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(tight_validation);
    
    // Calculate and display the actual quaternion angle distance for debugging
    Scalar angle_distance = problem.computeQuaternionAngleDistance(problem.q_, problem.q_gt_);
    ENTO_DEBUG("Quaternion angle distance (degrees): %f", angle_distance);
    
    // Test with a very tight threshold that should fail
    // Only do this if the angle distance is non-zero
    if (angle_distance > 1e-10) {
        bool super_tight_validation = problem.validate(angle_distance * 0.5);
        ENTO_DEBUG("Super tight validation result (%f deg threshold): %s", 
                  angle_distance * 0.5, super_tight_validation ? "PASSED" : "FAILED");
        ENTO_TEST_CHECK_FALSE(super_tight_validation);
    }
    
    ENTO_DEBUG("test_madgwick_imu_validation PASSED!");
}

//------------------------------------------------------------------------------
// Test for Madgwick MARG Validate
//------------------------------------------------------------------------------
void test_madgwick_marg_validation()
{
    ENTO_DEBUG("Running test_madgwick_marg_validation...");
    
    // Create the adapter with optimized parameters
    MadgwickAdapter<Scalar, true> adapter(0.001);
    
    // Create the problem with this adapter - use a more lenient threshold for MARG
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, true>, 
                                true> problem(adapter, 10.0);  // 10 degree threshold
    
    // Create a sample line that matches the expected format for deserialize_impl
    // Format for MARG (with mag): ax ay az gx gy gz mx my mz qw qx qy qz dt
    std::string input_line = "1.13147340585962E-01 1.5124603405802E-01 8.4355704699742E+00 1.31868102828397E-06 -2.71143525493424E-06 3.17779011484456E-06 -1.69710456595215E+04 -9.92469996952225E+02 3.46472685717592E+04 1.0 0.0 0.0 0.0 0.04";
    
    // Deserialize the input line
    bool deserialize_success = problem.deserialize_impl(input_line.c_str());
    ENTO_TEST_CHECK_TRUE(deserialize_success);
    
    // Make sure q_prev_ is set properly
    problem.q_prev_ = problem.q_gt_;
    
    // Calculate angle before solving
    Scalar initial_angle = problem.computeQuaternionAngleDistance(problem.q_prev_, problem.q_gt_);
    ENTO_DEBUG("Initial angle before solving (should be 0): %f", initial_angle);
    
    // Solve the problem to update q_
    problem.solve_impl();
    
    // Get the resulting quaternion and print it for debugging
    ENTO_DEBUG("Resulting quaternion after solve_impl():");
    ENTO_DEBUG_EIGEN_QUATERNION(problem.q_);
    
    // Calculate angle distance after solving
    Scalar angle_distance = problem.computeQuaternionAngleDistance(problem.q_, problem.q_gt_);
    ENTO_DEBUG("Quaternion angle distance after solving (degrees): %f", angle_distance);
    
    // Test validate_impl (should use the constructor threshold of 10 degrees)
    bool validation_result = problem.validate_impl();
    ENTO_DEBUG("Default validation result (10 deg threshold): %s", validation_result ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(validation_result);
    
    // Test with a threshold based on the expected error
    Scalar expected_error_deg = 0.5;  // 0.5 degrees should be enough based on test data
    bool adjusted_validation = problem.validate(expected_error_deg);
    ENTO_DEBUG("Adjusted validation result (%f deg threshold): %s", 
              expected_error_deg, adjusted_validation ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(adjusted_validation);
    
    // Check the current result against the expected quaternion with a looser tolerance
    // From the testing data, we know the madgwick error is very small (3.999928000346814e-06)
    Eigen::Quaternion<Scalar> expected_q(1.00000000e+00, 1.49870419e-06, 3.40537366e-06, 4.36343901e-07);
    
    // Check with appropriate tolerance
    const float custom_tol = 0.0005f;  
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ_TOL(problem.q_.coeffs(), expected_q.coeffs(), custom_tol);
    
    ENTO_DEBUG("test_madgwick_marg_validation PASSED!");
}


//------------------------------------------------------------------------------
// Main Test Runner
//------------------------------------------------------------------------------
int main( int argc, char** argv )
{
  using namespace EntoUtil;
  //int n;
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

  if (__ento_test_num(__n, 1)) test_madgwick_imu_update();
  if (__ento_test_num(__n, 2)) test_madgwick_marg_update();

  if (__ento_test_num(__n, 3)) test_madgwick_imu_problem();
  if (__ento_test_num(__n, 4)) test_madgwick_marg_problem();


  if (__ento_test_num(__n, 5)) test_madgwick_imu_serialization();
  if (__ento_test_num(__n, 6)) test_madgwick_marg_serialization();

  if (__ento_test_num(__n, 7)) test_madgwick_imu_validation();
  if (__ento_test_num(__n, 8)) test_madgwick_marg_validation();

  ENTO_TEST_END();
  //return 0;
}
