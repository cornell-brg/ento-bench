#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/mahoney.h>


using Scalar = double;
using namespace EntoMath;


template <typename Scalar, bool UseMag>
class MahoneyAdapter
{
private:
    // Additional parameters needed by Mahoney functions
    Scalar k_p_;
    Scalar k_i_;
    EntoMath::Vec3<Scalar> bias_;

public:
    MahoneyAdapter(Scalar k_p = static_cast<Scalar>(0.1), 
                  Scalar k_i = static_cast<Scalar>(0.01))
        : k_p_(k_p), k_i_(k_i), bias_(EntoMath::Vec3<Scalar>::Zero()) {}
    
    // Implement the interface required by AttitudeProblem
    void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                   const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& measurement,
                   Scalar dt,
                   Eigen::Quaternion<Scalar>* q_out)
    {
        if constexpr (UseMag) {
            // For MARG (with magnetometer)
            *q_out = EntoAttitude::mahoney_update_marg(
                q_prev,
                measurement.gyr,  // Gyroscope vector
                measurement.acc,  // Accelerometer vector
                measurement.mag,  // Magnetometer vector
                dt,
                k_p_,
                k_i_,
                bias_
            );
        } else {
            // For IMU (no magnetometer)
            *q_out = EntoAttitude::mahoney_update_imu(
                q_prev,
                measurement.gyr,  // Gyroscope vector
                measurement.acc,  // Accelerometer vector
                dt,
                k_p_,
                k_i_,
                bias_
            );
        }
    }
    
    // Name method for identification
    static constexpr const char* name()
    {
        if constexpr (UseMag) {
            return "Mahoney MARG Filter";
        } else {
            return "Mahoney IMU Filter";
        }
    }
};


constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

//------------------------------------------------------------------------------
// Test for mahoney IMU Update
//------------------------------------------------------------------------------
void test_mahoney_imu_update()
{
  ENTO_DEBUG("Running test_mahoney_imu_update...");

  // Initial quaternion
  Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
  // Sample IMU sensor readings (from RoboBee dataset)
  EntoMath::Vec3<Scalar> gyr(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06);  // Gyroscope (rad/s)
  EntoMath::Vec3<Scalar> acc(-0.07215829, 0.03096613, 8.31740944);                // Accelerometer (gravity vector)

  Scalar k_i = 0.01;       // Integral term
  Scalar k_p = 0.1;       // Proportional term
  Scalar dt  = 0.004f;  // Time step (s)
  Vec3<Scalar> bias = { 0., 0., 0. }; // IMU Bias

  // Print Out Matrices
  ENTO_DEBUG_EIGEN_MATRIX(q_init.coeffs());
  ENTO_DEBUG_EIGEN_MATRIX(gyr);
  ENTO_DEBUG_EIGEN_MATRIX(acc);

  // Run the mahoney IMU update
  Eigen::Quaternion<Scalar> q_updated = EntoAttitude::mahoney_update_imu(q_init, gyr, acc, dt, k_p, k_i, bias);

  // Hardcoded expected quaternion (from Python ahrs run on RoboBee dataset) 
  Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,6.66248740e-07,1.46525393e-06,4.36344465e-07); // Placeholder

  // Validate each quaternion component
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
  ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
  ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

  ENTO_DEBUG("test_mahoney_imu_update PASSED!");
}

//------------------------------------------------------------------------------
// Test for mahoney MARG Update
//------------------------------------------------------------------------------
void test_mahoney_marg_update()
{
  ENTO_DEBUG("Running test_mahoney_marg_update...");

  // Initial quaternion
  Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);

  // Sample IMU sensor readings
  Vec3<Scalar> gyr(-1.52713681e-04,-6.10919329e-05,-4.35697544e-06);  // Gyroscope (rad/s)
  Vec3<Scalar> acc(-0.07215829,0.03096613,8.31740944);   // Accelerometer (gravity vector)
  Vec3<Scalar> mag(16925.5042314,1207.22593348,34498.24159392);


  Scalar k_i = 0.01;       // Integral term
  Scalar k_p = 0.1;       // Proportional term
  Scalar dt  = 0.004f;  // Time step (s)
  Vec3<Scalar> bias = { 0., 0., 0. }; // IMU Bias

  // Run the mahoney MARG update
  Eigen::Quaternion<Scalar> q_updated = EntoAttitude::mahoney_update_marg(q_init, gyr, acc, mag, dt, k_p, k_i, bias);

  // Hardcoded expected quaternion (from Python ahrs run on RoboBee dataset) 
  Eigen::Quaternion<Scalar> q_expected(9.99999994e-01,-7.29375589e-05,-7.75753231e-05,3.93137128e-05); // Placeholder

  // Validate each quaternion component
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
  ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
  ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

  ENTO_DEBUG("test_mahoney_marg_update PASSED!");
}


//------------------------------------------------------------------------------
// Test for mahoney IMU Problem
//------------------------------------------------------------------------------
void test_mahoney_imu_problem()
{
    ENTO_DEBUG("Running test_mahoney_imu_problem...");
    
    // Create the adapter with optimized parameters
    MahoneyAdapter<Scalar, false> adapter(0.1, 0.01);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MahoneyAdapter<Scalar, false>, 
                                false> problem(adapter);
    
    // Create a test measurement
    EntoAttitude::AttitudeMeasurement<Scalar, false> measurement(
        Vec3<Scalar>(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06),  // Gyroscope
        Vec3<Scalar>(-0.07215829, 0.03096613, 8.31740944)                 // Accelerometer
    );
    
    // Set up the problem
    problem.measurement_ = measurement;
    problem.dt_ = 0.004;
    problem.q_gt_ = Eigen::Quaternion<Scalar>(1.0, 2.26892869e-07, -1.48352885e-07, 4.45058993e-07);
    problem.q_prev_ = problem.q_gt_;
    
    // Solve the problem
    problem.solve_impl();
    
    // Expected quaternion from previous test
    Eigen::Quaternion<Scalar> q_expected(1.00000000e+00, 6.66248740e-07, 1.46525393e-06, 4.36344465e-07);
    
    // Check results
    ENTO_DEBUG_EIGEN_QUATERNION(problem.q_);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(problem.q_.coeffs(), q_expected.coeffs());
    
    ENTO_DEBUG("test_mahoney_imu_problem PASSED!");
}

//------------------------------------------------------------------------------
// Test for mahoney MARG Problem
//------------------------------------------------------------------------------
void test_mahoney_marg_problem()
{
    ENTO_DEBUG("Running test_mahoney_marg_problem...");
    
    // Create the adapter with optimized parameters
    MahoneyAdapter<Scalar, true> adapter(0.1, 0.01);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MahoneyAdapter<Scalar, true>, 
                                true> problem(adapter);
    
    // Create a test measurement
    EntoAttitude::AttitudeMeasurement<Scalar, true> measurement(
        Vec3<Scalar>(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06),  // Gyroscope
        Vec3<Scalar>(-0.07215829, 0.03096613, 8.31740944),                // Accelerometer
        Vec3<Scalar>(16925.5042314, 1207.22593348, 34498.24159392)        // Magnetometer
    );
    
    // Set up the problem
    problem.measurement_ = measurement;
    problem.dt_ = 0.004;
    problem.q_gt_ = Eigen::Quaternion<Scalar>(1.0, 2.26892869e-07, -1.48352885e-07, 4.45058993e-07);
    problem.q_prev_ = problem.q_gt_;
    
    // Solve the problem
    problem.solve_impl();
    
    // Expected quaternion from previous test
    Eigen::Quaternion<Scalar> q_expected(9.99999994e-01, -7.29375589e-05, -7.75753231e-05, 3.93137128e-05);
    
    // Check results
    ENTO_DEBUG_EIGEN_QUATERNION(problem.q_);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(problem.q_.coeffs(), q_expected.coeffs());
    
    ENTO_DEBUG("test_mahoney_marg_problem PASSED!");
}


// Test serialization and deserialization for Mahoney IMU problem
void test_mahoney_imu_serialization()
{
    ENTO_DEBUG("Running test_mahoney_imu_serialization...");
    
    // Create the adapter with optimized parameters
    MahoneyAdapter<Scalar, false> adapter(0.1, 0.01);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MahoneyAdapter<Scalar, false>, 
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
                                MahoneyAdapter<Scalar, false>, 
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
    
    // Check that the result matches what we expect
    ENTO_DEBUG_EIGEN_QUATERNION(new_problem.q_);
    ENTO_DEBUG_EIGEN_QUATERNION(expected_q);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(new_problem.q_.coeffs(), expected_q.coeffs());
    
    ENTO_DEBUG("test_mahoney_imu_serialization PASSED!");
}

// Test serialization and deserialization for Mahoney MARG problem
void test_mahoney_marg_serialization()
{
    ENTO_DEBUG("Running test_mahoney_marg_serialization...");
    
    // Create the adapter with optimized parameters
    MahoneyAdapter<Scalar, true> adapter(0.1, 0.01);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MahoneyAdapter<Scalar, true>, 
                                true> problem(adapter);
    
    // Create a sample line that matches the expected format for deserialize_impl
    // Format for MARG (with mag): ax ay az gx gy gz mx my mz qw qx qy qz dt
    std::string input_line = "-0.07215829 0.03096613 8.31740944 -1.52713681e-04 -6.10919329e-05 -4.35697544e-06 16925.5042314 1207.22593348 34498.24159392 1.0 2.26892869e-07 -1.48352885e-07 4.45058993e-07 0.004";
    
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
                                MahoneyAdapter<Scalar, true>, 
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
    
    // Check that the result matches what we expect
    ENTO_DEBUG_EIGEN_QUATERNION(new_problem.q_);
    ENTO_DEBUG_EIGEN_QUATERNION(expected_q);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(new_problem.q_.coeffs(), expected_q.coeffs());
    
    ENTO_DEBUG("test_mahoney_marg_serialization PASSED!");
}


// Test validation for Mahoney IMU problem
void test_mahoney_imu_validation()
{
    ENTO_DEBUG("Running test_mahoney_imu_validation...");
    
    // Create the adapter with optimized parameters
    MahoneyAdapter<Scalar, false> adapter(0.1, 0.01);
    
    // Create the problem with this adapter - use a 5 degree threshold
    EntoAttitude::AttitudeProblem<Scalar, 
                                MahoneyAdapter<Scalar, false>, 
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
    // This should also pass because the filter should be accurate with these test values
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
    
    ENTO_DEBUG("test_mahoney_imu_validation PASSED!");
}

// Test validation for Mahoney MARG problem
void test_mahoney_marg_validation()
{
    ENTO_DEBUG("Running test_mahoney_marg_validation...");
    
    // Create the adapter with optimized parameters
    MahoneyAdapter<Scalar, true> adapter(0.1, 0.01);
    
    // Create the problem with this adapter - use a 5 degree threshold
    EntoAttitude::AttitudeProblem<Scalar, 
                                MahoneyAdapter<Scalar, true>, 
                                true> problem(adapter, 5.0);
    
    // Create a sample line that matches the expected format for deserialize_impl
    // Format for MARG (with mag): ax ay az gx gy gz mx my mz qw qx qy qz dt
    std::string input_line = "-0.07215829 0.03096613 8.31740944 -1.52713681e-04 -6.10919329e-05 -4.35697544e-06 16925.5042314 1207.22593348 34498.24159392 1.0 2.26892869e-07 -1.48352885e-07 4.45058993e-07 0.004";
    
    // Deserialize the input line
    bool deserialize_success = problem.deserialize_impl(input_line.c_str());
    ENTO_TEST_CHECK_TRUE(deserialize_success);
    
    // Solve the problem to update q_
    problem.solve_impl();
    
    // Test validate_impl (should use the default threshold of 5 degrees)
    bool validation_result = problem.validate_impl();
    ENTO_DEBUG("Default validation result (5 deg threshold): %s", validation_result ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(validation_result);
    
    // Test validate with a tighter threshold (1 degree)
    bool tight_validation = problem.validate(1.0);
    ENTO_DEBUG("Tight validation result (1 deg threshold): %s", tight_validation ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(tight_validation);
    
    // Calculate and display the actual quaternion angle distance for debugging
    Scalar angle_distance = problem.computeQuaternionAngleDistance(problem.q_, problem.q_gt_);
    ENTO_DEBUG("Quaternion angle distance (degrees): %f", angle_distance);
    
    // // Test with a very tight threshold that should fail
    // // Only do this if the angle distance is non-zero
    // if (angle_distance > 1e-10) {
    //     bool super_tight_validation = problem.validate(angle_distance * 0.5);
    //     ENTO_DEBUG("Super tight validation result (%f deg threshold): %s", 
    //               angle_distance * 0.5, super_tight_validation ? "PASSED" : "FAILED");
    //     ENTO_TEST_CHECK_FALSE(super_tight_validation);
    // }
    
    ENTO_DEBUG("test_mahoney_marg_validation PASSED!");
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

  if (__ento_test_num(__n, 1)) test_mahoney_imu_update();
  if (__ento_test_num(__n, 2)) test_mahoney_marg_update();

  if (__ento_test_num(__n, 3)) test_mahoney_imu_problem();
  if (__ento_test_num(__n, 4)) test_mahoney_marg_problem();

  if (__ento_test_num(__n, 5)) test_mahoney_imu_serialization();
  if (__ento_test_num(__n, 6)) test_mahoney_marg_serialization();

  if (__ento_test_num(__n, 7)) test_mahoney_imu_validation();
  if (__ento_test_num(__n, 8)) test_mahoney_marg_validation();

  ENTO_TEST_END();
  //return 0;
}