// #include <ento-util/debug.h>
// #include <ento-util/unittest.h>
// #include <ento-state-est/attitude-est/fourati_nonlinear.h>

// using Scalar = double;  // Define scalar type (float or double)
// using namespace EntoMath;

// constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

// //------------------------------------------------------------------------------
// // Test for fourati IMU Update
// //------------------------------------------------------------------------------
// void test_fourati_update()
// {
//   ENTO_DEBUG("Running test_fourati_update...");

//   // Initial quaternion
//   Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
//   // Sample IMU sensor readings (from RoboBee dataset)
//   Vec3<Scalar> gyr(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06);  // Gyroscope (rad/s)
//   Vec3<Scalar> acc(-0.07215829, 0.03096613, 8.31740944);                // Accelerometer (gravity vector)
//   Vec3<Scalar> mag(16925.5042314,1207.22593348,34498.24159392);

  
//   const Scalar gain = 0.05;
//   Scalar dt  = 0.004f;  // Time step (s)
//   const Eigen::Quaternion<Scalar> g_ref = {0., 0., 0., 1.}; // Reference gravity
//   const Eigen::Quaternion<Scalar> m_ref = {0.,0.44275986,0.02522524,0.89628533}; // Reference magnetic field

//   // Print Out Matrices
//   ENTO_DEBUG_EIGEN_MATRIX(q_init.coeffs());
//   ENTO_DEBUG_EIGEN_MATRIX(gyr);
//   ENTO_DEBUG_EIGEN_MATRIX(acc);

//   // Run the fourati IMU update
//   Eigen::Quaternion<Scalar> q_updated = EntoAttitude::fourati_update(q_init, gyr, acc, mag, dt, gain, g_ref, m_ref);

//   // Hardcoded expected quaternion (from Python ahrs run on RoboBee dataset) 
//   Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,-7.85120035e-08,-2.70590288e-07,4.36307438e-07); // Placeholder

//   // Validate each quaternion component
//   ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
//   ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
//   ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

//   ENTO_DEBUG("test_fourati_update PASSED!");
// }


// //------------------------------------------------------------------------------
// // Main Test Runner
// //------------------------------------------------------------------------------
// int main( int argc, char** argv )
// {
//   using namespace EntoUtil;
//   //int n;
//   if (argc > 1)
//   {
//     __n = atoi(argv[1]);
//   }
//   else
//   {
//     // For the case we are running on the MCU and we can't pass in args
//     // the same way args are passed for a native build.
//     __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
//     __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
//   }

//   ENTO_DEBUG("N: %i", __n);
//   ENTO_TEST_START();

//   if (__ento_test_num(__n, 1)) test_fourati_update();

//   ENTO_TEST_END();
//   //return 0;
// }



#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/fourati_nonlinear.h>

using Scalar = double;  // Define scalar type (float or double)
using namespace EntoMath;

constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

// Adapter class to make Fourati filter work with AttitudeProblem
template <typename Scalar, bool UseMag>
class FouratiAdapter
{
private:
    // Fourati filter parameters
    Scalar gain_;
    Eigen::Quaternion<Scalar> g_ref_;
    Eigen::Quaternion<Scalar> m_ref_;

public:
    FouratiAdapter(Scalar gain = static_cast<Scalar>(0.05))
        : gain_(gain), 
          g_ref_(0., 0., 0., 1.),  // Reference gravity quaternion
          m_ref_(0., 0.44275986, 0.02522524, 0.89628533)  // Reference magnetic field quaternion
    {}
    
    // Operator that matches the interface expected by AttitudeProblem
    void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                   const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& measurement,
                   Scalar dt,
                   Eigen::Quaternion<Scalar>* q_out)
    {
        if constexpr (UseMag) {
            // For MARG (with magnetometer)
            *q_out = EntoAttitude::fourati_update(
                q_prev,
                measurement.gyr,  // Gyroscope vector
                measurement.acc,  // Accelerometer vector
                measurement.mag,  // Magnetometer vector
                dt,
                gain_,
                g_ref_,
                m_ref_
            );
        } else {
            // For IMU (no magnetometer) - if needed
            // Currently, Fourati always uses magnetometer, but we could add
            // an IMU-only version if needed
            *q_out = EntoAttitude::fourati_update(
                q_prev,
                measurement.gyr,
                measurement.acc,
                measurement.mag,  // Will still use mag data
                dt,
                gain_,
                g_ref_,
                m_ref_
            );
        }
    }
    
    // Name method for identification
    static constexpr const char* name()
    {
        if constexpr (UseMag) {
            return "Fourati MARG Filter";
        } else {
            return "Fourati IMU Filter";
        }
    }
};

//------------------------------------------------------------------------------
// Test for Fourati MARG Update (existing test, renamed for consistency)
//------------------------------------------------------------------------------
void test_fourati_marg_update()
{
  ENTO_DEBUG("Running test_fourati_marg_update...");

  // Initial quaternion
  Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
  // Sample IMU sensor readings (from RoboBee dataset)
  Vec3<Scalar> gyr(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06);  // Gyroscope (rad/s)
  Vec3<Scalar> acc(-0.07215829, 0.03096613, 8.31740944);                // Accelerometer (gravity vector)
  Vec3<Scalar> mag(16925.5042314, 1207.22593348, 34498.24159392);       // Magnetometer

  const Scalar gain = 0.05;
  Scalar dt = 0.004f;  // Time step (s)
  const Eigen::Quaternion<Scalar> g_ref = {0., 0., 0., 1.}; // Reference gravity
  const Eigen::Quaternion<Scalar> m_ref = {0., 0.44275986, 0.02522524, 0.89628533}; // Reference magnetic field

  // Print Out Matrices
  ENTO_DEBUG_EIGEN_MATRIX(q_init.coeffs());
  ENTO_DEBUG_EIGEN_MATRIX(gyr);
  ENTO_DEBUG_EIGEN_MATRIX(acc);
  ENTO_DEBUG_EIGEN_MATRIX(mag);

  // Run the fourati update
  Eigen::Quaternion<Scalar> q_updated = EntoAttitude::fourati_update(
      q_init, gyr, acc, mag, dt, gain, g_ref, m_ref);

  // Hardcoded expected quaternion from Python ahrs run on RoboBee dataset
  Eigen::Quaternion<Scalar> q_expected(1.00000000e+00, -7.85120035e-08, -2.70590288e-07, 4.36307438e-07);

  // Validate each quaternion component
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
  ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
  ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

  ENTO_DEBUG("test_fourati_marg_update PASSED!");
}

//------------------------------------------------------------------------------
// Test for Fourati MARG Problem
//------------------------------------------------------------------------------
void test_fourati_marg_problem()
{
    ENTO_DEBUG("Running test_fourati_marg_problem...");
    
    // Create the adapter with optimized parameters
    FouratiAdapter<Scalar, true> adapter(0.05);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                FouratiAdapter<Scalar, true>, 
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
    Eigen::Quaternion<Scalar> q_expected(1.00000000e+00, -7.85120035e-08, -2.70590288e-07, 4.36307438e-07);
    
    // Check results
    ENTO_DEBUG_EIGEN_QUATERNION(problem.q_);
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(problem.q_.coeffs(), q_expected.coeffs());
    
    ENTO_DEBUG("test_fourati_marg_problem PASSED!");
}

//------------------------------------------------------------------------------
// Test for Fourati MARG Serialization + Deserialization
//------------------------------------------------------------------------------
void test_fourati_marg_serialization()
{
    ENTO_DEBUG("Running test_fourati_marg_serialization...");
    
    // Create the adapter with optimized parameters
    FouratiAdapter<Scalar, true> adapter(0.05);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                FouratiAdapter<Scalar, true>, 
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
                                FouratiAdapter<Scalar, true>, 
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
    
    ENTO_DEBUG("test_fourati_marg_serialization PASSED!");
}

//------------------------------------------------------------------------------
// Test for Fourati MARG Validate
//------------------------------------------------------------------------------
void test_fourati_marg_validation()
{
    ENTO_DEBUG("Running test_fourati_marg_validation...");
    
    // Create the adapter with optimized parameters
    FouratiAdapter<Scalar, true> adapter(0.05);
    
    // Create the problem with this adapter - use a suitable threshold
    EntoAttitude::AttitudeProblem<Scalar, 
                                FouratiAdapter<Scalar, true>, 
                                true> problem(adapter, 5.0);  // 5 degree threshold
    
    // Create a sample line that matches the expected format for deserialize_impl
    // Format for MARG (with mag): ax ay az gx gy gz mx my mz qw qx qy qz dt
    std::string input_line = "-0.07215829 0.03096613 8.31740944 -1.52713681e-04 -6.10919329e-05 -4.35697544e-06 16925.5042314 1207.22593348 34498.24159392 1.0 2.26892869e-07 -1.48352885e-07 4.45058993e-07 0.004";
    
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
    
    // Test validate_impl (should use the constructor threshold of 5 degrees)
    bool validation_result = problem.validate_impl();
    ENTO_DEBUG("Default validation result (5 deg threshold): %s", validation_result ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(validation_result);
    
    // Test with a tighter threshold that is still valid 
    // From the test data the Fourati error is very small so this should pass
    Scalar tight_threshold_deg = 0.1;  // 0.1 degrees
    bool tight_validation = problem.validate(tight_threshold_deg);
    ENTO_DEBUG("Tight validation result (%f deg threshold): %s", 
              tight_threshold_deg, tight_validation ? "PASSED" : "FAILED");
    ENTO_TEST_CHECK_TRUE(tight_validation);
    
    // Expected quaternion from previous test
    Eigen::Quaternion<Scalar> expected_q(1.00000000e+00, -7.85120035e-08, -2.70590288e-07, 4.36307438e-07);
    
    // Check with appropriate tolerance
    const float custom_tol = 0.0001f;  
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ_TOL(problem.q_.coeffs(), expected_q.coeffs(), custom_tol);
    
    ENTO_DEBUG("test_fourati_marg_validation PASSED!");
}

//------------------------------------------------------------------------------
// Main Test Runner
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  using namespace EntoUtil;
  //int n;
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

  ENTO_DEBUG("N: %i", __n);
  ENTO_TEST_START();

  if (__ento_test_num(__n, 1)) test_fourati_marg_update();
  if (__ento_test_num(__n, 2)) test_fourati_marg_problem();
  if (__ento_test_num(__n, 3)) test_fourati_marg_serialization();
  if (__ento_test_num(__n, 4)) test_fourati_marg_validation();

  ENTO_TEST_END();
  //return 0;
}


