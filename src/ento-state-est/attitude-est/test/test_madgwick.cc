// #include <ento-util/debug.h>
// #include <ento-util/unittest.h>
// #include <ento-state-est/attitude-est/madgwick.h>

// using Scalar = double;  // Define scalar type (float or double)

// constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

// //------------------------------------------------------------------------------
// // Test for Madgwick IMU Update
// //------------------------------------------------------------------------------
// void test_madgwick_imu_update()
// {
//   ENTO_DEBUG("Running test_madgwick_imu_update...");

//   // Initial quaternion (identity)
//   Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
//   // Sample IMU sensor readings (from RoboBee dataset)
//   EntoMath::Vec3<Scalar> gyr(-1.52713681e-04,-6.10919329e-05,-4.35697544e-06);  // Gyroscope (rad/s)
//   EntoMath::Vec3<Scalar> acc(-0.07215829,0.03096613,8.31740944);   // Accelerometer (gravity vector)

//   Scalar dt = 0.004;  // Time step (s)
//   Scalar gain = 0.001; // Madgwick filter gain

//   // Print Out Matrices
//   ENTO_DEBUG_EIGEN_MATRIX(q_init.coeffs());
//   ENTO_DEBUG_EIGEN_MATRIX(gyr);
//   ENTO_DEBUG_EIGEN_MATRIX(acc);

//   // Run the Madgwick IMU update
//   Eigen::Quaternion<Scalar> q_updated = EntoAttitude::madgwick_update_imu(q_init, gyr, acc, dt, gain);

//   // Hardcoded expected quaternion (replace with actual values from Python)
//   Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,1.49870419e-06,3.40537366e-06,4.36343901e-07); // Placeholder

//   // Validate each quaternion component
//   ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
//   ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
//   ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

//   ENTO_DEBUG("test_madgwick_imu_update PASSED!");
// }

// //------------------------------------------------------------------------------
// // Test for Madgwick MARG Update
// //------------------------------------------------------------------------------
// void test_madgwick_marg_update()
// {
//   ENTO_DEBUG("Running test_madgwick_marg_update...");

//   // Initial quaternion (identity)
//   Eigen::Quaternion<Scalar> q_init(1.0, 0.0, 0.0, 0.0);

//   // Sample IMU sensor readings
//   EntoMath::Vec3<Scalar> gyr(1.31868102828397E-06,-2.71143525493424E-06,3.17779011484456E-06);  // Gyroscope (rad/s)
//   EntoMath::Vec3<Scalar> acc(1.13147340585962E-01,1.5124603405802E-01,8.4355704699742E+00);   // Accelerometer (gravity vector)
//   EntoMath::Vec3<Scalar> mag(-1.69710456595215E+04,-9.92469996952225E+02,3.46472685717592E+04);

//   Scalar dt = 0.04f;  // Time step (s)
//   Scalar gain = 0.001f; // Madgwick filter gain

//   // Run the Madgwick MARG update
//   Eigen::Quaternion<Scalar> q_updated = EntoAttitude::madgwick_update_marg(q_init, gyr, acc, mag, dt, gain);

//   // Hardcoded expected quaternion (replace with actual values from Python)
//   Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,1.49870419e-06,3.40537366e-06,4.36343901e-07); // Placeholder

//   //ENTO_DEBUG("Expected Quaternion: (%f, %f, %f, %f)", q_expected.w(), q_expected.x(), q_expected.y(), q_expected.z());
//   //ENTO_DEBUG("Computed Quaternion: (%f, %f, %f, %f)", q_updated.w(), q_updated.x(), q_updated.y(), q_updated.z());

//   // Validate each quaternion component
//   ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());

//   ENTO_DEBUG("test_madgwick_marg_update PASSED!");
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

//   if (__ento_test_num(__n, 1)) test_madgwick_imu_update();
//   if (__ento_test_num(__n, 2)) test_madgwick_marg_update();

//   ENTO_TEST_END();
//   //return 0;
// }


#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/madgwick.h>

using Scalar = double;
using namespace EntoMath;

// Adapter class to make Madgwick filter work with AttitudeProblem
template <typename Scalar, bool UseMag>
class MadgwickAdapter
{
private:
    // Madgwick filter parameter
    Scalar gain_;

public:
    // Constructor with optimized gain parameter from your data generation
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

// Test function for IMU (no magnetometer) case
void test_madgwick_imu_problem()
{
    ENTO_DEBUG("Running test_madgwick_imu_problem...");
    
    // Create the adapter with optimized gain parameter from your script
    MadgwickAdapter<Scalar, false> adapter(0.001);
    
    // Create the problem with this adapter
    EntoAttitude::AttitudeProblem<Scalar, 
                                MadgwickAdapter<Scalar, false>, 
                                false> problem(adapter);
    
    // Create a test measurement using the same data from your original test
    EntoAttitude::AttitudeMeasurement<Scalar, false> measurement(
        Vec3<Scalar>(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06),  // Gyroscope
        Vec3<Scalar>(-0.07215829, 0.03096613, 8.31740944)                 // Accelerometer
    );
    
    // Set up the problem with initial quaternion from your data
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

// Test function for MARG (with magnetometer) case
void test_madgwick_marg_problem()
{
    ENTO_DEBUG("Running test_madgwick_marg_problem...");
    
    // Create the adapter with optimized gain parameter from your script
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
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_DEBUG("N: %i", __n);
  ENTO_TEST_START();

  // if (__ento_test_num(__n, 1)) test_madgwick_imu_update();
  // if (__ento_test_num(__n, 2)) test_madgwick_marg_update();

  if (__ento_test_num(__n, 1)) test_madgwick_imu_problem();
  if (__ento_test_num(__n, 2)) test_madgwick_marg_problem();

  ENTO_TEST_END();
  //return 0;
}
