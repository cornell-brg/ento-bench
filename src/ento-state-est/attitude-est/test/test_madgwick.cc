#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/madgwick.h>

using Scalar = double;  // Define scalar type (float or double)

constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

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

  if (__ento_test_num(__n, 1)) test_madgwick_imu_update();
  if (__ento_test_num(__n, 2)) test_madgwick_marg_update();

  ENTO_TEST_END();
  //return 0;
}
