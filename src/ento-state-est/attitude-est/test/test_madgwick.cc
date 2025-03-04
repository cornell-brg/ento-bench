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
  Eigen::Quaternion<Scalar> q_init(9.99999999999864E-01, 2.26892868785046E-07, -1.48352885438851E-07, 4.45058992918769E-07);
  
  // Sample IMU sensor readings
  EntoMath::Vec3<Scalar> gyr(1.31868102828397E-06,-2.71143525493424E-06,3.17779011484456E-06);  // Gyroscope (rad/s)
  EntoMath::Vec3<Scalar> acc(1.13147340585962E-01,1.5124603405802E-01,8.4355704699742E+00);   // Accelerometer (gravity vector)

  Scalar dt = 0.04;  // Time step (s)
  Scalar gain = 0.001; // Madgwick filter gain

  // Run the Madgwick IMU update
  Eigen::Quaternion<Scalar> q_updated = EntoAttitude::madgwick_update_imu(q_init, gyr, acc, dt, gain);

  // Hardcoded expected quaternion (replace with actual values from Python)
  Eigen::Quaternion<Scalar> q_expected(9.99999999937557E-01,5.51343221353965E-07,-1.85120342788294E-06,1.10070276640774E-05); // Placeholder

  ENTO_DEBUG("Expected Quaternion: (%f, %f, %f, %f)", q_expected.w(), q_expected.x(), q_expected.y(), q_expected.z());
  ENTO_DEBUG("Computed Quaternion: (%f, %f, %f, %f)", q_updated.w(), q_updated.x(), q_updated.y(), q_updated.z());

  // Validate each quaternion component
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.w(), q_expected.w());
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.x(), q_expected.x());
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.y(), q_expected.y());
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.z(), q_expected.z());

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
  Eigen::Quaternion<Scalar> q_expected(1.01139306110868E-05,-2.91020355759363E-06,2.43083422689691E-06,9.99999999941665E-01); // Placeholder

  ENTO_DEBUG("Expected Quaternion: (%f, %f, %f, %f)", q_expected.w(), q_expected.x(), q_expected.y(), q_expected.z());
  ENTO_DEBUG("Computed Quaternion: (%f, %f, %f, %f)", q_updated.w(), q_updated.x(), q_updated.y(), q_updated.z());

  // Validate each quaternion component
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.w(), q_expected.w());
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.x(), q_expected.x());
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.y(), q_expected.y());
  ENTO_TEST_CHECK_FLOAT_EQ(q_updated.z(), q_expected.z());

  ENTO_DEBUG("test_madgwick_marg_update PASSED!");
}

//------------------------------------------------------------------------------
// Main Test Runner
//------------------------------------------------------------------------------
int main( int argc, char** argv )
{
  using namespace EntoUtil;
  //int __n;
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

  if (__ento_test_num(__n, 1)) test_madgwick_imu_update();
  if (__ento_test_num(__n, 2)) test_madgwick_marg_update();
  ENTO_DEBUG("All tests completed successfully!");
  return 0;
}
