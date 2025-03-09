#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/fourati_nonlinear.h>

using Scalar = double;  // Define scalar type (float or double)
using namespace EntoMath;

constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

//------------------------------------------------------------------------------
// Test for fourati IMU Update
//------------------------------------------------------------------------------
void test_fourati_update()
{
  ENTO_DEBUG("Running test_fourati_update...");

  // Initial quaternion
  Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
  // Sample IMU sensor readings (from RoboBee dataset)
  Vec3<Scalar> gyr(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06);  // Gyroscope (rad/s)
  Vec3<Scalar> acc(-0.07215829, 0.03096613, 8.31740944);                // Accelerometer (gravity vector)
  Vec3<Scalar> mag(16925.5042314,1207.22593348,34498.24159392);

  
  const Scalar gain = 0.05;
  Scalar dt  = 0.004f;  // Time step (s)
  const Eigen::Quaternion<Scalar> g_ref = {0., 0., 0., 1.}; // Reference gravity
  const Eigen::Quaternion<Scalar> m_ref = {0.,0.44275986,0.02522524,0.89628533}; // Reference magnetic field

  // Print Out Matrices
  ENTO_DEBUG_EIGEN_MATRIX(q_init.coeffs());
  ENTO_DEBUG_EIGEN_MATRIX(gyr);
  ENTO_DEBUG_EIGEN_MATRIX(acc);

  // Run the fourati IMU update
  Eigen::Quaternion<Scalar> q_updated = EntoAttitude::fourati_update(q_init, gyr, acc, mag, dt, gain, g_ref, m_ref);

  // Hardcoded expected quaternion (from Python ahrs run on RoboBee dataset) 
  Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,-7.85120035e-08,-2.70590288e-07,4.36307438e-07); // Placeholder

  // Validate each quaternion component
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
  ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
  ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

  ENTO_DEBUG("test_fourati_update PASSED!");
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

  if (__ento_test_num(__n, 1)) test_fourati_update();

  ENTO_TEST_END();
  //return 0;
}


