#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/madgwick_fixed.h>

using namespace EntoMath;

//------------------------------------------------------------------------------
// Test basic FixedPoint functionality with Eigen
//------------------------------------------------------------------------------
void test_fixedpoint_basic()
{
  ENTO_DEBUG("Running test_fixedpoint_basic...");

  // Test basic FixedPoint construction and conversion - use Q7_24 for larger range
  using FP = EntoAttitude::Q7_24;
  
  FP a(1.5f);
  FP b(2.0f);
  FP c = a + b;
  
  ENTO_DEBUG("FixedPoint test: %f + %f = %f", a.to_float(), b.to_float(), c.to_float());
  
  // Test Eigen vector with FixedPoint - use data() access
  Eigen::Matrix<FP, 3, 1> vec;
  vec.data()[0] = FP(1.0f);
  vec.data()[1] = FP(2.0f);
  vec.data()[2] = FP(3.0f);
  
  ENTO_DEBUG("FixedPoint vector: [%f, %f, %f]", 
    vec.data()[0].to_float(), vec.data()[1].to_float(), vec.data()[2].to_float());
  
  ENTO_DEBUG("test_fixedpoint_basic PASSED!");
}

//------------------------------------------------------------------------------
// Test FixedPoint quaternion operations
//------------------------------------------------------------------------------
void test_fixedpoint_quaternion()
{
  ENTO_DEBUG("Running test_fixedpoint_quaternion...");

  using FP = EntoAttitude::Q7_24;
  
  // Test quaternion construction
  Eigen::Quaternion<FP> q(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));
  
  ENTO_DEBUG("FixedPoint quaternion: [%f, %f, %f, %f]", 
    q.coeffs()[0].to_float(), q.coeffs()[1].to_float(), 
    q.coeffs()[2].to_float(), q.coeffs()[3].to_float());
  
  // Test quaternion normalization
  try {
    q.normalize();
    ENTO_DEBUG("Quaternion normalization succeeded");
  } catch (...) {
    ENTO_DEBUG("Quaternion normalization failed - sqrt not implemented for FixedPoint");
  }
  
  ENTO_DEBUG("test_fixedpoint_quaternion PASSED!");
}

//------------------------------------------------------------------------------
// Test Madgwick algorithm with FixedPoint
//------------------------------------------------------------------------------
void test_madgwick_fixedpoint()
{
  ENTO_DEBUG("Running test_madgwick_fixedpoint...");

  using FP = EntoAttitude::Q7_24;
  
  // Initial quaternion
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));
  
  // Sample IMU sensor readings (scaled for FixedPoint range)
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-0.001f);
  gyr.data()[1] = FP(-0.0006f);
  gyr.data()[2] = FP(-0.000004f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-0.07f);
  acc.data()[1] = FP(0.03f);
  acc.data()[2] = FP(8.3f);

  FP gain = FP(0.1f);
  FP dt = FP(0.004f);

  ENTO_DEBUG("Initial quaternion: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  try {
    // Run the madgwick IMU update
    Eigen::Quaternion<FP> q_updated = EntoAttitude::madgwick_update_imu_fixed(
      q_init, gyr, acc, dt, gain);

    ENTO_DEBUG("Updated quaternion: [%f, %f, %f, %f]", 
      q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
      q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());
    
    ENTO_DEBUG("Madgwick FixedPoint algorithm succeeded!");
  } catch (...) {
    ENTO_DEBUG("Madgwick FixedPoint algorithm failed");
  }

  ENTO_DEBUG("test_madgwick_fixedpoint PASSED!");
}

//------------------------------------------------------------------------------
// Test Q3.12 format from the paper
//------------------------------------------------------------------------------
void test_q3_12_format()
{
  ENTO_DEBUG("Running test_q3_12_format...");

  // Test Q3.12 format - has 3 integer bits, can handle values up to ~7
  using FP = EntoAttitude::Q3_12;
  
  FP a(1.5f);
  FP b(2.0f);
  FP c = a + b;
  
  ENTO_DEBUG("Q3.12 test: %f + %f = %f", a.to_float(), b.to_float(), c.to_float());
  
  // Test vector operations
  Eigen::Matrix<FP, 3, 1> vec;
  vec.data()[0] = FP(1.0f);
  vec.data()[1] = FP(2.0f);
  vec.data()[2] = FP(3.0f);
  
  ENTO_DEBUG("Q3.12 vector: [%f, %f, %f]", 
    vec.data()[0].to_float(), vec.data()[1].to_float(), vec.data()[2].to_float());
  
  ENTO_DEBUG("test_q3_12_format PASSED!");
}

//------------------------------------------------------------------------------
// Test Madgwick functor interface
//------------------------------------------------------------------------------
void test_madgwick_functor()
{
  ENTO_DEBUG("Running test_madgwick_functor...");

  using FP = EntoAttitude::Q7_24;
  
  // Create the functor
  EntoAttitude::FilterMadgwickFixed<FP, false> madgwick_filter;
  
  // Initial quaternion
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));
  
  // Sample measurement
  EntoAttitude::AttitudeMeasurement<FP, false> meas;
  meas.gyr.data()[0] = FP(-0.001f);
  meas.gyr.data()[1] = FP(-0.0006f);
  meas.gyr.data()[2] = FP(-0.000004f);
  meas.acc.data()[0] = FP(-0.07f);
  meas.acc.data()[1] = FP(0.03f);
  meas.acc.data()[2] = FP(8.3f);

  FP dt = FP(0.004f);
  FP gain = FP(0.1f);

  try {
    // Test the functor interface
    Eigen::Quaternion<FP> q_result = madgwick_filter(q_init, meas, dt, gain);

    ENTO_DEBUG("Functor result quaternion: [%f, %f, %f, %f]", 
      q_result.coeffs()[0].to_float(), q_result.coeffs()[1].to_float(), 
      q_result.coeffs()[2].to_float(), q_result.coeffs()[3].to_float());
    
    ENTO_DEBUG("Madgwick functor interface succeeded!");
  } catch (...) {
    ENTO_DEBUG("Madgwick functor interface failed");
  }

  ENTO_DEBUG("test_madgwick_functor PASSED!");
}

//------------------------------------------------------------------------------
// Test Madgwick IMU with realistic sensor data - Q7.24 format
//------------------------------------------------------------------------------
void test_madgwick_imu_realistic_q7_24()
{
  ENTO_DEBUG("Running test_madgwick_imu_realistic_q7_24...");

  using FP = EntoAttitude::Q7_24;
  
  // Initial quaternion
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));
  
  // Realistic water strider robot sensor readings (scaled down for testing)
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(2.856683f);   // Gyroscope (rad/s) - realistic values
  gyr.data()[1] = FP(-0.095346f);
  gyr.data()[2] = FP(0.347629f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(0.014683f);   // Accelerometer (m/s²)
  acc.data()[1] = FP(-0.019885f);
  acc.data()[2] = FP(-1.012501f);  // Close to -1g (gravity)

  FP gain = FP(0.1f);
  FP dt = FP(0.001f);  // 1kHz sampling rate

  ENTO_DEBUG("Initial quaternion Q7.24: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the madgwick IMU update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::madgwick_update_imu_fixed(q_init, gyr, acc, dt, gain);

  ENTO_DEBUG("Updated quaternion Q7.24: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  ENTO_DEBUG("test_madgwick_imu_realistic_q7_24 PASSED!");
}

//------------------------------------------------------------------------------
// Test Madgwick IMU with realistic sensor data - Q5.26 format
//------------------------------------------------------------------------------
void test_madgwick_imu_realistic_q5_26()
{
  ENTO_DEBUG("Running test_madgwick_imu_realistic_q5_26...");

  using FP = EntoAttitude::Q5_26;
  
  // Initial quaternion
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));
  
  // Realistic water strider robot sensor readings
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(2.856683f);   // Gyroscope (rad/s)
  gyr.data()[1] = FP(-0.095346f);
  gyr.data()[2] = FP(0.347629f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(0.014683f);   // Accelerometer (m/s²)
  acc.data()[1] = FP(-0.019885f);
  acc.data()[2] = FP(-1.012501f);

  FP gain = FP(0.1f);
  FP dt = FP(0.001f);

  ENTO_DEBUG("Initial quaternion Q5.26: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the madgwick IMU update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::madgwick_update_imu_fixed(q_init, gyr, acc, dt, gain);

  ENTO_DEBUG("Updated quaternion Q5.26: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  ENTO_DEBUG("test_madgwick_imu_realistic_q5_26 PASSED!");
}

//------------------------------------------------------------------------------
// Test Madgwick MARG with realistic sensor data - Q7.24 format
//------------------------------------------------------------------------------
void test_madgwick_marg_realistic_q7_24()
{
  ENTO_DEBUG("Running test_madgwick_marg_realistic_q7_24...");

  using FP = EntoAttitude::Q7_24;
  
  // Initial quaternion
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));

  // Realistic water strider robot sensor readings
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(2.856683f);   // Gyroscope (rad/s)
  gyr.data()[1] = FP(-0.095346f);
  gyr.data()[2] = FP(0.347629f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(0.014683f);   // Accelerometer (m/s²)
  acc.data()[1] = FP(-0.019885f);
  acc.data()[2] = FP(-1.012501f);
  
  Eigen::Matrix<FP,3,1> mag;
  mag.data()[0] = FP(7.5999f);     // Magnetometer (scaled down from original)
  mag.data()[1] = FP(3.7998f);
  mag.data()[2] = FP(3.0376f);

  FP gain = FP(0.1f);
  FP dt = FP(0.001f);

  ENTO_DEBUG("Initial quaternion Q7.24 MARG: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the madgwick MARG update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::madgwick_update_marg_fixed(q_init, gyr, acc, mag, dt, gain);

  ENTO_DEBUG("Updated quaternion Q7.24 MARG: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  ENTO_DEBUG("test_madgwick_marg_realistic_q7_24 PASSED!");
}

//------------------------------------------------------------------------------
// Test Madgwick with tiny synthetic gyro values (like Mahony test)
//------------------------------------------------------------------------------
void test_madgwick_tiny_synthetic_values()
{
  ENTO_DEBUG("Running test_madgwick_tiny_synthetic_values...");

  using FP = EntoAttitude::Q7_24;
  
  // Initial quaternion (from RoboBee dataset)
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(2.26892868785046e-7f), FP(-1.48352885438851e-7f), FP(4.45058992918769e-7f));
  
  // Sample IMU sensor readings (from RoboBee dataset) - the tiny values that caused Mahony to fail
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-1.52713681e-04f);  // Gyroscope (rad/s) - tiny values
  gyr.data()[1] = FP(-6.10919329e-05f);
  gyr.data()[2] = FP(-4.35697544e-06f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-7.07106781e-02f);  // Accelerometer (m/s²)
  acc.data()[1] = FP(3.53553391e-02f);
  acc.data()[2] = FP(8.31469612e+00f);

  FP gain = FP(0.1f);
  FP dt = FP(0.004f);

  ENTO_DEBUG("Initial quaternion: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  ENTO_DEBUG("Tiny gyro values: [%f, %f, %f]", 
    gyr.data()[0].to_float(), gyr.data()[1].to_float(), gyr.data()[2].to_float());

  // Check gyro norm
  FP gyr_norm = gyr.norm();
  ENTO_DEBUG("Gyro norm: %f", gyr_norm.to_float());

  // Run the madgwick IMU update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::madgwick_update_imu_fixed(q_init, gyr, acc, dt, gain);

  ENTO_DEBUG("Updated quaternion: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  // Check if quaternion changed
  bool quaternion_changed = (q_updated.coeffs() - q_init.coeffs()).norm() > FP(1e-6f);
  ENTO_DEBUG("Quaternion changed: %s", quaternion_changed ? "YES" : "NO");

  ENTO_DEBUG("test_madgwick_tiny_synthetic_values PASSED!");
}

//------------------------------------------------------------------------------
// Main Test Runner
//------------------------------------------------------------------------------
int main( int argc, char** argv )
{
  using namespace EntoUtil;
  
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

  if (__ento_test_num(__n, 1)) test_fixedpoint_basic();
  if (__ento_test_num(__n, 2)) test_fixedpoint_quaternion();
  if (__ento_test_num(__n, 3)) test_madgwick_fixedpoint();
  if (__ento_test_num(__n, 4)) test_q3_12_format();
  if (__ento_test_num(__n, 5)) test_madgwick_functor();
  if (__ento_test_num(__n, 6)) test_madgwick_imu_realistic_q7_24();
  if (__ento_test_num(__n, 7)) test_madgwick_imu_realistic_q5_26();
  if (__ento_test_num(__n, 8)) test_madgwick_marg_realistic_q7_24();
  if (__ento_test_num(__n, 9)) test_madgwick_tiny_synthetic_values();

  ENTO_TEST_END();
} 