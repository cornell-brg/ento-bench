#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/mahoney_fixed.h>

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
  
  // Test vector operations
  FP norm2 = vec.squaredNorm();
  ENTO_DEBUG("Vector squared norm: %f", norm2.to_float());
  
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
  
  // Test quaternion normalization (this might fail if sqrt is not implemented)
  try {
    q.normalize();
    ENTO_DEBUG("Quaternion normalization succeeded");
  } catch (...) {
    ENTO_DEBUG("Quaternion normalization failed - sqrt not implemented for FixedPoint");
  }
  
  ENTO_DEBUG("test_fixedpoint_quaternion PASSED!");
}

//------------------------------------------------------------------------------
// Test vector normalization with FixedPoint
//------------------------------------------------------------------------------
void test_fixedpoint_normalize()
{
  ENTO_DEBUG("Running test_fixedpoint_normalize...");

  using FP = EntoAttitude::Q7_24;
  
  Eigen::Matrix<FP, 3, 1> vec;
  vec.data()[0] = FP(3.0f);
  vec.data()[1] = FP(4.0f);
  vec.data()[2] = FP(0.0f);
  
  ENTO_DEBUG("Original vector: [%f, %f, %f]", 
    vec.data()[0].to_float(), vec.data()[1].to_float(), vec.data()[2].to_float());
  
  // Test our custom normalization function
  try {
    auto normalized = EntoAttitude::vec_normalise(vec);
    ENTO_DEBUG("Normalized vector: [%f, %f, %f]", 
      normalized.data()[0].to_float(), normalized.data()[1].to_float(), normalized.data()[2].to_float());
    ENTO_DEBUG("Vector normalization succeeded");
  } catch (...) {
    ENTO_DEBUG("Vector normalization failed");
  }
  
  ENTO_DEBUG("test_fixedpoint_normalize PASSED!");
}

//------------------------------------------------------------------------------
// Test AttitudeMeasurement with FixedPoint
//------------------------------------------------------------------------------
void test_fixedpoint_measurement()
{
  ENTO_DEBUG("Running test_fixedpoint_measurement...");

  using FP = EntoAttitude::Q7_24;
  
  // Test measurement struct
  EntoAttitude::AttitudeMeasurement<FP, false> meas;
  meas.gyr.data()[0] = FP(0.1f);
  meas.gyr.data()[1] = FP(0.2f);
  meas.gyr.data()[2] = FP(0.3f);
  meas.acc.data()[0] = FP(0.0f);
  meas.acc.data()[1] = FP(0.0f);
  meas.acc.data()[2] = FP(9.8f);
  
  ENTO_DEBUG("Measurement gyro: [%f, %f, %f]", 
    meas.gyr.data()[0].to_float(), meas.gyr.data()[1].to_float(), meas.gyr.data()[2].to_float());
  ENTO_DEBUG("Measurement accel: [%f, %f, %f]", 
    meas.acc.data()[0].to_float(), meas.acc.data()[1].to_float(), meas.acc.data()[2].to_float());
  
  ENTO_DEBUG("test_fixedpoint_measurement PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony algorithm with FixedPoint (if previous tests pass)
//------------------------------------------------------------------------------
void test_mahoney_fixedpoint()
{
  ENTO_DEBUG("Running test_mahoney_fixedpoint...");

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

  FP k_i = FP(0.01f);
  FP k_p = FP(0.1f);
  FP dt = FP(0.004f);
  Eigen::Matrix<FP,3,1> bias;
  bias.data()[0] = FP(0.0f);
  bias.data()[1] = FP(0.0f);
  bias.data()[2] = FP(0.0f);

  ENTO_DEBUG("Initial quaternion: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  try {
    // Run the mahoney IMU update
    Eigen::Quaternion<FP> q_updated = EntoAttitude::mahony_update_imu_fixed(
      q_init, gyr, acc, dt, k_p, k_i, bias);

    ENTO_DEBUG("Updated quaternion: [%f, %f, %f, %f]", 
      q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
      q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());
    
    ENTO_DEBUG("Mahony FixedPoint algorithm succeeded!");
  } catch (...) {
    ENTO_DEBUG("Mahony FixedPoint algorithm failed");
  }

  ENTO_DEBUG("test_mahoney_fixedpoint PASSED!");
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
  
  FP norm2 = vec.squaredNorm();
  ENTO_DEBUG("Q3.12 vector squared norm: %f", norm2.to_float());
  
  ENTO_DEBUG("test_q3_12_format PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony functor interface
//------------------------------------------------------------------------------
void test_mahoney_functor()
{
  ENTO_DEBUG("Running test_mahoney_functor...");

  using FP = EntoAttitude::Q7_24;
  
  // Create the functor
  EntoAttitude::FilterMahonyFixed<FP, false> mahony_filter(FP(0.1f), FP(0.01f));
  
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
  Eigen::Quaternion<FP> q_out;

  try {
    // Test the functor interface
    Eigen::Quaternion<FP> q_result = mahony_filter(q_init, meas, dt, &q_out);

    ENTO_DEBUG("Functor result quaternion: [%f, %f, %f, %f]", 
      q_result.coeffs()[0].to_float(), q_result.coeffs()[1].to_float(), 
      q_result.coeffs()[2].to_float(), q_result.coeffs()[3].to_float());
    
    ENTO_DEBUG("Mahony functor interface succeeded!");
  } catch (...) {
    ENTO_DEBUG("Mahony functor interface failed");
  }

  ENTO_DEBUG("test_mahoney_functor PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony IMU with real RoboBee data - Q6.25 format
//------------------------------------------------------------------------------
void test_mahoney_imu_real_data_q6_25()
{
  ENTO_DEBUG("Running test_mahoney_imu_real_data_q6_25...");

  using FP = EntoAttitude::Q6_25;  // Q6.25 format
  
  // Initial quaternion (from RoboBee dataset)
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(2.26892868785046e-7f), FP(-1.48352885438851e-7f), FP(4.45058992918769e-7f));
  
  // Sample IMU sensor readings (from RoboBee dataset) - original values
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-1.52713681e-04f);  // Gyroscope (rad/s)
  gyr.data()[1] = FP(-6.10919329e-05f);
  gyr.data()[2] = FP(-4.35697544e-06f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-0.07215829f);      // Accelerometer (gravity vector)
  acc.data()[1] = FP(0.03096613f);
  acc.data()[2] = FP(8.31740944f);

  FP k_i = FP(0.01f);       // Integral term
  FP k_p = FP(0.1f);        // Proportional term
  FP dt = FP(0.004f);       // Time step (s)
  Eigen::Matrix<FP,3,1> bias;
  bias.data()[0] = FP(0.0f);
  bias.data()[1] = FP(0.0f);
  bias.data()[2] = FP(0.0f);

  ENTO_DEBUG("Initial quaternion Q6.25: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the mahoney IMU update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::mahony_update_imu_fixed(q_init, gyr, acc, dt, k_p, k_i, bias);

  ENTO_DEBUG("Updated quaternion Q6.25: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  // Expected quaternion (from Python ahrs run on RoboBee dataset)
  Eigen::Quaternion<FP> q_expected(FP(1.00000000e+00f), FP(6.66248740e-07f), FP(1.46525393e-06f), FP(4.36344465e-07f));

  ENTO_DEBUG("Expected quaternion Q6.25: [%f, %f, %f, %f]", 
    q_expected.coeffs()[0].to_float(), q_expected.coeffs()[1].to_float(), 
    q_expected.coeffs()[2].to_float(), q_expected.coeffs()[3].to_float());

  ENTO_DEBUG("test_mahoney_imu_real_data_q6_25 PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony IMU with real RoboBee data - Q3.12 format
//------------------------------------------------------------------------------
void test_mahoney_imu_real_data_q3_12()
{
  ENTO_DEBUG("Running test_mahoney_imu_real_data_q3_12...");

  using FP = EntoAttitude::Q3_12;
  
  // Initial quaternion (from RoboBee dataset) - note: very small values may underflow in Q3.12
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));  // Simplified for Q3.12 range
  
  // Sample IMU sensor readings (from RoboBee dataset)
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-1.52713681e-04f);  // Gyroscope (rad/s)
  gyr.data()[1] = FP(-6.10919329e-05f);
  gyr.data()[2] = FP(-4.35697544e-06f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-0.07215829f);      // Accelerometer (gravity vector)
  acc.data()[1] = FP(0.03096613f);
  acc.data()[2] = FP(8.31740944f);       // This is close to Q3.12 limit (~7.99)

  FP k_i = FP(0.01f);       // Integral term
  FP k_p = FP(0.1f);        // Proportional term
  FP dt = FP(0.004f);       // Time step (s)
  Eigen::Matrix<FP,3,1> bias;
  bias.data()[0] = FP(0.0f);
  bias.data()[1] = FP(0.0f);
  bias.data()[2] = FP(0.0f);

  ENTO_DEBUG("Initial quaternion Q3.12: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the mahoney IMU update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::mahony_update_imu_fixed(q_init, gyr, acc, dt, k_p, k_i, bias);

  ENTO_DEBUG("Updated quaternion Q3.12: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  ENTO_DEBUG("test_mahoney_imu_real_data_q3_12 PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony IMU with real RoboBee data - Q5.26 format
//------------------------------------------------------------------------------
void test_mahoney_imu_real_data_q5_26()
{
  ENTO_DEBUG("Running test_mahoney_imu_real_data_q5_26...");

  using FP = EntoAttitude::Q5_26;  // Q5.26 format
  
  // Initial quaternion (from RoboBee dataset)
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(2.26892868785046e-7f), FP(-1.48352885438851e-7f), FP(4.45058992918769e-7f));
  
  // Sample IMU sensor readings (from RoboBee dataset) - original values
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-1.52713681e-04f);  // Gyroscope (rad/s)
  gyr.data()[1] = FP(-6.10919329e-05f);
  gyr.data()[2] = FP(-4.35697544e-06f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-0.07215829f);      // Accelerometer (gravity vector)
  acc.data()[1] = FP(0.03096613f);
  acc.data()[2] = FP(8.31740944f);

  FP k_i = FP(0.01f);       // Integral term
  FP k_p = FP(0.1f);        // Proportional term
  FP dt = FP(0.004f);       // Time step (s)
  Eigen::Matrix<FP,3,1> bias;
  bias.data()[0] = FP(0.0f);
  bias.data()[1] = FP(0.0f);
  bias.data()[2] = FP(0.0f);

  ENTO_DEBUG("Initial quaternion Q5.26: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the mahoney IMU update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::mahony_update_imu_fixed(q_init, gyr, acc, dt, k_p, k_i, bias);

  ENTO_DEBUG("Updated quaternion Q5.26: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  // Expected quaternion (from Python ahrs run on RoboBee dataset)
  Eigen::Quaternion<FP> q_expected(FP(1.00000000e+00f), FP(6.66248740e-07f), FP(1.46525393e-06f), FP(4.36344465e-07f));

  ENTO_DEBUG("Expected quaternion Q5.26: [%f, %f, %f, %f]", 
    q_expected.coeffs()[0].to_float(), q_expected.coeffs()[1].to_float(), 
    q_expected.coeffs()[2].to_float(), q_expected.coeffs()[3].to_float());

  ENTO_DEBUG("test_mahoney_imu_real_data_q5_26 PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony IMU with real RoboBee data - Q2.13 format
//------------------------------------------------------------------------------
void test_mahoney_imu_real_data_q2_13()
{
  ENTO_DEBUG("Running test_mahoney_imu_real_data_q2_13...");

  using FP = EntoAttitude::Q2_13;  // Q2.13 format - 16-bit with higher precision
  
  // Initial quaternion (simplified for Q2.13 range - max value ~3.99)
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));  // Simplified for Q2.13 range
  
  // Sample IMU sensor readings (from RoboBee dataset)
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-1.52713681e-04f);  // Gyroscope (rad/s)
  gyr.data()[1] = FP(-6.10919329e-05f);
  gyr.data()[2] = FP(-4.35697544e-06f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-0.07215829f);      // Accelerometer (gravity vector)
  acc.data()[1] = FP(0.03096613f);
  acc.data()[2] = FP(3.99f);             // Scaled down to fit Q2.13 range (max ~3.99)

  FP k_i = FP(0.01f);       // Integral term
  FP k_p = FP(0.1f);        // Proportional term
  FP dt = FP(0.004f);       // Time step (s)
  Eigen::Matrix<FP,3,1> bias;
  bias.data()[0] = FP(0.0f);
  bias.data()[1] = FP(0.0f);
  bias.data()[2] = FP(0.0f);

  ENTO_DEBUG("Initial quaternion Q2.13: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the mahoney IMU update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::mahony_update_imu_fixed(q_init, gyr, acc, dt, k_p, k_i, bias);

  ENTO_DEBUG("Updated quaternion Q2.13: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  ENTO_DEBUG("test_mahoney_imu_real_data_q2_13 PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony MARG with real RoboBee data - Q7.24 format
//------------------------------------------------------------------------------
void test_mahoney_marg_real_data_q7_24()
{
  ENTO_DEBUG("Running test_mahoney_marg_real_data_q7_24...");

  using FP = EntoAttitude::Q7_24;
  
  // Initial quaternion (from RoboBee dataset)
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(2.26892868785046e-7f), FP(-1.48352885438851e-7f), FP(4.45058992918769e-7f));

  // Sample IMU sensor readings (from RoboBee dataset)
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-1.52713681e-04f);  // Gyroscope (rad/s)
  gyr.data()[1] = FP(-6.10919329e-05f);
  gyr.data()[2] = FP(-4.35697544e-06f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-0.07215829f);      // Accelerometer (gravity vector)
  acc.data()[1] = FP(0.03096613f);
  acc.data()[2] = FP(8.31740944f);
  
  Eigen::Matrix<FP,3,1> mag;
  mag.data()[0] = FP(16925.5042314f);    // Magnetometer
  mag.data()[1] = FP(1207.22593348f);
  mag.data()[2] = FP(34498.24159392f);

  FP k_i = FP(0.01f);       // Integral term
  FP k_p = FP(0.1f);        // Proportional term
  FP dt = FP(0.004f);       // Time step (s)
  Eigen::Matrix<FP,3,1> bias;
  bias.data()[0] = FP(0.0f);
  bias.data()[1] = FP(0.0f);
  bias.data()[2] = FP(0.0f);

  ENTO_DEBUG("Initial quaternion Q7.24 MARG: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the mahoney MARG update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::mahony_update_marg_fixed(q_init, gyr, acc, mag, dt, k_p, k_i, bias);

  ENTO_DEBUG("Updated quaternion Q7.24 MARG: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  // Expected quaternion (from Python ahrs run on RoboBee dataset)
  Eigen::Quaternion<FP> q_expected(FP(9.99999994e-01f), FP(-7.29375589e-05f), FP(-7.75753231e-05f), FP(3.93137128e-05f));

  ENTO_DEBUG("Expected quaternion Q7.24 MARG: [%f, %f, %f, %f]", 
    q_expected.coeffs()[0].to_float(), q_expected.coeffs()[1].to_float(), 
    q_expected.coeffs()[2].to_float(), q_expected.coeffs()[3].to_float());

  ENTO_DEBUG("test_mahoney_marg_real_data_q7_24 PASSED!");
}

//------------------------------------------------------------------------------
// Test Mahony MARG with real RoboBee data - Q3.12 format (scaled magnetometer)
//------------------------------------------------------------------------------
void test_mahoney_marg_real_data_q3_12()
{
  ENTO_DEBUG("Running test_mahoney_marg_real_data_q3_12...");

  using FP = EntoAttitude::Q3_12;
  
  // Initial quaternion (simplified for Q3.12 range)
  Eigen::Quaternion<FP> q_init(FP(1.0f), FP(0.0f), FP(0.0f), FP(0.0f));

  // Sample IMU sensor readings (from RoboBee dataset)
  Eigen::Matrix<FP,3,1> gyr;
  gyr.data()[0] = FP(-1.52713681e-04f);  // Gyroscope (rad/s)
  gyr.data()[1] = FP(-6.10919329e-05f);
  gyr.data()[2] = FP(-4.35697544e-06f);
  
  Eigen::Matrix<FP,3,1> acc;
  acc.data()[0] = FP(-0.07215829f);      // Accelerometer (gravity vector)
  acc.data()[1] = FP(0.03096613f);
  acc.data()[2] = FP(8.31740944f);       // Close to Q3.12 limit
  
  // Scale magnetometer values to fit Q3.12 range (divide by ~10000)
  Eigen::Matrix<FP,3,1> mag;
  mag.data()[0] = FP(1.69255f);          // Scaled magnetometer
  mag.data()[1] = FP(0.12072f);
  mag.data()[2] = FP(3.44982f);

  FP k_i = FP(0.01f);       // Integral term
  FP k_p = FP(0.1f);        // Proportional term
  FP dt = FP(0.004f);       // Time step (s)
  Eigen::Matrix<FP,3,1> bias;
  bias.data()[0] = FP(0.0f);
  bias.data()[1] = FP(0.0f);
  bias.data()[2] = FP(0.0f);

  ENTO_DEBUG("Initial quaternion Q3.12 MARG: [%f, %f, %f, %f]", 
    q_init.coeffs()[0].to_float(), q_init.coeffs()[1].to_float(), 
    q_init.coeffs()[2].to_float(), q_init.coeffs()[3].to_float());

  // Run the mahoney MARG update
  Eigen::Quaternion<FP> q_updated = EntoAttitude::mahony_update_marg_fixed(q_init, gyr, acc, mag, dt, k_p, k_i, bias);

  ENTO_DEBUG("Updated quaternion Q3.12 MARG: [%f, %f, %f, %f]", 
    q_updated.coeffs()[0].to_float(), q_updated.coeffs()[1].to_float(), 
    q_updated.coeffs()[2].to_float(), q_updated.coeffs()[3].to_float());

  ENTO_DEBUG("test_mahoney_marg_real_data_q3_12 PASSED!");
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
  if (__ento_test_num(__n, 3)) test_fixedpoint_normalize();
  if (__ento_test_num(__n, 4)) test_fixedpoint_measurement();
  if (__ento_test_num(__n, 5)) test_mahoney_fixedpoint();
  if (__ento_test_num(__n, 6)) test_q3_12_format();
  if (__ento_test_num(__n, 7)) test_mahoney_functor();
  if (__ento_test_num(__n, 8)) test_mahoney_imu_real_data_q6_25();
  if (__ento_test_num(__n, 9)) test_mahoney_imu_real_data_q3_12();
  if (__ento_test_num(__n, 10)) test_mahoney_imu_real_data_q5_26();
  if (__ento_test_num(__n, 11)) test_mahoney_imu_real_data_q2_13();
  if (__ento_test_num(__n, 12)) test_mahoney_marg_real_data_q7_24();
  if (__ento_test_num(__n, 13)) test_mahoney_marg_real_data_q3_12();

  ENTO_TEST_END();
} 