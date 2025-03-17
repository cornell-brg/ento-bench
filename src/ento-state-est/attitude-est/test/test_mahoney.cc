// #include <ento-util/debug.h>
// #include <ento-util/unittest.h>
// #include <ento-state-est/attitude-est/mahoney.h>

// using Scalar = double;  // Define scalar type (float or double)
// using namespace EntoMath;

// constexpr Scalar TOLERANCE = 1e-5;  // Adjust as needed

// //------------------------------------------------------------------------------
// // Test for mahoney IMU Update
// //------------------------------------------------------------------------------
// void test_mahoney_imu_update()
// {
//   ENTO_DEBUG("Running test_mahoney_imu_update...");

//   // Initial quaternion
//   Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
  
//   // Sample IMU sensor readings (from RoboBee dataset)
//   EntoMath::Vec3<Scalar> gyr(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06);  // Gyroscope (rad/s)
//   EntoMath::Vec3<Scalar> acc(-0.07215829, 0.03096613, 8.31740944);                // Accelerometer (gravity vector)

//   Scalar k_i = 0.01;       // Integral term
//   Scalar k_p = 0.1;       // Proportional term
//   Scalar dt  = 0.004f;  // Time step (s)
//   Vec3<Scalar> bias = { 0., 0., 0. }; // IMU Bias

//   // Print Out Matrices
//   ENTO_DEBUG_EIGEN_MATRIX(q_init.coeffs());
//   ENTO_DEBUG_EIGEN_MATRIX(gyr);
//   ENTO_DEBUG_EIGEN_MATRIX(acc);

//   // Run the mahoney IMU update
//   Eigen::Quaternion<Scalar> q_updated = EntoAttitude::mahoney_update_imu(q_init, gyr, acc, dt, k_p, k_i, bias);

//   // Hardcoded expected quaternion (from Python ahrs run on RoboBee dataset) 
//   Eigen::Quaternion<Scalar> q_expected(1.00000000e+00,6.66248740e-07,1.46525393e-06,4.36344465e-07); // Placeholder

//   // Validate each quaternion component
//   ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
//   ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
//   ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

//   ENTO_DEBUG("test_mahoney_imu_update PASSED!");
// }

// //------------------------------------------------------------------------------
// // Test for mahoney MARG Update
// //------------------------------------------------------------------------------
// void test_mahoney_marg_update()
// {
//   ENTO_DEBUG("Running test_mahoney_marg_update...");

//   // Initial quaternion
//   Eigen::Quaternion<Scalar> q_init(1., 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);

//   // Sample IMU sensor readings
//   Vec3<Scalar> gyr(-1.52713681e-04,-6.10919329e-05,-4.35697544e-06);  // Gyroscope (rad/s)
//   Vec3<Scalar> acc(-0.07215829,0.03096613,8.31740944);   // Accelerometer (gravity vector)
//   Vec3<Scalar> mag(16925.5042314,1207.22593348,34498.24159392);


//   Scalar k_i = 0.01;       // Integral term
//   Scalar k_p = 0.1;       // Proportional term
//   Scalar dt  = 0.004f;  // Time step (s)
//   Vec3<Scalar> bias = { 0., 0., 0. }; // IMU Bias

//   // Run the mahoney MARG update
//   Eigen::Quaternion<Scalar> q_updated = EntoAttitude::mahoney_update_marg(q_init, gyr, acc, mag, dt, k_p, k_i, bias);

//   // Hardcoded expected quaternion (from Python ahrs run on RoboBee dataset) 
//   Eigen::Quaternion<Scalar> q_expected(9.99999994e-01,-7.29375589e-05,-7.75753231e-05,3.93137128e-05); // Placeholder

//   // Validate each quaternion component
//   ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(q_updated.coeffs(), q_expected.coeffs());
//   ENTO_DEBUG_EIGEN_QUATERNION(q_updated);
//   ENTO_DEBUG_EIGEN_QUAT2(q_updated, q_expected);

//   ENTO_DEBUG("test_mahoney_marg_update PASSED!");
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

//   if (__ento_test_num(__n, 1)) test_mahoney_imu_update();
//   if (__ento_test_num(__n, 2)) test_mahoney_marg_update();

//   ENTO_TEST_END();
//   //return 0;
// }



#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/attitude-est/mahoney.h>
#include <ento-state-est/attitude-est/attitude_problem.h>

using Scalar = double;
using namespace EntoMath;

// A simple wrapper class to make Mahoney work with AttitudeProblem
template <typename Scalar, bool UseMag>
class MahoneyKernel
{
private:
    Scalar k_p_;
    Scalar k_i_;
    EntoMath::Vec3<Scalar> bias_;

public:
    MahoneyKernel(Scalar k_p = static_cast<Scalar>(0.1),
                 Scalar k_i = static_cast<Scalar>(0.01))
        : k_p_(k_p), k_i_(k_i), bias_(EntoMath::Vec3<Scalar>::Zero()) {}

    // This is the interface required by AttitudeProblem
    void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                   const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& measurement,
                   Scalar dt,
                   Eigen::Quaternion<Scalar>* q_out)
    {
        if constexpr (UseMag) {
            *q_out = EntoAttitude::mahoney_update_marg(
                q_prev,
                measurement.gyr,
                measurement.acc,
                measurement.mag,
                dt,
                k_p_,
                k_i_,
                bias_
            );
        } else {
            *q_out = EntoAttitude::mahoney_update_imu(
                q_prev,
                measurement.gyr,
                measurement.acc,
                dt,
                k_p_,
                k_i_,
                bias_
            );
        }
    }

    static constexpr const char* name()
    {
        if constexpr (UseMag) {
            return "Mahoney MARG Filter";
        } else {
            return "Mahoney IMU Filter";
        }
    }
};

// Test for Mahoney IMU integration with AttitudeProblem
void test_mahoney_imu_problem()
{
    ENTO_DEBUG("Running test_mahoney_imu_problem...");

    // Create the Mahoney kernel
    MahoneyKernel<Scalar, false> imuKernel(0.1, 0.01);

    // Create the problem with this kernel
    EntoAttitude::AttitudeProblem<Scalar, 
                               MahoneyKernel<Scalar, false>, 
                               false> imuProblem(imuKernel);

    // Create test measurement
    EntoAttitude::AttitudeMeasurement<Scalar, false> measurement(
        Vec3<Scalar>(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06),  // Gyroscope
        Vec3<Scalar>(-0.07215829, 0.03096613, 8.31740944)                 // Accelerometer
    );

    // Set up the problem
    imuProblem.measurement_ = measurement;
    imuProblem.dt_ = 0.004;
    imuProblem.q_gt_ = Eigen::Quaternion<Scalar>(1.0, 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
    
    // Initial quaternion
    imuProblem.q_prev_ = imuProblem.q_gt_;

    // Solve the problem
    imuProblem.solve_impl();

    // Expected quaternion from previous test
    Eigen::Quaternion<Scalar> q_expected(1.00000000e+00, 6.66248740e-07, 1.46525393e-06, 4.36344465e-07);

    // Check results
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(imuProblem.q_.coeffs(), q_expected.coeffs());
    
    // Also test the validation
    bool valid = imuProblem.validate(1.0);  // 1 degree threshold
    ENTO_TEST_CHECK(valid);

    ENTO_DEBUG("test_mahoney_imu_problem PASSED!");
}

// Test for Mahoney MARG integration with AttitudeProblem
void test_mahoney_marg_problem()
{
    ENTO_DEBUG("Running test_mahoney_marg_problem...");

    // Create the Mahoney kernel
    MahoneyKernel<Scalar, true> margKernel(0.1, 0.01);

    // Create the problem with this kernel
    EntoAttitude::AttitudeProblem<Scalar, 
                               MahoneyKernel<Scalar, true>, 
                               true> margProblem(margKernel);

    // Create test measurement
    EntoAttitude::AttitudeMeasurement<Scalar, true> measurement(
        Vec3<Scalar>(-1.52713681e-04, -6.10919329e-05, -4.35697544e-06),  // Gyroscope
        Vec3<Scalar>(-0.07215829, 0.03096613, 8.31740944),                // Accelerometer
        Vec3<Scalar>(16925.5042314, 1207.22593348, 34498.24159392)        // Magnetometer
    );

    // Set up the problem
    margProblem.measurement_ = measurement;
    margProblem.dt_ = 0.004;
    margProblem.q_gt_ = Eigen::Quaternion<Scalar>(1.0, 2.26892868785046e-7, -1.48352885438851e-7, 4.45058992918769e-7);
    
    // Initial quaternion
    margProblem.q_prev_ = margProblem.q_gt_;

    // Solve the problem
    margProblem.solve_impl();

    // Expected quaternion from previous test
    Eigen::Quaternion<Scalar> q_expected(9.99999994e-01, -7.29375589e-05, -7.75753231e-05, 3.93137128e-05);

    // Check results
    ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(margProblem.q_.coeffs(), q_expected.coeffs());
    
    // Also test the validation
    bool valid = margProblem.validate(1.0);  // 1 degree threshold
    ENTO_TEST_CHECK(valid);

    ENTO_DEBUG("test_mahoney_marg_problem PASSED!");
}

//------------------------------------------------------------------------------
// Main Test Runner
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    using namespace EntoUtil;
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

    if (__ento_test_num(__n, 1)) test_mahoney_imu_problem();
    if (__ento_test_num(__n, 2)) test_mahoney_marg_problem();

    ENTO_TEST_END();
}