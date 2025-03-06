#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/ekf.h>
#include <ento-state-est/robobee_ekf.h>

using Scalar = double;
constexpr Scalar TOLERANCE = 1e-5;

using namespace EntoUtil;

//------------------------------------------------------------------------------
// Test EKF Initialization
//------------------------------------------------------------------------------
void test_ekf_initialization()
{
  ENTO_DEBUG("Running test_ekf_initialization...");

  Eigen::Matrix<Scalar, 10, 10> Q = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  
  EKF<Scalar, 10, 4, 4> ekf(Q, R);

  Eigen::Matrix<Scalar, 10,  1> expected_state = Eigen::Matrix<Scalar, 10, 1>::Zero();
  Eigen::Matrix<Scalar, 10, 10> expected_cov   = Eigen::Matrix<Scalar, 10, 10>::Identity();

  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(ekf.getState(), expected_state);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(ekf.getCovariance(), expected_cov);

  ENTO_DEBUG("test_ekf_initialization PASSED!");
}

//------------------------------------------------------------------------------
// Test EKF Prediction Step
//------------------------------------------------------------------------------
void test_ekf_prediction()
{
  ENTO_DEBUG("Running test_ekf_prediction...");

  Eigen::Matrix<Scalar, 10, 10> Q    = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 10, 10> Qhat = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.01;
  Eigen::Matrix<Scalar, 4, 4> R      = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  Eigen::Matrix<Scalar, 4, 4> Rhat   = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.1;
  EKF<Scalar, 10, 4, 4> ekf(Q, R);

  Eigen::Matrix<Scalar, 10, 1> x = Eigen::Matrix<Scalar, 10, 1>::Zero();
  Eigen::Matrix<Scalar, 4, 1> u = Eigen::Matrix<Scalar, 4, 1>::Zero();
  RobobeeDynamicsModel<Scalar> dynamics;

  // Compute F (state transition Jacobian)
  Eigen::Matrix<Scalar, 10, 10> F = Eigen::Matrix<Scalar, 10, 10>::Zero();
  dynamics(x, u);
  dynamics.jacobian(F, x, u);

  // Predict next state
  ekf.predict(dynamics, u);

  Eigen::Matrix<Scalar, 10, 10> P_expected = F * Eigen::Matrix<Scalar, 10, 10>::Identity() * F.transpose() + Qhat;

  //ENTO_DEBUG_EIGEN_MATRIX(P_expected);
  //ENTO_DEBUG_EIGEN_MATRIX(ekf.getCovariance());
  ENTO_DEBUG_EIGEN_MATRIX_COMPARISON(P_expected, ekf.getCovariance(), TOLERANCE);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ_TOL(ekf.getCovariance(), P_expected, TOLERANCE);
  ENTO_DEBUG("test_ekf_prediction PASSED!");
}

//------------------------------------------------------------------------------
// Test EKF Update Step
//------------------------------------------------------------------------------
void test_ekf_update()
{
  ENTO_DEBUG("Running test_ekf_update...");

  // Initialize Q and R from Python implementation
  Eigen::Matrix<Scalar, 10, 10> Q = Eigen::Matrix<Scalar, 10, 10>::Identity();
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Zero();
  R(0, 0) = 0.07;
  R(1, 1) = 0.07;
  R(2, 2) = 0.07;
  R(3, 3) = 0.002;

  // Initial Covariance
  Eigen::Matrix<Scalar, 10, 10> P_init = Eigen::Matrix<Scalar, 10, 10>::Identity() * (M_PI / 2);

  // Initialize EKF
  EKF<Scalar, 10, 4, 4> ekf(Q, R);
  ekf.set_covariance(P_init);

  // Initialize dynamics and measurement models
  RobobeeDynamicsModel<Scalar> dynamics;
  RobobeeMeasurementModel<Scalar> meas_model;

  // Set initial measurement and control values from Python
  Eigen::Matrix<Scalar, 4, 1> z;
  z << 0.0, 0.0, 0.01745329, 0.072;

  Eigen::Matrix<Scalar, 4, 1> u;
  u << 0.0, -0.0, -0.0, 0.0;

  // Perform EKF prediction then update
  ekf.predict(dynamics, u);
  ekf.update(meas_model, z);

  // Expected updated state based on Python output
  Eigen::Matrix<Scalar, 4, 1> x_expected;
  x_expected << 0.0, 0.0, 0.01699066, 0.07194403;

  // Extract estimated measurement state
  Eigen::Matrix<Scalar, 4, 10> H = Eigen::Matrix<Scalar, 4, 10>::Zero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 2) = 1;
  H(3, 6) = 1;
  Eigen::Matrix<Scalar, 4, 1> x_meas = H * ekf.getState();

  // Debug print
  ENTO_DEBUG_EIGEN_MATRIX(x_meas);
  ENTO_DEBUG_EIGEN_MATRIX(x_expected);
  ENTO_DEBUG_EIGEN_MATRIX_COMPARISON(x_meas, x_expected, TOLERANCE);

  // Validate state update
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ_TOL(x_meas, x_expected, TOLERANCE);

  ENTO_DEBUG("test_ekf_update PASSED!");
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

  if (__ento_test_num(__n, 1)) test_ekf_initialization();
  if (__ento_test_num(__n, 2)) test_ekf_prediction();
  if (__ento_test_num(__n, 3)) test_ekf_update();

  ENTO_TEST_END();
}

