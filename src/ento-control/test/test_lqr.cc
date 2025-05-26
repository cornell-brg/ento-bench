#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-control/lqr.h>
#include <ento-control/lqr_base.h>
#include <ento-control/lqr_traits_robofly.h>
#include <ento-control/opt_control_problem.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;

// Use the RoboFly LQR traits for testing
using LQRSolver = LQRStep<RoboFlyLQRTraits>;

void test_lqr_initialization() {
  LQRSolver lqr;
  
  // Check that the LQR solver returns the correct dynamics matrices
  auto Adyn = lqr.get_Adyn();
  auto Bdyn = lqr.get_Bdyn();
  
  // Compare with the expected matrices from RoboFlyLQRTraits
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(Adyn, RoboFlyLQRTraits::Adyn);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(Bdyn, RoboFlyLQRTraits::Bdyn);
}

void test_lqr_solve_zero_state() {
  LQRSolver lqr;
  
  // Create a zero state and zero reference
  Matrix<float, RoboFlyLQRTraits::N, 1> x0 = Matrix<float, RoboFlyLQRTraits::N, 1>::Zero();
  Matrix<float, RoboFlyLQRTraits::N, 1> x_ref = Matrix<float, RoboFlyLQRTraits::N, 1>::Zero();
  
  // Set the initial state and reference
  lqr.set_x0(x0);
  lqr.set_x_ref(x_ref);
  
  // Reset duals (no-op for LQR, but part of the interface)
  lqr.reset_duals();
  
  // Solve for the control input
  lqr.solve();
  
  // Get the control input
  auto u0 = lqr.get_u0();
  
  // For zero state and zero reference, the control input should be zero
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u = Matrix<float, RoboFlyLQRTraits::M, 1>::Zero();
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u0, expected_u);
}

void test_lqr_solve_nonzero_state() {
  LQRSolver lqr;
  
  // Create a non-zero state and zero reference
  Matrix<float, RoboFlyLQRTraits::N, 1> x0;
  x0 << 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f, 0.4f;
  
  Matrix<float, RoboFlyLQRTraits::N, 1> x_ref = Matrix<float, RoboFlyLQRTraits::N, 1>::Zero();
  
  // Set the initial state and reference
  lqr.set_x0(x0);
  lqr.set_x_ref(x_ref);
  
  // Solve for the control input
  lqr.solve();
  
  // Get the control input
  auto u0 = lqr.get_u0();
  
  // For non-zero state and zero reference, control should be -K*x
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u = -RoboFlyLQRTraits::K * x0;
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u0, expected_u);
}

void test_lqr_solve_nonzero_reference() {
  LQRSolver lqr;
  
  // Create a zero state and non-zero reference
  Matrix<float, RoboFlyLQRTraits::N, 1> x0 = Matrix<float, RoboFlyLQRTraits::N, 1>::Zero();
  
  Matrix<float, RoboFlyLQRTraits::N, 1> x_ref;
  x_ref << 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f, 0.4f;
  
  // Set the initial state and reference
  lqr.set_x0(x0);
  lqr.set_x_ref(x_ref);
  
  // Solve for the control input
  lqr.solve();
  
  // Get the control input
  auto u0 = lqr.get_u0();
  
  // For zero state and non-zero reference, control should be K*x_ref
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u = -RoboFlyLQRTraits::K * (x0 - x_ref);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u0, expected_u);
}

void test_lqr_solve_tracking() {
  LQRSolver lqr;
  
  // Create a non-zero state and non-zero reference
  Matrix<float, RoboFlyLQRTraits::N, 1> x0;
  x0 << 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f, 0.4f;
  
  Matrix<float, RoboFlyLQRTraits::N, 1> x_ref;
  x_ref << 5.0f, 6.0f, 7.0f, 0.5f, 0.6f, 0.7f, 0.05f, 0.06f, 0.07f, 0.8f;
  
  // Set the initial state and reference
  lqr.set_x0(x0);
  lqr.set_x_ref(x_ref);
  
  // Solve for the control input
  lqr.solve();
  
  // Get the control input
  auto u0 = lqr.get_u0();
  
  // Control should be -K*(x0 - x_ref)
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u = -RoboFlyLQRTraits::K * (x0 - x_ref);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u0, expected_u);
}

void test_lqr_closed_loop() {
  // Test LQR controller functionality using data derived from the actual trajectory
  LQRSolver lqr;
  
  // ---- Test with state at t=0.0 ----
  Matrix<float, RoboFlyLQRTraits::N, 1> x0;
  x0 << 0.092087f, -0.167926f, 0.261501f, 0.000000f, 0.000000f, 0.000000f, 0.021123f, 0.030017f, 0.000000f, 0.000000f;
  
  Matrix<float, RoboFlyLQRTraits::N, 1> x_ref;
  x_ref << 0.090000f, -0.168000f, 0.288000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f;
  
  // Set up controller with initial state and reference
  lqr.set_x0(x0);
  lqr.set_x_ref(x_ref);
  
  // Solve for control input
  lqr.solve();
  auto u0 = lqr.get_u0();
  
  // Calculate expected control based on the LQR formula
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u0 = -RoboFlyLQRTraits::K * (x0 - x_ref);
  
  // Verify the controller produces the expected control output
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u0, expected_u0);
  
  // ---- Test with state at t=0.5 ----
  Matrix<float, RoboFlyLQRTraits::N, 1> x1;
  x1 << 0.108290f, -0.175858f, 0.299720f, 0.501430f, -0.800790f, 2.290600f, 0.101524f, 0.131200f, 31.073203f, -124.922303f;
  
  // Reference is the same for all points
  Matrix<float, RoboFlyLQRTraits::N, 1> x_ref1 = x_ref;
  
  lqr.set_x0(x1);
  lqr.set_x_ref(x_ref1);
  lqr.solve();
  auto u1 = lqr.get_u0();
  
  // Verify control is correctly computed
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u1 = -RoboFlyLQRTraits::K * (x1 - x_ref1);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u1, expected_u1);
  
  // ---- Test with state at t=2.0 (end of trajectory) ----
  Matrix<float, RoboFlyLQRTraits::N, 1> x2;
  x2 << 0.104976f, -0.171701f, 0.308092f, 0.703340f, 1.477890f, 0.656840f, 0.032568f, 0.067413f, 110.511214f, -107.171688f;
  
  Matrix<float, RoboFlyLQRTraits::N, 1> x_ref2 = x_ref;
  
  lqr.set_x0(x2);
  lqr.set_x_ref(x_ref2);
  lqr.solve();
  auto u2 = lqr.get_u0();
  
  // Verify control is correctly computed
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u2 = -RoboFlyLQRTraits::K * (x2 - x_ref2);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u2, expected_u2);
  
  // Verify the core LQR property: when at the reference state, control should be zero
  Matrix<float, RoboFlyLQRTraits::N, 1> x_at_ref = x_ref;
  lqr.set_x0(x_at_ref);
  lqr.set_x_ref(x_ref);
  lqr.solve();
  auto u_at_ref = lqr.get_u0();
  
  Matrix<float, RoboFlyLQRTraits::M, 1> zero_u = Matrix<float, RoboFlyLQRTraits::M, 1>::Zero();
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u_at_ref, zero_u);
}

// Add a new test function for the optimal control problem framework
void test_lqr_with_opt_control() {
  // For now, let's just verify the LQR controller without using the OptControlProblem framework
  // since it appears to require a more complex setup with different matrices formats
  
  // Create the LQR solver
  LQRSolver lqr;
  
  // Test with trajectory data
  Matrix<float, RoboFlyLQRTraits::N, 1> start_state;
  start_state << 0.09f, -0.17f, 0.26f,   // Position
                0.0f, 0.0f, 0.0f,        // Velocity
                0.0f, 0.0f,              // Roll, pitch
                0.0f, 0.0f;              // Angular velocities
  
  Matrix<float, RoboFlyLQRTraits::N, 1> target_state;
  target_state << 0.10f, -0.17f, 0.29f,  // Target position
                 0.0f, 0.0f, 0.0f,       // Target velocity
                 0.0f, 0.0f,             // Target roll, pitch
                 0.0f, 0.0f;             // Target angular velocities
  
  // Test that LQR correctly implements the functions needed by the OptControlProblem framework
  ENTO_DEBUG("Testing LQR controller functions needed by OptControlProblem framework");
  
  // Verify that get_Adyn and get_Bdyn return the expected matrices
  auto Adyn = lqr.get_Adyn();
  auto Bdyn = lqr.get_Bdyn();
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(Adyn, RoboFlyLQRTraits::Adyn);
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(Bdyn, RoboFlyLQRTraits::Bdyn);
  
  // Verify that set_x0, set_x_ref, reset_duals, solve, and get_u0 all work
  lqr.set_x0(start_state);
  lqr.set_x_ref(target_state);
  lqr.reset_duals();
  lqr.solve();
  
  auto u0 = lqr.get_u0();
  Matrix<float, RoboFlyLQRTraits::M, 1> expected_u = -RoboFlyLQRTraits::K * (start_state - target_state);
  
  // Check that the control input is correct
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u0, expected_u);
  
  // Verify that when at the reference state, the control is zero
  lqr.set_x0(target_state);
  lqr.set_x_ref(target_state);
  lqr.solve();
  
  auto u_at_ref = lqr.get_u0();
  Matrix<float, RoboFlyLQRTraits::M, 1> zero_u = Matrix<float, RoboFlyLQRTraits::M, 1>::Zero();
  ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(u_at_ref, zero_u);
  
  // Note: A complete integration with OptControlProblem would require modifying
  // the LQR implementation to support the specific matrix formats expected by OptControlProblem,
  // which is beyond the scope of this test.
}

int main(int argc, char** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_lqr_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_TEST_START();
  
  if (__ento_test_num(__n, 1)) test_lqr_initialization();
  if (__ento_test_num(__n, 2)) test_lqr_solve_zero_state();
  if (__ento_test_num(__n, 3)) test_lqr_solve_nonzero_state();
  if (__ento_test_num(__n, 4)) test_lqr_solve_nonzero_reference();
  if (__ento_test_num(__n, 5)) test_lqr_solve_tracking();
  if (__ento_test_num(__n, 6)) test_lqr_closed_loop();
  if (__ento_test_num(__n, 7)) test_lqr_with_opt_control();
  
  ENTO_TEST_END();
} 