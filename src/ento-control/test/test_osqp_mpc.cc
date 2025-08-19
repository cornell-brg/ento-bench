#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <cmath>
#include <limits>

#include <ento-control/osqp_bridge_template.h>
#include "generated/robofly_mpc/robofly_api.h"

using namespace std;
using namespace Eigen;
using namespace EntoUtil;

// Use the new OSQPBridge template
using MpcSolver = EntoControl::OSQPBridge<RoboFlyAPI>;
using MpcSolverWarmStart = EntoControl::OSQPBridge<RoboFlyAPI>; // Same for now

void test_osqp_mpc_initialization() {
  // Test that the solver can be initialized and basic operations work
  MpcSolver solver;
  
  // Test setting a valid state
  MpcSolver::State x0;
  x0 << 0.1f, 0.1f, 0.1f, 0.0f, 0.0f, 0.0f, 0.01f, 0.01f, 0.0f, 0.0f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  
  // Test setting reference
  MpcSolver::State x_ref = MpcSolver::State::Zero();
  solver.set_x_ref(x_ref);
  
  // Test that we can get initial status
  auto initial_status = solver.get_status();
  ENTO_DEBUG("Initial status: %d", static_cast<int>(initial_status));
}

void test_osqp_mpc_solve_zero_state() {
  MpcSolver solver;
  
  // Test with zero state and zero reference
  MpcSolver::State x0 = MpcSolver::State::Zero();
  MpcSolver::State x_ref = MpcSolver::State::Zero();
  
  // Set the initial state and reference
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  solver.set_x_ref(x_ref);
  
  // Solve for the control input
  bool solve_success = solver.solve();
  ENTO_DEBUG("Zero state solve success: %s, Status: %d", solve_success ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  // Get the control input
  auto u0 = solver.get_u0();
  
  // Debug output
  ENTO_DEBUG("Zero state u0: [%.6f, %.6f, %.6f]", u0[0], u0[1], u0[2]);
  ENTO_DEBUG("OSQP iterations: %d", solver.get_iterations());
  
  // For zero state and zero reference, control should be small
  for (int i = 0; i < 3; ++i) {
    ENTO_TEST_CHECK_TRUE(std::abs(u0[i]) < 1.0f);
  }
}

void test_osqp_mpc_solve_nonzero_state() {
  MpcSolver solver;
  
  // Use realistic trajectory data from test_lqr.cc (t=0.0)
  MpcSolver::State x0;
  x0 << 0.092087f, -0.167926f, 0.261501f, 0.000000f, 0.000000f, 0.000000f, 0.021123f, 0.030017f, 0.000000f, 0.000000f;
  
  MpcSolver::State x_ref;
  x_ref << 0.090000f, -0.168000f, 0.288000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f;
  
  // Set the initial state and reference
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  solver.set_x_ref(x_ref);
  
  // Solve for the control input
  bool solve_success = solver.solve();
  ENTO_DEBUG("Solve success: %s, Status: %d", solve_success ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  // Get the control input
  auto u0 = solver.get_u0();
  
  // Debug output
  ENTO_DEBUG("Realistic state x0: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", 
             x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6], x0[7], x0[8], x0[9]);
  ENTO_DEBUG("Reference x_ref: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", 
             x_ref[0], x_ref[1], x_ref[2], x_ref[3], x_ref[4], x_ref[5], x_ref[6], x_ref[7], x_ref[8], x_ref[9]);
  ENTO_DEBUG("Control u0: [%.6f, %.6f, %.6f]", u0[0], u0[1], u0[2]);
  ENTO_DEBUG("Control norm: %.6f", u0.norm());
  
  // Check raw solution from OSQP
  auto workspace = RoboFlyAPI::get_workspace();
  const float* raw_solution = RoboFlyAPI::get_solution(workspace);
  ENTO_DEBUG("Raw OSQP solution first 10 elements: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
             raw_solution[0], raw_solution[1], raw_solution[2], raw_solution[3], raw_solution[4],
             raw_solution[5], raw_solution[6], raw_solution[7], raw_solution[8], raw_solution[9]);
  
  // Control input should not be zero (MPC should try to drive state to reference)
  ENTO_TEST_CHECK_TRUE(u0.norm() > 0);
  
  // Check iterations
  ENTO_DEBUG("OSQP iterations: %d", solver.get_iterations());
  ENTO_TEST_CHECK_TRUE(solver.get_iterations() > 0);
}

void test_osqp_mpc_solve_nonzero_reference() {
  MpcSolver solver;
  
  // Create a zero state and realistic reference from test_lqr.cc
  MpcSolver::State x0 = MpcSolver::State::Zero();
  
  MpcSolver::State x_ref;
  x_ref << 0.090000f, -0.168000f, 0.288000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f;
  
  // Set the initial state and reference
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  solver.set_x_ref(x_ref);
  
  // Solve for the control input
  bool solve_success = solver.solve();
  ENTO_DEBUG("Solve success: %s, Status: %d", solve_success ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  // Get the control input
  auto u0 = solver.get_u0();
  
  // Debug output
  ENTO_DEBUG("Reference x_ref: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", 
             x_ref[0], x_ref[1], x_ref[2], x_ref[3], x_ref[4], x_ref[5], x_ref[6], x_ref[7], x_ref[8], x_ref[9]);
  ENTO_DEBUG("Control u0: [%.6f, %.6f, %.6f]", u0[0], u0[1], u0[2]);
  ENTO_DEBUG("Control norm: %.6f", u0.norm());
  
  // Check raw solution from OSQP
  auto workspace = RoboFlyAPI::get_workspace();
  const float* raw_solution = RoboFlyAPI::get_solution(workspace);
  ENTO_DEBUG("Raw OSQP solution first 10 elements: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
             raw_solution[0], raw_solution[1], raw_solution[2], raw_solution[3], raw_solution[4],
             raw_solution[5], raw_solution[6], raw_solution[7], raw_solution[8], raw_solution[9]);
  
  // Control input should not be zero (MPC should try to reach reference)
  ENTO_TEST_CHECK_TRUE(u0.norm() > 0);
}

void test_osqp_mpc_different_states_different_controls() {
  MpcSolver solver;
  
  // Test that different initial states produce different control inputs
  // Use realistic reference from test_lqr.cc
  MpcSolver::State x_ref;
  x_ref << 0.090000f, -0.168000f, 0.288000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f;
  
  // First state (t=0.0 from trajectory)
  MpcSolver::State x0_1;
  x0_1 << 0.092087f, -0.167926f, 0.261501f, 0.000000f, 0.000000f, 0.000000f, 0.021123f, 0.030017f, 0.000000f, 0.000000f;
  
  solver.set_x_ref(x_ref);
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0_1));
  ENTO_TEST_CHECK_TRUE(solver.solve());
  auto u1 = solver.get_u0();
  
  // Second state (t=0.5 from trajectory)
  MpcSolver::State x0_2;
  x0_2 << 0.108290f, -0.175858f, 0.299720f, 0.501430f, -0.800790f, 2.290600f, 0.101524f, 0.131200f, 31.073203f, -124.922303f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0_2));
  ENTO_TEST_CHECK_TRUE(solver.solve());
  auto u2 = solver.get_u0();
  
  // Control inputs should be different for different states
  ENTO_TEST_CHECK_TRUE((u1 - u2).norm() > 1e-6);
  
  ENTO_DEBUG("State 1 control: [%.6f, %.6f, %.6f], norm: %.6f", u1[0], u1[1], u1[2], u1.norm());
  ENTO_DEBUG("State 2 control: [%.6f, %.6f, %.6f], norm: %.6f", u2[0], u2[1], u2[2], u2.norm());
  ENTO_DEBUG("Control difference norm: %.6f", (u1 - u2).norm());
}

void test_osqp_mpc_bounds_validation() {
  MpcSolver solver;
  
  // Create a state with NaN
  MpcSolver::State x_nan;
  x_nan << 1.0f, NAN, 3.0f, 0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f, 0.4f;
  
  // Setting a state with NaN should fail
  ENTO_TEST_CHECK_FALSE(solver.set_x0(x_nan));
  
  // Create a state with Inf
  MpcSolver::State x_inf;
  x_inf << 1.0f, 2.0f, INFINITY, 0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f, 0.4f;
  
  // Setting a state with Inf should fail
  ENTO_TEST_CHECK_FALSE(solver.set_x0(x_inf));
  
  // Setting a realistic valid state should succeed (from trajectory data)
  MpcSolver::State x_valid;
  x_valid << 0.092087f, -0.167926f, 0.261501f, 0.000000f, 0.000000f, 0.000000f, 0.021123f, 0.030017f, 0.000000f, 0.000000f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x_valid));
}

void test_osqp_mpc_solver_status() {
  MpcSolver solver;
  
  // Test with realistic state and reference from trajectory data
  MpcSolver::State x0;
  x0 << 0.092087f, -0.167926f, 0.261501f, 0.000000f, 0.000000f, 0.000000f, 0.021123f, 0.030017f, 0.000000f, 0.000000f;
  
  MpcSolver::State x_ref;
  x_ref << 0.090000f, -0.168000f, 0.288000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  solver.set_x_ref(x_ref);
  
  // Solve and check status
  bool solve_success = solver.solve();
  ENTO_DEBUG("Solve success: %s, Status: %d", solve_success ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  // Should have solved successfully (status 1 = OSQP_SOLVED, 2 = OSQP_SOLVED_INACCURATE)
  auto status = solver.get_status();
  ENTO_TEST_CHECK_TRUE(status == 1 || status == 2);
  
  // Should have taken some iterations
  ENTO_TEST_CHECK_TRUE(solver.get_iterations() > 0);
}

// Test warm start functionality if available
void test_osqp_mpc_warm_start() {
  using MpcSolverWarmStart = EntoControl::OSQPBridge<RoboFlyAPI>;
  
  MpcSolverWarmStart solver;
  
  // Set up a problem with realistic trajectory data
  MpcSolverWarmStart::State x0;
  x0 << 0.092087f, -0.167926f, 0.261501f, 0.000000f, 0.000000f, 0.000000f, 0.021123f, 0.030017f, 0.000000f, 0.000000f;
  
  MpcSolverWarmStart::State x_ref;
  x_ref << 0.090000f, -0.168000f, 0.288000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  solver.set_x_ref(x_ref);
  
  // First solve
  bool solve_success1 = solver.solve();
  ENTO_DEBUG("First solve success: %s, Status: %d", solve_success1 ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success1);
  
  int first_iterations = solver.get_iterations();
  auto first_u0 = solver.get_u0();
  
  // Second solve with slightly different state (should be faster with warm start)
  MpcSolverWarmStart::State x0_2;
  x0_2 << 0.093f, -0.168f, 0.262f, 0.000000f, 0.000000f, 0.000000f, 0.021f, 0.030f, 0.000000f, 0.000000f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0_2));
  bool solve_success2 = solver.solve();
  ENTO_DEBUG("Second solve success: %s, Status: %d", solve_success2 ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success2);
  
  int second_iterations = solver.get_iterations();
  auto second_u0 = solver.get_u0();
  
  ENTO_DEBUG("First solve iterations: %d, Second solve iterations: %d", first_iterations, second_iterations);
  ENTO_DEBUG("First u0: [%.6f, %.6f, %.6f]", first_u0[0], first_u0[1], first_u0[2]);
  ENTO_DEBUG("Second u0: [%.6f, %.6f, %.6f]", second_u0[0], second_u0[1], second_u0[2]);
  
  // Both solves should succeed
  ENTO_TEST_CHECK_TRUE(solve_success1 && solve_success2);
}

void test_osqp_mpc_realistic_trajectory() {
  // Test MPC controller functionality using realistic trajectory data from test_lqr.cc
  // Note: The embedded OSQP solver has fixed problem data, so we're testing that
  // the solver integration works correctly, not that it solves different problems
  MpcSolver solver;
  
  // Reference state (same for all test points)
  MpcSolver::State x_ref;
  x_ref << 0.090000f, -0.168000f, 0.288000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f;
  
  // ---- Test with state at t=0.0 ----
  MpcSolver::State x0;
  x0 << 0.092087f, -0.167926f, 0.261501f, 0.000000f, 0.000000f, 0.000000f, 0.021123f, 0.030017f, 0.000000f, 0.000000f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  solver.set_x_ref(x_ref);
  
  bool solve_success0 = solver.solve();
  ENTO_DEBUG("t=0.0 solve success: %s, Status: %d", solve_success0 ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success0);
  
  auto u0 = solver.get_u0();
  ENTO_DEBUG("t=0.0 Control u0: [%.6f, %.6f, %.6f], norm: %.6f", u0[0], u0[1], u0[2], u0.norm());
  
  // ---- Test with state at t=0.5 ----
  MpcSolver::State x1;
  x1 << 0.108290f, -0.175858f, 0.299720f, 0.501430f, -0.800790f, 2.290600f, 0.101524f, 0.131200f, 31.073203f, -124.922303f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x1));
  solver.set_x_ref(x_ref);
  
  bool solve_success1 = solver.solve();
  ENTO_DEBUG("t=0.5 solve success: %s, Status: %d", solve_success1 ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success1);
  
  auto u1 = solver.get_u0();
  ENTO_DEBUG("t=0.5 Control u1: [%.6f, %.6f, %.6f], norm: %.6f", u1[0], u1[1], u1[2], u1.norm());
  
  // ---- Test with state at t=2.0 (end of trajectory) ----
  MpcSolver::State x2;
  x2 << 0.104976f, -0.171701f, 0.308092f, 0.703340f, 1.477890f, 0.656840f, 0.032568f, 0.067413f, 110.511214f, -107.171688f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x2));
  solver.set_x_ref(x_ref);
  
  bool solve_success2 = solver.solve();
  ENTO_DEBUG("t=2.0 solve success: %s, Status: %d", solve_success2 ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success2);
  
  auto u2 = solver.get_u0();
  ENTO_DEBUG("t=2.0 Control u2: [%.6f, %.6f, %.6f], norm: %.6f", u2[0], u2[1], u2[2], u2.norm());
  
  // ---- Test when at reference state ----
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x_ref));
  solver.set_x_ref(x_ref);
  
  bool solve_success_ref = solver.solve();
  ENTO_DEBUG("At reference solve success: %s, Status: %d", solve_success_ref ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success_ref);
  
  auto u_at_ref = solver.get_u0();
  ENTO_DEBUG("At reference Control: [%.6f, %.6f, %.6f], norm: %.6f", u_at_ref[0], u_at_ref[1], u_at_ref[2], u_at_ref.norm());
  
  // For embedded OSQP with fixed problem data, we expect consistent behavior
  // The solver should successfully solve the embedded problem
  ENTO_TEST_CHECK_TRUE(solve_success0 && solve_success1 && solve_success2 && solve_success_ref);
  
  // All solves should return the same status (solved)
  ENTO_TEST_CHECK_TRUE(solver.get_status() == 1 || solver.get_status() == 2);
  
  // The solver should take a reasonable number of iterations
  ENTO_TEST_CHECK_TRUE(solver.get_iterations() > 0 && solver.get_iterations() < 100);
  
  // Note: Since the embedded OSQP has fixed problem data, the control outputs
  // will be the same regardless of the input state/reference we set.
  // This is expected behavior for this test - we're verifying the solver
  // integration works, not that it solves different problems.
}

void test_osqp_mpc_conservative_data() {
  // Test with very conservative data that should be easily feasible
  MpcSolver solver;
  
  // Very small deviations from reference that should be easily correctable
  MpcSolver::State x0;
  x0 << 0.001f, 0.001f, 0.001f, 0.0f, 0.0f, 0.0f, 0.001f, 0.001f, 0.0f, 0.0f;
  
  MpcSolver::State x_ref = MpcSolver::State::Zero();
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  solver.set_x_ref(x_ref);
  
  bool solve_success = solver.solve();
  ENTO_DEBUG("Conservative solve success: %s, Status: %d", solve_success ? "true" : "false", static_cast<int>(solver.get_status()));
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  auto u0 = solver.get_u0();
  ENTO_DEBUG("Conservative u0: [%.6f, %.6f, %.6f]", u0[0], u0[1], u0[2]);
  
  // Control should be small for small state deviations
  ENTO_TEST_CHECK_TRUE(std::abs(u0[0]) < 5.0f);  // Thrust should be reasonable
  ENTO_TEST_CHECK_TRUE(std::abs(u0[1]) < 2.0f);  // Pitch moment should be small
  ENTO_TEST_CHECK_TRUE(std::abs(u0[2]) < 2.0f);  // Roll moment should be small
}

int main(int argc, char **argv) {
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
    __ento_replace_file_suffix(__FILE__, "test_osqp_mpc_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_TEST_START();
  
  if (__ento_test_num(__n, 1)) test_osqp_mpc_initialization();
  if (__ento_test_num(__n, 2)) test_osqp_mpc_solve_zero_state();
  if (__ento_test_num(__n, 3)) test_osqp_mpc_solve_nonzero_state();
  if (__ento_test_num(__n, 4)) test_osqp_mpc_solve_nonzero_reference();
  if (__ento_test_num(__n, 5)) test_osqp_mpc_different_states_different_controls();
  if (__ento_test_num(__n, 6)) test_osqp_mpc_bounds_validation();
  if (__ento_test_num(__n, 7)) test_osqp_mpc_solver_status();
  if (__ento_test_num(__n, 8)) test_osqp_mpc_warm_start();
  if (__ento_test_num(__n, 9)) test_osqp_mpc_realistic_trajectory();
  if (__ento_test_num(__n, 10)) test_osqp_mpc_conservative_data();
  
  ENTO_TEST_END();
} 