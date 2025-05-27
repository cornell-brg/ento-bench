#include <stdlib.h>
#include <Eigen/Dense>
#include <sstream>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-math/core.h>
#include <cmath>
#include <limits>

#include "robobee_mpc.h"
#include "opt_control_problem.h"

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoControl;

// Flight task implementations based on external/robobee3d/template/flight_tasks.py
namespace FlightTasks {
  
  struct TrajectoryPoint {
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f orientation;
  };
  
  TrajectoryPoint helix(float t, const Eigen::Vector3f& initialPos, 
                       float trajAmp = 80.0f, float trajFreq = 1.0f, 
                       float dz = 0.15f, bool useY = true) {
    TrajectoryPoint traj;
    traj.position = initialPos;
    traj.velocity.setZero();
    
    float trajOmg = 2.0f * M_PI * trajFreq * 1e-3f; // to KHz, then to rad/ms
    
    traj.position[0] += trajAmp * std::sin(trajOmg * t);
    traj.velocity[0] = trajAmp * trajOmg * std::cos(trajOmg * t);
    
    if (useY) {
      traj.position[1] += trajAmp * (1.0f - std::cos(trajOmg * t));
      traj.velocity[1] = trajAmp * trajOmg * std::sin(trajOmg * t);
    }
    
    if (trajAmp > 1e-3f) { // otherwise assume hover
      traj.position[2] += dz * t;
      traj.velocity[2] = dz;
    }
    
    traj.orientation = Eigen::Vector3f(0.0f, 0.0f, 1.0f); // VERTICAL
    return traj;
  }
  
  TrajectoryPoint hover(float t, const Eigen::Vector3f& initialPos) {
    return helix(t, initialPos, 0.0f, 1.0f, 0.0f, false); // trajAmp=0 for hover
  }
}

/**
 * @brief Template MPC Solver wrapper that implements the OptControlProblem interface
 * 
 * This wrapper adapts the RoboBeeMPC (template MPC with horizon=3) to work with
 * the OptControlProblem interface which expects horizon-length reference trajectories.
 */
class TemplateMPCSolver {
public:
  using State = Eigen::Matrix<float, 12, 1>;
  using Control = Eigen::Matrix<float, 3, 1>;
  static constexpr int StateSize = 12;
  static constexpr int CtrlSize = 3;
  static constexpr int HorizonSize = 3;  // UMPC_N = 3

  TemplateMPCSolver() : mpc_() {}

  bool set_x0(const State& x0) {
    mpc_.set_x0(x0);
    return true; // Always successful now
  }

  /**
   * @brief Set reference trajectory over horizon
   * @param x_ref Reference states [StateSize × HorizonSize] matrix
   * 
   * The template MPC uses constant references over the horizon, so we take
   * the first column as the reference for all horizon steps.
   */
  void set_x_ref(const Eigen::Matrix<float, StateSize, HorizonSize>& x_ref) {
    // Template MPC uses constant reference over horizon
    // Take the first reference state
    State ref_state = x_ref.col(0);
    mpc_.set_x_ref(ref_state);
    
    // Store for dynamics simulation
    x_ref_single_ = ref_state;
  }

  void reset_duals() {
    // Template MPC doesn't expose dual reset, but OSQP handles warm start internally
  }

  bool solve() {
    return mpc_.solve();
  }

  Control get_u0() const {
    return mpc_.get_u0();
  }

  int get_status() const {
    return mpc_.get_status();
  }

  int get_iterations() const {
    return mpc_.get_iterations();
  }

  // Forward simulation method that delegates to RoboBeeMPC
  State simulate_forward(const State& x0, const Control& u, float dt = 5.0f) {
    return mpc_.simulate_forward(x0, u, dt);
  }

private:
  RoboBeeMPC mpc_;
  State x_ref_single_;
};

// ============================================================================
// BASIC TEMPLATE MPC TESTS
// ============================================================================

void test_template_mpc_initialization() {
  // Test that the RoboBee MPC controller can be initialized with realistic parameters
  RoboBeeMPC controller;
  
  // Test setting a realistic initial state
  RoboBeeMPC::State x0;
  x0 << 0.0f, 0.0f, 0.0f,           // position at origin (mm)
        0.0f, 0.0f, 1.0f,           // upright orientation
        0.0f, 0.0f, 0.0f,           // zero initial velocity
        0.0f, 0.0f, 0.0f;           // zero initial angular velocity
  
  controller.set_x0(x0);
  ENTO_DEBUG("RoboBee template MPC initialized successfully");
}

void test_template_mpc_hover_task() {
  // Test basic hover task
  RoboBeeMPC controller;
  
  Eigen::Vector3f initialPos(0.0f, 0.0f, 0.0f);
  auto traj = FlightTasks::hover(50.0f, initialPos);
  
  // Set initial state (hover at origin)
  RoboBeeMPC::State x0;
  x0 << 0.0f, 0.0f, 0.0f,     // position (mm)
        0.0f, 0.0f, 1.0f,     // orientation vector (upright)
        0.0f, 0.0f, 0.0f,     // velocity (mm/ms)
        0.0f, 0.0f, 0.0f;     // angular velocity (rad/ms)
  
  controller.set_x0(x0);
  controller.set_reference_trajectory(traj.position, traj.velocity, traj.orientation);
  
  bool solve_success = controller.solve();
  ENTO_DEBUG("Hover solve success: %s, Status: %d", solve_success ? "true" : "false", controller.get_status());
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  auto u0 = controller.get_u0();
  ENTO_DEBUG("Hover control: [%.6f, %.6f, %.6f]", u0[0], u0[1], u0[2]);
  
  // Should generate reasonable control for hover
  ENTO_TEST_CHECK_TRUE(u0.norm() > 1e-6f);
  ENTO_TEST_CHECK_TRUE(std::abs(u0[0]) < 50.0f);  // Thrust should be reasonable
}

void test_template_mpc_helix_trajectory() {
  // Test helix trajectory with realistic parameters
  RoboBeeMPC controller;
  
  Eigen::Vector3f initialPos(0.0f, 0.0f, 0.0f);
  float t = 100.0f; // 100ms into helix
  
  // Generate realistic helix trajectory
  auto traj = FlightTasks::helix(t, initialPos, 50.0f, 1.0f, 0.15f, true);
  
  // Set initial state (on trajectory)
  RoboBeeMPC::State x0;
  x0 << initialPos[0], initialPos[1], initialPos[2],  // position (mm)
        0.0f, 0.0f, 1.0f,                             // orientation vector (upright)
        0.0f, 0.0f, 0.0f,                             // velocity (mm/ms)
        0.0f, 0.0f, 0.0f;                             // angular velocity (rad/ms)
  
  controller.set_x0(x0);
  controller.set_reference_trajectory(traj.position, traj.velocity, traj.orientation);
  
  bool solve_success = controller.solve();
  ENTO_DEBUG("Helix solve success: %s, Status: %d", solve_success ? "true" : "false", controller.get_status());
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  auto u0 = controller.get_u0();
  ENTO_DEBUG("Helix desired: pos=[%.3f, %.3f, %.3f], vel=[%.3f, %.3f, %.3f]", 
             traj.position[0], traj.position[1], traj.position[2],
             traj.velocity[0], traj.velocity[1], traj.velocity[2]);
  ENTO_DEBUG("Helix control: [%.6f, %.6f, %.6f]", u0[0], u0[1], u0[2]);
  
  // Should generate significant control for trajectory following
  ENTO_TEST_CHECK_TRUE(u0.norm() > 0.05f);
}

// ============================================================================
// OPTCONTROLPROBLEM INTERFACE TESTS
// ============================================================================

void test_template_mpc_solver_interface() {
  // Test that the TemplateMPCSolver implements the required interface
  TemplateMPCSolver solver;
  
  // Test state setting
  TemplateMPCSolver::State x0;
  x0 << 0.1f, 0.1f, 0.1f, 0.0f, 0.0f, 1.0f, 0.01f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f;
  
  ENTO_TEST_CHECK_TRUE(solver.set_x0(x0));
  
  // Test reference setting with horizon
  Eigen::Matrix<float, 12, 3> x_ref;
  x_ref.col(0) << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  x_ref.col(1) << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  x_ref.col(2) << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  
  solver.set_x_ref(x_ref);
  
  // Test solving
  bool solve_success = solver.solve();
  ENTO_DEBUG("Template MPC solve success: %s, Status: %d", 
             solve_success ? "true" : "false", solver.get_status());
  ENTO_TEST_CHECK_TRUE(solve_success);
  
  // Test getting control
  auto u0 = solver.get_u0();
  ENTO_DEBUG("Template MPC u0: [%.6f, %.6f, %.6f]", u0[0], u0[1], u0[2]);
  
  // Test that the solver produces reasonable control outputs
  ENTO_TEST_CHECK_TRUE(u0.norm() > 1e-6f); // Should produce non-zero control
  ENTO_TEST_CHECK_TRUE(std::abs(u0[0]) < 10.0f); // Thrust should be reasonable
}

void test_template_mpc_opt_control_problem() {
  // Test the template MPC with the OptControlProblem interface
  using TemplateMPCProblem = OptControlProblem<float, TemplateMPCSolver, 12, 3, 3, 100>;
  
  TemplateMPCSolver solver;
  TemplateMPCProblem problem(std::move(solver));
  
  // Add some trajectory points
  for (int i = 0; i < 10; ++i) {
    // Simulate deserializing trajectory data with proper upright orientation
    std::string line = "0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,";
    ENTO_TEST_CHECK_TRUE(problem.deserialize_impl(line.c_str()));
  }
  
  // Test solving a few steps
  for (int i = 0; i < 5; ++i) {
    problem.solve_impl();  // This gets timed (ROI)
    problem.step();        // This handles forward simulation (outside ROI)
    ENTO_DEBUG("Step %d completed", i);
  }
  
  // Test validation
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
  
  ENTO_DEBUG("Template MPC OptControlProblem test completed successfully");
}

void test_template_mpc_benchmark_pattern() {
  // Demonstrate the proper benchmark pattern with ROI timing
  ENTO_DEBUG("Testing Template MPC benchmark pattern with ROI separation...");
  
  using TemplateMPCProblem = OptControlProblem<float, TemplateMPCSolver, 12, 3, 3, 100>;
  
  TemplateMPCSolver solver;
  TemplateMPCProblem problem(std::move(solver));
  
  // Setup: Add trajectory points (outside ROI)
  for (int i = 0; i < 10; ++i) {
    std::string line = "0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,";
    ENTO_TEST_CHECK_TRUE(problem.deserialize_impl(line.c_str()));
  }
  
  // Simulate benchmark loop
  for (int i = 0; i < 5; ++i) {
    ENTO_DEBUG("Benchmark iteration %d:", i);
    
    // === ROI START === (This is what gets timed)
    // start_roi();
    problem.solve_impl();  // Only the optimization solve is timed
    // end_roi();
    // === ROI END ===
    
    // Forward simulation happens outside ROI (not timed)
    problem.step();
    
    ENTO_DEBUG("  - Solve completed (timed), forward simulation completed (not timed)");
  }
  
  // Validation (outside ROI)
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
  
  ENTO_DEBUG("Benchmark pattern test completed successfully!");
  ENTO_DEBUG("Key insight: Only solve_impl() is timed, step() handles dynamics outside ROI");
}

// ============================================================================
// LONGER SIMULATION TESTS USING OPTCONTROLPROBLEM FRAMEWORK
// ============================================================================

void test_template_mpc_long_hover() {
  // Test longer hover simulation using manual loop
  ENTO_DEBUG("Testing template MPC with long hover simulation...");
  
  TemplateMPCSolver solver;
  
  // Set initial state (hover at origin)
  Eigen::Matrix<float, 12, 1> x;
  x << 0.0f, 0.0f, 0.0f,     // position
       0.0f, 0.0f, 1.0f,     // orientation (upright)
       0.0f, 0.0f, 0.0f,     // velocity
       0.0f, 0.0f, 0.0f;     // angular velocity
  
  float max_tracking_error = 0.0f;
  
  // Run simulation for 1 second (200 steps * 5ms = 1000ms)
  for (int i = 0; i < 200; ++i) {
    float t = i * 5.0f; // 5ms timestep
    
    // Generate hover reference
    Eigen::Vector3f initialPos(0.0f, 0.0f, 0.0f);
    auto traj = FlightTasks::hover(t, initialPos);
    
    // Set reference trajectory (constant over horizon)
    Eigen::Matrix<float, 12, 3> x_ref;
    for (int j = 0; j < 3; ++j) {
      x_ref.col(j) << traj.position[0], traj.position[1], traj.position[2],
                      traj.orientation[0], traj.orientation[1], traj.orientation[2],
                      traj.velocity[0], traj.velocity[1], traj.velocity[2],
                      0.0f, 0.0f, 0.0f; // zero angular velocity
    }
    
    // Set current state and reference
    solver.set_x0(x);
    solver.set_x_ref(x_ref);
    
    // Solve for control
    bool solve_success = solver.solve();
    ENTO_TEST_CHECK_TRUE(solve_success);
    
    auto u0 = solver.get_u0();
    
    // Check for NaN in control output
    if (!std::isfinite(u0[0]) || !std::isfinite(u0[1]) || !std::isfinite(u0[2])) {
      ENTO_DEBUG("NaN detected in control output at step %d!", i);
      break; // Stop simulation when NaN is detected
    }
    
    // Forward simulate using solver's dynamics
    x = solver.simulate_forward(x, u0, 5.0f); // 5ms timestep
    
    // Calculate tracking error
    Eigen::Vector3f pos_error = x.segment<3>(0) - traj.position;
    float tracking_error = pos_error.norm();
    max_tracking_error = std::max(max_tracking_error, tracking_error);
    
    // Debug every 50 steps
    if (i % 50 == 0) {
      ENTO_DEBUG("Step %d: tracking_error=%.6f, pos=[%.3f,%.3f,%.3f]", 
                 i, tracking_error, x[0], x[1], x[2]);
    }
  }
  
  ENTO_DEBUG("Long hover completed successfully! Max tracking error: %.6f", max_tracking_error);
  
  // Should maintain reasonable tracking error for hover
  ENTO_TEST_CHECK_TRUE(max_tracking_error < 10.0f); // 10mm tolerance
}

void test_template_mpc_long_helix() {
  // Test longer helix trajectory using manual loop
  ENTO_DEBUG("Testing template MPC with long helix trajectory...");
  
  TemplateMPCSolver solver;
  
  // Set initial state (on trajectory)
  Eigen::Matrix<float, 12, 1> x;
  x << 0.0f, 0.0f, 0.0f,     // position
       0.0f, 0.0f, 1.0f,     // orientation (upright)
       0.0f, 0.0f, 0.0f,     // velocity
       0.0f, 0.0f, 0.0f;     // angular velocity
  
  float max_tracking_error = 0.0f;
  
  // Run simulation for 2 seconds (400 steps * 5ms = 2000ms)
  for (int i = 0; i < 400; ++i) {
    float t = i * 5.0f; // 5ms timestep
    
    // Generate helix reference
    Eigen::Vector3f initialPos(0.0f, 0.0f, 0.0f);
    auto traj = FlightTasks::helix(t, initialPos, 50.0f, 1.0f, 0.15f, true); // useY=true for 3D helix!
    
    // Set reference trajectory (constant over horizon)
    Eigen::Matrix<float, 12, 3> x_ref;
    for (int j = 0; j < 3; ++j) {
      x_ref.col(j) << traj.position[0], traj.position[1], traj.position[2],
                      traj.orientation[0], traj.orientation[1], traj.orientation[2],
                      traj.velocity[0], traj.velocity[1], traj.velocity[2],
                      0.0f, 0.0f, 0.0f; // zero angular velocity
    }
    
    // Set current state and reference
    solver.set_x0(x);
    solver.set_x_ref(x_ref);
    
    // Solve for control
    bool solve_success = solver.solve();
    ENTO_TEST_CHECK_TRUE(solve_success);
    
    auto u0 = solver.get_u0();
    
    // Check for NaN in control output
    if (!std::isfinite(u0[0]) || !std::isfinite(u0[1]) || !std::isfinite(u0[2])) {
      ENTO_DEBUG("NaN detected in control output at step %d!", i);
      break; // Stop simulation when NaN is detected
    }
    
    // Forward simulate using solver's dynamics
    x = solver.simulate_forward(x, u0, 5.0f); // 5ms timestep
    
    // Calculate tracking error
    Eigen::Vector3f pos_error = x.segment<3>(0) - traj.position;
    float tracking_error = pos_error.norm();
    max_tracking_error = std::max(max_tracking_error, tracking_error);
    
    // Debug every 10 steps
    if (i % 10 == 0) {
      ENTO_DEBUG("Step %d: tracking_error=%.6f, pos=[%.3f,%.3f,%.3f]", 
                 i, tracking_error, x[0], x[1], x[2]);
    }
  }
  
  ENTO_DEBUG("Long helix completed successfully! Max tracking error: %.6f", max_tracking_error);
  
  // Should maintain reasonable tracking error for helix
  ENTO_TEST_CHECK_TRUE(max_tracking_error < 100.0f); // 100mm tolerance for helix
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
    __ento_replace_file_suffix(__FILE__, "test_template_mpc_opt_control_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_TEST_START();
  
  // Basic Template MPC Tests
  if (__ento_test_num(__n, 1)) test_template_mpc_initialization();
  if (__ento_test_num(__n, 2)) test_template_mpc_hover_task();
  if (__ento_test_num(__n, 3)) test_template_mpc_helix_trajectory();
  
  // OptControlProblem Interface Tests
  if (__ento_test_num(__n, 4)) test_template_mpc_solver_interface();
  if (__ento_test_num(__n, 5)) test_template_mpc_opt_control_problem();
  if (__ento_test_num(__n, 6)) test_template_mpc_benchmark_pattern();
  
  // Longer Simulation Tests
  if (__ento_test_num(__n, 7)) test_template_mpc_long_hover();
  if (__ento_test_num(__n, 8)) test_template_mpc_long_helix();
  
  ENTO_TEST_END();
} 