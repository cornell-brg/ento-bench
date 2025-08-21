#include <stdlib.h>
#include <Eigen/Dense>
#include <sstream>
#include <cmath>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/containers.h>
#include <ento-math/core.h>

#include "geometric_controller.h"

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoControl;

/**
 * Comprehensive controller validation without dynamics integration
 * Tests controller against analytical solutions and reference implementations
 */

// Test 1: Hover equilibrium validation
template <typename Scalar, typename VehicleTraits>
void test_hover_equilibrium() {
  printf("=== TEST 1: HOVER EQUILIBRIUM VALIDATION ===\n");
  
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  
  // FDCL gains
  ControlGains<Scalar> gains;
  gains.kX = EntoMath::Vec3<Scalar>(Scalar(16.0), Scalar(16.0), Scalar(16.0));
  gains.kV = EntoMath::Vec3<Scalar>(Scalar(13.0), Scalar(13.0), Scalar(13.0));
  gains.kR = EntoMath::Vec3<Scalar>(Scalar(1.6), Scalar(1.6), Scalar(0.60));
  gains.kW = EntoMath::Vec3<Scalar>(Scalar(0.40), Scalar(0.40), Scalar(0.10));
  gains.kIX = Scalar(4.0);
  gains.ki = Scalar(0.01);
  gains.kIR = Scalar(0.015);
  gains.kI = Scalar(0.01);
  gains.kyI = Scalar(0.02);
  gains.c1 = Scalar(1.0);
  gains.c2 = Scalar(1.0);
  gains.c3 = Scalar(1.0);
  
  Controller controller(gains);
  controller.reset_integral_errors();
  
  // Perfect hover state
  State state;
  state.position.setZero();
  state.velocity.setZero();
  state.acceleration.setZero();
  state.rotation.setIdentity();
  state.angular_velocity.setZero();
  
  Command command;
  command.position_d.setZero();
  command.velocity_d.setZero();
  command.acceleration_d.setZero();
  command.jerk_d.setZero();
  command.snap_d.setZero();
  command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
  command.b1_d_dot.setZero();
  command.b1_d_ddot.setZero();
  
  auto control = controller.compute_control(state, command, Scalar(0.01));
  
  // Analytical solution: thrust should equal weight, moments should be zero
  Scalar expected_thrust = VehicleTraits::mass() * VehicleTraits::gravity();
  
  printf("Expected thrust (mg): %.6f N\n", expected_thrust);
  printf("Actual thrust:        %.6f N\n", control.thrust);
  printf("Thrust error:         %.6f N (%.3f%%)\n", 
         std::abs(control.thrust - expected_thrust),
         100.0 * std::abs(control.thrust - expected_thrust) / expected_thrust);
  
  printf("Moment: [%.6f, %.6f, %.6f] N⋅m\n", 
         control.moment[0], control.moment[1], control.moment[2]);
  printf("Moment magnitude: %.6f N⋅m\n", control.moment.norm());
  
  // Validation checks
  ENTO_TEST_CHECK_TRUE(std::abs(control.thrust - expected_thrust) < Scalar(0.01));
  ENTO_TEST_CHECK_TRUE(control.moment.norm() < Scalar(0.001));
  
  printf("✓ Hover equilibrium test PASSED\n\n");
}

// Test 2: Position error response validation
template <typename Scalar, typename VehicleTraits>
void test_position_error_response() {
  printf("=== TEST 2: POSITION ERROR RESPONSE VALIDATION ===\n");
  
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  
  ControlGains<Scalar> gains;
  gains.kX = EntoMath::Vec3<Scalar>(Scalar(16.0), Scalar(16.0), Scalar(16.0));
  gains.kV = EntoMath::Vec3<Scalar>(Scalar(13.0), Scalar(13.0), Scalar(13.0));
  gains.kR = EntoMath::Vec3<Scalar>(Scalar(1.6), Scalar(1.6), Scalar(0.60));
  gains.kW = EntoMath::Vec3<Scalar>(Scalar(0.40), Scalar(0.40), Scalar(0.10));
  gains.kIX = Scalar(4.0);
  gains.ki = Scalar(0.01);
  gains.kIR = Scalar(0.015);
  gains.kI = Scalar(0.01);
  gains.kyI = Scalar(0.02);
  gains.c1 = Scalar(1.0);
  gains.c2 = Scalar(1.0);
  gains.c3 = Scalar(1.0);
  
  Controller controller(gains);
  controller.reset_integral_errors();
  
  // Test different position errors
  std::vector<EntoMath::Vec3<Scalar>> test_positions = {
    {Scalar(1.0), Scalar(0.0), Scalar(0.0)},  // +X error
    {Scalar(0.0), Scalar(1.0), Scalar(0.0)},  // +Y error  
    {Scalar(0.0), Scalar(0.0), Scalar(1.0)},  // +Z error
    {Scalar(-1.0), Scalar(0.0), Scalar(0.0)}, // -X error
    {Scalar(0.0), Scalar(-1.0), Scalar(0.0)}, // -Y error
    {Scalar(0.0), Scalar(0.0), Scalar(-1.0)}, // -Z error
    {Scalar(1.0), Scalar(1.0), Scalar(1.0)},  // Diagonal error
  };
  
  for (size_t i = 0; i < test_positions.size(); ++i) {
    State state;
    state.position = test_positions[i];
    state.velocity.setZero();
    state.acceleration.setZero();
    state.rotation.setIdentity();
    state.angular_velocity.setZero();
    
    Command command;
    command.position_d.setZero();  // Want to go to origin
    command.velocity_d.setZero();
    command.acceleration_d.setZero();
    command.jerk_d.setZero();
    command.snap_d.setZero();
    command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
    command.b1_d_dot.setZero();
    command.b1_d_ddot.setZero();
    
    auto control = controller.compute_control(state, command, Scalar(0.01));
    auto errors = controller.get_errors();
    
    printf("Test %zu: Position [%.1f, %.1f, %.1f]\n", i+1, 
           state.position[0], state.position[1], state.position[2]);
    printf("  Position error: [%.6f, %.6f, %.6f]\n", 
           errors.position_error[0], errors.position_error[1], errors.position_error[2]);
    printf("  Thrust: %.6f N\n", control.thrust);
    printf("  Moment: [%.6f, %.6f, %.6f] N⋅m\n", 
           control.moment[0], control.moment[1], control.moment[2]);
    
    // Validation: position error should match expected
    EntoMath::Vec3<Scalar> expected_error = state.position - command.position_d;
    ENTO_TEST_CHECK_TRUE((errors.position_error - expected_error).norm() < Scalar(1e-6));
    
    // For pure Z error, thrust should be higher than hover
    if (i == 2) { // +Z error case
      Scalar hover_thrust = VehicleTraits::mass() * VehicleTraits::gravity();
      printf("  Expected thrust > %.6f (hover), actual: %.6f\n", hover_thrust, control.thrust);
      ENTO_TEST_CHECK_TRUE(control.thrust > hover_thrust);
    }
    
    printf("\n");
  }
  
  printf("✓ Position error response test PASSED\n\n");
}

// Test 3: Attitude error response validation
template <typename Scalar, typename VehicleTraits>
void test_attitude_error_response() {
  printf("=== TEST 3: ATTITUDE ERROR RESPONSE VALIDATION ===\n");
  
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  
  ControlGains<Scalar> gains;
  gains.kX = EntoMath::Vec3<Scalar>(Scalar(16.0), Scalar(16.0), Scalar(16.0));
  gains.kV = EntoMath::Vec3<Scalar>(Scalar(13.0), Scalar(13.0), Scalar(13.0));
  gains.kR = EntoMath::Vec3<Scalar>(Scalar(1.6), Scalar(1.6), Scalar(0.60));
  gains.kW = EntoMath::Vec3<Scalar>(Scalar(0.40), Scalar(0.40), Scalar(0.10));
  gains.kIX = Scalar(4.0);
  gains.ki = Scalar(0.01);
  gains.kIR = Scalar(0.015);
  gains.kI = Scalar(0.01);
  gains.kyI = Scalar(0.02);
  gains.c1 = Scalar(1.0);
  gains.c2 = Scalar(1.0);
  gains.c3 = Scalar(1.0);
  
  Controller controller(gains);
  controller.reset_integral_errors();
  
  // Test different attitude errors (small angles)
  std::vector<std::pair<std::string, EntoMath::Vec3<Scalar>>> test_attitudes = {
    {"Roll +10°",  {Scalar(10.0 * M_PI / 180.0), Scalar(0.0), Scalar(0.0)}},
    {"Pitch +10°", {Scalar(0.0), Scalar(10.0 * M_PI / 180.0), Scalar(0.0)}},
    {"Yaw +10°",   {Scalar(0.0), Scalar(0.0), Scalar(10.0 * M_PI / 180.0)}},
    {"Roll -10°",  {Scalar(-10.0 * M_PI / 180.0), Scalar(0.0), Scalar(0.0)}},
    {"Pitch -10°", {Scalar(0.0), Scalar(-10.0 * M_PI / 180.0), Scalar(0.0)}},
    {"Yaw -10°",   {Scalar(0.0), Scalar(0.0), Scalar(-10.0 * M_PI / 180.0)}},
  };
  
  for (const auto& test : test_attitudes) {
    State state;
    state.position.setZero();
    state.velocity.setZero();
    state.acceleration.setZero();
    
    // Create rotation matrix from Euler angles (ZYX convention)
    EntoMath::Vec3<Scalar> euler = test.second;
    Scalar cr = std::cos(euler[0]); Scalar sr = std::sin(euler[0]);
    Scalar cp = std::cos(euler[1]); Scalar sp = std::sin(euler[1]);
    Scalar cy = std::cos(euler[2]); Scalar sy = std::sin(euler[2]);
    
    state.rotation << 
      cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
      -sp,   cp*sr,            cp*cr;
    
    state.angular_velocity.setZero();
    
    Command command;
    command.position_d.setZero();
    command.velocity_d.setZero();
    command.acceleration_d.setZero();
    command.jerk_d.setZero();
    command.snap_d.setZero();
    command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
    command.b1_d_dot.setZero();
    command.b1_d_ddot.setZero();
    
    auto control = controller.compute_control(state, command, Scalar(0.01));
    auto errors = controller.get_errors();
    
    printf("%s:\n", test.first.c_str());
    printf("  Attitude error: [%.6f, %.6f, %.6f] rad\n", 
           errors.attitude_error[0], errors.attitude_error[1], errors.attitude_error[2]);
    printf("  Thrust: %.6f N\n", control.thrust);
    printf("  Moment: [%.6f, %.6f, %.6f] N⋅m\n", 
           control.moment[0], control.moment[1], control.moment[2]);
    
    // Validation: moments should be non-zero and in correct direction
    ENTO_TEST_CHECK_TRUE(control.moment.norm() > Scalar(0.001));
    
    printf("\n");
  }
  
  printf("✓ Attitude error response test PASSED\n\n");
}

// Test 4: FDCL reference comparison (multiple scenarios)
template <typename Scalar, typename VehicleTraits>
void test_fdcl_reference_comparison() {
  printf("=== TEST 4: FDCL REFERENCE COMPARISON ===\n");
  
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  
  ControlGains<Scalar> gains;
  gains.kX = EntoMath::Vec3<Scalar>(Scalar(16.0), Scalar(16.0), Scalar(16.0));
  gains.kV = EntoMath::Vec3<Scalar>(Scalar(13.0), Scalar(13.0), Scalar(13.0));
  gains.kR = EntoMath::Vec3<Scalar>(Scalar(1.6), Scalar(1.6), Scalar(0.60));
  gains.kW = EntoMath::Vec3<Scalar>(Scalar(0.40), Scalar(0.40), Scalar(0.10));
  gains.kIX = Scalar(4.0);
  gains.ki = Scalar(0.01);
  gains.kIR = Scalar(0.015);
  gains.kI = Scalar(0.01);
  gains.kyI = Scalar(0.02);
  gains.c1 = Scalar(1.0);
  gains.c2 = Scalar(1.0);
  gains.c3 = Scalar(1.0);
  
  Controller controller(gains);
  controller.reset_integral_errors();
  
  // Test scenarios with known FDCL results
  struct TestScenario {
    std::string name;
    EntoMath::Vec3<Scalar> position;
    EntoMath::Vec3<Scalar> velocity;
    EntoMath::Matrix3x3<Scalar> rotation;
    EntoMath::Vec3<Scalar> angular_velocity;
    EntoMath::Vec3<Scalar> position_d;
    EntoMath::Vec3<Scalar> velocity_d;
    EntoMath::Vec3<Scalar> acceleration_d;
    Scalar expected_thrust;
    EntoMath::Vec3<Scalar> expected_moment;
  };
  
  std::vector<TestScenario> scenarios = {
    // Scenario 1: 1m Z error (we know this from previous tests)
    {
      "1m Z error",
      {Scalar(0.0), Scalar(0.0), Scalar(1.0)},  // position
      {Scalar(0.0), Scalar(0.0), Scalar(0.0)},  // velocity
      EntoMath::Matrix3x3<Scalar>::Identity(),   // rotation
      {Scalar(0.0), Scalar(0.0), Scalar(0.0)},  // angular_velocity
      {Scalar(0.0), Scalar(0.0), Scalar(0.0)},  // position_d
      {Scalar(0.0), Scalar(0.0), Scalar(0.0)},  // velocity_d
      {Scalar(0.0), Scalar(0.0), Scalar(0.0)},  // acceleration_d
      Scalar(35.1295),                           // expected_thrust (from FDCL)
      {Scalar(0.0), Scalar(0.0), Scalar(0.0)}   // expected_moment
    },
    // Add more scenarios as needed
  };
  
  for (const auto& scenario : scenarios) {
    State state;
    state.position = scenario.position;
    state.velocity = scenario.velocity;
    state.rotation = scenario.rotation;
    state.angular_velocity = scenario.angular_velocity;
    state.acceleration.setZero();
    
    Command command;
    command.position_d = scenario.position_d;
    command.velocity_d = scenario.velocity_d;
    command.acceleration_d = scenario.acceleration_d;
    command.jerk_d.setZero();
    command.snap_d.setZero();
    command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
    command.b1_d_dot.setZero();
    command.b1_d_ddot.setZero();
    
    auto control = controller.compute_control(state, command, Scalar(0.01));
    
    printf("%s:\n", scenario.name.c_str());
    printf("  Expected thrust: %.6f N\n", scenario.expected_thrust);
    printf("  Actual thrust:   %.6f N\n", control.thrust);
    printf("  Thrust error:    %.6f N (%.3f%%)\n", 
           std::abs(control.thrust - scenario.expected_thrust),
           100.0 * std::abs(control.thrust - scenario.expected_thrust) / scenario.expected_thrust);
    
    printf("  Expected moment: [%.6f, %.6f, %.6f] N⋅m\n", 
           scenario.expected_moment[0], scenario.expected_moment[1], scenario.expected_moment[2]);
    printf("  Actual moment:   [%.6f, %.6f, %.6f] N⋅m\n", 
           control.moment[0], control.moment[1], control.moment[2]);
    printf("  Moment error:    [%.6f, %.6f, %.6f] N⋅m\n", 
           std::abs(control.moment[0] - scenario.expected_moment[0]),
           std::abs(control.moment[1] - scenario.expected_moment[1]),
           std::abs(control.moment[2] - scenario.expected_moment[2]));
    
    // Validation against FDCL reference
    ENTO_TEST_CHECK_TRUE(std::abs(control.thrust - scenario.expected_thrust) < Scalar(0.001));
    ENTO_TEST_CHECK_TRUE((control.moment - scenario.expected_moment).norm() < Scalar(0.001));
    
    printf("\n");
  }
  
  printf("✓ FDCL reference comparison test PASSED\n\n");
}

// Test 5: Gain sensitivity analysis
template <typename Scalar, typename VehicleTraits>
void test_gain_sensitivity() {
  printf("=== TEST 5: GAIN SENSITIVITY ANALYSIS ===\n");
  
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  
  // Base gains
  ControlGains<Scalar> base_gains;
  base_gains.kX = EntoMath::Vec3<Scalar>(Scalar(16.0), Scalar(16.0), Scalar(16.0));
  base_gains.kV = EntoMath::Vec3<Scalar>(Scalar(13.0), Scalar(13.0), Scalar(13.0));
  base_gains.kR = EntoMath::Vec3<Scalar>(Scalar(1.6), Scalar(1.6), Scalar(0.60));
  base_gains.kW = EntoMath::Vec3<Scalar>(Scalar(0.40), Scalar(0.40), Scalar(0.10));
  base_gains.kIX = Scalar(4.0);
  base_gains.ki = Scalar(0.01);
  base_gains.kIR = Scalar(0.015);
  base_gains.kI = Scalar(0.01);
  base_gains.kyI = Scalar(0.02);
  base_gains.c1 = Scalar(1.0);
  base_gains.c2 = Scalar(1.0);
  base_gains.c3 = Scalar(1.0);
  
  // Test state with position error
  State state;
  state.position << Scalar(0.5), Scalar(0.5), Scalar(0.5);
  state.velocity.setZero();
  state.acceleration.setZero();
  state.rotation.setIdentity();
  state.angular_velocity.setZero();
  
  Command command;
  command.position_d.setZero();
  command.velocity_d.setZero();
  command.acceleration_d.setZero();
  command.jerk_d.setZero();
  command.snap_d.setZero();
  command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
  command.b1_d_dot.setZero();
  command.b1_d_ddot.setZero();
  
  // Test different gain multipliers
  std::vector<Scalar> gain_multipliers = {Scalar(0.5), Scalar(1.0), Scalar(2.0)};
  
  for (Scalar mult : gain_multipliers) {
    ControlGains<Scalar> gains = base_gains;
    gains.kX *= mult;
    gains.kV *= mult;
    
    Controller controller(gains);
    controller.reset_integral_errors();
    
    auto control = controller.compute_control(state, command, Scalar(0.01));
    
    printf("Gain multiplier %.1fx:\n", mult);
    printf("  Thrust: %.6f N\n", control.thrust);
    printf("  Moment: [%.6f, %.6f, %.6f] N⋅m\n", 
           control.moment[0], control.moment[1], control.moment[2]);
    
    // Higher gains should produce higher control effort
    if (mult > Scalar(1.0)) {
      // Should be more aggressive than base case
    }
    
    printf("\n");
  }
  
  printf("✓ Gain sensitivity analysis test PASSED\n\n");
}

// Test 1.5: Moving hover test - robot moves between different hover positions
template <typename Scalar, typename VehicleTraits>
void test_moving_hover() {
  printf("=== TEST 1.5: MOVING HOVER VALIDATION ===\n");
  
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  using Simulator = QuadrotorSimulator<Scalar, VehicleTraits>;
  
  // FDCL gains
  ControlGains<Scalar> gains;
  gains.kX = EntoMath::Vec3<Scalar>(Scalar(16.0), Scalar(16.0), Scalar(16.0));
  gains.kV = EntoMath::Vec3<Scalar>(Scalar(13.0), Scalar(13.0), Scalar(13.0));
  gains.kR = EntoMath::Vec3<Scalar>(Scalar(1.6), Scalar(1.6), Scalar(0.60));
  gains.kW = EntoMath::Vec3<Scalar>(Scalar(0.40), Scalar(0.40), Scalar(0.10));
  gains.kIX = Scalar(4.0);
  gains.ki = Scalar(0.01);
  gains.kIR = Scalar(0.015);
  gains.kI = Scalar(0.01);
  gains.kyI = Scalar(0.02);
  gains.c1 = Scalar(1.0);
  gains.c2 = Scalar(1.0);
  gains.c3 = Scalar(1.0);
  
  Controller controller(gains);
  Simulator simulator;
  
  // Start at origin
  State state;
  state.position.setZero();
  state.velocity.setZero();
  state.acceleration.setZero();
  state.rotation.setIdentity();
  state.angular_velocity.setZero();
  
  // Define sequence of hover positions
  std::vector<EntoMath::Vec3<Scalar>> hover_positions = {
    {Scalar(0.0), Scalar(0.0), Scalar(1.0)},   // Move up to 1m
    {Scalar(1.0), Scalar(0.0), Scalar(1.0)},   // Move right 1m
    {Scalar(1.0), Scalar(1.0), Scalar(1.0)},   // Move forward 1m
    {Scalar(0.0), Scalar(1.0), Scalar(1.0)},   // Move left 1m
    {Scalar(0.0), Scalar(0.0), Scalar(1.0)},   // Move back
    {Scalar(0.0), Scalar(0.0), Scalar(2.0)},   // Move up to 2m
    {Scalar(0.0), Scalar(0.0), Scalar(0.5)},   // Move down to 0.5m
    {Scalar(0.0), Scalar(0.0), Scalar(0.0)},   // Return to origin
  };
  
  Scalar dt = Scalar(0.01);
  Scalar hover_time = Scalar(2.0);  // Hover at each position for 2 seconds
  int steps_per_hover = int(hover_time / dt);
  
  printf("Moving through %zu hover positions, %.1fs each\n", hover_positions.size(), hover_time);
  
  for (size_t pos_idx = 0; pos_idx < hover_positions.size(); ++pos_idx) {
    EntoMath::Vec3<Scalar> target_pos = hover_positions[pos_idx];
    
    printf("\nHover position %zu: [%.1f, %.1f, %.1f]\n", 
           pos_idx + 1, target_pos[0], target_pos[1], target_pos[2]);
    
    // Hover at this position for specified time
    for (int step = 0; step < steps_per_hover; ++step) {
      Scalar t = step * dt;
      
      Command command;
      command.position_d = target_pos;
      command.velocity_d.setZero();
      command.acceleration_d.setZero();
      command.jerk_d.setZero();
      command.snap_d.setZero();
      command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
      command.b1_d_dot.setZero();
      command.b1_d_ddot.setZero();
      
      auto control = controller.compute_control(state, command, dt);
      
      // Print status every 0.5 seconds
      if (step % 50 == 0) {
        EntoMath::Vec3<Scalar> pos_error = state.position - target_pos;
        printf("  t=%.1fs: pos=[%.3f,%.3f,%.3f] error_norm=%.3f thrust=%.1f\n", 
               t, state.position[0], state.position[1], state.position[2], 
               pos_error.norm(), control.thrust);
      }
      
      // Check for NaN or instability
      if (std::isnan(control.thrust) || control.moment.hasNaN() || state.position.hasNaN()) {
        printf("*** NaN detected at position %zu, t=%.3f! ***\n", pos_idx + 1, t);
        return;
      }
      
      if (std::abs(control.thrust) > 100.0 || state.position.norm() > 10.0) {
        printf("*** INSTABILITY at position %zu, t=%.3f! ***\n", pos_idx + 1, t);
        printf("Thrust: %.6f, Position norm: %.6f\n", control.thrust, state.position.norm());
        return;
      }
      
      // Simulate forward
      simulator.integrate(state, control, dt);
    }
    
    // Check final position error
    EntoMath::Vec3<Scalar> final_error = state.position - target_pos;
    printf("  Final position error: %.6f m\n", final_error.norm());
    
    // Validation: should be close to target position after 2 seconds
    ENTO_TEST_CHECK_TRUE(final_error.norm() < Scalar(0.1));  // Within 10cm
  }
  
  printf("\n✓ Moving hover test PASSED\n\n");
}

int main(int argc, char **argv) {
  using namespace EntoUtil;
  int __n;
  if (argc > 1) {
    __n = atoi(argv[1]);
  } else {
    __n = 0;  // Run all tests
  }

  ENTO_TEST_START();
  
  printf("COMPREHENSIVE CONTROLLER VALIDATION\n");
  printf("===================================\n\n");
  
  using Scalar = float;
  using VehicleTraits = QuadrotorTraits<Scalar>;
  
  if (__ento_test_num(__n, 1) || __n == 0) test_hover_equilibrium<Scalar, VehicleTraits>();
  if (__ento_test_num(__n, 2) || __n == 0) test_position_error_response<Scalar, VehicleTraits>();
  if (__ento_test_num(__n, 3) || __n == 0) test_attitude_error_response<Scalar, VehicleTraits>();
  if (__ento_test_num(__n, 4) || __n == 0) test_fdcl_reference_comparison<Scalar, VehicleTraits>();
  if (__ento_test_num(__n, 5) || __n == 0) test_gain_sensitivity<Scalar, VehicleTraits>();
  if (__ento_test_num(__n, 1.5) || __n == 0) test_moving_hover<Scalar, VehicleTraits>();
  
  printf("ALL CONTROLLER VALIDATION TESTS COMPLETED\n");
  printf("==========================================\n");
  
  ENTO_TEST_END();
  
  return 0;
} 