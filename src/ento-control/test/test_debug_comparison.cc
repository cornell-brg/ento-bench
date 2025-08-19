#include <stdlib.h>
#include <Eigen/Dense>
#include <sstream>
#include <cmath>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json/json.h>

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
 * Debug comparison test that outputs detailed intermediate values
 * for comparison with FDCL Python implementation
 */
void test_debug_single_step_comparison() {
  using Scalar = float;
  using VehicleTraits = QuadrotorTraits<Scalar>;
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  
  printf("=== C++ DEBUG SINGLE STEP COMPARISON ===\n");
  
  // Create controller with FDCL gains (EXACT same as Python)
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
  
  printf("Vehicle parameters: mass=%.2fkg, gravity=%.2fm/s^2\n", 
         VehicleTraits::mass(), VehicleTraits::gravity());
  printf("Gains: kx=[%.1f,%.1f,%.1f], kv=[%.1f,%.1f,%.1f], kR=[%.1f,%.1f,%.2f], kW=[%.2f,%.2f,%.2f]\n",
         gains.kX[0], gains.kX[1], gains.kX[2],
         gains.kV[0], gains.kV[1], gains.kV[2],
         gains.kR[0], gains.kR[1], gains.kR[2],
         gains.kW[0], gains.kW[1], gains.kW[2]);
  
  Controller controller(gains);
  
  // Reset integral errors to match FDCL behavior (they start from zero)
  controller.reset_integral_errors();
  
  // Set up state: 1 meter position error in Z (EXACT same test as FDCL)
  State state;
  state.position << Scalar(0.0), Scalar(0.0), Scalar(1.0);  // Current position
  state.velocity.setZero();          // Zero velocity
  state.acceleration.setZero();
  state.rotation.setIdentity();      // Identity rotation
  state.angular_velocity.setZero();  // Zero angular velocity
  
  // Set up command: desired position at origin
  Command command;
  command.position_d = EntoMath::Vec3<Scalar>(Scalar(0), Scalar(0), Scalar(0));  // Desired position at origin
  command.velocity_d.setZero();   // Zero desired velocity
  command.acceleration_d.setZero();  // Zero desired acceleration
  command.jerk_d.setZero();
  command.snap_d.setZero();
  command.b1_d = EntoMath::Vec3<Scalar>(Scalar(1.0), Scalar(0.0), Scalar(0.0));  // Desired heading
  command.b1_d_dot.setZero();
  command.b1_d_ddot.setZero();
  
  printf("\n=== DEBUG POSITION CONTROL at t=0.000000 ===\n");
  
  // Compute errors manually to match Python output
  EntoMath::Vec3<Scalar> position_error = state.position - command.position_d;
  EntoMath::Vec3<Scalar> velocity_error = state.velocity - command.velocity_d;
  
  printf("Position error: ex = [%.6f, %.6f, %.6f]\n", 
         position_error[0], position_error[1], position_error[2]);
  printf("Velocity error: ev = [%.6f, %.6f, %.6f]\n", 
         velocity_error[0], velocity_error[1], velocity_error[2]);
  
  // Compute intermediate terms
  EntoMath::Vec3<Scalar> kx_ex = gains.kX.cwiseProduct(position_error);
  EntoMath::Vec3<Scalar> kv_ev = gains.kV.cwiseProduct(velocity_error);
  EntoMath::Vec3<Scalar> mg_e3(Scalar(0), Scalar(0), VehicleTraits::mass() * VehicleTraits::gravity());
  EntoMath::Vec3<Scalar> m_xd_2dot = VehicleTraits::mass() * command.acceleration_d;
  
  printf("kx ⊙ ex = [%.6f, %.6f, %.6f]\n", kx_ex[0], kx_ex[1], kx_ex[2]);
  printf("kv ⊙ ev = [%.6f, %.6f, %.6f]\n", kv_ev[0], kv_ev[1], kv_ev[2]);
  printf("mg*e3 = [%.6f, %.6f, %.6f]\n", mg_e3[0], mg_e3[1], mg_e3[2]);
  printf("m*xd_2dot = [%.6f, %.6f, %.6f]\n", m_xd_2dot[0], m_xd_2dot[1], m_xd_2dot[2]);
  
  // Compute A vector (desired acceleration in inertial frame)
  EntoMath::Vec3<Scalar> A = kx_ex + kv_ev + mg_e3 - m_xd_2dot;
  printf("A = [%.6f, %.6f, %.6f] (norm=%.6f)\n", A[0], A[1], A[2], A.norm());
  
  // Compute thrust
  EntoMath::Vec3<Scalar> e3(Scalar(0), Scalar(0), Scalar(1));
  EntoMath::Vec3<Scalar> R_e3 = state.rotation * e3;
  printf("R*e3 = [%.6f, %.6f, %.6f]\n", R_e3[0], R_e3[1], R_e3[2]);
  
  Scalar f = A.dot(R_e3);
  printf("Thrust f = A·(R*e3) = %.6f\n", f);
  
  // Compute desired thrust direction
  EntoMath::Vec3<Scalar> b3_desired = -A / A.norm();  // Negative because thrust opposes A
  printf("b3_desired = [%.6f, %.6f, %.6f]\n", b3_desired[0], b3_desired[1], b3_desired[2]);
  
  // For attitude control, we'll use identity for simplicity (same as Python)
  EntoMath::Matrix3x3<Scalar> Rd = EntoMath::Matrix3x3<Scalar>::Identity();
  EntoMath::Vec3<Scalar> Wd = EntoMath::Vec3<Scalar>::Zero();
  
  // Compute attitude errors (using FDCL formulation)
  EntoMath::Matrix3x3<Scalar> eR_matrix = Scalar(0.5) * (Rd.transpose() * state.rotation - state.rotation.transpose() * Rd);
  EntoMath::Vec3<Scalar> eR(eR_matrix(2,1), eR_matrix(0,2), eR_matrix(1,0));  // vee map
  EntoMath::Vec3<Scalar> eW = state.angular_velocity - state.rotation.transpose() * (Rd * Wd);
  
  printf("Attitude error: eR = [%.6f, %.6f, %.6f]\n", eR[0], eR[1], eR[2]);
  printf("Angular vel error: eW = [%.6f, %.6f, %.6f]\n", eW[0], eW[1], eW[2]);
  
  // Compute moment
  EntoMath::Vec3<Scalar> kR_eR = gains.kR.cwiseProduct(eR);
  EntoMath::Vec3<Scalar> kW_eW = gains.kW.cwiseProduct(eW);
  
  // Gyroscopic term: ω × (J * ω)
  EntoMath::Vec3<Scalar> J_omega = VehicleTraits::inertia_matrix() * state.angular_velocity;
  EntoMath::Vec3<Scalar> gyroscopic = state.angular_velocity.cross(J_omega);
  
  printf("kR ⊙ eR = [%.6f, %.6f, %.6f]\n", kR_eR[0], kR_eR[1], kR_eR[2]);
  printf("kW ⊙ eW = [%.6f, %.6f, %.6f]\n", kW_eW[0], kW_eW[1], kW_eW[2]);
  printf("Gyroscopic = [%.6f, %.6f, %.6f]\n", gyroscopic[0], gyroscopic[1], gyroscopic[2]);
  
  EntoMath::Vec3<Scalar> M = -kR_eR - kW_eW + gyroscopic;
  printf("Moment M = [%.6f, %.6f, %.6f]\n", M[0], M[1], M[2]);
  
  // Now run the actual controller and compare
  printf("\n=== ACTUAL CONTROLLER OUTPUT ===\n");
  auto control_output = controller.compute_control(state, command, Scalar(0.01));
  auto errors = controller.get_errors();
  
  printf("Controller thrust: %.6f\n", control_output.thrust);
  printf("Controller moment: [%.6f, %.6f, %.6f]\n", 
         control_output.moment[0], control_output.moment[1], control_output.moment[2]);
  
  printf("\n=== COMPARISON ===\n");
  printf("Manual thrust: %.6f vs Controller thrust: %.6f (diff: %.6f)\n", 
         f, control_output.thrust, std::abs(f - control_output.thrust));
  printf("Manual moment: [%.6f, %.6f, %.6f]\n", M[0], M[1], M[2]);
  printf("Controller moment: [%.6f, %.6f, %.6f]\n", 
         control_output.moment[0], control_output.moment[1], control_output.moment[2]);
  printf("Moment diff: [%.6f, %.6f, %.6f]\n", 
         std::abs(M[0] - control_output.moment[0]),
         std::abs(M[1] - control_output.moment[1]),
         std::abs(M[2] - control_output.moment[2]));
  
  // Save debug data to JSON for comparison with Python
  Json::Value debug_data;
  debug_data["time"] = 0.0;
  debug_data["state"]["position"] = Json::arrayValue;
  debug_data["state"]["position"].append(state.position[0]);
  debug_data["state"]["position"].append(state.position[1]);
  debug_data["state"]["position"].append(state.position[2]);
  
  debug_data["state"]["velocity"] = Json::arrayValue;
  debug_data["state"]["velocity"].append(state.velocity[0]);
  debug_data["state"]["velocity"].append(state.velocity[1]);
  debug_data["state"]["velocity"].append(state.velocity[2]);
  
  debug_data["reference"]["position"] = Json::arrayValue;
  debug_data["reference"]["position"].append(command.position_d[0]);
  debug_data["reference"]["position"].append(command.position_d[1]);
  debug_data["reference"]["position"].append(command.position_d[2]);
  
  debug_data["errors"]["position_error"] = Json::arrayValue;
  debug_data["errors"]["position_error"].append(position_error[0]);
  debug_data["errors"]["position_error"].append(position_error[1]);
  debug_data["errors"]["position_error"].append(position_error[2]);
  
  debug_data["intermediate"]["A_vector"] = Json::arrayValue;
  debug_data["intermediate"]["A_vector"].append(A[0]);
  debug_data["intermediate"]["A_vector"].append(A[1]);
  debug_data["intermediate"]["A_vector"].append(A[2]);
  debug_data["intermediate"]["A_norm"] = A.norm();
  
  debug_data["control"]["thrust"] = control_output.thrust;
  debug_data["control"]["moment"] = Json::arrayValue;
  debug_data["control"]["moment"].append(control_output.moment[0]);
  debug_data["control"]["moment"].append(control_output.moment[1]);
  debug_data["control"]["moment"].append(control_output.moment[2]);
  
  debug_data["manual_calculation"]["thrust"] = f;
  debug_data["manual_calculation"]["moment"] = Json::arrayValue;
  debug_data["manual_calculation"]["moment"].append(M[0]);
  debug_data["manual_calculation"]["moment"].append(M[1]);
  debug_data["manual_calculation"]["moment"].append(M[2]);
  
  // Write to file
  std::ofstream file("cpp_debug_single_step.json");
  file << debug_data;
  file.close();
  
  printf("\nDebug data saved to: cpp_debug_single_step.json\n");
  printf("Compare with: fdcl_debug_single_step.json\n");
  
  // Test assertions
  ENTO_TEST_CHECK_TRUE(std::abs(f - control_output.thrust) < 0.001f);
  ENTO_TEST_CHECK_TRUE((M - control_output.moment).norm() < 0.001f);
}

/**
 * Debug helix trajectory test with detailed output
 */
void test_debug_helix_trajectory() {
  using Scalar = float;
  using VehicleTraits = QuadrotorTraits<Scalar>;
  using Controller = GeometricController<Scalar, VehicleTraits, true, true>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  
  printf("\n=== C++ DEBUG HELIX TRAJECTORY TEST ===\n");
  
  // Create controller with FDCL gains
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
  
  // Helix parameters (same as Python test)
  EntoMath::Vec3<Scalar> helix_center(Scalar(0), Scalar(0), Scalar(1));
  Scalar helix_radius = Scalar(2.0);
  Scalar helix_frequency = Scalar(0.1);
  Scalar helix_climb_rate = Scalar(0.2);
  Scalar omega = Scalar(2.0) * M_PI * helix_frequency;
  
  printf("Helix parameters: radius=%.1fm, freq=%.1fHz, climb=%.1fm/s\n", 
         helix_radius, helix_frequency, helix_climb_rate);
  
  // Initial state (at helix starting position)
  State state;
  state.position << Scalar(2.0), Scalar(0.0), Scalar(1.0);  // Start at helix starting point
  state.velocity.setZero();
  state.acceleration.setZero();
  state.rotation.setIdentity();
  state.angular_velocity.setZero();
  
  Json::Value debug_array = Json::arrayValue;
  
  // Test first few time steps
  Scalar dt = Scalar(0.01);
  for (int step = 0; step < 3; ++step) {
    Scalar t = step * dt;
    
    printf("\n--- Step %d, t=%.3fs ---\n", step, t);
    
    // Generate helix trajectory
    EntoMath::Vec3<Scalar> xd;
    xd[0] = helix_center[0] + helix_radius * std::cos(omega * t);
    xd[1] = helix_center[1] + helix_radius * std::sin(omega * t);
    xd[2] = helix_center[2] + helix_climb_rate * t;
    
    EntoMath::Vec3<Scalar> xd_dot;
    xd_dot[0] = -helix_radius * omega * std::sin(omega * t);
    xd_dot[1] = helix_radius * omega * std::cos(omega * t);
    xd_dot[2] = helix_climb_rate;
    
    EntoMath::Vec3<Scalar> xd_2dot;
    xd_2dot[0] = -helix_radius * omega * omega * std::cos(omega * t);
    xd_2dot[1] = -helix_radius * omega * omega * std::sin(omega * t);
    xd_2dot[2] = Scalar(0.0);
    
    printf("Desired pos: [%.6f, %.6f, %.6f]\n", xd[0], xd[1], xd[2]);
    printf("Desired vel: [%.6f, %.6f, %.6f]\n", xd_dot[0], xd_dot[1], xd_dot[2]);
    printf("Desired acc: [%.6f, %.6f, %.6f]\n", xd_2dot[0], xd_2dot[1], xd_2dot[2]);
    
    // Set up command
    Command command;
    command.position_d = xd;
    command.velocity_d = xd_dot;
    command.acceleration_d = xd_2dot;
    command.jerk_d.setZero();
    command.snap_d.setZero();
    command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
    command.b1_d_dot.setZero();
    command.b1_d_ddot.setZero();
    
    // Compute control with detailed debug output
    printf("\n=== DEBUG POSITION CONTROL at t=%.6f ===\n", t);
    
    // Manual calculation (same as single step test)
    EntoMath::Vec3<Scalar> position_error = state.position - command.position_d;
    EntoMath::Vec3<Scalar> velocity_error = state.velocity - command.velocity_d;
    
    printf("Position error: ex = [%.6f, %.6f, %.6f]\n", 
           position_error[0], position_error[1], position_error[2]);
    printf("Velocity error: ev = [%.6f, %.6f, %.6f]\n", 
           velocity_error[0], velocity_error[1], velocity_error[2]);
    
    // Compute control
    auto control_output = controller.compute_control(state, command, dt);
    
    printf("Thrust: %.6f\n", control_output.thrust);
    printf("Moment: [%.6f, %.6f, %.6f]\n", 
           control_output.moment[0], control_output.moment[1], control_output.moment[2]);
    
    // Save debug data
    Json::Value step_data;
    step_data["time"] = t;
    step_data["step"] = step;
    step_data["state"]["position"] = Json::arrayValue;
    step_data["state"]["position"].append(state.position[0]);
    step_data["state"]["position"].append(state.position[1]);
    step_data["state"]["position"].append(state.position[2]);
    step_data["control"]["thrust"] = control_output.thrust;
    step_data["control"]["moment"] = Json::arrayValue;
    step_data["control"]["moment"].append(control_output.moment[0]);
    step_data["control"]["moment"].append(control_output.moment[1]);
    step_data["control"]["moment"].append(control_output.moment[2]);
    
    debug_array.append(step_data);
    
    // Simple Euler integration for next step (if not last step)
    if (step < 2) {
      // Dynamics (same as Python test)
      EntoMath::Vec3<Scalar> e3(Scalar(0), Scalar(0), Scalar(1));
      EntoMath::Vec3<Scalar> gravity_accel(Scalar(0), Scalar(0), -VehicleTraits::gravity());  // NEGATIVE gravity (downward)
      EntoMath::Vec3<Scalar> thrust_accel = (control_output.thrust / VehicleTraits::mass()) * (state.rotation * e3);
      EntoMath::Vec3<Scalar> linear_accel = thrust_accel + gravity_accel;  // ADD gravity (it's already negative)
      
      // Angular acceleration
      EntoMath::Vec3<Scalar> J_omega = VehicleTraits::inertia_matrix() * state.angular_velocity;
      EntoMath::Vec3<Scalar> gyroscopic_torque = -state.angular_velocity.cross(J_omega);
      EntoMath::Vec3<Scalar> total_torque = control_output.moment + gyroscopic_torque;
      EntoMath::Vec3<Scalar> angular_accel = VehicleTraits::inertia_matrix().inverse() * total_torque;
      
      // Integrate
      state.position = state.position + dt * state.velocity;
      state.velocity = state.velocity + dt * linear_accel;
      state.angular_velocity = state.angular_velocity + dt * angular_accel;
      
      // Rotation matrix integration (simplified)
      EntoMath::Matrix3x3<Scalar> omega_skew = EntoMath::skew(state.angular_velocity);
      state.rotation = state.rotation + dt * state.rotation * omega_skew;
      
      // Orthonormalize rotation matrix (simplified)
      Eigen::JacobiSVD<EntoMath::Matrix3x3<Scalar>> svd(state.rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
      state.rotation = svd.matrixU() * svd.matrixV().transpose();
      
      printf("Next state: pos=[%.6f, %.6f, %.6f], vel=[%.6f, %.6f, %.6f]\n", 
             state.position[0], state.position[1], state.position[2],
             state.velocity[0], state.velocity[1], state.velocity[2]);
    }
  }
  
  // Save debug data
  std::ofstream file("cpp_debug_helix.json");
  file << debug_array;
  file.close();
  
  printf("\nHelix debug data saved to: cpp_debug_helix.json\n");
  printf("Compare with: fdcl_debug_helix.json\n");
}

int main(int argc, char **argv) {
  using namespace EntoUtil;
  int __n;
  if (argc > 1) {
    __n = atoi(argv[1]);
  } else {
    __n = 1;  // Default to test 1
  }

  ENTO_TEST_START();
  
  if (__ento_test_num(__n, 1)) test_debug_single_step_comparison();
  if (__ento_test_num(__n, 2)) test_debug_helix_trajectory();
  
  ENTO_TEST_END();
  
  return 0;
} 