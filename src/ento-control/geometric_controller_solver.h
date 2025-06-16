#ifndef GEOMETRIC_CONTROLLER_SOLVER_H
#define GEOMETRIC_CONTROLLER_SOLVER_H

#include "geometric_controller.h"
#include <ento-math/core.h>
#include <ento-util/debug.h>
#include <Eigen/Dense>

namespace EntoControl {

/**
 * @brief Solver wrapper for GeometricController that implements the interface
 * expected by OptControlProblem for benchmarking
 * 
 * This class wraps the GeometricController and provides the necessary interface
 * methods for integration with the ento-bench framework.
 * 
 * State vector format: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
 * Control vector format: [thrust, Mx, My, Mz]
 */
template <
  typename Scalar, 
  typename VehicleTraits = QuadrotorTraits<Scalar>,
  bool UseDecoupledYaw = true,
  bool UseIntegralControl = true
>
class GeometricControllerSolver
{
public:
  using Scalar_ = Scalar;
  using Traits = VehicleTraits;
  using Controller = GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>;
  using State = typename Controller::State;
  using Command = typename Controller::Command;
  using Control = typename Controller::Control;
  
  using Vector3 = EntoMath::Vec3<Scalar>;
  using Matrix3 = EntoMath::Matrix3x3<Scalar>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  
  // State and control dimensions for benchmarking
  static constexpr int StateSize = 13;  // [x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz]
  static constexpr int ControlSize = 4; // [thrust, Mx, My, Mz]
  
  using StateVector = Eigen::Matrix<Scalar, StateSize, 1>;
  using ControlVector = Eigen::Matrix<Scalar, ControlSize, 1>;
  
private:
  Controller controller_;
  Scalar dt_;
  
  // Current state and reference
  StateVector current_state_;
  StateVector reference_state_;
  ControlVector current_control_;
  StateVector next_state_;
  
  // Trajectory generation for reference commands
  Command current_command_;
  
public:
  /**
   * @brief Constructor with timestep
   */
  explicit GeometricControllerSolver(Scalar dt) 
    : dt_(dt)
  {
    current_state_.setZero();
    reference_state_.setZero();
    current_control_.setZero();
    next_state_.setZero();
    
    // Initialize with hover at origin
    current_state_(2) = Scalar(1.0);  // z = 1m
    current_state_(6) = Scalar(1.0);  // qw = 1 (identity quaternion)
    
    reference_state_(2) = Scalar(1.0);  // z = 1m  
    reference_state_(6) = Scalar(1.0);  // qw = 1 (identity quaternion)
  }
  
  /**
   * @brief Set current state (required by OptControlProblem)
   */
  void set_x0(const StateVector& state) {
    current_state_ = state;
  }
  
  /**
   * @brief Set reference state (required by OptControlProblem)
   */
  void set_x_ref(const StateVector& ref_state) {
    reference_state_ = ref_state;
    update_command_from_reference();
  }
  
  /**
   * @brief Solve control problem (required by OptControlProblem)
   */
  void solve() {
    // Convert state vector to geometric controller state
    State state = state_vector_to_geometric_state(current_state_);
    
    // Compute control using geometric controller
    Control control = controller_.compute_control(state, current_command_, dt_);
    
    // Convert control to vector format
    current_control_ << control.thrust, control.moment[0], control.moment[1], control.moment[2];
    
    // Simulate forward dynamics to get next state
    next_state_ = simulate_forward(current_state_, current_control_);
  }
  
  /**
   * @brief Get computed control (required by OptControlProblem)
   */
  ControlVector get_u0() const {
    return current_control_;
  }
  
  /**
   * @brief Get next state (required by OptControlProblem)
   */
  StateVector get_next_state() const {
    return next_state_;
  }
  
  /**
   * @brief Get current gains
   */
  const typename Controller::Gains& get_gains() const {
    return controller_.get_gains();
  }
  
  /**
   * @brief Set control gains
   */
  void set_gains(const typename Controller::Gains& gains) {
    controller_.set_gains(gains);
  }
  
  /**
   * @brief Reset integral errors
   */
  void reset_integral_errors() {
    controller_.reset_integral_errors();
  }
  
  /**
   * @brief Forward simulate dynamics (quadrotor nonlinear dynamics)
   */
  StateVector simulate_forward(const StateVector& state, const ControlVector& control) {
    // Extract state components
    Vector3 pos = state.template segment<3>(0);
    Vector3 vel = state.template segment<3>(3);
    Quaternion quat(state(6), state(7), state(8), state(9));  // [qw, qx, qy, qz]
    Vector3 omega = state.template segment<3>(10);
    
    // Normalize quaternion to handle numerical drift
    quat.normalize();
    Matrix3 R = quat.toRotationMatrix();
    
    // Extract control components
    Scalar thrust = control(0);
    Vector3 moment = control.template segment<3>(1);
    
    // === QUADROTOR NONLINEAR DYNAMICS (FDCL FORMULATION) ===
    
    // Linear acceleration: v_dot = g*e3 - f*R*e3/m (EXACT FDCL equation)
    Vector3 e3(Scalar(0), Scalar(0), Scalar(1));
    Vector3 gravity_term = VehicleTraits::gravity() * e3;  // g*e3 (positive, upward)
    Vector3 thrust_term = (thrust / VehicleTraits::mass()) * (R * e3);  // f*R*e3/m
    Vector3 linear_accel = gravity_term - thrust_term;  // g*e3 - f*R*e3/m
    
    // Angular acceleration: alpha = J^-1 * (M - omega × (J * omega))
    Matrix3 inertia = VehicleTraits::inertia_matrix();
    Vector3 J_omega = inertia * omega;
    Vector3 gyroscopic_torque = -omega.cross(J_omega);
    Vector3 total_torque = moment + gyroscopic_torque;
    Vector3 angular_accel = inertia.inverse() * total_torque;
    
    // === EULER INTEGRATION ===
    
    // Linear motion
    Vector3 pos_new = pos + dt_ * vel;
    Vector3 vel_new = vel + dt_ * linear_accel;
    
    // Angular motion
    Vector3 omega_new = omega + dt_ * angular_accel;
    
    // Quaternion integration using angular velocity
    // q_new = q * exp(0.5 * omega * dt)
    Vector3 omega_dt = omega * dt_;
    Scalar angle = omega_dt.norm();
    
    Quaternion quat_new;
    if (angle < Scalar(1e-8)) {
      // Small angle approximation
      Quaternion dq(Scalar(1.0), Scalar(0.5) * omega_dt[0], Scalar(0.5) * omega_dt[1], Scalar(0.5) * omega_dt[2]);
      quat_new = quat * dq;
    } else {
      // Full quaternion exponential
      Scalar half_angle = Scalar(0.5) * angle;
      Scalar sin_half = std::sin(half_angle);
      Scalar cos_half = std::cos(half_angle);
      Vector3 axis = omega_dt / angle;
      
      Quaternion dq(cos_half, sin_half * axis[0], sin_half * axis[1], sin_half * axis[2]);
      quat_new = quat * dq;
    }
    
    quat_new.normalize();
    
    // Pack into state vector
    StateVector next_state;
    next_state.template segment<3>(0) = pos_new;
    next_state.template segment<3>(3) = vel_new;
    next_state(6) = quat_new.w();
    next_state(7) = quat_new.x();
    next_state(8) = quat_new.y();
    next_state(9) = quat_new.z();
    next_state.template segment<3>(10) = omega_new;
    
    return next_state;
  }
  
private:
  /**
   * @brief Convert state vector to geometric controller state
   */
  State state_vector_to_geometric_state(const StateVector& state_vec) {
    State state;
    
    // Position and velocity
    state.position = state_vec.template segment<3>(0);
    state.velocity = state_vec.template segment<3>(3);
    state.acceleration.setZero();  // Not used by controller
    
    // Rotation matrix from quaternion
    Quaternion quat(state_vec(6), state_vec(7), state_vec(8), state_vec(9));
    quat.normalize();
    state.rotation = quat.toRotationMatrix();
    
    // Angular velocity
    state.angular_velocity = state_vec.template segment<3>(10);
    
    return state;
  }
  
  /**
   * @brief Update command from reference state vector
   */
  void update_command_from_reference() {
    // Extract reference position and velocity
    current_command_.position_d = reference_state_.template segment<3>(0);
    current_command_.velocity_d = reference_state_.template segment<3>(3);
    
    // Zero higher-order derivatives (simple tracking)
    current_command_.acceleration_d.setZero();
    current_command_.jerk_d.setZero();
    current_command_.snap_d.setZero();
    
    // Extract reference attitude from quaternion
    Quaternion ref_quat(reference_state_(6), reference_state_(7), reference_state_(8), reference_state_(9));
    ref_quat.normalize();
    current_command_.rotation_d = ref_quat.toRotationMatrix();
    
    // Reference angular velocity
    current_command_.angular_velocity_d = reference_state_.template segment<3>(10);
    current_command_.angular_acceleration_d.setZero();
    
    // Set desired body frame directions (simple case: align with world frame)
    current_command_.b1_d = Vector3::UnitX();
    current_command_.b1_d_dot.setZero();
    current_command_.b1_d_ddot.setZero();
    current_command_.b3_d = Vector3::UnitZ();
    current_command_.b3_d_dot.setZero();
    current_command_.b3_d_ddot.setZero();
  }
};

// Common type aliases for benchmarking
using GeometricControllerSolverFloat = GeometricControllerSolver<float, QuadrotorTraits<float>, true, true>;
using GeometricControllerSolverDouble = GeometricControllerSolver<double, QuadrotorTraits<double>, true, true>;

} // namespace EntoControl

#endif // GEOMETRIC_CONTROLLER_SOLVER_H 