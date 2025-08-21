#include <ento-bench/problem.h>
#include <ento-control/geometric_controller.h>
#include <ento-control/geometric_controller_solver.h>
#include <ento-control/quadrotor_traits.h>
#include <ento-math/core.h>
#include <iostream>
#include <iomanip>

using namespace ento;

// Copy the QuadrotorSimulator from the integration test
template <typename Scalar, typename VehicleTraits>
class QuadrotorSimulator {
public:
  using Vector3 = EntoMath::Vec3<Scalar>;
  using Matrix3 = EntoMath::Matrix3x3<Scalar>;
  using State = GeometricState<Scalar, VehicleTraits>;
  using Control = GeometricControl<Scalar, VehicleTraits>;
  
private:
  static constexpr Scalar mass_ = VehicleTraits::mass();
  static constexpr Scalar gravity_ = VehicleTraits::gravity();
  static const Matrix3 inertia_;
  static const Matrix3 inertia_inv_;
  
public:
  QuadrotorSimulator() = default;
  
  void integrate(State& state, const Control& control, Scalar dt) {
    // Extract current state
    Vector3 pos = state.position;
    Vector3 vel = state.velocity;
    Matrix3 R = state.rotation;
    Vector3 omega = state.angular_velocity;
    
    // === STEP 1: Compute accelerations ===
    
    // Linear acceleration: dv = (thrust/mass) * R * e3 - g * e3_world
    Vector3 e3(Scalar(0), Scalar(0), Scalar(1));
    Vector3 thrust_accel = (control.thrust / mass_) * (R * e3);
    Vector3 gravity_accel(Scalar(0), Scalar(0), -gravity_);
    Vector3 linear_accel = thrust_accel + gravity_accel;
    
    // Angular acceleration: domega = J^-1 * (M - omega × (J * omega))
    Vector3 J_omega = inertia_ * omega;
    Vector3 gyroscopic_torque = -omega.cross(J_omega);
    Vector3 total_torque = control.moment + gyroscopic_torque;
    Vector3 angular_accel = inertia_inv_ * total_torque;
    
    // === STEP 2: Euler integration ===
    
    // Linear motion: p_new = p + dt * v, v_new = v + dt * dv
    Vector3 pos_new = pos + dt * vel;
    Vector3 vel_new = vel + dt * linear_accel;
    
    // Angular motion: omega_new = omega + dt * domega
    Vector3 omega_new = omega + dt * angular_accel;
    
    // Rotation matrix integration: R_new = R * expm(skew(omega) * dt)
    Matrix3 omega_skew = EntoMath::skew(omega);
    Matrix3 omega_skew_dt = omega_skew * dt;
    
    // Matrix exponential using Rodrigues' formula
    Matrix3 R_new;
    Scalar angle = omega.norm() * dt;
    
    if (angle < Scalar(1e-8)) {
      // Small angle approximation
      R_new = R * (Matrix3::Identity() + omega_skew_dt);
    } else {
      // Rodrigues' formula: expm(K) = I + sin(θ)/θ * K + (1-cos(θ))/θ² * K²
      Scalar sin_angle = std::sin(angle);
      Scalar cos_angle = std::cos(angle);
      Scalar angle_sq = angle * angle;
      
      Matrix3 omega_skew_dt_sq = omega_skew_dt * omega_skew_dt;
      Matrix3 exp_omega_dt = Matrix3::Identity() + 
                            (sin_angle / angle) * omega_skew_dt + 
                            ((Scalar(1) - cos_angle) / angle_sq) * omega_skew_dt_sq;
      
      R_new = R * exp_omega_dt;
    }
    
    // Orthonormalize rotation matrix to prevent drift
    orthonormalize_rotation(R_new);
    
    // === STEP 3: Update state ===
    state.position = pos_new;
    state.velocity = vel_new;
    state.acceleration = linear_accel;
    state.rotation = R_new;
    state.angular_velocity = omega_new;
  }
  
private:
  void orthonormalize_rotation(Matrix3& R) {
    // Gram-Schmidt orthonormalization to maintain SO(3) properties
    Vector3 x = R.col(0);
    Vector3 y = R.col(1);
    Vector3 z = R.col(2);
    
    // Orthonormalize
    x.normalize();
    y = y - y.dot(x) * x;
    y.normalize();
    z = x.cross(y);  // Ensure right-handed coordinate system
    
    R << x, y, z;
  }
};

template <typename Scalar, typename VehicleTraits>
const EntoMath::Matrix3x3<Scalar> QuadrotorSimulator<Scalar, VehicleTraits>::inertia_ = 
  VehicleTraits::inertia_matrix();

template <typename Scalar, typename VehicleTraits>
const EntoMath::Matrix3x3<Scalar> QuadrotorSimulator<Scalar, VehicleTraits>::inertia_inv_ = 
  VehicleTraits::inertia_matrix().inverse();

void test_dynamics_comparison() {
  using Scalar = float;
  using VehicleTraits = QuadrotorTraits<Scalar>;
  using Vector3 = EntoMath::Vec3<Scalar>;
  using Matrix3 = EntoMath::Matrix3x3<Scalar>;
  using Quaternion = EntoMath::Quaternion<Scalar>;
  
  std::cout << "=== DYNAMICS COMPARISON TEST ===" << std::endl;
  
  // Create both dynamics implementations
  GeometricControllerSolver<Scalar, VehicleTraits, false, false> solver(0.01f);
  QuadrotorSimulator<Scalar, VehicleTraits> simulator;
  
  // Test state: hover with small perturbation
  Vector3 pos(0.1f, 0.1f, 1.0f);
  Vector3 vel(0.05f, 0.05f, 0.0f);
  Quaternion quat = Quaternion::Identity();
  Vector3 omega(0.01f, 0.01f, 0.0f);
  
  // Control: hover thrust + small moment
  Scalar thrust = 19.13f;  // Hover thrust
  Vector3 moment(0.1f, 0.1f, 0.05f);
  
  // === Test GeometricControllerSolver dynamics ===
  
  // Convert to solver format
  typename GeometricControllerSolver<Scalar, VehicleTraits, false, false>::StateVector state_solver(13);
  state_solver.segment<3>(0) = pos;
  state_solver.segment<3>(3) = vel;
  state_solver(6) = quat.w();
  state_solver(7) = quat.x();
  state_solver(8) = quat.y();
  state_solver(9) = quat.z();
  state_solver.segment<3>(10) = omega;
  
  typename GeometricControllerSolver<Scalar, VehicleTraits, false, false>::ControlVector control_solver(4);
  control_solver(0) = thrust;
  control_solver.segment<3>(1) = moment;
  
  auto state_solver_new = solver.simulate_forward(state_solver, control_solver);
  
  // === Test QuadrotorSimulator dynamics ===
  
  // Convert to simulator format
  GeometricState<Scalar, VehicleTraits> state_sim;
  state_sim.position = pos;
  state_sim.velocity = vel;
  state_sim.rotation = quat.toRotationMatrix();
  state_sim.angular_velocity = omega;
  
  GeometricControl<Scalar, VehicleTraits> control_sim;
  control_sim.thrust = thrust;
  control_sim.moment = moment;
  
  simulator.integrate(state_sim, control_sim, 0.01f);
  
  // === Compare results ===
  
  std::cout << std::fixed << std::setprecision(8);
  
  std::cout << "\nInitial state:" << std::endl;
  std::cout << "  Position: [" << pos.transpose() << "]" << std::endl;
  std::cout << "  Velocity: [" << vel.transpose() << "]" << std::endl;
  std::cout << "  Omega:    [" << omega.transpose() << "]" << std::endl;
  
  std::cout << "\nControl:" << std::endl;
  std::cout << "  Thrust: " << thrust << std::endl;
  std::cout << "  Moment: [" << moment.transpose() << "]" << std::endl;
  
  std::cout << "\nSolver result:" << std::endl;
  std::cout << "  Position: [" << state_solver_new.segment<3>(0).transpose() << "]" << std::endl;
  std::cout << "  Velocity: [" << state_solver_new.segment<3>(3).transpose() << "]" << std::endl;
  std::cout << "  Omega:    [" << state_solver_new.segment<3>(10).transpose() << "]" << std::endl;
  
  std::cout << "\nSimulator result:" << std::endl;
  std::cout << "  Position: [" << state_sim.position.transpose() << "]" << std::endl;
  std::cout << "  Velocity: [" << state_sim.velocity.transpose() << "]" << std::endl;
  std::cout << "  Omega:    [" << state_sim.angular_velocity.transpose() << "]" << std::endl;
  
  // Compute differences
  Vector3 pos_diff = state_solver_new.segment<3>(0) - state_sim.position;
  Vector3 vel_diff = state_solver_new.segment<3>(3) - state_sim.velocity;
  Vector3 omega_diff = state_solver_new.segment<3>(10) - state_sim.angular_velocity;
  
  std::cout << "\nDifferences:" << std::endl;
  std::cout << "  Position: [" << pos_diff.transpose() << "] (norm=" << pos_diff.norm() << ")" << std::endl;
  std::cout << "  Velocity: [" << vel_diff.transpose() << "] (norm=" << vel_diff.norm() << ")" << std::endl;
  std::cout << "  Omega:    [" << omega_diff.transpose() << "] (norm=" << omega_diff.norm() << ")" << std::endl;
  
  // Check if differences are significant
  Scalar pos_tol = 1e-6f;
  Scalar vel_tol = 1e-6f;
  Scalar omega_tol = 1e-6f;
  
  bool pos_match = pos_diff.norm() < pos_tol;
  bool vel_match = vel_diff.norm() < vel_tol;
  bool omega_match = omega_diff.norm() < omega_tol;
  
  std::cout << "\nComparison results:" << std::endl;
  std::cout << "  Position match: " << (pos_match ? "✓" : "✗") << std::endl;
  std::cout << "  Velocity match: " << (vel_match ? "✓" : "✗") << std::endl;
  std::cout << "  Omega match:    " << (omega_match ? "✓" : "✗") << std::endl;
  
  if (pos_match && vel_match && omega_match) {
    std::cout << "\n🎉 SUCCESS: Both dynamics implementations match!" << std::endl;
  } else {
    std::cout << "\n❌ FAILURE: Dynamics implementations differ!" << std::endl;
  }
}

int main() {
  test_dynamics_comparison();
  return 0;
} 