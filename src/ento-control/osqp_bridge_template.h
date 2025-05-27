#ifndef ENTO_CONTROL_OSQP_BRIDGE_TEMPLATE_H
#define ENTO_CONTROL_OSQP_BRIDGE_TEMPLATE_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "osqp.h"

namespace EntoControl {

template<typename APIType>
class OSQPBridge {
public:
  using State = Eigen::Matrix<float, APIType::n_x, 1>;
  using Control = Eigen::Matrix<float, APIType::n_u, 1>;
  
  OSQPBridge() : last_status_(OSQP_UNSOLVED), last_iterations_(0) {
    u0_.setZero();
    x_ref_.setZero();
  }
  
  bool set_x0(const State& x0) {
    // Validate input
    for (int i = 0; i < APIType::n_x; ++i) {
      if (!std::isfinite(x0[i])) {
        return false;
      }
    }
    x0_ = x0;
    return true;
  }
  
  void set_x_ref(const State& x_ref) {
    x_ref_ = x_ref;
  }
  
  bool solve() {
    // Get workspace
    auto* workspace = APIType::get_workspace();
    if (!workspace) {
      return false;
    }

    // For now, we'll use the embedded OSQP solver directly
    // The problem data is already embedded in the generated code
    
    // Solve the QP
    OSQPInt status = APIType::solve(workspace);

    // Check if solve was successful
    auto solver_status = APIType::get_status(workspace);
    last_status_ = static_cast<OSQPInt>(solver_status);
    last_iterations_ = APIType::get_iterations(workspace);

    // Get solution
    const OSQPFloat* solution = APIType::get_solution(workspace);
    if (!solution) {
      return false;
    }

    // Extract first control input (u0)
    for (int i = 0; i < APIType::n_u; ++i) {
      u0_[i] = static_cast<float>(solution[i]);
    }
    
    return (last_status_ == OSQP_SOLVED || last_status_ == OSQP_SOLVED_INACCURATE);
  }
  
  Control get_u0() const {
    return u0_;
  }
  
  OSQPInt get_status() const {
    return last_status_;
  }
  
  int get_iterations() const {
    return last_iterations_;
  }

  // Forward simulation method for template MPC dynamics
  State simulate_forward(const State& x0, const Control& u, float dt = 0.005f) const {
    State x_next = x0;
    
    // Template MPC state: [pos(3), orient_vec(3), vel(3), ang_vel(3)] = 12 states
    // Control: [thrust, pitch_moment, roll_moment] = 3 inputs
    
    // Physical parameters (matching template MPC)
    const float mass = 100e-6f; // 100mg in kg
    const float g = 9.81f; // gravity
    
    // Position integration: pos += vel * dt
    x_next.template segment<3>(0) += x0.template segment<3>(6) * dt;
    
    // Velocity integration: vel += accel * dt
    // For template MPC: thrust acts along orientation vector (body z-axis)
    // Orientation vector is normalized [sx, sy, sz] where typically sz ≈ 1 for upright
    Eigen::Vector3f s = x0.template segment<3>(3); // orientation vector
    float thrust = u[0];
    
    // Thrust acceleration: a_thrust = (thrust / mass) * s
    Eigen::Vector3f a_thrust = (thrust / mass) * s;
    
    // Gravity acceleration: a_gravity = [0, 0, -g]
    Eigen::Vector3f a_gravity(0.0f, 0.0f, -g);
    
    // Total acceleration
    Eigen::Vector3f a_total = a_thrust + a_gravity;
    
    // Update velocities
    x_next.template segment<3>(6) += a_total * dt;
    
    // Orientation vector integration (simplified)
    // Moments affect orientation rates: ds/dt ≈ moments × scale_factor
    Eigen::Vector2f moments = u.template segment<2>(1); // [pitch_moment, roll_moment]
    
    // Simple orientation dynamics: pitch affects sx, roll affects sy
    // This is a simplified model - real template MPC has more complex orientation dynamics
    float orientation_scale = 0.1f; // tuning parameter
    x_next[3] += moments[1] * dt * orientation_scale; // roll affects sx
    x_next[4] += moments[0] * dt * orientation_scale; // pitch affects sy
    
    // Normalize orientation vector to maintain unit length constraint
    Eigen::Vector3f s_new = x_next.template segment<3>(3);
    float s_norm = s_new.norm();
    if (s_norm > 1e-6f) {
      x_next.template segment<3>(3) = s_new / s_norm;
    }
    
    // Angular velocity integration (simplified)
    // Moments directly affect angular velocities
    float angular_scale = 1.0f; // tuning parameter  
    x_next.template segment<3>(9) += u.template segment<3>(0) * dt * angular_scale;
    
    return x_next;
  }

private:
  State x0_;
  State x_ref_;
  Control u0_;
  OSQPInt last_status_;
  int last_iterations_;
};

} // namespace EntoControl

#endif // ENTO_CONTROL_OSQP_BRIDGE_TEMPLATE_H 