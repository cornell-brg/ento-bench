#ifndef ENTO_CONTROL_ROBOBEE_MPC_CONTROLLER_H
#define ENTO_CONTROL_ROBOBEE_MPC_CONTROLLER_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

// Include the generated RoboBee MPC API
#include "generated/robobee_mpc/robobee_api.h"

// Traits class for RoboBee MPC controller
struct RoboBeeControllerTraits {
  using Scalar = float;
  static constexpr int N = 10;  // State dimension (RoboBee state)
  static constexpr int M = 3;   // Control input dimension (thrust, roll, pitch)
  static constexpr int H = 20;  // Prediction horizon
  
  // Physical parameters for RoboBee (for simulation)
  static constexpr float mass = 1.06e-4f;           // kg
  static constexpr float marker_mass = 1.0e-5f;     // kg
  static constexpr float Jxx = 2.32e-9f;            // kg·m²
  static constexpr float Jyy = 1.97e-9f;            // kg·m²
  static constexpr float Jzz = 0.45e-9f;            // kg·m²
  static constexpr float k_T = 3.27e-5f;            // N / V
  static constexpr float k_r = 0.50e-6f;            // N·m / V
  static constexpr float k_p = 0.15e-6f;            // N·m / V
  static constexpr float g = 9.81f;                 // m/s²
};

template<typename Traits>
class RoboBeeControllerStep
{
public:
  using Scalar = typename Traits::Scalar;
  static constexpr int N = Traits::N;
  static constexpr int M = Traits::M;
  static constexpr int H = Traits::H;

  // ----- I/O structs ----------------------------------------------------- //
  struct Input
  {
    Eigen::Matrix<Scalar, N, 1> x;         // Current state
    Eigen::Matrix<Scalar, N, H, Eigen::RowMajor> x_ref;  // Reference trajectory
  };
  
  struct Output
  {
    Eigen::Matrix<Scalar, M, 1> u;        // Control output
  };

  // ----- Dynamics functions for simulation ------------------------------- //
  // Get the continuous-time A matrix
  static Eigen::Matrix<Scalar, N, N> get_A_continuous() {
    Eigen::Matrix<Scalar, N, N> A = Eigen::Matrix<Scalar, N, N>::Zero();
    
    // Position derivatives
    A(0, 3) = A(1, 4) = A(2, 5) = 1.0f;
    
    // Gravity effects
    A(3, 7) =  Traits::g;  // g*sin(θ) ≈ g*θ for small θ
    A(4, 6) = -Traits::g;  // -g*sin(φ) ≈ -g*φ for small φ
    
    // Angular velocity to angle
    A(6, 8) = 1.0f;  // φ_dot = p
    A(7, 9) = 1.0f;  // θ_dot = q
    
    return A;
  }
  
  // Get the continuous-time B matrix
  static Eigen::Matrix<Scalar, N, M> get_B_continuous() {
    Eigen::Matrix<Scalar, N, M> B = Eigen::Matrix<Scalar, N, M>::Zero();
    
    // Thrust to z-acceleration
    B(5, 0) = Traits::k_T / (Traits::mass + Traits::marker_mass);
    
    // Torques to angular accelerations
    B(8, 1) = Traits::k_r / Traits::Jxx;  // Roll torque
    B(9, 2) = Traits::k_p / Traits::Jyy;  // Pitch torque
    
    return B;
  }
  
  // Get the discrete-time A matrix
  static Eigen::Matrix<Scalar, N, N> get_A_discrete(Scalar dt) {
    return Eigen::Matrix<Scalar, N, N>::Identity() + get_A_continuous() * dt;
  }
  
  // Get the discrete-time B matrix
  static Eigen::Matrix<Scalar, N, M> get_B_discrete(Scalar dt) {
    return get_B_continuous() * dt;
  }

  // Default constructor with 5ms time step
  RoboBeeControllerStep() : dt_(0.005f) {
    reset_controller();
  }
  
  // Constructor with custom time step
  RoboBeeControllerStep(Scalar dt) : dt_(dt) {
    reset_controller();
  }
  
  // Reset the controller to its initial state
  void reset_controller() {
    // Get or initialize the workspace
    workspace_ = RoboBeeAPI::get_workspace();
    
    // Initialize state and reference trajectories
    x_.setZero();
    xref_matrix_.setZero();
    u_.setZero();
    u_warm_.setZero();
    
    // Resize arrays with proper size
    x0_array_.resize(N, 0.0f);
    xref_array_.resize(N * H, 0.0f);
    u_array_.resize(M * H, 0.0f);
    
    // Calculate discrete-time matrices for simulation
    Ad_ = get_A_discrete(dt_);
    Bd_ = get_B_discrete(dt_);
  }

  void set_x0(const Eigen::Matrix<Scalar, N, 1>& x0) {
    x_ = x0;
    
    // Convert to array for API
    for (int i = 0; i < N; ++i) {
      x0_array_[i] = static_cast<float>(x0[i]);
    }
  }
  
  void set_x_ref(const Eigen::Matrix<Scalar, N, H, Eigen::RowMajor>& xref) {
    xref_matrix_ = xref;
    
    // Convert the reference trajectory to a flattened array for API
    for (int k = 0; k < H; ++k) {
      for (int i = 0; i < N; ++i) {
        xref_array_[k * N + i] = static_cast<float>(xref(i, k));
      }
    }
  }
  
  // Set single reference point (replicated for entire horizon)
  void set_x_ref(const Eigen::Matrix<Scalar, N, 1>& xref) {
    // Create a reference trajectory with the same point repeated
    Eigen::Matrix<Scalar, N, H, Eigen::RowMajor> full_traj;
    for (int k = 0; k < H; ++k) {
      full_traj.col(k) = xref;
    }
    
    set_x_ref(full_traj);
  }
  
  void reset_duals() {
    // Reset the solver to its initial state
    if (workspace_) {
      RoboBeeAPI::init(workspace_);
    }
  }
  
  // Get the current time step
  Scalar get_dt() const { return dt_; }
  
  // Set a new time step
  void set_dt(Scalar dt) { 
    dt_ = dt; 
    // Update discrete-time matrices
    Ad_ = get_A_discrete(dt_);
    Bd_ = get_B_discrete(dt_);
  }

  void solve() {
    if (!workspace_) {
      std::cerr << "Error: RoboBee MPC workspace not initialized" << std::endl;
      return;
    }
    
    // Update the constraints with current state and reference trajectory
    RoboBeeAPI::update_rhs(workspace_, x0_array_.data(), xref_array_.data());
    
    // Warm start with previous solution if available
    if (has_solution_) {
      RoboBeeAPI::warm_start(workspace_, u_array_.data());
    }
    
    // Solve the MPC problem
    RoboBeeAPI::solve(workspace_);
    
    // Check if the solution is optimal
    if (RoboBeeAPI::is_optimal(workspace_)) {
      // Get the solution (optimal control inputs)
      const float* solution = RoboBeeAPI::get_solution(workspace_);
      
      // Store the first control input
      for (int i = 0; i < M; ++i) {
        u_[i] = static_cast<Scalar>(solution[i]);
      }
      
      // Store the entire solution for warm starting next time
      for (int i = 0; i < M * H; ++i) {
        u_array_[i] = static_cast<Scalar>(solution[i]);
        u_warm_[i] = u_array_[i];
      }
      
      has_solution_ = true;
    } else {
      // Handle non-optimal solution
      std::cerr << "Warning: RoboBee MPC solver did not reach optimal solution" << std::endl;
      // Use the previous control input as fallback
    }
  }

  const Eigen::Matrix<Scalar, M, 1>& get_u0() const { return u_; }
  
  // Get next state by simulating the system with the computed control input
  Eigen::Matrix<Scalar, N, 1> get_next_state() const {
    // Simulate one step forward using discrete-time dynamics
    return Ad_ * x_ + Bd_ * u_;
  }

private:
  // State and reference data
  Eigen::Matrix<Scalar, N, 1> x_;
  Eigen::Matrix<Scalar, N, H, Eigen::RowMajor> xref_matrix_;
  
  // Control data
  Eigen::Matrix<Scalar, M, 1> u_;
  Eigen::Matrix<Scalar, M * H, 1> u_warm_;
  
  // Arrays for API interface (using std::vector for proper memory management)
  std::vector<float> x0_array_;
  std::vector<float> xref_array_;
  std::vector<float> u_array_;
  
  // MPC workspace
  RoboBeeAPI::Workspace* workspace_ = nullptr;
  
  // Time step
  Scalar dt_;
  
  // Flag to indicate if we have a previous solution for warm starting
  bool has_solution_ = false;
  
  // Cached discrete-time system matrices for simulation
  Eigen::Matrix<Scalar, N, N> Ad_;
  Eigen::Matrix<Scalar, N, M> Bd_;
};

namespace EntoControl
{
  using RoboBeeController = RoboBeeControllerStep<RoboBeeControllerTraits>;
}

#endif // ENTO_CONTROL_ROBOBEE_MPC_CONTROLLER_H 