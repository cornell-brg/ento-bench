#ifndef ENTO_CONTROL_ADAPTIVE_CONTROLLER_H
#define ENTO_CONTROL_ADAPTIVE_CONTROLLER_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

// Include Simulink-generated headers
extern "C" {
  #include "robobee_adaptive_controller/integrated_controller.h"
}

// We'll use a traits class to make it configurable, similar to other controllers
struct AdaptiveControllerTraits {
  using Scalar = float;
  static constexpr int N = 6;   // State dimension: [x, y, z, roll, pitch, yaw]
  static constexpr int M = 3;   // Control input dimension (thrust, roll, pitch)
};

template<typename Traits>
class AdaptiveControllerStep
{
public:
  using Scalar = typename Traits::Scalar;
  static constexpr int N = Traits::N;
  static constexpr int M = Traits::M;

  // ----- I/O structs ----------------------------------------------------- //
  struct Input
  {
    Eigen::Matrix<Scalar, N, 1> x;
    Eigen::Matrix<Scalar, N, 1> x_ref;
  };
  
  struct Output
  {
    Eigen::Matrix<Scalar, M, 1> u;
  };

  // ----- OptControl hooks ------------------------------------------------ //
  // The controller has its own internal dynamics model - these are just placeholders
  static Eigen::Matrix<Scalar, N, N> get_Adyn() { 
    return Eigen::Matrix<Scalar, N, N>::Identity(); 
  }
  
  static Eigen::Matrix<Scalar, N, M> get_Bdyn() { 
    return Eigen::Matrix<Scalar, N, M>::Zero(); 
  }

  // Default constructor with 10ms time step
  AdaptiveControllerStep() : dt_(0.01f) {
    // Initialize the Simulink controller
    reset_controller();
  }
  
  // Constructor with custom time step
  AdaptiveControllerStep(Scalar dt) : dt_(dt) {
    // Initialize the Simulink controller
    reset_controller();
  }
  
  // Reset the controller to its initial state
  void reset_controller() {
    integrated_controller_initialize();
    
    // Initialize state and control vectors
    x_.setZero();
    x_ref_.setZero();
    u_.setZero();
    next_state_.setZero();
    
    // Initialize the Simulink controller inputs and outputs
    rtU.Xm = 0.0;
    rtU.Ym = 0.0;
    rtU.Zm = 0.0;
    rtU.alpharadians = 0.0;
    rtU.betaradians = 0.0;
    rtU.gammaradians = 0.0;
    
    // Call step once to initialize all internal states
    integrated_controller_step();
  }

  void set_x0(const Eigen::Matrix<Scalar, N, 1>& x0) { 
    x_ = x0;
    
    // Map our state vector to the Simulink controller inputs (rtU)
    // Positions (x, y, z)
    rtU.Xm = static_cast<real_T>(x0[0]);           
    rtU.Ym = static_cast<real_T>(x0[1]);           
    rtU.Zm = static_cast<real_T>(x0[2]);           
    
    // Orientation (roll, pitch, yaw)
    rtU.alpharadians = static_cast<real_T>(x0[3]); 
    rtU.betaradians = static_cast<real_T>(x0[4]);  
    rtU.gammaradians = static_cast<real_T>(x0[5]); 
  }
  
  void set_x_ref(const Eigen::Matrix<Scalar, N, 1>& xref) { 
    x_ref_ = xref;
    
    // The Simulink controller doesn't have direct reference inputs
    // as it's designed to stabilize around a hardcoded position/orientation
  }
  
  void reset_duals() {
    // Reinitialize the controller to reset all internal states
    reset_controller();
  }
  
  // Get the current time step
  Scalar get_dt() const { return dt_; }
  
  // Set a new time step
  void set_dt(Scalar dt) { dt_ = dt; }

  void solve() {
    // Run one step of the Simulink controller
    integrated_controller_step();
    
    // Map the control outputs from rtY to our control vector
    u_[0] = static_cast<Scalar>(rtY.force);  // Thrust
    u_[1] = static_cast<Scalar>(rtY.roll);   // Roll torque
    u_[2] = static_cast<Scalar>(rtY.pitch);  // Pitch torque
    
    // Map the controller's output state (rtY) to our next state vector
    // Check for NaN or inf values that indicate potential issues
    if (std::isfinite(rtY.Xm1) && std::isfinite(rtY.Ym1) && std::isfinite(rtY.Zm1) &&
        std::isfinite(rtY.alpharadians1) && std::isfinite(rtY.betaradians1) && std::isfinite(rtY.gammaradians1)) {
      
      next_state_[0] = static_cast<Scalar>(rtY.Xm1);           // x position
      next_state_[1] = static_cast<Scalar>(rtY.Ym1);           // y position
      next_state_[2] = static_cast<Scalar>(rtY.Zm1);           // z position
      next_state_[3] = static_cast<Scalar>(rtY.alpharadians1); // roll
      next_state_[4] = static_cast<Scalar>(rtY.betaradians1);  // pitch
      next_state_[5] = static_cast<Scalar>(rtY.gammaradians1); // yaw
    } else {
      // If we detect NaN values, use current state as fallback
      // This indicates a problem with the controller model
      std::cerr << "Warning: NaN values detected in Simulink controller output, using current state" << std::endl;
      next_state_ = x_;
    }
  }

  const Eigen::Matrix<Scalar, M, 1>& get_u0() const { return u_; }
  
  // Get the next state as computed by the controller
  const Eigen::Matrix<Scalar, N, 1>& get_next_state() const { return next_state_; }

private:
  Eigen::Matrix<Scalar, N, 1> x_ {}, x_ref_ {}, next_state_ {};
  Eigen::Matrix<Scalar, M, 1> u_ {};
  
  // Time step
  Scalar dt_;
};

namespace EntoControl
{
  using AdaptiveController = AdaptiveControllerStep<AdaptiveControllerTraits>;
}

#endif // ENTO_CONTROL_ADAPTIVE_CONTROLLER_H 