#ifndef ENTO_CONTROL_ROBOBEE_MPC_H
#define ENTO_CONTROL_ROBOBEE_MPC_H

#include <Eigen/Dense>

// Include the uprightmpc2 C headers directly
extern "C" {
#include "uprightmpc2.h"
}

namespace EntoControl {

/**
 * @brief RoboBee MPC controller using the uprightmpc2 template controller
 * 
 * This wraps the template MPC controller from external/robobee3d/template/uprightmpc2
 * to provide a clean C++ interface for ento-bench.
 * 
 * State vector (12D): [position(3), orientation_vector(3), velocity(3), angular_velocity(3)]
 * Control vector (3D): [thrust, pitch_moment, roll_moment]
 */
class RoboBeeMPC {
public:
  using State = Eigen::Matrix<float, 12, 1>;
  using Control = Eigen::Matrix<float, 3, 1>;

  /**
   * @brief Constructor with default parameters
   */
  RoboBeeMPC();

  /**
   * @brief Constructor with custom parameters
   * @param dt Control timestep (ms)
   * @param g Gravity acceleration (mm/ms^2)
   * @param TtoWmax Thrust to weight ratio maximum
   * @param ws Position weight
   * @param wds Velocity weight  
   * @param wpr Position weight (running cost)
   * @param wpf Position weight (final cost)
   * @param wvr Velocity weight (running cost)
   * @param wvf Velocity weight (final cost)
   * @param wthrust Thrust control weight
   * @param wmom Moment control weight
   * @param Ib Inertia matrix diagonal [Ixx, Iyy, Izz] (g*mm^2)
   * @param maxIter Maximum OSQP iterations
   */
  RoboBeeMPC(float dt, float g, float TtoWmax, float ws, float wds, 
             float wpr, float wpf, float wvr, float wvf, 
             float wthrust, float wmom, const float Ib[3], int maxIter);

  /**
   * @brief Destructor
   */
  ~RoboBeeMPC();

  /**
   * @brief Set the current state
   * @param x0 Current state [pos(3), orient_vec(3), vel(3), ang_vel(3)]
   */
  void set_x0(const State& x0);

  /**
   * @brief Set the reference state
   * @param x_ref Reference state
   */
  void set_x_ref(const State& x_ref);

  /**
   * @brief Solve the MPC optimization problem
   * @return true if solved successfully
   */
  bool solve();

  /**
   * @brief Get the optimal control input
   * @return Control input [thrust, pitch_moment, roll_moment]
   */
  Control get_u0() const;

  /**
   * @brief Get solver status
   * @return OSQP status code
   */
  int get_status() const;

  /**
   * @brief Get number of solver iterations
   * @return Number of iterations
   */
  int get_iterations() const;

  /**
   * @brief Simulate forward dynamics
   * @param x0 Current state
   * @param u Control input
   * @param dt Time step
   * @return Next state
   */
  State simulate_forward(const State& x0, const Control& u, float dt) const;

  /**
   * @brief Set reference trajectory (position and velocity)
   * @param pdes Desired position (mm)
   * @param dpdes Desired velocity (mm/ms)
   * @param sdes Desired orientation vector (unit vector)
   */
  void set_reference_trajectory(const Eigen::Vector3f& pdes, 
                               const Eigen::Vector3f& dpdes,
                               const Eigen::Vector3f& sdes);

private:
  UprightMPC_t mpc_controller_;   ///< The C MPC controller (stack allocated)
  State x0_;                      ///< Current state
  State x_ref_;                   ///< Reference state
  Control u0_;                    ///< Current control input
  
  // Reference trajectory components
  Eigen::Vector3f pdes_, dpdes_, sdes_;
  
  // Internal state: maintain full rotation matrix like original Python code
  mutable Eigen::Matrix3f Rb_internal_;
  
  // Control output and status
  int last_status_;
  int last_iterations_;
  
  /**
   * @brief Convert orientation vector to rotation matrix and store internally
   */
  void orientation_vector_to_rotation_matrix(const Eigen::Vector3f& orient_vec, Eigen::Matrix3f& R) const;
  
  /**
   * @brief Convert orientation vector to rotation matrix (legacy method for array output)
   */
  void orientation_vector_to_rotation_matrix(const Eigen::Vector3f& orient_vec, float R[9]) const;
};

} // namespace EntoControl

#endif // ENTO_CONTROL_ROBOBEE_MPC_H 