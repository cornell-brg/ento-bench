#ifndef ENTO_GEOMETRIC_CONTROLLER_H
#define ENTO_GEOMETRIC_CONTROLLER_H

#include <ento-math/core.h>
#include <ento-util/debug.h>
#include <Eigen/Dense>
#include <cmath>
#include <type_traits>

namespace EntoControl
{

/**
 * @brief Vehicle configuration traits for compile-time specialization
 */
template <typename Scalar>
struct QuadrotorTraits
{
  using Scalar_ = Scalar;
  static constexpr int StateDim = 13;    // [pos(3), vel(3), R(9), W(3)]
  static constexpr int ControlDim = 4;   // [thrust, torque_x, torque_y, torque_z]
  static constexpr int PositionDim = 3;
  static constexpr int VelocityDim = 3;
  static constexpr int AttitudeDim = 9;  // SO(3) rotation matrix
  static constexpr int AngularVelDim = 3;
  
  // Python simulation parameters to match FDCL reference
  static constexpr Scalar mass() { return Scalar(1.95); }  // kg (from Python simulation)
  static constexpr Scalar gravity() { return Scalar(9.81); }  // m/s^2
  static constexpr Scalar arm_length() { return Scalar(0.315); }  // m (d parameter)
  static constexpr Scalar thrust_to_torque_ratio() { return Scalar(8.004e-4); }  // cτf parameter
  
  // Python simulation inertia matrix J = diag([0.02, 0.02, 0.04])
  static EntoMath::Matrix3x3<Scalar> inertia_matrix() {
    EntoMath::Matrix3x3<Scalar> J;
    J << Scalar(0.02), Scalar(0.0),  Scalar(0.0),
         Scalar(0.0),  Scalar(0.02), Scalar(0.0),
         Scalar(0.0),  Scalar(0.0),  Scalar(0.04);
    return J;
  }
};

/**
 * @brief RoboBee configuration specialization
 */
template <typename Scalar>
struct RoboBeeTraits
{
  using Scalar_ = Scalar;
  static constexpr int StateDim = 13;
  static constexpr int ControlDim = 4;
  static constexpr int PositionDim = 3;
  static constexpr int VelocityDim = 3;
  static constexpr int AttitudeDim = 9;
  static constexpr int AngularVelDim = 3;
  
  // RoboBee-specific parameters
  static constexpr Scalar mass() { return Scalar(0.100e-3); }  // 100mg
  static constexpr Scalar gravity() { return Scalar(9.81); }   // m/s^2
  static constexpr Scalar arm_length() { return Scalar(3.0e-3); }  // 3mm
  static constexpr Scalar thrust_to_torque_ratio() { return Scalar(0.001); }  // Estimated for RoboBee
  
  static EntoMath::Matrix3x3<Scalar> inertia_matrix() {
    EntoMath::Matrix3x3<Scalar> J;
    J.setIdentity();
    J *= Scalar(1.0e-9); // Very small for RoboBee
    return J;
  }
};

/**
 * @brief Control gains configuration (matching FDCL implementation exactly)
 */
template <typename Scalar>
struct ControlGains
{
  using Matrix3 = EntoMath::Matrix3x3<Scalar>;
  using Vector3 = EntoMath::Vec3<Scalar>;
  
  // Position control gains (diagonal matrices like FDCL)
  Matrix3 kX = Matrix3::Zero();   // Position gains
  Matrix3 kV = Matrix3::Zero();   // Velocity gains
  
  // Attitude control gains (diagonal matrices like FDCL)
  Matrix3 kR = Matrix3::Zero();   // Attitude gains
  Matrix3 kW = Matrix3::Zero();   // Angular rate gains
  
  // Integral control gains (scalars like FDCL)
  Scalar kIX = Scalar(4.0);    // Position integral gain
  Scalar ki = Scalar(0.01);    // Position integral coupling
  Scalar kIR = Scalar(0.015);  // Attitude integral gain
  Scalar kI = Scalar(0.01);    // Roll/pitch integral gain
  Scalar kyI = Scalar(0.02);   // Yaw integral gain
  
  // Integral control parameters (scalars like FDCL)
  Scalar c1 = Scalar(1.0);     // Position integral coupling
  Scalar c2 = Scalar(1.0);     // Attitude integral coupling
  Scalar c3 = Scalar(1.0);     // Decoupled yaw integral coupling
  
  // Constructor to initialize diagonal matrices with Python simulation values
  ControlGains() {
    // Python simulation parameters: kX=8.0, kV=4.0, kR=0.5, kW=0.1
    kX(0,0) = Scalar(8.0); kX(1,1) = Scalar(8.0); kX(2,2) = Scalar(8.0);
    kV(0,0) = Scalar(4.0); kV(1,1) = Scalar(4.0); kV(2,2) = Scalar(4.0);
    
    // Attitude gains from Python simulation
    kR(0,0) = Scalar(0.5); kR(1,1) = Scalar(0.5); kR(2,2) = Scalar(0.5);
    kW(0,0) = Scalar(0.1); kW(1,1) = Scalar(0.1); kW(2,2) = Scalar(0.1);
  }
  
  // Convenience setters for diagonal values
  void set_kX(Scalar x, Scalar y, Scalar z) {
    kX.setZero(); kX(0,0) = x; kX(1,1) = y; kX(2,2) = z;
  }
  
  void set_kV(Scalar x, Scalar y, Scalar z) {
    kV.setZero(); kV(0,0) = x; kV(1,1) = y; kV(2,2) = z;
  }
  
  void set_kR(Scalar x, Scalar y, Scalar z) {
    kR.setZero(); kR(0,0) = x; kR(1,1) = y; kR(2,2) = z;
  }
  
  void set_kW(Scalar x, Scalar y, Scalar z) {
    kW.setZero(); kW(0,0) = x; kW(1,1) = y; kW(2,2) = z;
  }
};

/**
 * @brief State representation for geometric controller
 */
template <typename Scalar, typename VehicleTraits = QuadrotorTraits<Scalar>>
struct GeometricState
{
  using Vector3 = EntoMath::Vec3<Scalar>;
  using Matrix3 = EntoMath::Matrix3x3<Scalar>;
  using Scalar_ = Scalar;
  using Traits = VehicleTraits;
  
  Vector3 position = Vector3::Zero();     // Position in world frame
  Vector3 velocity = Vector3::Zero();     // Velocity in world frame 
  Vector3 acceleration = Vector3::Zero(); // Acceleration in world frame
  Matrix3 rotation = Matrix3::Identity(); // Rotation matrix (SO(3))
  Vector3 angular_velocity = Vector3::Zero(); // Angular velocity in body frame
  
  // Convenience accessors following FDCL naming
  const Vector3& x() const { return position; }
  const Vector3& v() const { return velocity; }
  const Vector3& a() const { return acceleration; }
  const Matrix3& R() const { return rotation; }
  const Vector3& W() const { return angular_velocity; }
  
  Vector3& x() { return position; }
  Vector3& v() { return velocity; }
  Vector3& a() { return acceleration; }
  Matrix3& R() { return rotation; }
  Vector3& W() { return angular_velocity; }
  
  // Body frame vectors
  Vector3 b1() const { return rotation.col(0); }
  Vector3 b2() const { return rotation.col(1); }
  Vector3 b3() const { return rotation.col(2); }
};

/**
 * @brief Command/reference trajectory for geometric controller
 */
template <typename Scalar, typename VehicleTraits = QuadrotorTraits<Scalar>>
struct GeometricCommand
{
  using Vector3 = EntoMath::Vec3<Scalar>;
  using Matrix3 = EntoMath::Matrix3x3<Scalar>;
  using Scalar_ = Scalar;
  using Traits = VehicleTraits;
  
  // Position trajectory
  Vector3 position_d = Vector3::Zero();      // xd
  Vector3 velocity_d = Vector3::Zero();      // xd_dot
  Vector3 acceleration_d = Vector3::Zero();  // xd_2dot
  Vector3 jerk_d = Vector3::Zero();          // xd_3dot
  Vector3 snap_d = Vector3::Zero();          // xd_4dot
  
  // Attitude trajectory  
  Matrix3 rotation_d = Matrix3::Identity();  // Rd
  Vector3 angular_velocity_d = Vector3::Zero(); // Wd
  Vector3 angular_acceleration_d = Vector3::Zero(); // Wd_dot
  
  // Desired body frame directions
  Vector3 b1_d = Vector3::UnitX();           // Desired x-axis direction
  Vector3 b1_d_dot = Vector3::Zero();        // Rate of b1_d
  Vector3 b1_d_ddot = Vector3::Zero();       // Acceleration of b1_d
  
  Vector3 b3_d = Vector3::UnitZ();           // Desired z-axis direction
  Vector3 b3_d_dot = Vector3::Zero();        // Rate of b3_d  
  Vector3 b3_d_ddot = Vector3::Zero();       // Acceleration of b3_d
  
  // Computed intermediate variables
  Vector3 b1_c = Vector3::UnitX();           // Computed x-axis
  Scalar wc3 = Scalar(0);                    // Computed yaw rate
  Scalar wc3_dot = Scalar(0);                // Computed yaw acceleration
  
  // Convenience accessors
  const Vector3& xd() const { return position_d; }
  const Vector3& xd_dot() const { return velocity_d; }
  const Vector3& xd_2dot() const { return acceleration_d; }
  const Vector3& xd_3dot() const { return jerk_d; }
  const Vector3& xd_4dot() const { return snap_d; }
  const Matrix3& Rd() const { return rotation_d; }
  const Vector3& Wd() const { return angular_velocity_d; }
  const Vector3& Wd_dot() const { return angular_acceleration_d; }
  const Vector3& b1d() const { return b1_d; }
  const Vector3& b1d_dot() const { return b1_d_dot; }
  const Vector3& b1d_ddot() const { return b1_d_ddot; }
  const Vector3& b3d() const { return b3_d; }
  const Vector3& b3d_dot() const { return b3_d_dot; }
  const Vector3& b3d_ddot() const { return b3_d_ddot; }
  
  Vector3& xd() { return position_d; }
  Vector3& xd_dot() { return velocity_d; }
  Vector3& xd_2dot() { return acceleration_d; }
  Vector3& xd_3dot() { return jerk_d; }
  Vector3& xd_4dot() { return snap_d; }
  Matrix3& Rd() { return rotation_d; }
  Vector3& Wd() { return angular_velocity_d; }
  Vector3& Wd_dot() { return angular_acceleration_d; }
  Vector3& b1d() { return b1_d; }
  Vector3& b1d_dot() { return b1_d_dot; }
  Vector3& b1d_ddot() { return b1_d_ddot; }
  Vector3& b3d() { return b3_d; }
  Vector3& b3d_dot() { return b3_d_dot; }
  Vector3& b3d_ddot() { return b3_d_ddot; }
};

/**
 * @brief Control output from geometric controller
 */
template <typename Scalar, typename VehicleTraits = QuadrotorTraits<Scalar>>
struct GeometricControl
{
  using Vector3 = EntoMath::Vec3<Scalar>;
  using Vector4 = EntoMath::Vec4<Scalar>;
  using Scalar_ = Scalar;
  using Traits = VehicleTraits;
  
  Scalar thrust = Scalar(0);      // Total thrust magnitude
  Vector3 moment = Vector3::Zero(); // Control moments [Mx, My, Mz]
  Vector4 force_moment = Vector4::Zero(); // [thrust, Mx, My, Mz]
  Vector4 motor_forces = Vector4::Zero(); // Individual motor forces
  
  // Convenience accessors
  Scalar f() const { return thrust; }
  const Vector3& M() const { return moment; }
  const Vector4& fM() const { return force_moment; }
  
  Scalar& f() { return thrust; }
  Vector3& M() { return moment; }
  Vector4& fM() { return force_moment; }
  
  void update_force_moment() {
    force_moment << thrust, moment[0], moment[1], moment[2];
  }
};

/**
 * @brief Integral error tracking for geometric controller
 */
template <typename Scalar>
class IntegralError
{
public:
  using Vector3 = EntoMath::Vec3<Scalar>;
  
private:
  Vector3 error_ = Vector3::Zero();
  bool saturated_ = false;
  Scalar saturation_limit_ = Scalar(10.0);
  
public:
  const Vector3& error() const { return error_; }
  bool is_saturated() const { return saturated_; }
  
  void integrate(const Vector3& error_rate, Scalar dt) {
    error_ += error_rate * dt;
    
    // Apply saturation
    if (error_.norm() > saturation_limit_) {
      error_ = error_.normalized() * saturation_limit_;
      saturated_ = true;
    } else {
      saturated_ = false;
    }
  }
  
  void set_zero() {
    error_.setZero();
    saturated_ = false;
  }
  
  void set_saturation_limit(Scalar limit) {
    saturation_limit_ = limit;
  }
};

/**
 * @brief Heavily templated geometric controller with compile-time configuration
 * 
 * This controller implements the FDCL geometric controller with full template
 * parameterization for vehicle type, scalar type, and control method.
 * 
 * @tparam Scalar Floating point type (float, double)
 * @tparam VehicleTraits Vehicle configuration (QuadrotorTraits, RoboBeeTraits, etc.)
 * @tparam UseDecoupledYaw Whether to use decoupled yaw control (compile-time)
 * @tparam UseIntegralControl Whether to enable integral control terms (compile-time)
 */
template <
  typename Scalar, 
  typename VehicleTraits = QuadrotorTraits<Scalar>,
  bool UseDecoupledYaw = true,
  bool UseIntegralControl = true
>
class GeometricController
{
public:
  using Scalar_ = Scalar;
  using Traits = VehicleTraits;
  using State = GeometricState<Scalar, VehicleTraits>;
  using Command = GeometricCommand<Scalar, VehicleTraits>;
  using Control = GeometricControl<Scalar, VehicleTraits>;
  using Gains = ControlGains<Scalar>;
  
  using Vector3 = EntoMath::Vec3<Scalar>;
  using Matrix3 = EntoMath::Matrix3x3<Scalar>;
  using Vector4 = EntoMath::Vec4<Scalar>;
  
  static constexpr bool UseDecoupledYaw_ = UseDecoupledYaw;
  static constexpr bool UseIntegralControl_ = UseIntegralControl;
  
private:
  // Vehicle parameters (compile-time constants)
  static constexpr Scalar mass_ = VehicleTraits::mass();
  static constexpr Scalar gravity_ = VehicleTraits::gravity();
  static constexpr Scalar arm_length_ = VehicleTraits::arm_length();
  static const Matrix3 inertia_;
  
  // Control gains
  Gains gains_;
  
  // Integral error states (only used if UseIntegralControl is true)
  IntegralError<Scalar> position_integral_error_;
  IntegralError<Scalar> attitude_integral_error_;
  
  // Control errors (for debugging/analysis)
  Vector3 position_error_ = Vector3::Zero();
  Vector3 velocity_error_ = Vector3::Zero();
  Vector3 attitude_error_ = Vector3::Zero();
  Vector3 angular_velocity_error_ = Vector3::Zero();
  
  // World frame unit vectors (const for efficiency)
  static const Vector3 e1_;
  static const Vector3 e2_;
  static const Vector3 e3_;
  
  // Force-to-motor mapping matrix (computed once in constructor)
  Eigen::Matrix<Scalar, 4, 4> force_to_motor_inv_;
  
  // Thrust computed in position control (used by attitude control)
  Scalar control_thrust_ = Scalar(0);
  
public:
  /**
   * @brief Constructor with custom gains
   */
  explicit GeometricController(const Gains& gains = Gains{})
    : gains_(gains)
  {
    initialize_force_mapping();
    
    if constexpr (UseIntegralControl) {
      position_integral_error_.set_saturation_limit(Scalar(1.8));
      attitude_integral_error_.set_saturation_limit(Scalar(1.8));
    }
  }
  
  /**
   * @brief Main control computation
   * @param state Current vehicle state
   * @param command Desired trajectory/reference
   * @param dt Control timestep
   * @return Control output (thrust and moments)
   */
  Control compute_control(const State& state, const Command& command, Scalar dt)
  {
    Control control_output;
    
    // Mutable command for intermediate computations
    Command& cmd = const_cast<Command&>(command);
    
    // Step 1: Position control to generate desired attitude
    compute_position_control(state, cmd, dt);
    
    // Step 2: Attitude control to generate moments
    if constexpr (UseDecoupledYaw) {
      compute_attitude_control_decoupled_yaw(state, cmd, control_output, dt);
    } else {
      compute_attitude_control_coupled(state, cmd, control_output, dt);
    }
    
    // Step 3: Convert to motor forces if needed
    control_output.update_force_moment();
    control_output.motor_forces = force_to_motor_inv_ * control_output.fM();
    
    return control_output;
  }
  
  /**
   * @brief Reset integral error terms
   */
  void reset_integral_errors()
  {
    if constexpr (UseIntegralControl) {
      position_integral_error_.set_zero();
      attitude_integral_error_.set_zero();
    }
  }
  
  /**
   * @brief Get current control gains
   */
  const Gains& get_gains() const { return gains_; }
  
  /**
   * @brief Set control gains
   */
  void set_gains(const Gains& gains) { gains_ = gains; }
  
  /**
   * @brief Get current control errors (for analysis)
   */
  struct ErrorState {
    Vector3 position_error;
    Vector3 velocity_error;
    Vector3 attitude_error;
    Vector3 angular_velocity_error;
  };
  
  ErrorState get_errors() const {
    return ErrorState{
      .position_error = position_error_,
      .velocity_error = velocity_error_,
      .attitude_error = attitude_error_,
      .angular_velocity_error = angular_velocity_error_
    };
  }
  
private:
  /**
   * @brief Initialize force-to-motor mapping matrix
   */
  void initialize_force_mapping()
  {
    // FDCL quadrotor force/moment to motor force mapping
    // Motor layout (+ configuration):
    //     1
    //   2   4  
    //     3
    // 
    // Forward mapping: [F, Mx, My, Mz]^T = A * [f1, f2, f3, f4]^T
    // Where A = [1   1   1   1 ]
    //           [0  -l   0   l ]  (l = arm_length)
    //           [l   0  -l   0 ]
    //           [-c_tf  c_tf  -c_tf  c_tf]  (c_tf = thrust_to_torque_ratio)
    //
    // Inverse mapping: [f1, f2, f3, f4]^T = A^(-1) * [F, Mx, My, Mz]^T
    
    Scalar l = VehicleTraits::arm_length();
    Scalar c_tf = VehicleTraits::thrust_to_torque_ratio();
    
    // Build the forward mapping matrix first
    Eigen::Matrix<Scalar, 4, 4> fM_to_forces;
    fM_to_forces << Scalar(1.0),  Scalar(1.0),  Scalar(1.0),  Scalar(1.0),
                    Scalar(0.0), -l,            Scalar(0.0),  l,
                    l,           Scalar(0.0), -l,            Scalar(0.0),
                   -c_tf,        c_tf,        -c_tf,         c_tf;
    
    // Compute the inverse
    force_to_motor_inv_ = fM_to_forces.inverse();
  }
  
  /**
   * @brief Position control implementation (generates desired attitude)
   */
  void compute_position_control(const State& state, Command& command, Scalar dt);
  
  /**
   * @brief Coupled attitude control (original geometric controller)
   */
  void compute_attitude_control_coupled(const State& state, const Command& command, 
                                       Control& control_output, Scalar dt);
  
  /**
   * @brief Decoupled yaw attitude control (FDCL extension)
   */
  void compute_attitude_control_decoupled_yaw(const State& state, const Command& command, 
                                             Control& control_output, Scalar dt);
  
  /**
   * @brief Utility: Skew-symmetric matrix from vector
   */
  static Matrix3 hat(const Vector3& v) {
    return EntoMath::skew(v);
  }
  
  /**
   * @brief Utility: Vector from skew-symmetric matrix
   */
  static Vector3 vee(const Matrix3& M) {
    return Vector3(M(2,1), M(0,2), M(1,0));
  }
  
  /**
   * @brief Utility: Derivative of unit vector
   */
  static void deriv_unit_vector(const Vector3& A, const Vector3& A_dot, const Vector3& A_ddot,
                               Vector3& u, Vector3& u_dot, Vector3& u_ddot);
};

// Static member definitions
template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
const EntoMath::Matrix3x3<Scalar> GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::inertia_ = 
  VehicleTraits::inertia_matrix();

template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
const EntoMath::Vec3<Scalar> GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::e1_ = 
  EntoMath::Vec3<Scalar>::UnitX();

template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
const EntoMath::Vec3<Scalar> GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::e2_ = 
  EntoMath::Vec3<Scalar>::UnitY();

template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
const EntoMath::Vec3<Scalar> GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::e3_ = 
  EntoMath::Vec3<Scalar>::UnitZ();

} // namespace EntoControl

#endif // ENTO_GEOMETRIC_CONTROLLER_H 