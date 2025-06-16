#include "geometric_controller.h"
#include <ento-util/debug.h>

namespace EntoControl
{

// ============================================================================
// UTILITY FUNCTIONS (HEADER-ONLY IMPLEMENTATIONS)
// ============================================================================

template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
void GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::
deriv_unit_vector(const Vector3& A, const Vector3& A_dot, const Vector3& A_ddot,
                  Vector3& u, Vector3& u_dot, Vector3& u_ddot)
{
  Scalar A_norm = A.norm();
  
  if (A_norm < Scalar(1e-8)) {
    // Handle near-zero case
    u = Vector3::UnitZ();
    u_dot = Vector3::Zero();
    u_ddot = Vector3::Zero();
    return;
  }
  
  u = A / A_norm;
  
  Scalar A_norm_inv = Scalar(1) / A_norm;
  Scalar A_norm_inv3 = A_norm_inv * A_norm_inv * A_norm_inv;
  
  u_dot = A_norm_inv * A_dot - A_norm_inv3 * A.dot(A_dot) * A;
  
  Scalar A_dot_norm_sq = A_dot.squaredNorm();
  Scalar A_A_dot = A.dot(A_dot);
  Scalar A_A_ddot = A.dot(A_ddot);
  
  u_ddot = A_norm_inv * A_ddot 
         - A_norm_inv3 * (A_A_ddot + A_dot_norm_sq) * A
         - A_norm_inv3 * A_A_dot * A_dot
         + Scalar(3) * A_norm_inv3 * A_norm_inv * A_norm_inv * A_A_dot * A_A_dot * A;
}

// ============================================================================
// POSITION CONTROL IMPLEMENTATION
// ============================================================================

template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
void GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::
compute_position_control(const State& state, Command& command, Scalar dt)
{
  // EXACT COPY OF FDCL position_control() function
  
  // translational error functions
  Vector3 eX = state.x() - command.xd();     // position error - eq (11)
  Vector3 eV = state.v() - command.xd_dot(); // velocity error - eq (12)
  
  // Store for debugging
  position_error_ = eX;
  velocity_error_ = eV;

  // position integral terms
  Vector3 integral_error = Vector3::Zero();
  if constexpr (UseIntegralControl) {
    position_integral_error_.integrate(gains_.c1 * eX + eV, dt); // eq (13)
    integral_error = gains_.kIX * position_integral_error_.error();
  }

  // Step 3: Compute desired acceleration (force direction) - FDCL implementation
  Vector3 A = -gains_.kX * position_error_ - gains_.kV * velocity_error_ - integral_error - mass_ * gravity_ * e3_ + mass_ * command.xd_2dot();
  
  // DEBUG: Print A vector computation (matching FDCL debug output)
  if (false) {  // Disable debug for now
    printf("[ENTO CONTROLLER DEBUG] A vector computation:\n");
    printf("  position_error = [%.6f, %.6f, %.6f]\n", position_error_[0], position_error_[1], position_error_[2]);
    Vector3 kX_term = -gains_.kX * position_error_;
    Vector3 kV_term = -gains_.kV * velocity_error_;
    Vector3 gravity_term = -mass_ * gravity_ * e3_;
    Vector3 accel_term = mass_ * command.xd_2dot();
    printf("  -kX * pos_err = [%.6f, %.6f, %.6f]\n", kX_term[0], kX_term[1], kX_term[2]);
    printf("  -kV * vel_err = [%.6f, %.6f, %.6f]\n", kV_term[0], kV_term[1], kV_term[2]);
    printf("  -m*g*e3 = [%.6f, %.6f, %.6f]\n", gravity_term[0], gravity_term[1], gravity_term[2]);
    printf("  m*accel_d = [%.6f, %.6f, %.6f]\n", accel_term[0], accel_term[1], accel_term[2]);
    printf("  A = [%.6f, %.6f, %.6f] (norm=%.6f)\n", A[0], A[1], A[2], A.norm());
  }
  
  // Step 4: Extract current body frame vectors
  Vector3 b3 = state.R() * e3_;
  Vector3 b3_dot = state.R() * hat(state.W()) * e3_;
  
  // Step 5: Compute total thrust
  Scalar f_total = -A.dot(b3);
  
  // DEBUG: Print thrust computation (matching FDCL debug output)
  if (false) {  // Disable debug for now
    printf("  b3 = [%.6f, %.6f, %.6f]\n", b3[0], b3[1], b3[2]);
    printf("  A.dot(b3) = %.6f\n", A.dot(b3));
    printf("  f_total = -A.dot(b3) = %.6f\n", f_total);
  }

  // intermediate terms for rotational errors
  Vector3 ea = gravity_ * e3_ - f_total / mass_ * b3 - command.xd_2dot();
  Vector3 A_dot = -gains_.kX * eV - gains_.kV * ea + mass_ * command.xd_3dot();

  Scalar fdot = -A_dot.dot(b3) - A.dot(b3_dot);
  Vector3 eb = -fdot / mass_ * b3 - f_total / mass_ * b3_dot - command.xd_3dot();
  Vector3 A_ddot = -gains_.kX * ea - gains_.kV * eb + mass_ * command.xd_4dot();

  Vector3 b3c, b3c_dot, b3c_ddot;
  deriv_unit_vector(-A, -A_dot, -A_ddot, b3c, b3c_dot, b3c_ddot);

  Vector3 A2 = -hat(command.b1d()) * b3c;
  Vector3 A2_dot = -hat(command.b1d_dot()) * b3c - hat(command.b1d()) * b3c_dot;
  Vector3 A2_ddot = -hat(command.b1d_ddot()) * b3c
                  - Scalar(2.0) * hat(command.b1d_dot()) * b3c_dot
                  - hat(command.b1d()) * b3c_ddot;

  Vector3 b2c, b2c_dot, b2c_ddot;
  deriv_unit_vector(A2, A2_dot, A2_ddot, b2c, b2c_dot, b2c_ddot);

  Vector3 b1c = hat(b2c) * b3c;
  Vector3 b1c_dot = hat(b2c_dot) * b3c + hat(b2c) * b3c_dot;
  Vector3 b1c_ddot = hat(b2c_ddot) * b3c
                   + Scalar(2.0) * hat(b2c_dot) * b3c_dot
                   + hat(b2c) * b3c_ddot;

  Matrix3 Rd_dot, Rd_ddot;
  command.Rd() << b1c, b2c, b3c;
  Rd_dot << b1c_dot, b2c_dot, b3c_dot;
  Rd_ddot << b1c_ddot, b2c_ddot, b3c_ddot;

  command.Wd() = vee(command.Rd().transpose() * Rd_dot);
  command.Wd_dot() = vee(command.Rd().transpose() * Rd_ddot
                       - hat(command.Wd()) * hat(command.Wd()));

  // roll / pitch
  command.b3d() = b3c;
  command.b3d_dot() = b3c_dot;
  command.b3d_ddot() = b3c_ddot;

  // yaw
  command.b1_c = b1c;
  command.wc3 = e3_.dot(state.R().transpose() * command.Rd() * command.Wd());
  command.wc3_dot = e3_.dot(state.R().transpose() * command.Rd() * command.Wd_dot())
                  - e3_.dot(hat(state.W()) * state.R().transpose() * command.Rd() * command.Wd());

  // Store thrust for attitude control
  control_thrust_ = f_total;
}

// ============================================================================
// COUPLED ATTITUDE CONTROL IMPLEMENTATION
// ============================================================================

template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
void GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::
compute_attitude_control_coupled(const State& state, const Command& command, 
                                Control& control_output, Scalar dt)
{
  // EXACT COPY OF FDCL attitude_control() function
  //  This uses the controller defined in "Control of Complex Maneuvers
  //  for a Quadrotor UAV using Geometric Methods on SE(3)"
  //  URL: https://arxiv.org/pdf/1003.2005.pdf
  
  Matrix3 RdtR = command.Rd().transpose() * state.R();
  Vector3 eR = Scalar(0.5) * vee(RdtR - RdtR.transpose());
  Vector3 eW = state.W() - state.R().transpose() * command.Rd() * command.Wd();
  
  // Store for debugging
  attitude_error_ = eR;
  angular_velocity_error_ = eW;

  Vector3 integral_error = Vector3::Zero();
  if constexpr (UseIntegralControl) {
    attitude_integral_error_.integrate(eW + gains_.c2 * eR, dt);
    integral_error = gains_.kI * attitude_integral_error_.error();
  }

  Vector3 Wd_body = state.R().transpose() * command.Rd() * command.Wd();
  Vector3 Wd_dot_body = state.R().transpose() * command.Rd() * command.Wd_dot();

  Vector3 M = -gains_.kR * eR
            - gains_.kW * eW
            - integral_error
            + hat(Wd_body) * inertia_ * Wd_body
            + inertia_ * Wd_dot_body;

  control_output.thrust = control_thrust_;
  control_output.moment = M;
}

// ============================================================================
// DECOUPLED YAW ATTITUDE CONTROL IMPLEMENTATION  
// ============================================================================

template <typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
void GeometricController<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>::
compute_attitude_control_decoupled_yaw(const State& state, const Command& command,
                                      Control& control_output, Scalar dt)
{
  // EXACT COPY OF FDCL attitude_control_decoupled_yaw() function
  // This uses the controller defined in "Geometric Controls of a Quadrotor
  // with a Decoupled Yaw Control" 
  // URL: https://doi.org/10.23919/ACC.2019.8815189
  
  Vector3 b1 = state.R() * e1_;
  Vector3 b2 = state.R() * e2_;
  Vector3 b3 = state.R() * e3_;

  Scalar ky = gains_.kR(2, 2);
  Scalar kwy = gains_.kW(2, 2);

  // roll/pitch angular velocity vector
  Vector3 W_12 = state.W()(0) * b1 + state.W()(1) * b2;
  Vector3 b3_dot = hat(W_12) * b3; // eq (26)

  Vector3 W_12d = hat(command.b3d()) * command.b3d_dot();
  Vector3 W_12d_dot = hat(command.b3d()) * command.b3d_ddot();

  Vector3 eb = hat(command.b3d()) * b3;           // eq (27)
  Vector3 ew = W_12 + hat(b3) * hat(b3) * W_12d; // eq (28)

  // yaw
  Scalar ey = -b2.dot(command.b1_c);
  Scalar ewy = state.W()(2) - command.wc3;

  // attitude integral terms
  Vector3 eI = ew + gains_.c2 * eb;

  Vector3 integral_b1_b2 = Vector3::Zero();
  Scalar integral_yaw = Scalar(0);
  
  if constexpr (UseIntegralControl) {
    // For simplicity, we'll use the same integral error object but extract components
    Vector3 integral_input;
    integral_input(0) = eI.dot(b1);  // b1 axis integral
    integral_input(1) = eI.dot(b2);  // b2 axis integral  
    integral_input(2) = ewy + gains_.c3 * ey;  // yaw integral
    
    attitude_integral_error_.integrate(integral_input, dt);
    Vector3 integral_errors = attitude_integral_error_.error();
    
    integral_b1_b2 = gains_.kI * integral_errors(0) * b1 + gains_.kI * integral_errors(1) * b2;
    integral_yaw = gains_.kyI * integral_errors(2);
  }

  // control moment for the roll/pitch dynamics - eq (31)
  Vector3 tau = -gains_.kR(0, 0) * eb
              - gains_.kW(0, 0) * ew
              - inertia_(0, 0) * b3.transpose() * W_12d * b3_dot
              - inertia_(0, 0) * hat(b3) * hat(b3) * W_12d_dot;
  
  if constexpr (UseIntegralControl) {
    tau -= integral_b1_b2;
  }

  // control moment around b1 axis - roll - eq (24)
  Scalar M1 = b1.transpose() * tau + inertia_(2, 2) * state.W()(2) * state.W()(1);

  // control moment around b2 axis - pitch - eq (24)
  Scalar M2 = b2.transpose() * tau - inertia_(2, 2) * state.W()(2) * state.W()(0);

  // control moment around b3 axis - yaw - eq (52)
  Scalar M3 = -ky * ey - kwy * ewy + inertia_(2, 2) * command.wc3_dot;
  if constexpr (UseIntegralControl) {
    M3 -= integral_yaw;
  }

  Vector3 M(M1, M2, M3);

  control_output.thrust = control_thrust_;
  control_output.moment = M;

  // Store errors for analysis (matching FDCL format)
  Matrix3 RdtR = command.Rd().transpose() * state.R();
  attitude_error_ = Scalar(0.5) * vee(RdtR - RdtR.transpose());
  angular_velocity_error_ = state.W() - state.R().transpose() * command.Rd() * command.Wd();
}

// ============================================================================
// EXPLICIT TEMPLATE INSTANTIATIONS
// ============================================================================

// Common instantiations for float precision
template class GeometricController<float, QuadrotorTraits<float>, true, true>;
template class GeometricController<float, QuadrotorTraits<float>, true, false>;
template class GeometricController<float, QuadrotorTraits<float>, false, true>;
template class GeometricController<float, QuadrotorTraits<float>, false, false>;

template class GeometricController<float, RoboBeeTraits<float>, true, true>;
template class GeometricController<float, RoboBeeTraits<float>, true, false>;
template class GeometricController<float, RoboBeeTraits<float>, false, true>;
template class GeometricController<float, RoboBeeTraits<float>, false, false>;

// Common instantiations for double precision
template class GeometricController<double, QuadrotorTraits<double>, true, true>;
template class GeometricController<double, QuadrotorTraits<double>, true, false>;
template class GeometricController<double, QuadrotorTraits<double>, false, true>;
template class GeometricController<double, QuadrotorTraits<double>, false, false>;

template class GeometricController<double, RoboBeeTraits<double>, true, true>;
template class GeometricController<double, RoboBeeTraits<double>, true, false>;
template class GeometricController<double, RoboBeeTraits<double>, false, true>;
template class GeometricController<double, RoboBeeTraits<double>, false, false>;

} // namespace EntoControl 