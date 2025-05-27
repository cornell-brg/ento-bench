#include "robobee_mpc.h"
#include <ento-util/debug.h>
#include <ento-math/core.h>
#include <cmath>
#include <cstring>

namespace EntoControl {

RoboBeeMPC::RoboBeeMPC() 
  : pdes_(Eigen::Vector3f::Zero()),
    dpdes_(Eigen::Vector3f::Zero()),
    sdes_(Eigen::Vector3f::UnitZ()),
    Rb_internal_(Eigen::Matrix3f::Identity()),
    last_status_(0),
    last_iterations_(0) {
  // Initialize the uprightmpc2 controller with default parameters
  float Ib[3] = {3333.0f, 3333.0f, 1000.0f}; // Inertia matrix diagonal
  umpcInit(&mpc_controller_, 
           5.0f,      // dt = 5ms
           9.81e-3f,  // g = 9.81 mm/ms^2
           3.0f,      // TtoWmax = 3
           1e1f,      // ws = 10
           1e3f,      // wds = 1000
           1.0f,      // wpr = 1
           5.0f,      // wpf = 5
           1e3f,      // wvr = 1000
           2e3f,      // wvf = 2000
           1e-1f,     // wthrust = 0.1
           1e-2f,     // wmom = 0.01
           Ib,        // Ib inertia
           50);       // maxIter = 50
}

RoboBeeMPC::RoboBeeMPC(float dt, float g, float TtoWmax, float ws, float wds, 
                       float wpr, float wpf, float wvr, float wvf, 
                       float wthrust, float wmom, const float Ib[3], int maxIter) 
  : x0_(State::Zero()), x_ref_(State::Zero()), u0_(Control::Zero()),
    pdes_(Eigen::Vector3f::Zero()), dpdes_(Eigen::Vector3f::Zero()), 
    sdes_(Eigen::Vector3f::UnitZ()), last_status_(0), last_iterations_(0) {
  
  // Initialize the C MPC controller (stack allocated)
  umpcInit(&mpc_controller_, dt, g, TtoWmax, ws, wds, wpr, wpf, wvr, wvf, 
           wthrust, wmom, Ib, maxIter);
  
  ENTO_DEBUG("RoboBeeMPC initialized with dt=%.3f, g=%.6f, TtoWmax=%.1f", dt, g, TtoWmax);
}

RoboBeeMPC::~RoboBeeMPC() {
  // No cleanup needed for stack-allocated struct
}

void RoboBeeMPC::set_x0(const State& x0) {
  x0_ = x0;
  
  // Extract orientation vector and convert to full rotation matrix
  // Store this internally to avoid reconstruction issues
  Eigen::Vector3f orient_vec = x0.segment<3>(3);
  orientation_vector_to_rotation_matrix(orient_vec, Rb_internal_);
}

void RoboBeeMPC::set_x_ref(const State& x_ref) {
  x_ref_ = x_ref;
  
  // Extract reference trajectory components from state
  pdes_ = x_ref.segment<3>(0);    // Position
  dpdes_ = x_ref.segment<3>(6);   // Velocity
  
  // Convert orientation vector to unit vector for sdes
  Eigen::Vector3f orient_vec = x_ref.segment<3>(3);
  if (orient_vec.norm() > 1e-6f) {
    sdes_ = orient_vec.normalized();
  } else {
    sdes_ = Eigen::Vector3f::UnitZ(); // Default to z-axis
  }
}

void RoboBeeMPC::set_reference_trajectory(const Eigen::Vector3f& pdes, 
                                          const Eigen::Vector3f& dpdes,
                                          const Eigen::Vector3f& sdes) {
  pdes_ = pdes;
  dpdes_ = dpdes;
  sdes_ = sdes.normalized(); // Ensure unit vector
}

bool RoboBeeMPC::solve() {
  // Convert current state to uprightmpc2 format
  float p0[3], R0[9], dq0[6];
  
  // Extract position
  p0[0] = x0_[0];
  p0[1] = x0_[1]; 
  p0[2] = x0_[2];
  
  // Use internal rotation matrix (already computed in set_x0)
  // Store in column-major order for uprightmpc2
  R0[0] = Rb_internal_(0,0); R0[1] = Rb_internal_(1,0); R0[2] = Rb_internal_(2,0);  // 1st column
  R0[3] = Rb_internal_(0,1); R0[4] = Rb_internal_(1,1); R0[5] = Rb_internal_(2,1);  // 2nd column  
  R0[6] = Rb_internal_(0,2); R0[7] = Rb_internal_(1,2); R0[8] = Rb_internal_(2,2);  // 3rd column
  
  // Extract velocity and angular velocity
  dq0[0] = x0_[6];  dq0[1] = x0_[7];  dq0[2] = x0_[8];   // velocity
  dq0[3] = x0_[9];  dq0[4] = x0_[10]; dq0[5] = x0_[11];  // angular velocity
  
  // Validate state before solving
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(p0[i]) || !std::isfinite(dq0[i]) || !std::isfinite(dq0[i+3])) {
      ENTO_DEBUG("Invalid state before MPC solve: p0[%d]=%f, dq0[%d]=%f, dq0[%d]=%f", 
                 i, p0[i], i, dq0[i], i+3, dq0[i+3]);
      return false;
    }
  }
  for (int i = 0; i < 9; ++i) {
    if (!std::isfinite(R0[i])) {
      ENTO_DEBUG("Invalid rotation matrix before MPC solve: R0[%d]=%f", i, R0[i]);
      return false;
    }
  }
  
  // Debug: Log state before calling umpcUpdate
  ENTO_DEBUG("MPC input: p=[%.6f,%.6f,%.6f], dq=[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]", 
             p0[0], p0[1], p0[2], dq0[0], dq0[1], dq0[2], dq0[3], dq0[4], dq0[5]);
  ENTO_DEBUG("MPC input R0: [%.6f,%.6f,%.6f; %.6f,%.6f,%.6f; %.6f,%.6f,%.6f]",
             R0[0], R0[3], R0[6], R0[1], R0[4], R0[7], R0[2], R0[5], R0[8]);
  
  // Prepare outputs
  float uquad[3], accdes[6];
  
  // Call the uprightmpc2 update function
  // Note: actualT0 parameter is set to -1 to preserve accumulated thrust
  // (passing actualT0 >= 0 would overwrite the accumulated thrust)
  int status = umpcUpdate(&mpc_controller_, uquad, accdes, p0, R0, dq0, 
                         pdes_.data(), dpdes_.data(), sdes_.data(), -1.0f);
  
  // Validate MPC outputs
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(uquad[i])) {
      ENTO_DEBUG("Invalid MPC output: uquad[%d]=%f, status=%d", i, uquad[i], status);
      // Set to safe default values
      uquad[0] = 9.81e-3f; // hover thrust
      uquad[1] = 0.0f;     // no pitch moment
      uquad[2] = 0.0f;     // no roll moment
      status = 1; // mark as failed
      break;
    }
  }
  
  // Debug: Log control output
  if (status == 0) {
    ENTO_DEBUG("MPC output: uquad=[%.6f,%.6f,%.6f]", uquad[0], uquad[1], uquad[2]);
  }
  
  // Store results
  u0_ << uquad[0], uquad[1], uquad[2];
  last_status_ = status;
  
  // The uprightmpc2 returns 0 for success, 1 for failure
  bool success = (status == 0);
  
  if (!success) {
    ENTO_DEBUG("RoboBeeMPC solve failed with status: %d", status);
  }
  
  return success;
}

RoboBeeMPC::Control RoboBeeMPC::get_u0() const {
  return u0_;
}

int RoboBeeMPC::get_status() const {
  return last_status_;
}

int RoboBeeMPC::get_iterations() const {
  return last_iterations_;
}

RoboBeeMPC::State RoboBeeMPC::simulate_forward(const State& x0, const Control& u, float dt) const {
  // Template MPC nonlinear dynamics simulation
  // This exactly matches the original quadrotorNLDyn from genqp.py
  // State: [pos(3), orient_vec(3), vel(3), ang_vel(3)] = 12 states
  // Control: [thrust, pitch_moment, roll_moment] = 3 inputs
  // Units: position in mm, velocity in mm/ms, time in ms
  
  // Debug: Log input state and control
  ENTO_DEBUG("simulate_forward input: u=[%.6f,%.6f,%.6f], dt=%.3f", u[0], u[1], u[2], dt);
  ENTO_DEBUG("simulate_forward input state: pos=[%.6f,%.6f,%.6f], vel=[%.6f,%.6f,%.6f]", 
             x0[0], x0[1], x0[2], x0[6], x0[7], x0[8]);
  
  // Physical parameters (matching template MPC)
  const float g = 9.81e-3f; // gravity in mm/ms²
  
  // Extract state components
  EntoMath::Vec3<float> p = x0.template segment<3>(0);       // position (mm)
  EntoMath::Vec3<float> vel = x0.template segment<3>(6);     // velocity (mm/ms)
  EntoMath::Vec3<float> omega = x0.template segment<3>(9);   // angular velocity (rad/ms)
  
  // Use the internal rotation matrix (computed from orientation vector in set_x0)
  // This ensures consistency with what was passed to the MPC solver
  EntoMath::Matrix3x3<float> Rb = Rb_internal_;
  
  // Validate inputs
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(p[i]) || !std::isfinite(vel[i]) || !std::isfinite(omega[i])) {
      ENTO_DEBUG("Invalid input state at simulate_forward");
      return x0; // Return original state if invalid
    }
  }
  if (!std::isfinite(u[0]) || !std::isfinite(u[1]) || !std::isfinite(u[2])) {
    ENTO_DEBUG("Invalid control input at simulate_forward");
    return x0;
  }
  
  // === STEP 1: Compute accelerations (quadrotorNLVF) ===
  // dv = u[0] * Rb @ [0,0,1] - [0, 0, g]
  EntoMath::Vec3<float> e3(0, 0, 1);
  EntoMath::Vec3<float> thrust_accel = u[0] * (Rb * e3); // thrust along body z-axis
  EntoMath::Vec3<float> gravity_accel(0, 0, -g);
  EntoMath::Vec3<float> dv = thrust_accel + gravity_accel;
  
  // domega = Ib^-1 @ (-omega × (Ib @ omega) + [u[1], u[2], 0])
  // Template MPC inertia: Ib = diag([3333, 3333, 1000]) in template MPC units
  EntoMath::Vec3<float> Ib_diag(3333.0f, 3333.0f, 1000.0f); // Use original template MPC units
  
  EntoMath::Vec3<float> Ib_omega(Ib_diag[0] * omega[0], Ib_diag[1] * omega[1], Ib_diag[2] * omega[2]);
  EntoMath::Vec3<float> gyroscopic = -omega.cross(Ib_omega);
  EntoMath::Vec3<float> applied_moments(u[1], u[2], 0.0f); // [pitch, roll, yaw=0]
  
  EntoMath::Vec3<float> domega(
    (gyroscopic[0] + applied_moments[0]) / Ib_diag[0],
    (gyroscopic[1] + applied_moments[1]) / Ib_diag[1],
    (gyroscopic[2] + applied_moments[2]) / Ib_diag[2]
  );
  
  // Debug: Log computed accelerations
  ENTO_DEBUG("simulate_forward accel: dv=[%.6f,%.6f,%.6f], domega=[%.6f,%.6f,%.6f]", 
             dv[0], dv[1], dv[2], domega[0], domega[1], domega[2]);
  
  // === STEP 2: Euler integration (exactly matching quadrotorNLDyn) ===
  // p2 = p + dt * dq[0:3]  (where dq[0:3] = vel)
  EntoMath::Vec3<float> p_new = p + dt * vel;
  
  // Rb2 = Rb @ expm(skew(omega) * dt)
  EntoMath::Matrix3x3<float> omega_skew = EntoMath::skew(omega);
  EntoMath::Matrix3x3<float> omega_skew_dt = omega_skew * dt;
  
  // Matrix exponential for rotation update
  // For small dt, use second-order approximation: expm(A) ≈ I + A + A²/2
  EntoMath::Matrix3x3<float> I = EntoMath::Matrix3x3<float>::Identity();
  EntoMath::Matrix3x3<float> expm_omega_dt;
  
  float omega_dt_norm = omega_skew_dt.norm();
  if (omega_dt_norm < 1e-6f) {
    expm_omega_dt = I;
  } else if (omega_dt_norm < 0.1f) {
    // Second-order approximation
    EntoMath::Matrix3x3<float> omega_skew_dt_sq = omega_skew_dt * omega_skew_dt;
    expm_omega_dt = I + omega_skew_dt + 0.5f * omega_skew_dt_sq;
  } else {
    // Use Rodrigues' formula for larger rotations
    float omega_norm = omega.norm() * dt;
    if (omega_norm > 1e-6f) {
      float sin_omega = std::sin(omega_norm);
      float cos_omega = std::cos(omega_norm);
      float omega_norm_sq = omega_norm * omega_norm;
      
      EntoMath::Matrix3x3<float> omega_skew_dt_sq = omega_skew_dt * omega_skew_dt;
      expm_omega_dt = I + (sin_omega / omega_norm) * omega_skew_dt + 
                      ((1.0f - cos_omega) / omega_norm_sq) * omega_skew_dt_sq;
    } else {
      expm_omega_dt = I;
    }
  }
  
  EntoMath::Matrix3x3<float> Rb_new = Rb * expm_omega_dt;
  
  // dq2 = dq + dt * ddq  (where dq = [vel, omega], ddq = [dv, domega])
  EntoMath::Vec3<float> vel_new = vel + dt * dv;
  EntoMath::Vec3<float> omega_new = omega + dt * domega;
  
  // Debug: Log new state
  ENTO_DEBUG("simulate_forward output: pos=[%.6f,%.6f,%.6f], vel=[%.6f,%.6f,%.6f]", 
             p_new[0], p_new[1], p_new[2], vel_new[0], vel_new[1], vel_new[2]);
  
  // === STEP 3: Convert back to orientation vector format ===
  // Extract orientation vector from new rotation matrix: s = Rb[:,2]
  EntoMath::Vec3<float> s_new = Rb_new.col(2);
  
  // Validate outputs
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(p_new[i]) || !std::isfinite(s_new[i]) || 
        !std::isfinite(vel_new[i]) || !std::isfinite(omega_new[i])) {
      ENTO_DEBUG("Invalid output state at simulate_forward, returning original");
      return x0; // Return original state if invalid
    }
  }
  
  // Update internal rotation matrix for consistency
  // Note: This is important so that subsequent solve() calls use the correct rotation matrix
  // We need to cast away const to update the internal state
  const_cast<RoboBeeMPC*>(this)->Rb_internal_ = Rb_new;
  
  // Assemble next state
  State x_next;
  x_next.template segment<3>(0) = p_new;
  x_next.template segment<3>(3) = s_new;
  x_next.template segment<3>(6) = vel_new;
  x_next.template segment<3>(9) = omega_new;
  
  return x_next;
}

void RoboBeeMPC::orientation_vector_to_rotation_matrix(const Eigen::Vector3f& orient_vec, Eigen::Matrix3f& R) const {
  // The orientation vector represents the thrust direction (3rd column of rotation matrix)
  // For upright flight, orient_vec = [0,0,1] should give identity matrix
  
  // Normalize the orientation vector to get the z-axis (thrust direction)
  Eigen::Vector3f z_axis = orient_vec;
  float norm = z_axis.norm();
  
  if (norm < 1e-6f) {
    // Invalid orientation vector, use default upright orientation
    R = Eigen::Matrix3f::Identity();
    return;
  } else {
    z_axis = z_axis / norm;
  }
  
  // For the template MPC, we want a rotation matrix where:
  // - z_axis (3rd column) = normalized orientation vector (thrust direction)
  // - x_axis and y_axis complete the orthonormal basis
  
  // Choose x-axis to be orthogonal to z-axis
  // If z is close to [0,0,1], use [1,0,0] as reference
  // If z is close to [1,0,0], use [0,1,0] as reference
  Eigen::Vector3f x_axis;
  if (std::abs(z_axis[2]) > 0.9f) {
    // z is close to vertical, use x = [1,0,0] as reference
    x_axis = Eigen::Vector3f(1, 0, 0);
  } else {
    // z is not vertical, use z = [0,0,1] as reference
    x_axis = Eigen::Vector3f(0, 0, 1);
  }
  
  // Make x_axis orthogonal to z_axis using Gram-Schmidt
  x_axis = x_axis - (x_axis.dot(z_axis)) * z_axis;
  x_axis = x_axis.normalized();
  
  // y_axis = z_axis × x_axis (right-handed coordinate system)
  Eigen::Vector3f y_axis = z_axis.cross(x_axis);
  
  // Assemble rotation matrix [x_axis, y_axis, z_axis]
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;
}

void RoboBeeMPC::orientation_vector_to_rotation_matrix(const Eigen::Vector3f& orient_vec, float R[9]) const {
  // The orientation vector represents the thrust direction (3rd column of rotation matrix)
  // We need to reconstruct a full rotation matrix from this single column
  
  // Normalize the orientation vector to get the z-axis (thrust direction)
  Eigen::Vector3f z_axis = orient_vec;
  float norm = z_axis.norm();
  
  if (norm < 1e-6f) {
    // Invalid orientation vector, use default upright orientation
    R[0] = 1.0f; R[1] = 0.0f; R[2] = 0.0f;
    R[3] = 0.0f; R[4] = 1.0f; R[5] = 0.0f;
    R[6] = 0.0f; R[7] = 0.0f; R[8] = 1.0f;
    return;
  } else {
    z_axis = z_axis / norm;
  }
  
  // Construct x and y axes to complete the rotation matrix
  // Choose x-axis to be orthogonal to z-axis and in the x-y plane if possible
  Eigen::Vector3f x_axis, y_axis;
  
  if (std::abs(z_axis[2]) < 0.9f) {
    // z-axis is not too close to world z, use world z × thrust_dir for x-axis
    x_axis = Eigen::Vector3f(0, 0, 1).cross(z_axis);
  } else {
    // z-axis is close to world z, use world x × thrust_dir for x-axis  
    x_axis = Eigen::Vector3f(1, 0, 0).cross(z_axis);
  }
  
  float x_norm = x_axis.norm();
  if (x_norm < 1e-6f) {
    // Fallback: use a different vector
    x_axis = Eigen::Vector3f(0, 1, 0).cross(z_axis);
    x_norm = x_axis.norm();
    if (x_norm < 1e-6f) {
      // Last resort: construct manually
      if (std::abs(z_axis[0]) < 0.9f) {
        x_axis = Eigen::Vector3f(1, 0, 0);
      } else {
        x_axis = Eigen::Vector3f(0, 1, 0);
      }
      // Make it orthogonal to z_axis
      x_axis = x_axis - x_axis.dot(z_axis) * z_axis;
      x_axis = x_axis.normalized();
    } else {
      x_axis = x_axis / x_norm;
    }
  } else {
    x_axis = x_axis / x_norm;
  }
  
  // y-axis = z-axis × x-axis (right-hand rule)
  y_axis = z_axis.cross(x_axis);
  y_axis = y_axis.normalized();
  
  // Store in column-major order for uprightmpc2
  // R = [x_axis, y_axis, z_axis] (columns)
  R[0] = x_axis[0]; R[1] = x_axis[1]; R[2] = x_axis[2];  // 1st column
  R[3] = y_axis[0]; R[4] = y_axis[1]; R[5] = y_axis[2];  // 2nd column  
  R[6] = z_axis[0]; R[7] = z_axis[1]; R[8] = z_axis[2];  // 3rd column (thrust direction)
}

} // namespace EntoControl 