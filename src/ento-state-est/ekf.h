#ifndef EKF_HH
#define EKF_HH

#include <Eigen/Dense>
#include <concepts>
#include <ento-util/debug.h>

template<typename T, typename Scalar, size_t StateDim, typename ControlInputs>
concept DynamicsModelConcept = requires(T t, Eigen::Matrix<Scalar, StateDim, 1>& x, ControlInputs u, Eigen::Matrix<Scalar, StateDim, StateDim>& jacobian_matrix, Scalar dt) {
    // Accept either interface: (x, u) OR (x, u, dt)
    { t(x, u) } -> std::same_as<void>;
} || requires(T t, Eigen::Matrix<Scalar, StateDim, 1>& x, ControlInputs u, Scalar dt) {
    { t(x, u, dt) } -> std::same_as<void>;
} && requires(T t, Eigen::Matrix<Scalar, StateDim, 1>& x, ControlInputs u, Eigen::Matrix<Scalar, StateDim, StateDim>& jacobian_matrix) {
    // Jacobian method should exist with const parameters
    { t.jacobian(jacobian_matrix, x, u) } -> std::same_as<void>;
};

template<typename T, typename Scalar, size_t StateDim, size_t MeasurementDim>
concept MeasurementModelConcept = requires(T t, const Eigen::Matrix<Scalar, StateDim, 1>& x,
                                           Eigen::Matrix<Scalar, MeasurementDim, 1>& z,
                                           Eigen::Matrix<Scalar, MeasurementDim, StateDim>& jacobian_matrix,
                                           Eigen::Matrix<Scalar, MeasurementDim, 1>& pred)
{
  { t(x, z, pred) } -> std::same_as<void>;
  // Accept either const or non-const x in jacobian
  { t.jacobian(jacobian_matrix, x) } -> std::same_as<void>;
} || requires(T t, Eigen::Matrix<Scalar, StateDim, 1>& x, Eigen::Matrix<Scalar, MeasurementDim, StateDim>& jacobian_matrix) {
  { t.jacobian(jacobian_matrix, x) } -> std::same_as<void>;
};

template<typename Scalar, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
class EKF
{
public:
  using Scalar_ = Scalar;
  static constexpr size_t StateDim_ = StateDim;
  static constexpr size_t MeasurementDim_ = MeasurementDim;
  static constexpr size_t ControlDim_ = ControlDim;

  EKF(const Eigen::Matrix<Scalar, StateDim, StateDim>& Q, // Process Noise covariance matrix
      const Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim>& R) // Measurement Noise covariance matrix
      : x_(Eigen::Matrix<Scalar, StateDim, 1>::Zero()),
        P_(Eigen::Matrix<Scalar, StateDim, StateDim>::Identity()), // P_: State Covariance Matrix
        predicted_measurement_(Eigen::Matrix<Scalar, MeasurementDim, 1>::Zero()), // Measurement predicted from dynamics
        measurement_residual_(Eigen::Matrix<Scalar, MeasurementDim, 1>::Zero()), // Residual (measurement - predicted measurement)
        Q_(Q), // Process Noise Covariance Matrix
        R_(R), // Measurement Covariance Matrix
        F_(Eigen::Matrix<Scalar, StateDim, StateDim>::Zero())
  {}

  template <typename DynamicsModel, typename ControlInputs>
  requires DynamicsModelConcept<DynamicsModel, Scalar, StateDim, ControlInputs>
  void predict(const DynamicsModel& dynamics, ControlInputs u)
  {
    ENTO_DEBUG("=== EKF PREDICT START ===");
    
    // Log state before dynamics update
    ENTO_DEBUG("State BEFORE dynamics: [%.6f, %.6f, %.6f, %.6f]", x_(0), x_(1), x_(2), x_(3));
    ENTO_DEBUG("Control input u: [%.6f]", u(0));
    
    // Check if state is finite before dynamics
    for (int i = 0; i < StateDim; i++) {
      if (!std::isfinite(x_(i))) {
        ENTO_DEBUG("ERROR: State x_(%d) = %.6f is not finite BEFORE dynamics!", i, x_(i));
      }
    }
    
    // Try the simpler interface first (RoboBee style)
    if constexpr (requires { dynamics(x_, u); }) {
      dynamics(x_, u);
      ENTO_DEBUG("Used RoboBee-style dynamics interface");
    } 
    // Fall back to dt interface (RoboFly style) - use member dt or default
    else if constexpr (requires { dynamics(x_, u, Scalar(0.004)); }) {
      // Try to access dt from the dynamics model, or use default
      if constexpr (requires { dynamics.dt_; }) {
        ENTO_DEBUG("Used RoboFly-style dynamics interface with model dt = %.6f", dynamics.dt_);
        dynamics(x_, u, dynamics.dt_);
      } else {
        ENTO_DEBUG("Used RoboFly-style dynamics interface with default dt = 0.004");
        dynamics(x_, u, Scalar(0.004)); // Default dt
      }
    }
    
    // Log state after dynamics update
    ENTO_DEBUG("State AFTER dynamics: [%.6f, %.6f, %.6f, %.6f]", x_(0), x_(1), x_(2), x_(3));
    
    // Check if dynamics introduced NaN values
    for (int i = 0; i < StateDim; i++) {
      if (!std::isfinite(x_(i))) {
        ENTO_DEBUG("ERROR: Dynamics introduced NaN at state index %d!", i);
      }
    }

    // Compute dynamics jacobian
    dynamics.jacobian(F_, x_, u);
    
    // Log jacobian matrix  
    ENTO_DEBUG("Dynamics Jacobian F_:");
    for (int i = 0; i < StateDim; i++) {
      ENTO_DEBUG("  F_[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, F_(i,0), F_(i,1), F_(i,2), F_(i,3));
    }
    
    // Check if jacobian has any non-finite values
    for (int i = 0; i < StateDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        if (!std::isfinite(F_(i,j))) {
          ENTO_DEBUG("ERROR: Dynamics jacobian F_(%d,%d) = %.6f is not finite!", i, j, F_(i,j));
        }
      }
    }

    // Log covariance before update
    ENTO_DEBUG("Covariance P_ BEFORE update:");
    ENTO_DEBUG_EIGEN_MATRIX(P_);
    
    // Check if covariance is finite before update
    for (int i = 0; i < StateDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        if (!std::isfinite(P_(i,j))) {
          ENTO_DEBUG("ERROR: Covariance P_(%d,%d) = %.6f is not finite BEFORE update!", i, j, P_(i,j));
        }
      }
    }
    
    // Update the covariance step by step
    auto F_P = F_ * P_;
    ENTO_DEBUG("Intermediate F*P:");
    for (int i = 0; i < StateDim; i++) {
      ENTO_DEBUG("  (F*P)[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, F_P(i,0), F_P(i,1), F_P(i,2), F_P(i,3));
    }
    
    auto F_P_Ft = F_P * F_.transpose();
    ENTO_DEBUG("Intermediate F*P*F^T:");
    for (int i = 0; i < StateDim; i++) {
      ENTO_DEBUG("  (F*P*F^T)[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, F_P_Ft(i,0), F_P_Ft(i,1), F_P_Ft(i,2), F_P_Ft(i,3));
    }
    
    ENTO_DEBUG("Process noise Q_:");
    for (int i = 0; i < StateDim; i++) {
      ENTO_DEBUG("  Q_[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, Q_(i,0), Q_(i,1), Q_(i,2), Q_(i,3));
    }
    
    P_ = F_P_Ft + Q_;
    
    // Log covariance after update
    ENTO_DEBUG("Covariance P_ AFTER update:");
    ENTO_DEBUG_EIGEN_MATRIX(P_);
    
    // Check if covariance update introduced NaN values
    for (int i = 0; i < StateDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        if (!std::isfinite(P_(i,j))) {
          ENTO_DEBUG("ERROR: Covariance update introduced NaN at P_(%d,%d) = %.6f!", i, j, P_(i,j));
        }
      }
    }
    
    ENTO_DEBUG("=== EKF PREDICT END ===");
  }

  template <typename MeasurementModel>
  requires MeasurementModelConcept<MeasurementModel, Scalar, StateDim, MeasurementDim>
  void update(const MeasurementModel& meas_model, const Eigen::Matrix<Scalar, MeasurementDim, 1>& z)
  {
    ENTO_DEBUG("=== EKF MEASUREMENT UPDATE START ===");
    
    // Log state before measurement update
    ENTO_DEBUG("State BEFORE measurement update: [%.6f, %.6f, %.6f, %.6f]", x_(0), x_(1), x_(2), x_(3));
    ENTO_DEBUG("Measurements z: [%.6f, %.6f, %.6f, %.6f]", z(0), z(1), z(2), z(3));
    
    // Check if state is finite before measurement update
    for (int i = 0; i < StateDim; i++) {
      if (!std::isfinite(x_(i))) {
        ENTO_DEBUG("ERROR: State x_(%d) = %.6f is not finite BEFORE measurement update!", i, x_(i));
      }
    }
    
    // Check if measurements are finite
    for (int i = 0; i < MeasurementDim; i++) {
      if (!std::isfinite(z(i))) {
        ENTO_DEBUG("ERROR: Measurement z(%d) = %.6f is not finite!", i, z(i));
      }
    }

    // Predict measurement
    ENTO_DEBUG("Computing predicted measurement...");
    meas_model(x_, u_, predicted_measurement_);
    
    ENTO_DEBUG("Predicted measurements: [%.6f, %.6f, %.6f, %.6f]", 
               predicted_measurement_(0), predicted_measurement_(1), predicted_measurement_(2), predicted_measurement_(3));
    
    // Check if predicted measurements are finite
    for (int i = 0; i < MeasurementDim; i++) {
      if (!std::isfinite(predicted_measurement_(i))) {
        ENTO_DEBUG("ERROR: Predicted measurement [%d] = %.6f is not finite!", i, predicted_measurement_(i));
        ENTO_DEBUG("This indicates a problem in the measurement model evaluation!");
        return; // Exit early to prevent NaN propagation
      }
    }

    // Compute the measurement Jacobian evaluated at x.
    ENTO_DEBUG("Computing measurement jacobian...");
    Eigen::Matrix<Scalar, MeasurementDim, StateDim> H;
    meas_model.jacobian(H, x_);
    
    // Log measurement jacobian
    ENTO_DEBUG("Measurement Jacobian H:");
    for (int i = 0; i < MeasurementDim; i++) {
      ENTO_DEBUG("  H[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, H(i,0), H(i,1), H(i,2), H(i,3));
    }
    
    // Check if measurement jacobian has any non-finite values
    for (int i = 0; i < MeasurementDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        if (!std::isfinite(H(i,j))) {
          ENTO_DEBUG("ERROR: Measurement jacobian H(%d,%d) = %.6f is not finite!", i, j, H(i,j));
          return; // Exit early to prevent NaN propagation
        }
      }
    }

    // Compute innovation covariance S and Kalman gain K.
    ENTO_DEBUG("Computing innovation covariance S = H*P*H^T + R...");
    auto H_P = H * P_;
    ENTO_DEBUG("Intermediate H*P computed successfully");
    
    auto H_P_Ht = H_P * H.transpose();
    ENTO_DEBUG("Intermediate H*P*H^T computed successfully");
    
    Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> S = H_P_Ht + R_;
    
    ENTO_DEBUG("Innovation covariance S:");
    for (int i = 0; i < MeasurementDim; i++) {
      ENTO_DEBUG("  S[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, S(i,0), S(i,1), S(i,2), S(i,3));
    }
    
    // Check if S is finite and invertible
    for (int i = 0; i < MeasurementDim; i++) {
      for (int j = 0; j < MeasurementDim; j++) {
        if (!std::isfinite(S(i,j))) {
          ENTO_DEBUG("ERROR: Innovation covariance S(%d,%d) = %.6f is not finite!", i, j, S(i,j));
          return; // Exit early to prevent NaN propagation
        }
      }
    }
    
    // Check determinant to ensure S is invertible
    Scalar det_S = S.determinant();
    ENTO_DEBUG("Determinant of S: %.6f", det_S);
    if (std::abs(det_S) < 1e-12) {
      ENTO_DEBUG("ERROR: Innovation covariance S is nearly singular (det = %.6e)!", det_S);
      return; // Exit early to prevent numerical issues
    }
    
    ENTO_DEBUG("Computing Kalman gain K = P*H^T*S^(-1)...");
    Eigen::Matrix<Scalar, StateDim, MeasurementDim> K = P_ * H.transpose() * S.inverse();
    
    ENTO_DEBUG("Kalman gain K:");
    for (int i = 0; i < StateDim; i++) {
      ENTO_DEBUG("  K[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, K(i,0), K(i,1), K(i,2), K(i,3));
    }
    
    // Check if Kalman gain is finite
    for (int i = 0; i < StateDim; i++) {
      for (int j = 0; j < MeasurementDim; j++) {
        if (!std::isfinite(K(i,j))) {
          ENTO_DEBUG("ERROR: Kalman gain K(%d,%d) = %.6f is not finite!", i, j, K(i,j));
          return; // Exit early to prevent NaN propagation
        }
      }
    }

    // Compute the measurement residual (measured - predicted)
    measurement_residual_ = z - predicted_measurement_;
    
    ENTO_DEBUG("Measurement residual: [%.6f, %.6f, %.6f, %.6f]", 
               measurement_residual_(0), measurement_residual_(1), measurement_residual_(2), measurement_residual_(3));
    
    // Check if measurement residual is finite
    for (int i = 0; i < MeasurementDim; i++) {
      if (!std::isfinite(measurement_residual_(i))) {
        ENTO_DEBUG("ERROR: Measurement residual(%d) = %.6f is not finite!", i, measurement_residual_(i));
        return; // Exit early to prevent NaN propagation
      }
    }

    // Update the state estimate using the residual and Kalman Gain.
    ENTO_DEBUG("Updating state estimate...");
    auto state_correction = K * measurement_residual_;
    
    ENTO_DEBUG("State correction: [%.6f, %.6f, %.6f, %.6f]", 
               state_correction(0), state_correction(1), state_correction(2), state_correction(3));
    
    // Check if state correction is finite
    for (int i = 0; i < StateDim; i++) {
      if (!std::isfinite(state_correction(i))) {
        ENTO_DEBUG("ERROR: State correction(%d) = %.6f is not finite!", i, state_correction(i));
        return; // Exit early to prevent NaN propagation
      }
    }
    
    x_ += state_correction;
    
    ENTO_DEBUG("State AFTER correction: [%.6f, %.6f, %.6f, %.6f]", x_(0), x_(1), x_(2), x_(3));
    
    // Check if updated state is finite
    for (int i = 0; i < StateDim; i++) {
      if (!std::isfinite(x_(i))) {
        ENTO_DEBUG("ERROR: Updated state x_(%d) = %.6f is not finite!", i, x_(i));
      }
    }
    
    Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();

    // Update the covariance.
    ENTO_DEBUG("Updating covariance P = (I - K*H)*P...");
    auto I_minus_KH = I - K * H;
    
    ENTO_DEBUG("(I - K*H) matrix:");
    for (int i = 0; i < StateDim; i++) {
      ENTO_DEBUG("  (I-KH)[%d,:] = [%.6f, %.6f, %.6f, %.6f]", i, I_minus_KH(i,0), I_minus_KH(i,1), I_minus_KH(i,2), I_minus_KH(i,3));
    }
    
    // Check if (I - K*H) is finite
    for (int i = 0; i < StateDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        if (!std::isfinite(I_minus_KH(i,j))) {
          ENTO_DEBUG("ERROR: (I-KH)(%d,%d) = %.6f is not finite!", i, j, I_minus_KH(i,j));
          return; // Exit early to prevent NaN propagation
        }
      }
    }
    
    P_ = I_minus_KH * P_;
    
    ENTO_DEBUG("Covariance P AFTER measurement update:");
    ENTO_DEBUG_EIGEN_MATRIX(P_);
    
    // Check if updated covariance is finite
    for (int i = 0; i < StateDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        if (!std::isfinite(P_(i,j))) {
          ENTO_DEBUG("ERROR: Updated covariance P_(%d,%d) = %.6f is not finite!", i, j, P_(i,j));
        }
      }
    }
    
    ENTO_DEBUG("=== EKF MEASUREMENT UPDATE END ===");
  }

  template <typename MeasurementModel>
  requires MeasurementModelConcept<MeasurementModel, Scalar, StateDim, MeasurementDim>
  void update_sequential(const MeasurementModel& meas_model,
                         const Eigen::Matrix<Scalar, MeasurementDim, 1>& z,
                         [[maybe_unused]] const Eigen::Matrix<Scalar, ControlDim, 1>& u,
                         std::array<bool, MeasurementDim>& avail_sensors)
  {
    static constexpr Scalar xi = 1e6;
    Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> I_meas =
        Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim>::Identity();
    for (size_t j = 0; j < MeasurementDim; ++j)
    {
      // Step (4)
      Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> Rtilde = xi * I_meas;

      // Step (5)
      if (avail_sensors[j])
      {
        // Step (5)
        Rtilde(j, j) = R_(j, j);

        // Step (6)
        Eigen::Matrix<Scalar, MeasurementDim, 1> pred_meas;
        meas_model(x_, u, pred_meas);
        Eigen::Matrix<Scalar, MeasurementDim, StateDim> Hk;
        meas_model.jacobian(Hk, x_);

        // Step (7)
        Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> S = Hk * P_ * Hk.transpose() + Rtilde;
        Eigen::Matrix<Scalar, StateDim, MeasurementDim> K = P_ * Hk.transpose() * S.inverse();

        // Step (8)
        measurement_residual_.setZero();
        measurement_residual_(j) = z(j) - pred_meas(j);
        x_ += K * measurement_residual_;

        // Step (9)
        Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();
        P_ = (I - K * Hk) * P_;
      }
    }
  }
  
  template <typename MeasurementModel>
  requires MeasurementModelConcept<MeasurementModel, Scalar, StateDim, MeasurementDim>
  void update_truncated(const MeasurementModel& meas_model,
                        const Eigen::Matrix<Scalar, MeasurementDim, 1>& z,
                        [[maybe_unused]] const Eigen::Matrix<Scalar, ControlDim, 1>& u,
                        std::array<bool, MeasurementDim> & avail_sensors)
  {
    // First count available sensors
    size_t m = 0;
    for (size_t j = 0; j < MeasurementDim; ++j) {
      if (avail_sensors[j]) {
        ++m;
      }
    }
    
    if (m == 0) return; // No sensors available
    
    // Initialize dynamic matrices with proper sizes
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, StateDim, StateDim> H_tilde(m, StateDim);
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MeasurementDim, MeasurementDim> R_tilde(m, m);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MeasurementDim, 1> y_tilde(m);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MeasurementDim, 1> z_tilde(m);
    
    // Initialize R_tilde to zero
    R_tilde.setZero();

    Eigen::Matrix<Scalar, MeasurementDim, 1> pred_meas;
    meas_model(x_, u, pred_meas);

    Eigen::Matrix<Scalar, MeasurementDim, StateDim> Hk;
    meas_model.jacobian(Hk, x_);

    // Fill the truncated matrices
    size_t idx = 0;
    for (size_t j = 0; j < MeasurementDim; ++j)
    {
      if (avail_sensors[j])
      {
        y_tilde(idx)     = z(j);
        z_tilde(idx)     = pred_meas(j);
        R_tilde(idx, idx)  = R_(j, j);
        H_tilde.row(idx) = Hk.row(j);
        ++idx;
      }
    }

    // Innovation covariance and Kalman gain
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MeasurementDim, MeasurementDim> S(m, m);
    S = H_tilde * P_ * H_tilde.transpose() + R_tilde;
    
    Eigen::Matrix<Scalar, StateDim, Eigen::Dynamic, 0, StateDim, MeasurementDim> K(StateDim, m);
    K = P_ * H_tilde.transpose() * S.inverse();

    // Measurement residual
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MeasurementDim, 1> measurement_residual(m);
    measurement_residual = y_tilde - z_tilde;

    // State and covariance update
    x_ += K * measurement_residual;

    Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();
    P_ = (I - K * H_tilde) * P_;
  }

  Eigen::Matrix<Scalar, StateDim, 1> getState() const
  {
      return x_;
  }

  Eigen::Matrix<Scalar, StateDim, StateDim> getCovariance() const
  {
      return P_;
  }

  void set_covariance(Eigen::Matrix<Scalar, StateDim, StateDim> P)
  {
    P_ = P;
  }

  void set_state(const Eigen::Matrix<Scalar, StateDim, 1>& x)
  {
    x_ = x;
  }

  // Access to noise matrices (for debugging)
  const Eigen::Matrix<Scalar, StateDim, StateDim>& getQ() const
  {
    return Q_;
  }
  
  const Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim>& getR() const
  {
    return R_;
  }

private:
  Eigen::Matrix<Scalar, StateDim, 1> x_; // State vector
  Eigen::Matrix<Scalar, ControlDim, 1> u_; // Control vector
  Eigen::Matrix<Scalar, StateDim, StateDim> P_; // Covariance matrix
  Eigen::Matrix<Scalar, MeasurementDim, 1> predicted_measurement_;
  Eigen::Matrix<Scalar, MeasurementDim, 1> measurement_residual_;
  Eigen::Matrix<Scalar, StateDim, StateDim> Q_; // Process noise covariance matrix
  Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> R_; // Measurement noise covariance matrix
  Eigen::Matrix<Scalar, StateDim, StateDim> F_; // Fixed state transition Jacobian
  Eigen::Matrix<Scalar, MeasurementDim, StateDim> H_; // Fixed measurement Jacobian
};

#endif // EKF_HH
