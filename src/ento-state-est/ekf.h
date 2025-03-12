#ifndef EKF_HH
#define EKF_HH

#include <Eigen/Dense>
#include <concepts>

template<typename T, typename Scalar, size_t StateDim, typename ControlInputs>
concept DynamicsModelConcept = requires(T t, Eigen::Matrix<Scalar, StateDim, 1>& x, ControlInputs u, Eigen::Matrix<Scalar, StateDim, StateDim>& jacobian_matrix) {
    { t(x, u) } -> std::same_as<void>;
    { t.jacobian(jacobian_matrix, x, u) } -> std::same_as<void>;
};

template<typename T, typename Scalar, size_t StateDim, size_t MeasurementDim>
concept MeasurementModelConcept = requires(T t, const Eigen::Matrix<Scalar, StateDim, 1>& x,
                                           Eigen::Matrix<Scalar, MeasurementDim, 1>& z,
                                           Eigen::Matrix<Scalar, MeasurementDim, StateDim>& jacobian_matrix,
                                           Eigen::Matrix<Scalar, MeasurementDim, 1>& pred)
{
  { t(x, z, pred) } -> std::same_as<void>;
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
    // Predict state using dynamics model
    dynamics(x_, u);

    //Eigen::Matrix<Scalar, StateDim, StateDim> F;
    dynamics.jacobian(F_, x_, u);

    // Update the covariance.
    ENTO_DEBUG_EIGEN_MATRIX(P_);
    P_ = F_ * P_ * F_.transpose() + Q_;
    ENTO_DEBUG_EIGEN_MATRIX(P_);
  }

  template <typename MeasurementModel>
  requires MeasurementModelConcept<MeasurementModel, Scalar, StateDim, MeasurementDim>
  void update(const MeasurementModel& meas_model, const Eigen::Matrix<Scalar, MeasurementDim, 1>& z)
  {
    // Predict measurement
    meas_model(x_, u_, predicted_measurement_);

    // Compute the measurement Jacobian evaluated at x.
    Eigen::Matrix<Scalar, MeasurementDim, StateDim> H;
    meas_model.jacobian(H, x_);

    // Compute innovation covariance S and Kalman gain K.
    Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> S = H * P_ * H.transpose() + R_;
    Eigen::Matrix<Scalar, StateDim, MeasurementDim> K = P_ * H.transpose() * S.inverse();

    // Compute the measurement residual (measured - predicted)
    measurement_residual_ = z - predicted_measurement_;

    // Update the state estimate using the residual and Kalman Gain.
    x_ += K * measurement_residual_;
    Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();

    // Update the covariance.
    P_ = (I - K * H) * P_;
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
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, StateDim, StateDim> H_tilde;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MeasurementDim, MeasurementDim> R_tilde;

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MeasurementDim, 1> y_tilde;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MeasurementDim, 1> z_tilde;

    Eigen::Matrix<Scalar, MeasurementDim, 1> pred_meas;
    meas_model(x_, u, pred_meas);

    Eigen::Matrix<Scalar, MeasurementDim, StateDim> Hk;
    meas_model.jacobian(Hk, x_);

    size_t m = 0;
    for (size_t j = 0; j < MeasurementDim; ++j)
    {
      if (avail_sensors[j])
      {
        y_tilde(m)     = z(j);
        z_tilde(m)     = pred_meas(j);
        R_tilde(m, m)  = R_(j, j);
        H_tilde.row(m) = Hk.row(j);
        ++m;
      }
    }

    if (m == 0) return;

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MeasurementDim, MeasurementDim> S;
    S = H_tilde.topRows(m) * P_ * H_tilde.topRows(m).transpose() + R_tilde.topLeftCorner(m, m);
    Eigen::Matrix<Scalar, StateDim, Eigen::Dynamic, 0, StateDim, MeasurementDim> K;
    K = P_ * H_tilde.topRows(m).transpose() * S.inverse();

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MeasurementDim, 1> measurement_residual =
      y_tilde.topRows(m) - z_tilde.topRows(m);

    x_ += K * measurement_residual;

    Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();
    P_ = ( I - K * H_tilde.topRows(m) ) * P_;
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
