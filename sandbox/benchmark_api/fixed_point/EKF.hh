// Concept for DynamicsModel
template<typename T, typename Scalar, int StateDim, typename... ControlInputs>
concept DynamicsModelConcept = requires(T t, Eigen::Matrix<Scalar, StateDim, 1>& x, ControlInputs... u, Eigen::Matrix<Scalar, StateDim, StateDim>& jacobian_matrix) {
    { t(x, u...) } -> std::same_as<void>;
    { t.jacobian(jacobian_matrix, x, u...) } -> std::same_as<void>;
};

// Concept for MeasurementModel
template<typename T, typename Scalar, int StateDim, int MeasurementDim>
concept MeasurementModelConcept = requires(T t, const Eigen::Matrix<Scalar, StateDim, 1>& x, Eigen::Matrix<Scalar, MeasurementDim, 1>& z, Eigen::Matrix<Scalar, MeasurementDim, StateDim>& jacobian_matrix) {
    { t(x, z) } -> std::same_as<void>;
    { t.jacobian(jacobian_matrix, x) } -> std::same_as<void>;
};

template<typename Scalar, int StateDim, int MeasurementDim, typename DynamicsModel, typename MeasurementModel>
requires DynamicsModelConcept<DynamicsModel, Scalar, StateDim> && MeasurementModelConcept<MeasurementModel, Scalar, StateDim, MeasurementDim>
class EKF {
private:
    Eigen::Matrix<Scalar, StateDim, 1> x_; // State vector
    Eigen::Matrix<Scalar, StateDim, StateDim> P_; // Covariance matrix
    Eigen::Matrix<Scalar, MeasurementDim, 1> predicted_measurement_;
    Eigen::Matrix<Scalar, MeasurementDim, 1> measurement_residual_;
    Eigen::Matrix<Scalar, StateDim, StateDim> Q_; // Process noise covariance matrix
public:
    EKF() 
      : x_(Eigen::Matrix<Scalar, StateDim, 1>::Zero()),
        P_(Eigen::Matrix<Scalar, StateDim, StateDim>::Identity()),
        predicted_measurement_(Eigen::Matrix<Scalar, MeasurementDim, 1>::Zero()),
        measurement_residual_(Eigen::Matrix<Scalar, MeasurementDim, 1>::Zero()) {}

    template<typename... ControlInputs>
    void predict(const DynamicsModel& dynamics, ControlInputs... u) {
        // Predict state using dynamics model
        dynamics(x_, u...);
        // Compute Jacobian and update covariance
        Eigen::Matrix<Scalar, StateDim, StateDim> F;
        dynamics.jacobian(F, x_, u...);
        P_ = F * P_ * F.transpose() + Q_; // Q_ is the process noise covariance matrix
    }

    void update(const MeasurementModel& measurement, const Eigen::Matrix<Scalar, MeasurementDim, 1>& z) {
        // Predict measurement
        measurement(x_, predicted_measurement_);

        // Compute Jacobian
        Eigen::Matrix<Scalar, MeasurementDim, StateDim> H;
        measurement.jacobian(H, x_);

        // Kalman gain
        Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> R; // Assume R is provided or calculated elsewhere
        Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> S = H * P_ * H.transpose() + R;
        Eigen::Matrix<Scalar, StateDim, MeasurementDim> K = P_ * H.transpose() * S.inverse();

        // Measurement residual
        measurement_residual_ = z - predicted_measurement_;
        x_ += K * measurement_residual_;

        // Update covariance
        Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();
        P_ = (I - K * H) * P_;
    }

};
