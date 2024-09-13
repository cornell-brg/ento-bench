#ifndef EKF_HH
#define EKF_HH

#include <Eigen/Dense>
#include <concepts>
#include <optional>

template<typename T, typename Scalar, int StateDim, typename... ControlInputs>
concept DynamicsModelConcept = requires(T t, Eigen::Matrix<Scalar, StateDim, 1>& x, ControlInputs... u, Eigen::Matrix<Scalar, StateDim, StateDim>& jacobian_matrix) {
    { t(x, u...) } -> std::same_as<void>;
    { t.jacobian(jacobian_matrix, x, u...) } -> std::same_as<void>;
};

template<typename T, typename Scalar, int StateDim, int MeasurementDim>
concept MeasurementModelConcept = requires(T t, const Eigen::Matrix<Scalar, StateDim, 1>& x, Eigen::Matrix<Scalar, MeasurementDim, 1>& z, Eigen::Matrix<Scalar, MeasurementDim, StateDim>& jacobian_matrix) {
    { t(x, z) } -> std::same_as<void>;
    { t.jacobian(jacobian_matrix, x) } -> std::same_as<void>;
};

template<typename Scalar, int StateDim, int MeasurementDim, typename DynamicsModel, typename MeasurementModel, bool UseFixedJacobians = false>
requires DynamicsModelConcept<DynamicsModel, Scalar, StateDim> && MeasurementModelConcept<MeasurementModel, Scalar, StateDim, MeasurementDim>
class EKF {
private:
    Eigen::Matrix<Scalar, StateDim, 1> x_; // State vector
    Eigen::Matrix<Scalar, StateDim, StateDim> P_; // Covariance matrix
    Eigen::Matrix<Scalar, MeasurementDim, 1> predicted_measurement_;
    Eigen::Matrix<Scalar, MeasurementDim, 1> measurement_residual_;
    Eigen::Matrix<Scalar, StateDim, StateDim> Q_; // Process noise covariance matrix
    Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> R_; // Measurement noise covariance matrix
    Eigen::Matrix<Scalar, StateDim, StateDim> F_; // Fixed state transition Jacobian
    Eigen::Matrix<Scalar, MeasurementDim, StateDim> H_; // Fixed measurement Jacobian
public:
    EKF(const Eigen::Matrix<Scalar, StateDim, StateDim>& Q, const Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim>& R, 
        const Eigen::Matrix<Scalar, StateDim, StateDim>& F = Eigen::Matrix<Scalar, StateDim, StateDim>::Zero(), 
        const Eigen::Matrix<Scalar, MeasurementDim, StateDim>& H = Eigen::Matrix<Scalar, MeasurementDim, StateDim>::Zero())
        : x_(Eigen::Matrix<Scalar, StateDim, 1>::Zero()),
          P_(Eigen::Matrix<Scalar, StateDim, StateDim>::Identity()),
          predicted_measurement_(Eigen::Matrix<Scalar, MeasurementDim, 1>::Zero()),
          measurement_residual_(Eigen::Matrix<Scalar, MeasurementDim, 1>::Zero()),
          Q_(Q),
          R_(R),
          F_(F),
          H_(H) {}

    template<typename... ControlInputs>
    void predict(const DynamicsModel& dynamics, ControlInputs... u) {
        // Predict state using dynamics model
        dynamics(x_, u...);

        if constexpr (UseFixedJacobians) {
            P_ = F_ * P_ * F_.transpose() + Q_;
        } else {
            Eigen::Matrix<Scalar, StateDim, StateDim> F;
            dynamics.jacobian(F, x_, u...);
            P_ = F * P_ * F.transpose() + Q_;
        }
    }

    void update(const MeasurementModel& measurement, const Eigen::Matrix<Scalar, MeasurementDim, 1>& z) {
        // Predict measurement
        measurement(x_, predicted_measurement_);

        if constexpr (UseFixedJacobians) {
            Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> S = H_ * P_ * H_.transpose() + R_;
            Eigen::Matrix<Scalar, StateDim, MeasurementDim> K = P_ * H_.transpose() * S.inverse();
            measurement_residual_ = z - predicted_measurement_;
            x_ += K * measurement_residual_;
            Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();
            P_ = (I - K * H_) * P_;
        } else {
            Eigen::Matrix<Scalar, MeasurementDim, StateDim> H;
            measurement.jacobian(H, x_);
            Eigen::Matrix<Scalar, MeasurementDim, MeasurementDim> S = H * P_ * H.transpose() + R_;
            Eigen::Matrix<Scalar, StateDim, MeasurementDim> K = P_ * H.transpose() * S.inverse();
            measurement_residual_ = z - predicted_measurement_;
            x_ += K * measurement_residual_;
            Eigen::Matrix<Scalar, StateDim, StateDim> I = Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();
            P_ = (I - K * H) * P_;
        }
    }

    Eigen::Matrix<Scalar, StateDim, 1> getState() const {
        return x_;
    }

    Eigen::Matrix<Scalar, StateDim, StateDim> getCovariance() const {
        return P_;
    }
};

#endif // EKF_HH
