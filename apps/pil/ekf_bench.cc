#include <workload/state_est/EKF.hh>
#include <Eigen/Dense>
#include <random>

// Define dimensions
constexpr int StateDim = 3;
constexpr int MeasDim = 2;
constexpr int num_steps = 100;
using Scalar = double;
using StateVector = Eigen::Matrix<Scalar, StateDim, 1>;
using StateMatrix = Eigen::Matrix<Scalar, StateDim, StateDim>;
using MeasVector = Eigen::Matrix<Scalar, MeasDim, 1>;
using MeasMatrix = Eigen::Matrix<Scalar, MeasDim, MeasDim>;
using MeasStateMatrix = Eigen::Matrix<Scalar, MeasDim, StateDim>;

StateVector generateProcessNoise(const StateMatrix& Q) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0, 1);
    StateVector noise;
    for (int i = 0; i < StateDim; ++i) {
        noise(i) = d(gen);
    }
    return Q * noise;
}

MeasVector generateMeasurementNoise(const MeasMatrix& R) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0, 1);
    MeasVector noise;
    for (int i = 0; i < MeasDim; ++i) {
        noise(i) = d(gen);
    }
    return R * noise;
}

// Example dynamics model
struct DynamicsModel {
    void operator()(StateVector& x, const Scalar u) const {
        // State prediction
        x = A * x + B * u;
    }

    StateMatrix A;
    StateMatrix B;
    StateMatrix F;
};

// Measurement model
struct MeasurementModel {
    void operator()(const StateVector& x, MeasVector& z) const {
        z = H * x;
    }

    void jacobian(MeasStateMatrix& H_jacobian, const StateVector& x) const {
      H = 
    }

    MeasStateMatrix H;
};

int main() {
    // Time step
    Scalar dt = 0.1;

    // Define system matrices
    StateMatrix A;
    A << 1, dt, 0.5 * dt * dt,
         0, 1, dt,
         0, 0, 1;

    StateMatrix B;
    B << 0.5 * dt * dt,
         dt,
         1;

    StateMatrix Q;
    Q.setIdentity();

    MeasStateMatrix H;
    H << 1, 0, 0,
         0, 0, 1;

    MeasMatrix R;
    R.setIdentity();

    // Initialize EKF with dynamic Jacobians
    EKF<Scalar, StateDim, MeasDim, DynamicsModel, MeasurementModel> ekf(Q, R);

    // Initial state
    StateVector x_true;
    x_true << 0, 0, 9.81; // Initial position, velocity, and acceleration

    // Initialize models
    DynamicsModel dynamics{A, B, F};
    MeasurementModel measurement{H};

    // Fixed-size arrays for synthetic data and EKF results
    Eigen::Matrix<Scalar, StateDim, num_steps> true_states;
    Eigen::Matrix<Scalar, MeasDim, num_steps> measurements;
    Eigen::Matrix<Scalar, StateDim, num_steps> estimated_states;

    Scalar u = 0;

    for (int i = 0; i < num_steps; ++i) {
        // Simulate true state
        StateVector process_noise = generateProcessNoise(Q);
        dynamics(x_true, u);
        x_true += process_noise;
        true_states.col(i) = x_true;

        // Simulate measurement
        MeasVector measurement_noise = generateMeasurementNoise(R);
        MeasVector z;
        measurement(x_true, z);
        z += measurement_noise;
        measurements.col(i) = z;

        // EKF prediction and update
        ekf.predict(dynamics, u);
        ekf.update(measurement, z);
        estimated_states.col(i) = ekf.getState();
    }


    return 0;
}
