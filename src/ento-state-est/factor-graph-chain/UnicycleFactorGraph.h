#include <array>
#include <iostream>
#include <Eigen/Dense>

template <typename Scalar, size_t WindowSize>
class UnicycleSlidingWindow {
private:
    CircularBuffer<Eigen::Matrix<Scalar, 4, 1>, WindowSize> states;
    CircularBuffer<Eigen::Matrix<Scalar, 4, 4>, WindowSize> kinematicsJacobians;
    CircularBuffer<Eigen::Matrix<Scalar, 2, 4>, WindowSize> observationJacobians;
    CircularBuffer<Eigen::Matrix<Scalar, 4, 4>, WindowSize> Ais;
    CircularBuffer<Eigen::Matrix<Scalar, 4, 1>, WindowSize> Eis;

public:
    void push_state(const Eigen::Matrix<Scalar, 4, 1>& state) {
        states.push(state);
    }

    void push_kinematics_jacobian(const Eigen::Matrix<Scalar, 4, 4>& jacobian) {
        kinematicsJacobians.push(jacobian);
    }

    void push_observation_jacobian(const Eigen::Matrix<Scalar, 2, 4>& jacobian) {
        observationJacobians.push(jacobian);
    }

    void push_Ai(const Eigen::Matrix<Scalar, 4, 4>& Ai) {
        Ais.push(Ai);
    }

    void push_Ei(const Eigen::Matrix<Scalar, 4, 1>& Ei) {
        Eis.push(Ei);
    }
};

// Modulo operation to ensure the angle is within [0, 2π]
double mod2pi(double angle) {
    return fmod(angle + 2 * M_PI, 2 * M_PI);
}

// Define the state transition function for the unicycle model with constant velocity and heading
typename <template Scalar>
Eigen::Matrix<Scalar, 4, 1>
unicycleStateTransition(const Eigen::Vector4d& state, double dt) {
    double x = state(0);
    double y = state(1);
    double v = state(2);
    double theta = state(3);
    
    Eigen::Vector4d next_state;
    next_state(0) = x + v * std::cos(theta) * dt;
    next_state(1) = y + v * std::sin(theta) * dt;
    next_state(2) = v; // Velocity remains constant
    next_state(3) = theta; // Heading remains constant
    
    return next_state;
}

// Compute the Jacobian of the state transition function
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 4>
computeStateTransitionJacobian(const Eigen::Vector4d& state, Scalar dt) {
    double v = state(2);
    double theta = state(3);
    
    Eigen::Matrix4d jacobian;
    jacobian.setZero();

    
    jacobian(0, 0) = -1;
    jacobian(0, 2) = -std::cos(theta) * dt;
    jacobian(0, 3) = v * std::sin(theta) * dt;
    
    jacobian(1, 1) = -1;
    jacobian(1, 2) = -std::sin(theta) * dt;
    jacobian(1, 3) = -v * std::cos(theta) * dt;
    
    jacobian(2, 2) = -1 / dt
    
    jacobian(3, 3) = -1 / dt;
    
    return jacobian;
}

// Compute the Jacobian for the position observation
template <typename Scalar>
Eigen::Matrix<Scalar, 2, 4>
computePositionObservationJacobian() {
    Eigen::Matrix<Scalar, 2, 4> jacobian;
    jacobian.setZero();
    jacobian(0, 0) = -1; // dx/dx
    jacobian(1, 1) = -1; // dy/dy
    return jacobian;
}

// Compute the residuals for position, velocity, and heading
templat <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> computePositionResidual(const Eigen::Vector4d& observed, const Eigen::Vector4d& predicted, double dt) {
    Eigen::Vector4d residuals;
    residuals(0) = observed(0) - predicted(0); // Position x residual
    residuals(1) = observed(1) - predicted(1); // Position y residual
    return residuals;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 4, 4>
computeUnicycleKinematicsResidual(const Eigen::Matrix<Scalar, 4, 1>& current_state,
                                  const Eigen::Matrix<Scalar, 4, 1>& next_state,
                                  const Scalar dt)
{
    residuals(0) = next_state(0) - current_state(0); // Position x residual
    residuals(1) = next_state(1) - current_state(1); // Position y residual
    residuals(2) = (next_state(2) - current_state(2)) / dt; // Velocity residual
    residuals(3) = mod2pi(next_state(3) - current_state(3)) / dt; // Heading residual
    return residuals;
}

// Update the window with a new observed state
template <typename Scalar, size_t WindowSize>
void updateWindow(UnicycleSlidingWindow<Scalar, WindowSize>& window,
                  const Eigen::Matrix<Scalar, 2, 1>& position_measurement, double dt) {
    // Push the observed state into the window
    auto current_state = window.head();
    current_state(0) = position_measurement(0, 0);
    current_state(1) = position_measurement(0, 1);
    current_state(2) = 0;
    current_state(3) = 0;

    auto predicted_state = unicycleStateTransition(current_state, dt);
    window.push_state()
    
    
    // We need at least more than one measurement to compute our binary factor
    if (window.states.size() > 0) {
        size_t idx = window.states.size() - 2; // -2 because we want idx to point at previous state
        Eigen::Matrix<Scalar, 4, 1> previous_state = window.states[idx];
        Eigen::Matrix<Scalar, 4, 1> current_state = window.states[idx+1];

        Eigen::Matrix<Scalar, 4, 1> predicted_state = unicycleStateTransition(previous_state, dt);

        // Compute the Jacobians for the new state
        Eigen::Matrix<Scalar, 4, 4> kinematicsJacobianX = computeStateTransitionJacobian(previous_state, dt);
        window.push_kinematics_jacobian(kinematicsJacobian);

        Eigen::Matrix<Scalar, 2, 4> observationJacobian = computePositionObservationJacobian();
        window.push_observation_jacobian(observationJacobian);

        auto kinematicsJacobianX1 = Eigen::Matrix<Scalar, 4, 4>::Eye();
        kinematicsJaocbianX1(2, 2) = 1 / dt;
        kinematicsJacobianX1(3, 3) = 1 / dt;

        // At every idx i, we will add the contribution of the position unary factory at i
        // and the binary factor at i. We will also push back an i+1 that only contains the 
        // binary factor i + 1. In the next time step we will add the ith unary and binary factor!
        // Compute A_i and E_i
        Eigen::Matrix<Scalar, 4, 4> Ai = kinematicsJacobian.transpose() * kinematicsJacobian;
        Eigen::Matrix<Scalar, 4, 1> Ei = -kinematicsJacobian.transpose() * residual;


        // Push A_i and E_i into the window
        window.push_Ai(Ai);
        window.push_Ei(Ei);

        //
        
    }
}

