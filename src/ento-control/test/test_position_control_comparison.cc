#include <stdlib.h>
#include <Eigen/Dense>
#include <sstream>
#include <cmath>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/containers.h>
#include <ento-math/core.h>

#include "geometric_controller.h"

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoControl;

// ============================================================================
// FDCL REFERENCE IMPLEMENTATION (EXACT COPY)
// ============================================================================

struct FDCLController {
    using Vector3 = Eigen::Vector3d;
    using Matrix3 = Eigen::Matrix3d;
    
    // FDCL parameters (from uav.cfg)
    double m = 1.95;  // kg
    double g = 9.81;  // m/s^2
    Vector3 e3{0.0, 0.0, 1.0};
    
    // FDCL gains (from uav.cfg)
    Matrix3 kX = Matrix3::Zero();
    Matrix3 kV = Matrix3::Zero();
    double kIX = 4.0;
    double c1 = 1.0;
    
    // Integral error state
    Vector3 eIX_error = Vector3::Zero();
    
    FDCLController() {
        kX.diagonal() << 16.0, 16.0, 16.0;
        kV.diagonal() << 13.0, 13.0, 13.0;
    }
    
    double position_control_fdcl(const Vector3& x, const Vector3& v, 
                                const Vector3& xd, const Vector3& xd_dot, 
                                const Vector3& xd_2dot, const Matrix3& R,
                                bool use_integral = false, double dt = 0.01) {
        
        printf("=== FDCL REFERENCE IMPLEMENTATION ===\n");
        
        // Step 1: Compute errors (FDCL lines 75-76)
        Vector3 eX = x - xd;     // position error - eq (11)
        Vector3 eV = v - xd_dot; // velocity error - eq (12)
        
        printf("FDCL Step 1 - Errors:\n");
        printf("  eX = [%.10f, %.10f, %.10f]\n", eX[0], eX[1], eX[2]);
        printf("  eV = [%.10f, %.10f, %.10f]\n", eV[0], eV[1], eV[2]);
        
        // Step 2: Integral control (FDCL lines 78-87)
        Vector3 integral_term = Vector3::Zero();
        if (use_integral) {
            // FDCL: eIX.integrate(c1 * eX + eV, dt);
            Vector3 integral_input = c1 * eX + eV;
            eIX_error += integral_input * dt;  // Simple Euler integration
            integral_term = kIX * eIX_error;
            
            printf("FDCL Step 2 - Integral:\n");
            printf("  integral_input = [%.10f, %.10f, %.10f]\n", integral_input[0], integral_input[1], integral_input[2]);
            printf("  eIX_error = [%.10f, %.10f, %.10f]\n", eIX_error[0], eIX_error[1], eIX_error[2]);
            printf("  integral_term = [%.10f, %.10f, %.10f]\n", integral_term[0], integral_term[1], integral_term[2]);
        }
        
        // Step 3: Compute A vector (FDCL lines 89-95)
        Vector3 A = -kX * eX - kV * eV - integral_term - m * g * e3 + m * xd_2dot;
        
        printf("FDCL Step 3 - A vector:\n");
        Vector3 kX_term = -kX * eX;
        Vector3 kV_term = -kV * eV;
        Vector3 gravity_term = -m * g * e3;
        Vector3 accel_term = m * xd_2dot;
        printf("  -kX * eX = [%.10f, %.10f, %.10f]\n", kX_term[0], kX_term[1], kX_term[2]);
        printf("  -kV * eV = [%.10f, %.10f, %.10f]\n", kV_term[0], kV_term[1], kV_term[2]);
        printf("  -integral = [%.10f, %.10f, %.10f]\n", -integral_term[0], -integral_term[1], -integral_term[2]);
        printf("  -m*g*e3 = [%.10f, %.10f, %.10f]\n", gravity_term[0], gravity_term[1], gravity_term[2]);
        printf("  m*xd_2dot = [%.10f, %.10f, %.10f]\n", accel_term[0], accel_term[1], accel_term[2]);
        printf("  A = [%.10f, %.10f, %.10f] (norm=%.10f)\n", A[0], A[1], A[2], A.norm());
        
        // Step 4: Compute thrust (FDCL lines 97-99)
        Vector3 b3 = R * e3;
        double f_total = -A.dot(b3);
        
        printf("FDCL Step 4 - Thrust:\n");
        printf("  b3 = [%.10f, %.10f, %.10f]\n", b3[0], b3[1], b3[2]);
        printf("  A.dot(b3) = %.10f\n", A.dot(b3));
        printf("  f_total = -A.dot(b3) = %.10f\n", f_total);
        
        return f_total;
    }
};

// ============================================================================
// OUR IMPLEMENTATION WRAPPER
// ============================================================================

template<bool UseIntegral>
double test_our_implementation_template(const Eigen::Vector3d& x, const Eigen::Vector3d& v,
                              const Eigen::Vector3d& xd, const Eigen::Vector3d& xd_dot,
                              const Eigen::Vector3d& xd_2dot, const Eigen::Matrix3d& R,
                              double dt = 0.01) {
    
    printf("\n=== OUR IMPLEMENTATION ===\n");
    
    using Scalar = double;
    using VehicleTraits = QuadrotorTraits<Scalar>;
    using Controller = GeometricController<Scalar, VehicleTraits, true, UseIntegral>;
    using State = typename Controller::State;
    using Command = typename Controller::Command;
    
    // Create controller with FDCL gains
    ControlGains<Scalar> gains;
    gains.kX = EntoMath::Vec3<Scalar>(16.0, 16.0, 16.0);
    gains.kV = EntoMath::Vec3<Scalar>(13.0, 13.0, 13.0);
    gains.kR = EntoMath::Vec3<Scalar>(1.6, 1.6, 0.60);
    gains.kW = EntoMath::Vec3<Scalar>(0.40, 0.40, 0.10);
    gains.kIX = 4.0;
    gains.ki = 0.01;
    gains.kIR = 0.015;
    gains.kI = 0.01;
    gains.kyI = 0.02;
    gains.c1 = 1.0;
    gains.c2 = 1.0;
    gains.c3 = 1.0;
    
    Controller controller(gains);
    controller.reset_integral_errors();
    
    // Set up state
    State state;
    state.position = x.cast<Scalar>();
    state.velocity = v.cast<Scalar>();
    state.acceleration.setZero();
    state.rotation = R.cast<Scalar>();
    state.angular_velocity.setZero();
    
    // Set up command
    Command command;
    command.position_d = xd.cast<Scalar>();
    command.velocity_d = xd_dot.cast<Scalar>();
    command.acceleration_d = xd_2dot.cast<Scalar>();
    command.jerk_d.setZero();
    command.snap_d.setZero();
    command.b1_d = EntoMath::Vec3<Scalar>::UnitX();
    command.b1_d_dot.setZero();
    command.b1_d_ddot.setZero();
    
    // Run control computation
    auto control_output = controller.compute_control(state, command, Scalar(dt));
    
    printf("Our implementation: thrust=%.10f\n", control_output.thrust);
    
    return control_output.thrust;
}

double test_our_implementation(const Eigen::Vector3d& x, const Eigen::Vector3d& v,
                              const Eigen::Vector3d& xd, const Eigen::Vector3d& xd_dot,
                              const Eigen::Vector3d& xd_2dot, const Eigen::Matrix3d& R,
                              bool use_integral = false, double dt = 0.01) {
    if (use_integral) {
        return test_our_implementation_template<true>(x, v, xd, xd_dot, xd_2dot, R, dt);
    } else {
        return test_our_implementation_template<false>(x, v, xd, xd_dot, xd_2dot, R, dt);
    }
}

// ============================================================================
// COMPARISON TESTS
// ============================================================================

void test_position_control_comparison() {
    printf("=== POSITION CONTROL LINE-BY-LINE COMPARISON ===\n\n");
    
    // Test case 1: 1 meter position error in Z (same as FDCL debug test)
    Eigen::Vector3d x(0.0, 0.0, 1.0);      // Current position
    Eigen::Vector3d v(0.0, 0.0, 0.0);      // Current velocity
    Eigen::Vector3d xd(0.0, 0.0, 0.0);     // Desired position
    Eigen::Vector3d xd_dot(0.0, 0.0, 0.0); // Desired velocity
    Eigen::Vector3d xd_2dot(0.0, 0.0, 0.0); // Desired acceleration
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); // Current rotation
    
    printf("TEST CASE 1: 1m position error in Z, no integral control\n");
    printf("State: x=[%.1f, %.1f, %.1f], v=[%.1f, %.1f, %.1f]\n", x[0], x[1], x[2], v[0], v[1], v[2]);
    printf("Desired: xd=[%.1f, %.1f, %.1f], xd_dot=[%.1f, %.1f, %.1f], xd_2dot=[%.1f, %.1f, %.1f]\n", 
           xd[0], xd[1], xd[2], xd_dot[0], xd_dot[1], xd_dot[2], xd_2dot[0], xd_2dot[1], xd_2dot[2]);
    printf("Rotation: Identity\n\n");
    
    FDCLController fdcl;
    double fdcl_thrust = fdcl.position_control_fdcl(x, v, xd, xd_dot, xd_2dot, R, false, 0.01);
    
    double our_thrust = test_our_implementation(x, v, xd, xd_dot, xd_2dot, R, false, 0.01);
    
    printf("\n=== COMPARISON RESULTS ===\n");
    printf("FDCL thrust:  %.10f\n", fdcl_thrust);
    printf("Our thrust:   %.10f\n", our_thrust);
    printf("Difference:   %.10f\n", our_thrust - fdcl_thrust);
    printf("Relative err: %.2e\n", abs(our_thrust - fdcl_thrust) / abs(fdcl_thrust));
    
    // Test case 2: With integral control
    printf("\n\nTEST CASE 2: Same case but with integral control\n");
    
    FDCLController fdcl2;
    double fdcl_thrust2 = fdcl2.position_control_fdcl(x, v, xd, xd_dot, xd_2dot, R, true, 0.01);
    
    double our_thrust2 = test_our_implementation(x, v, xd, xd_dot, xd_2dot, R, true, 0.01);
    
    printf("\n=== COMPARISON RESULTS (WITH INTEGRAL) ===\n");
    printf("FDCL thrust:  %.10f\n", fdcl_thrust2);
    printf("Our thrust:   %.10f\n", our_thrust2);
    printf("Difference:   %.10f\n", our_thrust2 - fdcl_thrust2);
    printf("Relative err: %.2e\n", abs(our_thrust2 - fdcl_thrust2) / abs(fdcl_thrust2));
    
    // Test case 3: Different position error
    printf("\n\nTEST CASE 3: Different position error [0.1, 0.2, 0.5]\n");
    
    Eigen::Vector3d x3(0.1, 0.2, 0.5);
    Eigen::Vector3d v3(0.0, 0.0, 0.0);
    Eigen::Vector3d xd3(0.0, 0.0, 0.0);
    Eigen::Vector3d xd_dot3(0.0, 0.0, 0.0);
    Eigen::Vector3d xd_2dot3(0.0, 0.0, 0.0);
    
    FDCLController fdcl3;
    double fdcl_thrust3 = fdcl3.position_control_fdcl(x3, v3, xd3, xd_dot3, xd_2dot3, R, false, 0.01);
    
    double our_thrust3 = test_our_implementation(x3, v3, xd3, xd_dot3, xd_2dot3, R, false, 0.01);
    
    printf("\n=== COMPARISON RESULTS (DIFFERENT ERROR) ===\n");
    printf("FDCL thrust:  %.10f\n", fdcl_thrust3);
    printf("Our thrust:   %.10f\n", our_thrust3);
    printf("Difference:   %.10f\n", our_thrust3 - fdcl_thrust3);
    printf("Relative err: %.2e\n", abs(our_thrust3 - fdcl_thrust3) / abs(fdcl_thrust3));
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main() {
    test_position_control_comparison();
    return 0;
} 