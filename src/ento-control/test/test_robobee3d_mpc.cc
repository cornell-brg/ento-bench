#include <gtest/gtest.h>
#include "robobee_mpc_solver.h"
#include <Eigen/Dense>

TEST(Robobee3DMPC, BasicFunctionality) {
    // Set up MPC parameters
    const double dt = 0.01;  // 10ms time step
    const double g = 9.81;   // gravity
    const double mass = 0.1; // 100g mass
    
    // Create solver
    RobobeeMPCSolver solver(dt, g, mass);
    
    // Set initial state [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
    Eigen::Matrix<double, 12, 1> x0;
    x0 << 0.0, 0.0, 0.0,  // position
          0.0, 0.0, 0.0,  // velocity
          1.0, 0.0, 0.0, 0.0,  // quaternion (identity)
          0.0, 0.0, 0.0;  // angular velocity
    
    solver.set_x0(x0);
    
    // Set reference trajectory (hover at 1m height)
    Eigen::Matrix<double, 12, 1> x_ref;
    x_ref << 0.0, 0.0, 1.0,  // position
             0.0, 0.0, 0.0,  // velocity
             1.0, 0.0, 0.0, 0.0,  // quaternion (identity)
             0.0, 0.0, 0.0;  // angular velocity
    
    solver.set_x_ref(x_ref);
    
    // Solve MPC
    bool success = solver.solve();
    EXPECT_TRUE(success);
    
    // Get control inputs
    Eigen::Matrix<double, 3, 1> u = solver.get_u();
    
    // Check that thrust is close to gravity
    EXPECT_NEAR(u(0), g * mass, 0.1);
    
    // Check that moments are small
    EXPECT_NEAR(u(1), 0.0, 0.1);
    EXPECT_NEAR(u(2), 0.0, 0.1);
}

TEST(Robobee3DMPC, Tracking) {
    // Set up MPC parameters
    const double dt = 0.01;  // 10ms time step
    const double g = 9.81;   // gravity
    const double mass = 0.1; // 100g mass
    
    // Create solver
    RobobeeMPCSolver solver(dt, g, mass);
    
    // Set initial state
    Eigen::Matrix<double, 12, 1> x0;
    x0 << 0.0, 0.0, 0.0,  // position
          0.0, 0.0, 0.0,  // velocity
          1.0, 0.0, 0.0, 0.0,  // quaternion (identity)
          0.0, 0.0, 0.0;  // angular velocity
    
    solver.set_x0(x0);
    
    // Set reference trajectory (move to 1m height with 0.5m/s)
    Eigen::Matrix<double, 12, 1> x_ref;
    x_ref << 0.0, 0.0, 1.0,  // position
             0.0, 0.0, 0.5,  // velocity
             1.0, 0.0, 0.0, 0.0,  // quaternion (identity)
             0.0, 0.0, 0.0;  // angular velocity
    
    solver.set_x_ref(x_ref);
    
    // Solve MPC
    bool success = solver.solve();
    EXPECT_TRUE(success);
    
    // Get control inputs
    Eigen::Matrix<double, 3, 1> u = solver.get_u();
    
    // Check that thrust is greater than gravity (to accelerate upward)
    EXPECT_GT(u(0), g * mass);
    
    // Check that moments are small
    EXPECT_NEAR(u(1), 0.0, 0.1);
    EXPECT_NEAR(u(2), 0.0, 0.1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 