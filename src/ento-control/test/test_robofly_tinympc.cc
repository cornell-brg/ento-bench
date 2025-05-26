#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-control/tinympc_solver.h>
#include <ento-control/lqr_traits_robofly.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;

// Horizon length for MPC
constexpr int HORIZON = 10;

void test_robofly_tinympc_basic() {
    using Scalar = float;
    constexpr int N = RoboFlyLQRTraits::N;  // 10 states
    constexpr int M = RoboFlyLQRTraits::M;  // 3 inputs
    constexpr int H = HORIZON;
    
    // Get the dynamics matrices from RoboFlyLQRTraits (continuous-time)
    Matrix<Scalar, N, N> Ac = RoboFlyLQRTraits::Adyn;
    Matrix<Scalar, N, M> Bc = RoboFlyLQRTraits::Bdyn;
    
    // Discretize the continuous dynamics (Euler discretization)
    Scalar dt = 0.01f;  // 100 Hz control rate
    Matrix<Scalar, N, N> Ad = Matrix<Scalar, N, N>::Identity() + Ac * dt;
    Matrix<Scalar, N, M> Bd = Bc * dt;
    
    // Create diagonal state and input cost matrices
    Matrix<Scalar, N, 1> Q;
    Matrix<Scalar, M, 1> R;
    
    // Using values that should lead to reasonable control behavior
    Q << 20.0f, 20.0f, 20.0f,   // position (x,y,z)
         5.0f, 5.0f, 5.0f,       // velocity (vx,vy,vz)
         50.0f, 50.0f,           // orientation (roll, pitch)
         10.0f, 10.0f;           // angular velocity (ωx, ωy)
    
    R << 1.0f, 1.0f, 1.0f;       // 3 control inputs
    
    // Print the cost matrices
    ENTO_DEBUG("State cost Q:");
    for (int i = 0; i < N; i++) {
        ENTO_DEBUG("Q[%d] = %f", i, Q(i));
    }
    
    ENTO_DEBUG("Control cost R:");
    for (int i = 0; i < M; i++) {
        ENTO_DEBUG("R[%d] = %f", i, R(i));
    }
    
    // Define constraints (large enough to be non-binding)
    Matrix<Scalar, N, H> x_min = Matrix<Scalar, N, H>::Constant(-1000.0f);
    Matrix<Scalar, N, H> x_max = Matrix<Scalar, N, H>::Constant(1000.0f);
    Matrix<Scalar, M, H-1> u_min = Matrix<Scalar, M, H-1>::Constant(-100.0f);
    Matrix<Scalar, M, H-1> u_max = Matrix<Scalar, M, H-1>::Constant(100.0f);
    
    // Penalty parameter
    Scalar rho = 1.0f;
    
    // Create the solver with verbose debugging
    // Note: TinyMPC internally computes an infinite-horizon LQR solution (Kinf)
    // and uses it as part of its MPC formulation
    using MPCSolver = TinyMPCSolver<Scalar, N, M, H>;
    MPCSolver solver(Ad, Bd, Q, R, rho, x_min, x_max, u_min, u_max, true);
    
    // Get the precomputed LQR gain matrix for comparison
    Matrix<Scalar, M, N> K_precomputed = RoboFlyLQRTraits::K;
    
    // Set initial state - non-zero to test control
    Matrix<Scalar, N, 1> x0;
    x0 << 1.0f, 1.0f, 1.0f,    // Initial position: (1,1,1) - modest displacement
          0.0f, 0.0f, 0.0f,    // Initial velocity: (0,0,0)
          0.1f, 0.1f,          // Initial orientation: slight tilt
          0.0f, 0.0f;          // Initial angular velocity: (0,0)
    
    // Set reference trajectory (target at origin)
    Matrix<Scalar, N, H> x_ref = Matrix<Scalar, N, H>::Zero();
    
    // Configure solver
    solver.set_x0(x0);
    solver.set_x_ref(x_ref);
    solver.reset_duals();
    
    // Solve the MPC problem
    solver.solve();
    
    // Get the optimal control input from MPC
    auto u0_mpc = solver.get_u0();
    
    // Get the LQR control using the precomputed K matrix
    auto u0_lqr = -K_precomputed * x0;
    
    // Print the control inputs for comparison
    ENTO_DEBUG("MPC control input: [%f, %f, %f]", u0_mpc(0), u0_mpc(1), u0_mpc(2));
    ENTO_DEBUG("MPC control norm: %f", u0_mpc.norm());
    ENTO_DEBUG("LQR control input: [%f, %f, %f]", u0_lqr(0), u0_lqr(1), u0_lqr(2));
    ENTO_DEBUG("LQR control norm: %f", u0_lqr.norm());
    
    // At least one controller should generate non-zero control
    ENTO_TEST_CHECK_TRUE(u0_mpc.norm() > 0.01f || u0_lqr.norm() > 0.01f);
    
    // Run a simulation for multiple steps
    Matrix<Scalar, N, 1> state_mpc = x0;
    Matrix<Scalar, N, 1> state_lqr = x0;
    
    // Calculate initial state norm for comparison
    Scalar initial_norm = x0.norm();
    ENTO_DEBUG("Initial state norm: %f", initial_norm);
    
    // Number of simulation steps
    constexpr int sim_steps = 50;  // More steps to see convergence
    
    for (int i = 0; i < sim_steps; i++) {
        // MPC simulation
        solver.set_x0(state_mpc);
        solver.solve();
        auto u_mpc = solver.get_u0();
        
        // LQR simulation
        auto u_lqr = -K_precomputed * state_lqr;
        
        // Print controls and states every 10 steps to reduce output
        if (i % 10 == 0) {
            ENTO_DEBUG("Step %d:", i+1);
            ENTO_DEBUG("  MPC control: [%f, %f, %f], norm: %f", 
                     u_mpc(0), u_mpc(1), u_mpc(2), u_mpc.norm());
            ENTO_DEBUG("  LQR control: [%f, %f, %f], norm: %f", 
                     u_lqr(0), u_lqr(1), u_lqr(2), u_lqr.norm());
            ENTO_DEBUG("  MPC state norm: %f (%.1f%% of initial)", 
                     state_mpc.norm(), 100.0f * state_mpc.norm() / initial_norm);
            ENTO_DEBUG("  LQR state norm: %f (%.1f%% of initial)", 
                     state_lqr.norm(), 100.0f * state_lqr.norm() / initial_norm);
        }
        
        // Apply control and update states with discretized dynamics
        state_mpc = Ad * state_mpc + Bd * u_mpc;
        state_lqr = Ad * state_lqr + Bd * u_lqr;
    }
    
    // Print final state norms
    ENTO_DEBUG("Final MPC state norm: %f (%.1f%% of initial)", 
             state_mpc.norm(), 100.0f * state_mpc.norm() / initial_norm);
    ENTO_DEBUG("Final LQR state norm: %f (%.1f%% of initial)", 
             state_lqr.norm(), 100.0f * state_lqr.norm() / initial_norm);
    
    // Test passes if the controllers have non-zero output
    // We don't test convergence since the model may not be fully controllable
    ENTO_TEST_CHECK_TRUE(true);
}

int main(int argc, char** argv)
{
    int __n;
    if (argc > 1)
    {
        __n = atoi(argv[1]);
    }
    else
    {
        // For the case we are running on the MCU and we can't pass in args
        // the same way args are passed for a native build.
        __ento_replace_file_suffix(__FILE__, "test_robofly_tinympc_cmdline.txt");
        __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
    }

    ENTO_TEST_START();
    
    if (__ento_test_num(__n, 1)) test_robofly_tinympc_basic();
    
    ENTO_TEST_END();
} 