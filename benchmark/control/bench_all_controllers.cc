#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>

#include <cstdio>

// Include all controller headers
#include <ento-control/robofly_lqr_solver.h>
#include <ento-control/tinympc_solver.h>
#include <ento-control/geometric_controller_problem.h>
#include <ento-control/opt_control_problem.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoControl;

// Common parameters
constexpr int path_len = 315;
constexpr int num_states = 12;
constexpr int num_inputs = 4;
constexpr int len_horizon = 10;

void benchmark_robofly_lqr() {
    printf("\n=== Benchmarking Robofly LQR Controller ===\n");
    
    using Scalar_t = float;
    using Solver = RoboflyLQRSolver<Scalar_t, num_states, num_inputs>;
    using Problem = OptControlProblem<Scalar_t, Solver, num_states, num_inputs, len_horizon, path_len>;
    
    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/fig8.csv";
    char dataset_path[512];
    char output_path[256];
    
    if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path))) {
        ENTO_DEBUG("ERROR! Could not build file path for Robofly LQR");
        return;
    }
    
    printf("Dataset: %s\n", dataset_path);
    
    // Configure LQR gains for Robofly (smaller MAV)
    Eigen::Matrix<Scalar_t, num_states, num_states> Q = Eigen::Matrix<Scalar_t, num_states, num_states>::Identity();
    Q.diagonal() << 50.0, 50.0, 100.0, 2.0, 2.0, 10.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;
    
    Eigen::Matrix<Scalar_t, num_inputs, num_inputs> R = Eigen::Matrix<Scalar_t, num_inputs, num_inputs>::Identity();
    R.diagonal() << 1.0, 1.0, 1.0, 1.0;
    
    static Solver solver(Q, R);
    Problem problem(solver);
    
    EntoBench::Harness<Problem, false, 5> harness(problem, "Robofly LQR Controller", 
                                                   dataset_path, output_path);
    harness.run();
}

void benchmark_robofly_tinympc() {
    printf("\n=== Benchmarking Robofly TinyMPC Controller ===\n");
    
    using Scalar_t = float;
    using Solver = TinyMPCSolver<Scalar_t, num_states, num_inputs, len_horizon>;
    using Problem = OptControlProblem<Scalar_t, Solver, num_states, num_inputs, len_horizon, path_len>;
    
    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/fig8.csv";
    char dataset_path[512];
    char output_path[256];
    
    if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path))) {
        ENTO_DEBUG("ERROR! Could not build file path for Robofly TinyMPC");
        return;
    }
    
    printf("Dataset: %s\n", dataset_path);
    
    // Robofly dynamics matrices (smaller MAV)
    Eigen::Matrix<Scalar_t, num_states, num_states> Adyn = (Eigen::Matrix<Scalar_t, num_states, num_states>() <<
        1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000,  0.0000000, 0.0002044, 0.0000000,
        0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0250000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0250000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0250000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0122625, 0.0000000,
        0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000).finished();
    
    Eigen::Matrix<Scalar_t, num_states, num_inputs> Bdyn = (Eigen::Matrix<Scalar_t, num_states, num_inputs>() <<
        -0.0007069,   0.0007773,  0.0007091,  -0.0007795,
         0.0007034,   0.0007747, -0.0007042,  -0.0007739,
         0.0052554,   0.0052554,  0.0052554,   0.0052554,
        -0.1720966,  -0.1895213,  0.1722891,   0.1893288,
        -0.1729419,   0.1901740,  0.1734809,  -0.1907131,
         0.0123423,  -0.0045148, -0.0174024,   0.0095748,
        -0.0565520,   0.0621869,  0.0567283,  -0.0623632,
         0.0562756,   0.0619735, -0.0563386,  -0.0619105,
         0.2102143,   0.2102143,  0.2102143,   0.2102143,
       -13.7677303, -15.1617018, 13.7831318,  15.1463003,
       -13.8353509,  15.2139209, 13.8784751, -15.2570451,
         0.9873856,  -0.3611820, -1.3921880,   0.7659845).finished();
    
    Eigen::Matrix<Scalar_t, num_states, 1> Q{100.0, 100.0, 100.0, 4.0, 4.0, 400.0, 4.0, 4.0, 4.0, 2.0408163, 2.0408163, 4.0};
    Eigen::Matrix<Scalar_t, num_inputs, 1> R{4.0, 4.0, 4.0, 4.0};
    
    Eigen::Matrix<Scalar_t, num_states, len_horizon> x_min = Eigen::Matrix<Scalar_t, num_states, len_horizon>::Constant(-5);
    Eigen::Matrix<Scalar_t, num_states, len_horizon> x_max = Eigen::Matrix<Scalar_t, num_states, len_horizon>::Constant(5);
    Eigen::Matrix<Scalar_t, num_inputs, len_horizon - 1> u_min = Eigen::Matrix<Scalar_t, num_inputs, len_horizon - 1>::Constant(-0.5);
    Eigen::Matrix<Scalar_t, num_inputs, len_horizon - 1> u_max = Eigen::Matrix<Scalar_t, num_inputs, len_horizon - 1>::Constant(0.5);
    
    float rho_value = 5.0f;
    
    static Solver solver(Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true);
    
    auto tiny_settings = solver.get_settings();
    solver.update_settings(tiny_settings.abs_pri_tol, tiny_settings.abs_dua_tol, 100,
                          tiny_settings.check_termination, tiny_settings.en_state_bound, tiny_settings.en_input_bound);
    
    Problem problem(solver);
    
    EntoBench::Harness<Problem, false, 5> harness(problem, "Robofly TinyMPC Controller", 
                                                   dataset_path, output_path);
    harness.run();
}

void benchmark_robobee_geometric() {
    printf("\n=== Benchmarking RoboBee Geometric Controller ===\n");
    
    using Scalar = float;
    using Problem = GeometricControllerProblem<Scalar>;
    
    const char* base_path = DATASET_PATH;
    const char* rel_path = "geometric-control/helix.csv";
    char dataset_path[512];
    char output_path[256];
    
    if (!EntoUtil::build_file_path(base_path, rel_path, dataset_path, sizeof(dataset_path))) {
        ENTO_DEBUG("ERROR! Could not build file path for RoboBee Geometric");
        return;
    }
    
    printf("Dataset: %s\n", dataset_path);
    
    Problem problem;
    problem.set_default_gains();  // Uses Python simulation parameters
    
    EntoBench::Harness<Problem, false, 5> harness(problem, "RoboBee Geometric Controller", 
                                                   dataset_path, output_path);
    harness.run();
}

int main() {
    initialise_monitor_handles();
    
    // Configure system
    sys_clk_cfg();
    SysTick_Setup();
    __enable_irq();
    
    // Enable caches for performance
    enable_instruction_cache();
    enable_instruction_cache_prefetch();
    icache_enable();
    
    printf("=== EntoBench: Comprehensive Controller Benchmark ===\n");
    printf("Testing 5 controllers for flapping wing MAVs:\n");
    printf("1. Robofly LQR\n");
    printf("2. Robofly TinyMPC\n");
    printf("3. RoboBee Geometric Control\n");
    printf("Note: RoboBee MPC and Sliding Mode controllers require additional implementation\n\n");
    
    // Run benchmarks for implemented controllers
    benchmark_robofly_lqr();
    benchmark_robofly_tinympc();
    benchmark_robobee_geometric();
    
    printf("\n=== Benchmark Complete ===\n");
    printf("All controllers produce torque/force commands that would be fed to inverse mapping.\n");
    printf("Each controller uses appropriate datasets and parameters for fair comparison.\n");
    
    exit(1);
    return 0;
} 