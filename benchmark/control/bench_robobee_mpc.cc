#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/experiment_io.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>

#include <ento-control/opt_control_problem.h>
#include <ento-control/tinympc_solver.h>

#include <cstdio>

extern "C" void initialise_monitor_handles(void);

using namespace EntoControl;

// RoboBee MPC parameters
constexpr int path_len = 315;
constexpr int num_states = 12;
constexpr int num_inputs = 4;
constexpr int len_horizon = 10;
constexpr double rho_value = 5.0;

void bench_robobee_mpc() {
    using Scalar = float;
    using Solver = TinyMPCSolver<Scalar, num_states, num_inputs, len_horizon>;
    using Problem = OptControlProblem<Scalar, Solver, num_states, num_inputs, len_horizon, path_len>;
    
    ENTO_DEBUG("=== RoboBee MPC Controller Benchmark ===");
    
    // System dynamics matrices (from test data)
    Eigen::Matrix<Scalar, num_states, num_states> Adyn = (Eigen::Matrix<Scalar, num_states, num_states>() <<
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
        
    Eigen::Matrix<Scalar, num_states, num_inputs> Bdyn = (Eigen::Matrix<Scalar, num_states, num_inputs>() <<
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
          
    Eigen::Matrix<Scalar, num_states, 1> Q{100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, 4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000};
    Eigen::Matrix<Scalar, num_inputs, 1> R{4.0, 4.0, 4.0, 4.0};

    Eigen::Matrix<Scalar, num_states, len_horizon> x_min = Eigen::Matrix<Scalar, num_states, len_horizon>::Constant(-5);
    Eigen::Matrix<Scalar, num_states, len_horizon> x_max = Eigen::Matrix<Scalar, num_states, len_horizon>::Constant(5);
    Eigen::Matrix<Scalar, num_inputs, len_horizon - 1> u_min = Eigen::Matrix<Scalar, num_inputs, len_horizon - 1>::Constant(-0.5);
    Eigen::Matrix<Scalar, num_inputs, len_horizon - 1> u_max = Eigen::Matrix<Scalar, num_inputs, len_horizon - 1>::Constant(0.5);

    Solver solver(Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true);

    auto tiny_settings = solver.get_settings();
    solver.update_settings(tiny_settings.abs_pri_tol, tiny_settings.abs_dua_tol, 100,
                          tiny_settings.check_termination, tiny_settings.en_state_bound, tiny_settings.en_input_bound);

    // Load trajectory data
    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/fig8.csv";
    char full_path[256];
    
    if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path))) {
        ENTO_DEBUG("ERROR! Could not build file path for RoboBee MPC benchmark!");
        return;
    }
    
    Problem problem(solver);
    ENTO_DEBUG("Loading trajectory data from: %s", full_path);
    EntoUtil::ExperimentIO reader(full_path);

    // Benchmark metrics
    uint32_t total_cycles = 0;
    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0;
    int num_experiments = 0;

    // Run benchmark
    while (reader.read_next(problem)) {
        // Measure performance
        uint32_t start_cycles = DWT->CYCCNT;
        problem.solve();
        uint32_t end_cycles = DWT->CYCCNT;
        
        uint32_t cycles = end_cycles - start_cycles;
        total_cycles += cycles;
        
        if (cycles < min_cycles) min_cycles = cycles;
        if (cycles > max_cycles) max_cycles = cycles;
        
        num_experiments++;
        
        if (num_experiments <= 5) {  // Log first few iterations
            ENTO_DEBUG("Experiment %d: Cycles=%lu", num_experiments, cycles);
        }
    }
    
    if (num_experiments > 0) {
        problem.validate();
        
        // Calculate statistics
        uint32_t avg_cycles = total_cycles / num_experiments;
        float avg_time_us = (float)avg_cycles / (SystemCoreClock / 1000000.0f);
        float min_time_us = (float)min_cycles / (SystemCoreClock / 1000000.0f);
        float max_time_us = (float)max_cycles / (SystemCoreClock / 1000000.0f);
        
        ENTO_DEBUG("\n=== RoboBee MPC Performance Results ===");
        ENTO_DEBUG("Experiments: %d", num_experiments);
        ENTO_DEBUG("Average: %lu cycles (%.2f μs)", avg_cycles, avg_time_us);
        ENTO_DEBUG("Min:     %lu cycles (%.2f μs)", min_cycles, min_time_us);
        ENTO_DEBUG("Max:     %lu cycles (%.2f μs)", max_cycles, max_time_us);
        ENTO_DEBUG("Frequency: %.1f Hz", 1000000.0f / avg_time_us);
    } else {
        ENTO_DEBUG("ERROR: No experiments loaded from trajectory file!");
    }
}

int main() {
    initialise_monitor_handles();
    
    // Initialize MCU
    EntoMCU::cache_init();
    EntoMCU::flash_init();
    EntoMCU::clk_init();
    EntoMCU::systick_config();
    
    // Enable cycle counter for performance measurement
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    
    ENTO_DEBUG("Starting RoboBee MPC benchmark...");
    
    bench_robobee_mpc();
    
    ENTO_DEBUG("Benchmark completed.");
    
    return 0;
} 