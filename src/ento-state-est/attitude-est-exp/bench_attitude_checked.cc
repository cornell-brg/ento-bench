#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

// Core attitude estimation
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>

// Checked filter implementations
#include <ento-state-est/attitude-est-exp/kernels/madgwick_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/madgwick_fixed_checked.h>
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>
#include <ento-state-est/attitude-est-exp/core/fp_overflow_tracker.h>

// Utilities
#include <ento-util/debug.h>

using namespace EntoAttitude;
using namespace EntoAttitudeExp;

// Failure statistics structure
struct FailureStats {
    size_t total_samples = 0;
    size_t overflow_count = 0;
    size_t bad_norm_count = 0;
    size_t near_zero_div_count = 0;
    size_t excessive_err_count = 0;
    size_t any_failure_count = 0;
    
    void add_failure(const FailureFlags& flags) {
        if (flags[FailureReason::Overflow]) overflow_count++;
        if (flags[FailureReason::BadNorm]) bad_norm_count++;
        if (flags[FailureReason::NearZeroDiv]) near_zero_div_count++;
        if (flags[FailureReason::ExcessiveErr]) excessive_err_count++;
        if (flags.any()) any_failure_count++;
        total_samples++;
    }
    
    void print_summary() const {
        std::cout << "\n=== FAILURE ANALYSIS SUMMARY ===\n";
        std::cout << "Total samples processed: " << total_samples << "\n";
        std::cout << "Samples with any failure: " << any_failure_count 
                  << " (" << std::fixed << std::setprecision(2) 
                  << (100.0 * any_failure_count / total_samples) << "%)\n";
        std::cout << "\nFailure breakdown:\n";
        std::cout << "  Overflow/saturation: " << overflow_count 
                  << " (" << (100.0 * overflow_count / total_samples) << "%)\n";
        std::cout << "  Bad quaternion norm: " << bad_norm_count 
                  << " (" << (100.0 * bad_norm_count / total_samples) << "%)\n";
        std::cout << "  Near-zero divisors: " << near_zero_div_count 
                  << " (" << (100.0 * near_zero_div_count / total_samples) << "%)\n";
        std::cout << "  Excessive error: " << excessive_err_count 
                  << " (" << (100.0 * excessive_err_count / total_samples) << "%)\n";
    }
};

// Template function to run benchmark for any kernel type
template<typename KernelType, typename ScalarType>
FailureStats run_kernel_benchmark(const char* kernel_name, const char* dataset_path) {
    ENTO_INFO("Testing: %s", kernel_name);
    
    // Create the checked kernel and problem
    KernelType kernel;
    FailureFlags failure_flags;
    kernel.setFailureFlags(&failure_flags);
    
    // Create problem using generic constructor
    AttitudeProblem<ScalarType, KernelType, false> problem(kernel);
    
    // Open dataset file
    std::ifstream input_file(dataset_path);
    if (!input_file.is_open()) {
        ENTO_INFO("ERROR: Failed to open dataset file: %s", dataset_path);
        return FailureStats();
    }
    
    FailureStats stats;
    std::string line;
    size_t line_count = 0;
    
    // Skip header line if present
    if (std::getline(input_file, line) && line.find("Attitude Estimation Problem") != std::string::npos) {
        // Header found, continue
    } else {
        // No header, rewind to beginning
        input_file.clear();
        input_file.seekg(0);
    }
    
    // Process each sample
    while (std::getline(input_file, line) && !line.empty()) {
        line_count++;
        
        // Deserialize the line
        if (!problem.deserialize_impl(line.c_str())) {
            ENTO_INFO("WARNING: Failed to deserialize line %zu", line_count);
            continue;
        }
        
        // Run the filter (this will set failure flags)
        problem.solve_impl();
        
        // Record failure statistics
        stats.add_failure(failure_flags);
        
        // Progress indicator every 1000 samples
        if (line_count % 1000 == 0) {
            std::cout << "Processed " << line_count << " samples...\r" << std::flush;
        }
    }
    
    input_file.close();
    std::cout << "\n";
    
    ENTO_INFO("Completed %s: Processed %zu samples.", kernel_name, line_count);
    return stats;
}

int main() {
    ENTO_INFO("Starting Q-format sweep for Madgwick filter...");
    
    // Hard-coded dataset path (IMU-only)
    const char* dataset_path = "../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt";
    ENTO_INFO("Dataset: %s", dataset_path);
    
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "MADGWICK FILTER Q-FORMAT FAILURE ANALYSIS\n";
    std::cout << std::string(70, '=') << "\n\n";
    
    // Store all results for final comparison
    std::vector<std::pair<std::string, FailureStats>> all_results;
    
    // Test 1: Float Madgwick (baseline - should have no failures)
    std::cout << "1/6: Testing Float (baseline)...\n";
    auto stats_float = run_kernel_benchmark<FilterMadgwickChecked<float, false>, float>(
        "Madgwick Float IMU", dataset_path);
    stats_float.print_summary();
    all_results.emplace_back("Float", stats_float);
    
    std::cout << "\n" << std::string(70, '-') << "\n";
    
    // Test 2: Q7.24 (32-bit: 7 integer, 24 fractional)
    std::cout << "2/6: Testing Q7.24 (32-bit: 7I.24F)...\n";
    auto stats_q7_24 = run_kernel_benchmark<FilterMadgwickQ7_24Checked<false>, Q7_24>(
        "Madgwick Q7.24 IMU", dataset_path);
    stats_q7_24.print_summary();
    all_results.emplace_back("Q7.24", stats_q7_24);
    
    std::cout << "\n" << std::string(70, '-') << "\n";
    
    // Test 3: Q6.25 (32-bit: 6 integer, 25 fractional)
    std::cout << "3/6: Testing Q6.25 (32-bit: 6I.25F)...\n";
    auto stats_q6_25 = run_kernel_benchmark<FilterMadgwickQ6_25Checked<false>, Q6_25>(
        "Madgwick Q6.25 IMU", dataset_path);
    stats_q6_25.print_summary();
    all_results.emplace_back("Q6.25", stats_q6_25);
    
    std::cout << "\n" << std::string(70, '-') << "\n";
    
    // Test 4: Q5.26 (32-bit: 5 integer, 26 fractional)
    std::cout << "4/6: Testing Q5.26 (32-bit: 5I.26F)...\n";
    auto stats_q5_26 = run_kernel_benchmark<FilterMadgwickQ5_26Checked<false>, Q5_26>(
        "Madgwick Q5.26 IMU", dataset_path);
    stats_q5_26.print_summary();
    all_results.emplace_back("Q5.26", stats_q5_26);
    
    std::cout << "\n" << std::string(70, '-') << "\n";
    
    // Test 5: Q3.12 (16-bit: 3 integer, 12 fractional)
    std::cout << "5/6: Testing Q3.12 (16-bit: 3I.12F)...\n";
    auto stats_q3_12 = run_kernel_benchmark<FilterMadgwickQ3_12Checked<false>, Q3_12>(
        "Madgwick Q3.12 IMU", dataset_path);
    stats_q3_12.print_summary();
    all_results.emplace_back("Q3.12", stats_q3_12);
    
    std::cout << "\n" << std::string(70, '-') << "\n";
    
    // Test 6: Q2.13 (16-bit: 2 integer, 13 fractional)
    std::cout << "6/6: Testing Q2.13 (16-bit: 2I.13F)...\n";
    auto stats_q2_13 = run_kernel_benchmark<FilterMadgwickQ2_13Checked<false>, Q2_13>(
        "Madgwick Q2.13 IMU", dataset_path);
    stats_q2_13.print_summary();
    all_results.emplace_back("Q2.13", stats_q2_13);
    
    // Final comparison table
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "COMPLETE Q-FORMAT COMPARISON\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << "Format     | Total  | Failures | Rate    | Overflow | BadNorm | NearZero | ExcessErr\n";
    std::cout << std::string(70, '-') << "\n";
    
    for (const auto& [name, stats] : all_results) {
        printf("%-10s | %6zu | %8zu | %6.2f%% | %8zu | %7zu | %8zu | %9zu\n",
               name.c_str(),
               stats.total_samples,
               stats.any_failure_count,
               (100.0 * stats.any_failure_count / stats.total_samples),
               stats.overflow_count,
               stats.bad_norm_count,
               stats.near_zero_div_count,
               stats.excessive_err_count);
    }
    
    std::cout << std::string(70, '=') << "\n";
    std::cout << "Legend: I=Integer bits, F=Fractional bits\n";
    std::cout << "32-bit formats: Q7.24, Q6.25, Q5.26\n";
    std::cout << "16-bit formats: Q3.12, Q2.13\n";
    
    return 0;
} 