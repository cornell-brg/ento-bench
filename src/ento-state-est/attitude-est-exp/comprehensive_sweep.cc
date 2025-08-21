#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <cstdio>

// Core includes
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>
#include <ento-state-est/attitude-est-exp/core/fp_overflow_tracker.h>
#include <ento-state-est/attitude-est-exp/kernels/qformat_generator.h>

// Utilities
#include <ento-util/debug.h>

using namespace EntoAttitude;
using namespace EntoAttitudeExp;

//------------------------------------------------------------------------------
// Result structure for each format test
//------------------------------------------------------------------------------
struct FormatResult {
    QFormatSpec format_spec;
    FailureStats stats;
    double runtime_ms;
    bool test_succeeded;
    std::string error_message;
    
    double failure_rate() const {
        return (stats.total_samples > 0) ? 
               (100.0 * stats.any_failure_count / stats.total_samples) : 0.0;
    }
    
    void print_summary() const {
        printf("%-12s | %6zu | %8zu | %6.2f%% | %8zu | %7zu | %8zu | %9zu | %7.1fms\n",
               format_spec.name().c_str(),
               stats.total_samples,
               stats.any_failure_count,
               failure_rate(),
               stats.overflow_count,
               stats.bad_norm_count,
               stats.near_zero_div_count,
               stats.excessive_err_count,
               runtime_ms);
    }
};

//------------------------------------------------------------------------------
// Template dispatch mechanism using variadic templates
//------------------------------------------------------------------------------
template<typename KernelType, typename ScalarType>
FormatResult run_single_format_test(const QFormatSpec& spec, const char* dataset_path) {
    FormatResult result;
    result.format_spec = spec;
    result.test_succeeded = false;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Create the checked kernel and problem
        KernelType kernel;
        FailureFlags failure_flags;
        kernel.setFailureFlags(&failure_flags);
        
        // Create problem using generic constructor
        AttitudeProblem<ScalarType, KernelType, false> problem(kernel);
        
        // Open dataset file
        std::ifstream input_file(dataset_path);
        if (!input_file.is_open()) {
            result.error_message = "Failed to open dataset file";
            return result;
        }
        
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
                continue; // Skip invalid lines
            }
            
            // Run the filter (this will set failure flags)
            problem.solve_impl();
            
            // Record failure statistics
            result.stats.add_failure(failure_flags);
        }
        
        input_file.close();
        result.test_succeeded = true;
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    } catch (...) {
        result.error_message = "Unknown exception occurred";
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.runtime_ms = duration.count();
    
    return result;
}

//------------------------------------------------------------------------------
// Helper function to replace macro (avoids braced initializer issues)
//------------------------------------------------------------------------------
template<typename FilterType, typename ScalarType>
FormatResult test_format_helper(const QFormatSpec& spec, const char* dataset_path) {
    std::cout << "Testing " << spec.name() << "..." << std::flush;
    auto result = run_single_format_test<FilterType, ScalarType>(spec, dataset_path);
    std::cout << " " << (result.test_succeeded ? "OK" : "FAILED") << std::endl;
    if (!result.test_succeeded) {
        std::cout << "  Error: " << result.error_message << std::endl;
    }
    return result;
}

std::vector<FormatResult> run_comprehensive_sweep(const char* dataset_path) {
    std::vector<FormatResult> all_results;
    all_results.reserve(48);
    
    std::cout << "\n" << std::string(80, '=') << "\n";
    std::cout << "COMPREHENSIVE Q-FORMAT SWEEP (48 FORMATS)\n";
    std::cout << std::string(80, '=') << "\n\n";
    
    std::cout << "Progress:\n";
    
    // Floating-point formats (2 formats)
    std::cout << "\n[1/3] Floating-point formats (2 formats):\n";
    all_results.push_back(test_format_helper<FilterFloat<false>, float>(
        QFormatSpec{23, 8, 32, true, "float"}, dataset_path));
    all_results.push_back(test_format_helper<FilterDouble<false>, double>(
        QFormatSpec{52, 11, 64, true, "double"}, dataset_path));
    
    // 16-bit fixed-point formats (15 formats)
    std::cout << "\n[2/3] 16-bit Q-formats (15 formats):\n";
    all_results.push_back(test_format_helper<FilterQ1_15<false>, FixedPoint<1,15,int16_t>>(
        QFormatSpec{1, 15, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ2_14<false>, FixedPoint<2,14,int16_t>>(
        QFormatSpec{2, 14, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ3_13<false>, FixedPoint<3,13,int16_t>>(
        QFormatSpec{3, 13, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ4_12<false>, FixedPoint<4,12,int16_t>>(
        QFormatSpec{4, 12, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ5_11<false>, FixedPoint<5,11,int16_t>>(
        QFormatSpec{5, 11, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ6_10<false>, FixedPoint<6,10,int16_t>>(
        QFormatSpec{6, 10, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ7_9<false>, FixedPoint<7,9,int16_t>>(
        QFormatSpec{7, 9, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ8_8<false>, FixedPoint<8,8,int16_t>>(
        QFormatSpec{8, 8, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ9_7<false>, FixedPoint<9,7,int16_t>>(
        QFormatSpec{9, 7, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ10_6<false>, FixedPoint<10,6,int16_t>>(
        QFormatSpec{10, 6, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ11_5<false>, FixedPoint<11,5,int16_t>>(
        QFormatSpec{11, 5, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ12_4<false>, FixedPoint<12,4,int16_t>>(
        QFormatSpec{12, 4, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ13_3<false>, FixedPoint<13,3,int16_t>>(
        QFormatSpec{13, 3, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ14_2<false>, FixedPoint<14,2,int16_t>>(
        QFormatSpec{14, 2, 16, false, "int16_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ15_1<false>, FixedPoint<15,1,int16_t>>(
        QFormatSpec{15, 1, 16, false, "int16_t"}, dataset_path));
    
    // 32-bit fixed-point formats (31 formats)
    std::cout << "\n[3/3] 32-bit Q-formats (31 formats):\n";
    all_results.push_back(test_format_helper<FilterQ1_31<false>, FixedPoint<1,31,int32_t>>(
        QFormatSpec{1, 31, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ2_30<false>, FixedPoint<2,30,int32_t>>(
        QFormatSpec{2, 30, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ3_29<false>, FixedPoint<3,29,int32_t>>(
        QFormatSpec{3, 29, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ4_28<false>, FixedPoint<4,28,int32_t>>(
        QFormatSpec{4, 28, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ5_27<false>, FixedPoint<5,27,int32_t>>(
        QFormatSpec{5, 27, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ6_26<false>, FixedPoint<6,26,int32_t>>(
        QFormatSpec{6, 26, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ7_25<false>, FixedPoint<7,25,int32_t>>(
        QFormatSpec{7, 25, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ8_24<false>, FixedPoint<8,24,int32_t>>(
        QFormatSpec{8, 24, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ9_23<false>, FixedPoint<9,23,int32_t>>(
        QFormatSpec{9, 23, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ10_22<false>, FixedPoint<10,22,int32_t>>(
        QFormatSpec{10, 22, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ11_21<false>, FixedPoint<11,21,int32_t>>(
        QFormatSpec{11, 21, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ12_20<false>, FixedPoint<12,20,int32_t>>(
        QFormatSpec{12, 20, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ13_19<false>, FixedPoint<13,19,int32_t>>(
        QFormatSpec{13, 19, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ14_18<false>, FixedPoint<14,18,int32_t>>(
        QFormatSpec{14, 18, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ15_17<false>, FixedPoint<15,17,int32_t>>(
        QFormatSpec{15, 17, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ16_16<false>, FixedPoint<16,16,int32_t>>(
        QFormatSpec{16, 16, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ17_15<false>, FixedPoint<17,15,int32_t>>(
        QFormatSpec{17, 15, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ18_14<false>, FixedPoint<18,14,int32_t>>(
        QFormatSpec{18, 14, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ19_13<false>, FixedPoint<19,13,int32_t>>(
        QFormatSpec{19, 13, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ20_12<false>, FixedPoint<20,12,int32_t>>(
        QFormatSpec{20, 12, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ21_11<false>, FixedPoint<21,11,int32_t>>(
        QFormatSpec{21, 11, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ22_10<false>, FixedPoint<22,10,int32_t>>(
        QFormatSpec{22, 10, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ23_9<false>, FixedPoint<23,9,int32_t>>(
        QFormatSpec{23, 9, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ24_8<false>, FixedPoint<24,8,int32_t>>(
        QFormatSpec{24, 8, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ25_7<false>, FixedPoint<25,7,int32_t>>(
        QFormatSpec{25, 7, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ26_6<false>, FixedPoint<26,6,int32_t>>(
        QFormatSpec{26, 6, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ27_5<false>, FixedPoint<27,5,int32_t>>(
        QFormatSpec{27, 5, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ28_4<false>, FixedPoint<28,4,int32_t>>(
        QFormatSpec{28, 4, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ29_3<false>, FixedPoint<29,3,int32_t>>(
        QFormatSpec{29, 3, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ30_2<false>, FixedPoint<30,2,int32_t>>(
        QFormatSpec{30, 2, 32, false, "int32_t"}, dataset_path));
    all_results.push_back(test_format_helper<FilterQ31_1<false>, FixedPoint<31,1,int32_t>>(
        QFormatSpec{31, 1, 32, false, "int32_t"}, dataset_path));
    
    return all_results;
}

//------------------------------------------------------------------------------
// CSV export function for heatmap generation
//------------------------------------------------------------------------------
void export_results_to_csv(const std::vector<FormatResult>& results, const char* csv_filename) {
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        ENTO_INFO("ERROR: Failed to create CSV file: %s", csv_filename);
        return;
    }
    
    // CSV header
    csv_file << "format_name,integer_bits,fractional_bits,total_bits,is_float,underlying_type,"
             << "total_samples,failures,failure_rate,overflow,bad_norm,near_zero,excess_err,"
             << "runtime_ms,test_succeeded\n";
    
    // Data rows
    for (const auto& result : results) {
        csv_file << result.format_spec.name() << ","
                 << result.format_spec.integer_bits << ","
                 << result.format_spec.fractional_bits << ","
                 << result.format_spec.total_bits << ","
                 << (result.format_spec.is_float ? "true" : "false") << ","
                 << result.format_spec.underlying_type << ","
                 << result.stats.total_samples << ","
                 << result.stats.any_failure_count << ","
                 << std::fixed << std::setprecision(3) << result.failure_rate() << ","
                 << result.stats.overflow_count << ","
                 << result.stats.bad_norm_count << ","
                 << result.stats.near_zero_div_count << ","
                 << result.stats.excessive_err_count << ","
                 << std::fixed << std::setprecision(1) << result.runtime_ms << ","
                 << (result.test_succeeded ? "true" : "false") << "\n";
    }
    
    csv_file.close();
    ENTO_INFO("Results exported to: %s", csv_filename);
}

//------------------------------------------------------------------------------
// Main execution
//------------------------------------------------------------------------------
int main() {
    ENTO_INFO("Starting comprehensive Q-format sweep (48 formats)...");
    
    // Hard-coded dataset path (IMU-only)
    const char* dataset_path = "../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt";
    ENTO_INFO("Dataset: %s", dataset_path);
    
    // Run the comprehensive sweep
    auto all_results = run_comprehensive_sweep(dataset_path);
    
    // Print summary table
    std::cout << "\n" << std::string(90, '=') << "\n";
    std::cout << "COMPREHENSIVE Q-FORMAT COMPARISON (48 FORMATS)\n";
    std::cout << std::string(90, '=') << "\n";
    std::cout << "Format       | Total  | Failures | Rate    | Overflow | BadNorm | NearZero | ExcessErr | Runtime\n";
    std::cout << std::string(90, '-') << "\n";
    
    size_t total_successful = 0;
    double total_runtime = 0.0;
    
    for (const auto& result : all_results) {
        result.print_summary();
        if (result.test_succeeded) total_successful++;
        total_runtime += result.runtime_ms;
    }
    
    std::cout << std::string(90, '=') << "\n";
    std::cout << "Summary: " << total_successful << "/" << all_results.size() 
              << " formats tested successfully in " << std::fixed << std::setprecision(1) 
              << total_runtime << "ms total\n";
    
    // Export to CSV for heatmap generation
    export_results_to_csv(all_results, "qformat_sweep_results.csv");
    
    // Print next steps
    std::cout << "\n" << std::string(80, '=') << "\n";
    std::cout << "NEXT STEPS FOR HEATMAP GENERATION:\n";
    std::cout << std::string(80, '=') << "\n";
    std::cout << "1. CSV data exported to: qformat_sweep_results.csv\n";
    std::cout << "2. Run Python script: tools/generate_qformat_heatmap.py\n";
    std::cout << "3. Heatmap will be saved as: qformat_failure_heatmap.png\n";
    std::cout << "4. View heatmap for publication-ready visualization!\n";
    
    return 0;
} 