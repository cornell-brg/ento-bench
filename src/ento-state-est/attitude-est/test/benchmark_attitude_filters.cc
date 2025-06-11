#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-bench/harness.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <ento-state-est/attitude-est/mahoney_fixed.h>
#include <ento-state-est/attitude-est/madgwick_fixed.h>
#include <ento-state-est/attitude-est/mahoney.h>
#include <ento-state-est/attitude-est/madgwick.h>
#include <ento-state-est/attitude-est/fourati_nonlinear.h>
#include <math/FixedPointMath.hh>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>

const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 256;

char dir_path[FILEPATH_SIZE];
char marg_input_path[FILEPATH_SIZE];
char imu_input_path[FILEPATH_SIZE];
char marg_output_path[FILEPATH_SIZE];
char imu_output_path[FILEPATH_SIZE];

using namespace EntoAttitude;

// =============================================================================
// Specialized Benchmark Problem Class with Appropriate Precision Ground Truth
// =============================================================================
template<typename EstimatedScalar, typename GroundTruthScalar>
double compute_quat_angle_distance(const Eigen::Quaternion<EstimatedScalar>& q_estimated, const Eigen::Quaternion<GroundTruthScalar>& q_gt) {
    // Use the highest precision available for computation
    using ComputeType = typename std::conditional_t<std::is_same_v<GroundTruthScalar, double> || std::is_same_v<EstimatedScalar, double>, double, float>;
    
    // Convert both quaternions to the computation type
    ComputeType est_w = static_cast<ComputeType>(q_estimated.w());
    ComputeType est_x = static_cast<ComputeType>(q_estimated.x());
    ComputeType est_y = static_cast<ComputeType>(q_estimated.y());
    ComputeType est_z = static_cast<ComputeType>(q_estimated.z());
    
    ComputeType gt_w = static_cast<ComputeType>(q_gt.w());
    ComputeType gt_x = static_cast<ComputeType>(q_gt.x());
    ComputeType gt_y = static_cast<ComputeType>(q_gt.y());
    ComputeType gt_z = static_cast<ComputeType>(q_gt.z());
    
    ComputeType dot_product = std::abs(est_w * gt_w + est_x * gt_x + est_y * gt_y + est_z * gt_z);
    dot_product = std::min(static_cast<ComputeType>(1.0), std::max(static_cast<ComputeType>(-1.0), dot_product));
    double angle_distance = 2.0 * std::acos(static_cast<double>(dot_product));
    return angle_distance * 180.0 / M_PI;  // Convert to degrees
}

// =============================================================================
// Direct Filter Type Definitions (using actual filter types from normal files)
// =============================================================================

// Floating-Point Baseline Filters (from mahoney.h and madgwick.h)
using MahonyFloat_IMU = FilterMahoney<float, false>;
using MahonyFloat_MARG = FilterMahoney<float, true>;
using MadgwickFloat_IMU = FilterMadgwick<float, false>;
using MadgwickFloat_MARG = FilterMadgwick<float, true>;

// Double-Precision Filters
using MahonyDouble_IMU = FilterMahoney<double, false>;
using MahonyDouble_MARG = FilterMahoney<double, true>;
using MadgwickDouble_IMU = FilterMadgwick<double, false>;
using MadgwickDouble_MARG = FilterMadgwick<double, true>;

// Fourati Filter (MARG only, float and double)
using FouratiFloat_MARG = FilterFourati<float>;
using FouratiDouble_MARG = FilterFourati<double>;

// Fixed-Point Filters (Q7.24 format)
using MahonyQ7_24_IMU = FilterMahonyFixed<Q7_24, false>;
using MahonyQ7_24_MARG = FilterMahonyFixed<Q7_24, true>;
using MadgwickQ7_24_IMU = FilterMadgwickFixed<Q7_24, false>;
using MadgwickQ7_24_MARG = FilterMadgwickFixed<Q7_24, true>;

// Fixed-Point Filters (Q5.26 format - better precision for small values)
using MahonyQ5_26_IMU = FilterMahonyFixed<Q5_26, false>;
using MahonyQ5_26_MARG = FilterMahonyFixed<Q5_26, true>;
using MadgwickQ5_26_IMU = FilterMadgwickFixed<Q5_26, false>;
using MadgwickQ5_26_MARG = FilterMadgwickFixed<Q5_26, true>;

// Fixed-Point Filters (Q3.12 format - should fail with small values)
using MahonyQ3_12_IMU = FilterMahonyFixed<Q3_12, false>;
using MahonyQ3_12_MARG = FilterMahonyFixed<Q3_12, true>;
using MadgwickQ3_12_IMU = FilterMadgwickFixed<Q3_12, false>;
using MadgwickQ3_12_MARG = FilterMadgwickFixed<Q3_12, true>;

// =============================================================================
// Problem Type Definitions (using AttitudeProblem with float ground truth)
// =============================================================================

// Floating-Point Problems (baseline)
using MahonyFloat_IMUProblem = AttitudeProblem<float, MahonyFloat_IMU, false>;
using MahonyFloat_MARGProblem = AttitudeProblem<float, MahonyFloat_MARG, true>;
using MadgwickFloat_IMUProblem = AttitudeProblem<float, MadgwickFloat_IMU, false>;
using MadgwickFloat_MARGProblem = AttitudeProblem<float, MadgwickFloat_MARG, true>;

// Double-Precision Problems
using MahonyDouble_IMUProblem = AttitudeProblem<double, MahonyDouble_IMU, false>;
using MahonyDouble_MARGProblem = AttitudeProblem<double, MahonyDouble_MARG, true>;
using MadgwickDouble_IMUProblem = AttitudeProblem<double, MadgwickDouble_IMU, false>;
using MadgwickDouble_MARGProblem = AttitudeProblem<double, MadgwickDouble_MARG, true>;

// Fourati Problems (MARG only)
using FouratiFloat_MARGProblem = AttitudeProblem<float, FouratiFloat_MARG, true>;
using FouratiDouble_MARGProblem = AttitudeProblem<double, FouratiDouble_MARG, true>;

// Fixed-Point Problems (Q7.24)
using MahonyQ7_24_IMUProblem = AttitudeProblem<Q7_24, MahonyQ7_24_IMU, false>;
using MahonyQ7_24_MARGProblem = AttitudeProblem<Q7_24, MahonyQ7_24_MARG, true>;
using MadgwickQ7_24_IMUProblem = AttitudeProblem<Q7_24, MadgwickQ7_24_IMU, false>;
using MadgwickQ7_24_MARGProblem = AttitudeProblem<Q7_24, MadgwickQ7_24_MARG, true>;

// Fixed-Point Problems (Q5.26)
using MahonyQ5_26_IMUProblem = AttitudeProblem<Q5_26, MahonyQ5_26_IMU, false>;
using MahonyQ5_26_MARGProblem = AttitudeProblem<Q5_26, MahonyQ5_26_MARG, true>;
using MadgwickQ5_26_IMUProblem = AttitudeProblem<Q5_26, MadgwickQ5_26_IMU, false>;
using MadgwickQ5_26_MARGProblem = AttitudeProblem<Q5_26, MadgwickQ5_26_MARG, true>;

// Fixed-Point Problems (Q3.12 - should now work better with float comparison)
using MahonyQ3_12_IMUProblem = AttitudeProblem<Q3_12, MahonyQ3_12_IMU, false>;
using MahonyQ3_12_MARGProblem = AttitudeProblem<Q3_12, MahonyQ3_12_MARG, true>;
using MadgwickQ3_12_IMUProblem = AttitudeProblem<Q3_12, MadgwickQ3_12_IMU, false>;
using MadgwickQ3_12_MARGProblem = AttitudeProblem<Q3_12, MadgwickQ3_12_MARG, true>;

// =============================================================================
// Error Tracking and Results Structure
// =============================================================================
struct BenchmarkResult {
    const char* filter_name;
    const char* precision_type;
    const char* sensor_type;
    size_t total_samples;
    size_t valid_samples;
    size_t failed_samples;
    double success_rate;
    double mean_error_deg;
    double max_error_deg;
    double std_error_deg;
    bool completed_successfully;
};

std::vector<BenchmarkResult> benchmark_results;

// Helper function for computing quaternion angle distance with proper type handling

// =============================================================================
// Custom Error-Tracking Harness
// =============================================================================
template<typename Problem, typename Kernel, typename... GainArgs>
BenchmarkResult run_error_benchmark(const char* filter_name, 
                                   const char* precision_type,
                                   const char* sensor_type,
                                   const char* input_path,
                                   Kernel kernel,
                                   GainArgs... gains) {
    
    BenchmarkResult result = {};
    result.filter_name = filter_name;
    result.precision_type = precision_type;
    result.sensor_type = sensor_type;
    result.completed_successfully = false;
    
    std::vector<double> errors;
    size_t valid_count = 0;
    size_t failed_count = 0;
    
    try {
        ENTO_DEBUG("Running error benchmark: %s (%s, %s)", filter_name, precision_type, sensor_type);
        
        // Create problem instance with gains passed to constructor (THE RULE)
        Problem problem(kernel, gains...);
        
        // Create simple harness for file I/O (no performance measurement)
        EntoBench::Harness<Problem, false, 1, 0> harness(problem, filter_name, input_path, "/dev/null");
        
        // We'll manually track errors during execution
        // Reset problem state
        problem.clear_impl();
        
        // Open input file manually for error tracking
        std::ifstream input_file(input_path);
        if (!input_file.is_open()) {
            ENTO_DEBUG("Failed to open input file: %s", input_path);
            return result;
        }
        
        std::string line;
        size_t sample_count = 0;
        
        while (std::getline(input_file, line) && !line.empty()) {
            // Deserialize the line
            if (!problem.deserialize_impl(line.c_str())) {
                ENTO_DEBUG("Failed to deserialize line %zu", sample_count);
                failed_count++;
                continue;
            }
            
            // Solve the problem (run the filter)
            problem.solve_impl();
            
            // DEBUG: Print quaternion values for first few samples to diagnose fixed-point issue
            if (sample_count < 3 && (precision_type[0] == 'Q')) {
                ENTO_DEBUG("[%s %s] Sample %zu: GT=(%.6f,%.6f,%.6f,%.6f) EST=(%.6f,%.6f,%.6f,%.6f)", 
                          filter_name, precision_type, sample_count,
                          problem.q_gt_.w(), problem.q_gt_.x(), problem.q_gt_.y(), problem.q_gt_.z(),
                          static_cast<float>(problem.q_.w()), static_cast<float>(problem.q_.x()), 
                          static_cast<float>(problem.q_.y()), static_cast<float>(problem.q_.z()));
            }
            
            // Compute the actual error using float ground truth for precision consistency
            double error_deg = compute_quat_angle_distance(problem.q_, problem.q_gt_);
            errors.push_back(error_deg);
            
            // DEBUG: Print error for first few samples of fixed-point filters
            if (sample_count < 3 && (precision_type[0] == 'Q')) {
                ENTO_DEBUG("[%s %s] Sample %zu: Error = %.6f degrees", filter_name, precision_type, sample_count, error_deg);
            }
            
            // Check if it passes validation threshold
            if (problem.validate_impl()) {
                valid_count++;
            } else {
                failed_count++;
            }
            
            sample_count++;
        }
        
        input_file.close();
        
        // Compute statistics
        result.total_samples = sample_count;
        result.valid_samples = valid_count;
        result.failed_samples = failed_count;
        result.success_rate = (double)valid_count / sample_count * 100.0;
        
        if (!errors.empty()) {
            // Compute mean error
            double sum = 0.0;
            for (double e : errors) sum += e;
            result.mean_error_deg = sum / errors.size();
            
            // Find max error
            result.max_error_deg = *std::max_element(errors.begin(), errors.end());
            
            // Compute standard deviation
            double variance = 0.0;
            for (double e : errors) {
                variance += (e - result.mean_error_deg) * (e - result.mean_error_deg);
            }
            result.std_error_deg = std::sqrt(variance / errors.size());
        }
        
        result.completed_successfully = true;
        
        ENTO_DEBUG("Completed: %s - %zu samples, %.1f%% success rate, %.2f° mean error", 
                  filter_name, result.total_samples, result.success_rate, result.mean_error_deg);
        
    } catch (const std::exception& e) {
        ENTO_DEBUG("Benchmark failed for %s: %s", filter_name, e.what());
        result.completed_successfully = false;
    }
    
    return result;
}

// =============================================================================
// Individual Benchmark Functions (all filter types)
// =============================================================================

void benchmark_floating_point_filters() {
    ENTO_DEBUG("=== Floating-Point Filter Error Benchmarks (Baseline) ===");
    
    // Mahony Float IMU
    {
        MahonyFloat_IMU kernel;  // Default constructor
        auto result = run_error_benchmark<MahonyFloat_IMUProblem>("Mahony", "Float", "IMU", 
                                                                 imu_input_path, kernel, 1.0f, 0.1f);  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Mahony Float MARG
    {
        MahonyFloat_MARG kernel;  // Default constructor
        auto result = run_error_benchmark<MahonyFloat_MARGProblem>("Mahony", "Float", "MARG", 
                                                                  marg_input_path, kernel, 1.0f, 0.1f);  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Madgwick Float IMU
    {
        MadgwickFloat_IMU kernel;  // Default constructor
        auto result = run_error_benchmark<MadgwickFloat_IMUProblem>("Madgwick", "Float", "IMU", 
                                                                   imu_input_path, kernel, 0.1f);  // gain
        benchmark_results.push_back(result);
    }
    
    // Madgwick Float MARG
    {
        MadgwickFloat_MARG kernel;  // Default constructor
        auto result = run_error_benchmark<MadgwickFloat_MARGProblem>("Madgwick", "Float", "MARG", 
                                                                    marg_input_path, kernel, 0.1f);  // gain
        benchmark_results.push_back(result);
    }
}

void benchmark_fixed_point_q7_24_filters() {
    ENTO_DEBUG("=== Q7.24 Fixed-Point Filter Error Benchmarks ===");
    
    // Mahony Q7.24 IMU
    {
        MahonyQ7_24_IMU kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MahonyQ7_24_IMUProblem>("Mahony", "Q7.24", "IMU", 
                                                                 imu_input_path, kernel, Q7_24(1.0f), Q7_24(0.1f));  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Mahony Q7.24 MARG
    {
        MahonyQ7_24_MARG kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MahonyQ7_24_MARGProblem>("Mahony", "Q7.24", "MARG", 
                                                                  marg_input_path, kernel, Q7_24(1.0f), Q7_24(0.1f));  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q7.24 IMU
    {
        MadgwickQ7_24_IMU kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MadgwickQ7_24_IMUProblem>("Madgwick", "Q7.24", "IMU", 
                                                                   imu_input_path, kernel, Q7_24(0.1f));  // gain
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q7.24 MARG
    {
        MadgwickQ7_24_MARG kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MadgwickQ7_24_MARGProblem>("Madgwick", "Q7.24", "MARG", 
                                                                    marg_input_path, kernel, Q7_24(0.1f));  // gain
        benchmark_results.push_back(result);
    }
}

void benchmark_fixed_point_q5_26_filters() {
    ENTO_DEBUG("=== Q5.26 Fixed-Point Filter Error Benchmarks ===");
    
    // Mahony Q5.26 IMU
    {
        MahonyQ5_26_IMU kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MahonyQ5_26_IMUProblem>("Mahony", "Q5.26", "IMU", 
                                                                 imu_input_path, kernel, Q5_26(1.0f), Q5_26(0.1f));  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Mahony Q5.26 MARG
    {
        MahonyQ5_26_MARG kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MahonyQ5_26_MARGProblem>("Mahony", "Q5.26", "MARG", 
                                                                  marg_input_path, kernel, Q5_26(1.0f), Q5_26(0.1f));  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q5.26 IMU
    {
        MadgwickQ5_26_IMU kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MadgwickQ5_26_IMUProblem>("Madgwick", "Q5.26", "IMU", 
                                                                   imu_input_path, kernel, Q5_26(0.1f));  // gain
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q5.26 MARG
    {
        MadgwickQ5_26_MARG kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MadgwickQ5_26_MARGProblem>("Madgwick", "Q5.26", "MARG", 
                                                                    marg_input_path, kernel, Q5_26(0.1f));  // gain
        benchmark_results.push_back(result);
    }
}

void benchmark_fixed_point_q3_12_filters() {
    ENTO_DEBUG("=== Q3.12 Fixed-Point Filter Error Benchmarks (With Float Ground Truth) ===");
    
    // Mahony Q3.12 IMU
    {
        MahonyQ3_12_IMU kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MahonyQ3_12_IMUProblem>("Mahony", "Q3.12", "IMU", 
                                                                 imu_input_path, kernel, Q3_12(1.0f), Q3_12(0.1f));  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Mahony Q3.12 MARG
    {
        MahonyQ3_12_MARG kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MahonyQ3_12_MARGProblem>("Mahony", "Q3.12", "MARG", 
                                                                  marg_input_path, kernel, Q3_12(1.0f), Q3_12(0.1f));  // kp, ki
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q3.12 IMU
    {
        MadgwickQ3_12_IMU kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MadgwickQ3_12_IMUProblem>("Madgwick", "Q3.12", "IMU", 
                                                                   imu_input_path, kernel, Q3_12(0.1f));  // gain
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q3.12 MARG
    {
        MadgwickQ3_12_MARG kernel;  // Default constructor (THE RULE)
        auto result = run_error_benchmark<MadgwickQ3_12_MARGProblem>("Madgwick", "Q3.12", "MARG", 
                                                                    marg_input_path, kernel, Q3_12(0.1f));  // gain
        benchmark_results.push_back(result);
    }
}

// =============================================================================
// Results Reporting
// =============================================================================

void print_benchmark_summary() {
    ENTO_DEBUG("\n=== ATTITUDE FILTER ERROR BENCHMARK SUMMARY ===");
    ENTO_DEBUG("%-12s %-8s %-6s %-7s %-7s %-8s %-8s %-8s %s", 
              "Filter", "Precision", "Sensor", "Samples", "Success", "Mean(°)", "Max(°)", "Std(°)", "Status");
    ENTO_DEBUG("%-12s %-8s %-6s %-7s %-7s %-8s %-8s %-8s %s", 
              "--------", "--------", "------", "-------", "-------", "--------", "-------", "-------", "------");
    
    for (const auto& result : benchmark_results) {
        const char* status = result.completed_successfully ? "OK" : "FAIL";
        
        if (result.completed_successfully) {
            ENTO_DEBUG("%-12s %-8s %-6s %7zu %6.1f%% %8.6f %8.6f %8.6f %s", 
                      result.filter_name, result.precision_type, result.sensor_type,
                      result.total_samples, result.success_rate,
                      result.mean_error_deg, result.max_error_deg, result.std_error_deg,
                      status);
        } else {
            ENTO_DEBUG("%-12s %-8s %-6s %7s %7s %8s %8s %8s %s", 
                      result.filter_name, result.precision_type, result.sensor_type,
                      "N/A", "N/A", "N/A", "N/A", "N/A", status);
        }
    }
    
    // Summary statistics
    size_t successful_tests = 0;
    size_t total_tests = benchmark_results.size();
    
    for (const auto& result : benchmark_results) {
        if (result.completed_successfully) {
            successful_tests++;
        }
    }
    
    ENTO_DEBUG("\n=== SUMMARY ===");
    ENTO_DEBUG("Total tests: %zu", total_tests);
    ENTO_DEBUG("Successful: %zu", successful_tests);
    ENTO_DEBUG("Failed: %zu", total_tests - successful_tests);
    ENTO_DEBUG("Success rate: %.1f%%", (100.0 * successful_tests) / total_tests);
}

// =============================================================================
// Debug Analysis Functions
// =============================================================================

void analyze_fixed_point_precision() {
    ENTO_DEBUG("=== Fixed-Point Precision Analysis ===");
    
    // Test small values that are typical in our dataset
    std::vector<float> test_values = {
        0.000001f, 0.00001f, 0.0001f, 0.001f, 0.01f,
        -0.002930f, -0.001221f, -0.000488f  // Actual values from debug output
    };
    
    std::ofstream analysis_file("fixed_point_analysis.txt");
    analysis_file << "Value,Q3_12,Q7_24,Q5_26,Q3_12_sq,Q7_24_sq,Q5_26_sq\n";
    
    for (float val : test_values) {
        Q3_12 q3_val(val);
        Q7_24 q7_val(val);
        Q5_26 q5_val(val);
        
        float q3_sq = static_cast<float>(q3_val * q3_val);
        float q7_sq = static_cast<float>(q7_val * q7_val);
        float q5_sq = static_cast<float>(q5_val * q5_val);
        
        analysis_file << val << "," 
                     << static_cast<float>(q3_val) << ","
                     << static_cast<float>(q7_val) << ","
                     << static_cast<float>(q5_val) << ","
                     << q3_sq << ","
                     << q7_sq << ","
                     << q5_sq << "\n";
    }
    analysis_file.close();
    
    ENTO_DEBUG("Fixed-point precision analysis saved to fixed_point_analysis.txt");
}

// =============================================================================
// Main Benchmark Function
// =============================================================================

void test_attitude_filter_benchmark() {
    // Setup file paths using simple string operations
    const char* base_dir = "../../datasets/state-est";
    
    snprintf(marg_input_path, FILEPATH_SIZE, "%s/tuned_icm42688_1khz_marg_dataset.txt", base_dir);
    snprintf(imu_input_path, FILEPATH_SIZE, "%s/tuned_icm42688_1khz_imu_dataset.txt", base_dir);
    snprintf(marg_output_path, FILEPATH_SIZE, "./benchmark_marg_results.csv");
    snprintf(imu_output_path, FILEPATH_SIZE, "./benchmark_imu_results.csv");
    
    ENTO_DEBUG("Benchmark dataset paths:");
    ENTO_DEBUG("  MARG input: %s", marg_input_path);
    ENTO_DEBUG("  IMU input: %s", imu_input_path);
    
    // Clear previous results
    benchmark_results.clear();
    
    // COMPREHENSIVE ATTITUDE FILTER BENCHMARK: All Precision Types + Fourati
    ENTO_DEBUG("=== COMPREHENSIVE ATTITUDE FILTER BENCHMARK ===");
    
    // ========== FLOAT PRECISION TESTS (baseline) ==========
    ENTO_DEBUG("=== FLOAT PRECISION FILTERS ===");
    
    // Mahony Float (IMU)
    {
        MahonyFloat_IMU kernel;
        auto result = run_error_benchmark<MahonyFloat_IMUProblem>("Mahony", "Float", "IMU", 
                                                                 imu_input_path, kernel, 0.01f, 0.001f);
        benchmark_results.push_back(result);
    }
    
    // Mahony Float (MARG) - investigate performance issue
    {
        MahonyFloat_MARG kernel;
        auto result = run_error_benchmark<MahonyFloat_MARGProblem>("Mahony", "Float", "MARG", 
                                                                  marg_input_path, kernel, 0.01f, 0.001f);
        benchmark_results.push_back(result);
    }
    
    // Madgwick Float (IMU)
    {
        MadgwickFloat_IMU kernel;
        auto result = run_error_benchmark<MadgwickFloat_IMUProblem>("Madgwick", "Float", "IMU", 
                                                                   imu_input_path, kernel, 0.001f);
        benchmark_results.push_back(result);
    }
    
    // Madgwick Float (MARG)
    {
        MadgwickFloat_MARG kernel;
        auto result = run_error_benchmark<MadgwickFloat_MARGProblem>("Madgwick", "Float", "MARG", 
                                                                    marg_input_path, kernel, 0.001f);
        benchmark_results.push_back(result);
    }
    
    // Fourati Float (MARG only)
    {
        FouratiFloat_MARG kernel;
        auto result = run_error_benchmark<FouratiFloat_MARGProblem>("Fourati", "Float", "MARG", 
                                                                   marg_input_path, kernel, 0.1f);  // Fourati gain
        benchmark_results.push_back(result);
    }
    
    // ========== DOUBLE PRECISION TESTS ==========
    ENTO_DEBUG("=== DOUBLE PRECISION FILTERS ===");
    
    // Mahony Double (IMU)
    {
        MahonyDouble_IMU kernel;
        auto result = run_error_benchmark<MahonyDouble_IMUProblem>("Mahony", "Double", "IMU", 
                                                                  imu_input_path, kernel, 0.01, 0.001);
        benchmark_results.push_back(result);
    }
    
    // Mahony Double (MARG)
    {
        MahonyDouble_MARG kernel;
        auto result = run_error_benchmark<MahonyDouble_MARGProblem>("Mahony", "Double", "MARG", 
                                                                   marg_input_path, kernel, 0.01, 0.001);
        benchmark_results.push_back(result);
    }
    
    // Madgwick Double (IMU)
    {
        MadgwickDouble_IMU kernel;
        auto result = run_error_benchmark<MadgwickDouble_IMUProblem>("Madgwick", "Double", "IMU", 
                                                                    imu_input_path, kernel, 0.001);
        benchmark_results.push_back(result);
    }
    
    // Madgwick Double (MARG)
    {
        MadgwickDouble_MARG kernel;
        auto result = run_error_benchmark<MadgwickDouble_MARGProblem>("Madgwick", "Double", "MARG", 
                                                                     marg_input_path, kernel, 0.001);
        benchmark_results.push_back(result);
    }
    
    // Fourati Double (MARG only)
    {
        FouratiDouble_MARG kernel;
        auto result = run_error_benchmark<FouratiDouble_MARGProblem>("Fourati", "Double", "MARG", 
                                                                    marg_input_path, kernel, 0.1);  // Fourati gain
        benchmark_results.push_back(result);
    }
    
    // ========== Q7.24 FIXED-POINT TESTS ==========
    ENTO_DEBUG("=== Q7.24 FIXED-POINT FILTERS ===");
    
    // Mahony Q7.24 (IMU)
    {
        MahonyQ7_24_IMU kernel;
        auto result = run_error_benchmark<MahonyQ7_24_IMUProblem>("Mahony", "Q7.24", "IMU", 
                                                                 imu_input_path, kernel, Q7_24(0.01f), Q7_24(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q7.24 (MARG)
    {
        MahonyQ7_24_MARG kernel;
        auto result = run_error_benchmark<MahonyQ7_24_MARGProblem>("Mahony", "Q7.24", "MARG", 
                                                                  marg_input_path, kernel, Q7_24(0.01f), Q7_24(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q7.24 (IMU)
    {
        MadgwickQ7_24_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ7_24_IMUProblem>("Madgwick", "Q7.24", "IMU", 
                                                                   imu_input_path, kernel, Q7_24(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q7.24 (MARG)
    {
        MadgwickQ7_24_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ7_24_MARGProblem>("Madgwick", "Q7.24", "MARG", 
                                                                    marg_input_path, kernel, Q7_24(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q5.26 FIXED-POINT TESTS ==========
    ENTO_DEBUG("=== Q5.26 FIXED-POINT FILTERS ===");
    
    // Mahony Q5.26 (IMU)
    {
        MahonyQ5_26_IMU kernel;
        auto result = run_error_benchmark<MahonyQ5_26_IMUProblem>("Mahony", "Q5.26", "IMU", 
                                                                 imu_input_path, kernel, Q5_26(0.01f), Q5_26(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q5.26 (MARG)
    {
        MahonyQ5_26_MARG kernel;
        auto result = run_error_benchmark<MahonyQ5_26_MARGProblem>("Mahony", "Q5.26", "MARG", 
                                                                  marg_input_path, kernel, Q5_26(0.01f), Q5_26(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q5.26 (IMU)
    {
        MadgwickQ5_26_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ5_26_IMUProblem>("Madgwick", "Q5.26", "IMU", 
                                                                   imu_input_path, kernel, Q5_26(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q5.26 (MARG)
    {
        MadgwickQ5_26_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ5_26_MARGProblem>("Madgwick", "Q5.26", "MARG", 
                                                                    marg_input_path, kernel, Q5_26(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== PERFORMANCE ANALYSIS SECTION ==========
    ENTO_DEBUG("=== MAHONY MARG PERFORMANCE ANALYSIS ===");
    
    // Test Mahony MARG with alternative gains to understand the performance difference
    {
        MahonyFloat_MARG kernel;
        auto result = run_error_benchmark<MahonyFloat_MARGProblem>("Mahony", "Float-Alt", "MARG", 
                                                                  marg_input_path, kernel, 1.0f, 0.1f);  // Default gains
        benchmark_results.push_back(result);
    }
    
    {
        MahonyFloat_MARG kernel;
        auto result = run_error_benchmark<MahonyFloat_MARGProblem>("Mahony", "Float-High", "MARG", 
                                                                  marg_input_path, kernel, 0.1f, 0.01f);  // Higher gains
        benchmark_results.push_back(result);
    }
    
    // Print summary
    print_benchmark_summary();
}

// =============================================================================
// Test Entry Point
// =============================================================================

int main() {
    ENTO_DEBUG("Starting Attitude Filter Error Benchmark Suite...");
    
    try {
        // Run precision analysis first
        analyze_fixed_point_precision();
        
        // Run the main benchmark test
        test_attitude_filter_benchmark();
        
        ENTO_DEBUG("Benchmark suite completed successfully!");
        return 0;
    } catch (const std::exception& e) {
        ENTO_DEBUG("Benchmark suite failed with exception: %s", e.what());
        return 1;
    }
} 