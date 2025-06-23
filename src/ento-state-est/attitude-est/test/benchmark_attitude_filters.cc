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
#include <ento-state-est/attitude-est/fourati_fixed.h>
#include <math/FixedPointMath.hh>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <iostream>
#include <cstring>

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

// Fixed-Point Fourati Filters (Q7.24 format)
using FouratiQ7_24_MARG = FilterFouratiFixed<Q7_24>;

// Fixed-Point Fourati Filters (Q5.26 format - better precision for small values)
using FouratiQ5_26_MARG = FilterFouratiFixed<Q5_26>;

// Fixed-Point Fourati Filters (Q3.12 format - should fail with small values)
using FouratiQ3_12_MARG = FilterFouratiFixed<Q3_12>;

// Additional 32-bit Q formats for comprehensive coverage
// Q6.25 - Good balance, more integer bits than Q5.26
using Q6_25 = FixedPoint<6, 25, int32_t>;
using MahonyQ6_25_IMU = FilterMahonyFixed<Q6_25, false>;
using MahonyQ6_25_MARG = FilterMahonyFixed<Q6_25, true>;
using MadgwickQ6_25_IMU = FilterMadgwickFixed<Q6_25, false>;
using MadgwickQ6_25_MARG = FilterMadgwickFixed<Q6_25, true>;

// Q8.23 - Higher integer range for aggressive gyro rates (32-bit: 8+23=31, +1 sign bit = 32 total)
using Q8_23 = FixedPoint<8, 23, int32_t>;
using MahonyQ8_23_IMU = FilterMahonyFixed<Q8_23, false>;
using MahonyQ8_23_MARG = FilterMahonyFixed<Q8_23, true>;
using MadgwickQ8_23_IMU = FilterMadgwickFixed<Q8_23, false>;
using MadgwickQ8_23_MARG = FilterMadgwickFixed<Q8_23, true>;

// Q4.27 - Very high fractional precision, moderate integer range (32-bit: 4+27=31, +1 sign bit = 32 total)
using Q4_27 = FixedPoint<4, 27, int32_t>;
using MahonyQ4_27_IMU = FilterMahonyFixed<Q4_27, false>;
using MahonyQ4_27_MARG = FilterMahonyFixed<Q4_27, true>;
using MadgwickQ4_27_IMU = FilterMadgwickFixed<Q4_27, false>;
using MadgwickQ4_27_MARG = FilterMadgwickFixed<Q4_27, true>;

// Q2.29 - Extreme fractional precision, narrow integer range (32-bit: 2+29=31, +1 sign bit = 32 total)
using Q2_29 = FixedPoint<2, 29, int32_t>;
using MahonyQ2_29_IMU = FilterMahonyFixed<Q2_29, false>;
using MahonyQ2_29_MARG = FilterMahonyFixed<Q2_29, true>;
using MadgwickQ2_29_IMU = FilterMadgwickFixed<Q2_29, false>;
using MadgwickQ2_29_MARG = FilterMadgwickFixed<Q2_29, true>;

// Additional 16-bit Q formats for comparison
// Q2.13 - Higher precision than Q3.12
using Q2_13 = FixedPoint<2, 13, int16_t>;
using MahonyQ2_13_IMU = FilterMahonyFixed<Q2_13, false>;
using MahonyQ2_13_MARG = FilterMahonyFixed<Q2_13, true>;
using MadgwickQ2_13_IMU = FilterMadgwickFixed<Q2_13, false>;
using MadgwickQ2_13_MARG = FilterMadgwickFixed<Q2_13, true>;

// Q4.11 - More integer range, less fractional precision than Q3.12
using Q4_11 = FixedPoint<4, 11, int16_t>;
using MahonyQ4_11_IMU = FilterMahonyFixed<Q4_11, false>;
using MahonyQ4_11_MARG = FilterMahonyFixed<Q4_11, true>;
using MadgwickQ4_11_IMU = FilterMadgwickFixed<Q4_11, false>;
using MadgwickQ4_11_MARG = FilterMadgwickFixed<Q4_11, true>;

// Additional Fourati filters for new Q formats
using FouratiQ6_25_MARG = FilterFouratiFixed<Q6_25>;
using FouratiQ8_23_MARG = FilterFouratiFixed<Q8_23>;
using FouratiQ4_27_MARG = FilterFouratiFixed<Q4_27>;
using FouratiQ2_29_MARG = FilterFouratiFixed<Q2_29>;
using FouratiQ2_13_MARG = FilterFouratiFixed<Q2_13>;
using FouratiQ4_11_MARG = FilterFouratiFixed<Q4_11>;

// Additional 16-bit Q formats for stress testing precision limits
// Q13.2 - Maximum integer range, minimal fractional precision (16-bit: 13+2=15, +1 sign bit = 16 total)
using Q13_2 = FixedPoint<13, 2, int16_t>;
using MahonyQ13_2_IMU = FilterMahonyFixed<Q13_2, false>;
using MahonyQ13_2_MARG = FilterMahonyFixed<Q13_2, true>;
using MadgwickQ13_2_IMU = FilterMadgwickFixed<Q13_2, false>;
using MadgwickQ13_2_MARG = FilterMadgwickFixed<Q13_2, true>;

// Q11.5 - High integer range, very low fractional precision  
using Q11_5 = FixedPoint<11, 5, int16_t>;
using MahonyQ11_5_IMU = FilterMahonyFixed<Q11_5, false>;
using MahonyQ11_5_MARG = FilterMahonyFixed<Q11_5, true>;
using MadgwickQ11_5_IMU = FilterMadgwickFixed<Q11_5, false>;
using MadgwickQ11_5_MARG = FilterMadgwickFixed<Q11_5, true>;

// Q8.7 - Equal split 16-bit format (16-bit: 8+7=15, +1 sign bit = 16 total)
using Q8_7 = FixedPoint<8, 7, int16_t>;
using MahonyQ8_7_IMU = FilterMahonyFixed<Q8_7, false>;
using MahonyQ8_7_MARG = FilterMahonyFixed<Q8_7, true>;
using MadgwickQ8_7_IMU = FilterMadgwickFixed<Q8_7, false>;
using MadgwickQ8_7_MARG = FilterMadgwickFixed<Q8_7, true>;
using FouratiQ8_7_MARG = FilterFouratiFixed<Q8_7>;

// Q6.10 - Moderate range, moderate fractional precision
using Q6_10 = FixedPoint<6, 10, int16_t>;
using MahonyQ6_10_IMU = FilterMahonyFixed<Q6_10, false>;
using MahonyQ6_10_MARG = FilterMahonyFixed<Q6_10, true>;
using MadgwickQ6_10_IMU = FilterMadgwickFixed<Q6_10, false>;
using MadgwickQ6_10_MARG = FilterMadgwickFixed<Q6_10, true>;

// Q1.14 - Extreme fractional precision, very narrow range (16-bit: 1+14=15, +1 sign bit = 16 total)
using Q1_14 = FixedPoint<1, 14, int16_t>;
using MahonyQ1_14_IMU = FilterMahonyFixed<Q1_14, false>;
using MahonyQ1_14_MARG = FilterMahonyFixed<Q1_14, true>;
using MadgwickQ1_14_IMU = FilterMadgwickFixed<Q1_14, false>;
using MadgwickQ1_14_MARG = FilterMadgwickFixed<Q1_14, true>;

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
using FouratiQ7_24_MARGProblem = AttitudeProblem<Q7_24, FouratiQ7_24_MARG, true>;

// Fixed-Point Problems (Q5.26)
using MahonyQ5_26_IMUProblem = AttitudeProblem<Q5_26, MahonyQ5_26_IMU, false>;
using MahonyQ5_26_MARGProblem = AttitudeProblem<Q5_26, MahonyQ5_26_MARG, true>;
using MadgwickQ5_26_IMUProblem = AttitudeProblem<Q5_26, MadgwickQ5_26_IMU, false>;
using MadgwickQ5_26_MARGProblem = AttitudeProblem<Q5_26, MadgwickQ5_26_MARG, true>;
using FouratiQ5_26_MARGProblem = AttitudeProblem<Q5_26, FouratiQ5_26_MARG, true>;

// Fixed-Point Problems (Q3.12)
using MahonyQ3_12_IMUProblem = AttitudeProblem<Q3_12, MahonyQ3_12_IMU, false>;
using MahonyQ3_12_MARGProblem = AttitudeProblem<Q3_12, MahonyQ3_12_MARG, true>;
using MadgwickQ3_12_IMUProblem = AttitudeProblem<Q3_12, MadgwickQ3_12_IMU, false>;
using MadgwickQ3_12_MARGProblem = AttitudeProblem<Q3_12, MadgwickQ3_12_MARG, true>;
using FouratiQ3_12_MARGProblem = AttitudeProblem<Q3_12, FouratiQ3_12_MARG, true>;

// Fixed-Point Problems (additional 32-bit Q formats)
using MahonyQ6_25_IMUProblem = AttitudeProblem<Q6_25, MahonyQ6_25_IMU, false>;
using MahonyQ6_25_MARGProblem = AttitudeProblem<Q6_25, MahonyQ6_25_MARG, true>;
using MadgwickQ6_25_IMUProblem = AttitudeProblem<Q6_25, MadgwickQ6_25_IMU, false>;
using MadgwickQ6_25_MARGProblem = AttitudeProblem<Q6_25, MadgwickQ6_25_MARG, true>;
using FouratiQ6_25_MARGProblem = AttitudeProblem<Q6_25, FouratiQ6_25_MARG, true>;
using FouratiQ8_23_MARGProblem = AttitudeProblem<Q8_23, FouratiQ8_23_MARG, true>;
using FouratiQ4_27_MARGProblem = AttitudeProblem<Q4_27, FouratiQ4_27_MARG, true>;
using FouratiQ2_29_MARGProblem = AttitudeProblem<Q2_29, FouratiQ2_29_MARG, true>;
using FouratiQ2_13_MARGProblem = AttitudeProblem<Q2_13, FouratiQ2_13_MARG, true>;
using FouratiQ4_11_MARGProblem = AttitudeProblem<Q4_11, FouratiQ4_11_MARG, true>;
using FouratiQ8_7_MARGProblem = AttitudeProblem<Q8_7, FouratiQ8_7_MARG, true>;

// Fixed-Point Problems (Q8.23)
using MahonyQ8_23_IMUProblem = AttitudeProblem<Q8_23, MahonyQ8_23_IMU, false>;
using MahonyQ8_23_MARGProblem = AttitudeProblem<Q8_23, MahonyQ8_23_MARG, true>;
using MadgwickQ8_23_IMUProblem = AttitudeProblem<Q8_23, MadgwickQ8_23_IMU, false>;
using MadgwickQ8_23_MARGProblem = AttitudeProblem<Q8_23, MadgwickQ8_23_MARG, true>;

// Fixed-Point Problems (Q4.27)
using MahonyQ4_27_IMUProblem = AttitudeProblem<Q4_27, MahonyQ4_27_IMU, false>;
using MahonyQ4_27_MARGProblem = AttitudeProblem<Q4_27, MahonyQ4_27_MARG, true>;
using MadgwickQ4_27_IMUProblem = AttitudeProblem<Q4_27, MadgwickQ4_27_IMU, false>;
using MadgwickQ4_27_MARGProblem = AttitudeProblem<Q4_27, MadgwickQ4_27_MARG, true>;

// Fixed-Point Problems (Q2.29)
using MahonyQ2_29_IMUProblem = AttitudeProblem<Q2_29, MahonyQ2_29_IMU, false>;
using MahonyQ2_29_MARGProblem = AttitudeProblem<Q2_29, MahonyQ2_29_MARG, true>;
using MadgwickQ2_29_IMUProblem = AttitudeProblem<Q2_29, MadgwickQ2_29_IMU, false>;
using MadgwickQ2_29_MARGProblem = AttitudeProblem<Q2_29, MadgwickQ2_29_MARG, true>;

// Fixed-Point Problems (Q2.13)
using MahonyQ2_13_IMUProblem = AttitudeProblem<Q2_13, MahonyQ2_13_IMU, false>;
using MahonyQ2_13_MARGProblem = AttitudeProblem<Q2_13, MahonyQ2_13_MARG, true>;
using MadgwickQ2_13_IMUProblem = AttitudeProblem<Q2_13, MadgwickQ2_13_IMU, false>;
using MadgwickQ2_13_MARGProblem = AttitudeProblem<Q2_13, MadgwickQ2_13_MARG, true>;

// Fixed-Point Problems (Q4.11)
using MahonyQ4_11_IMUProblem = AttitudeProblem<Q4_11, MahonyQ4_11_IMU, false>;
using MahonyQ4_11_MARGProblem = AttitudeProblem<Q4_11, MahonyQ4_11_MARG, true>;
using MadgwickQ4_11_IMUProblem = AttitudeProblem<Q4_11, MadgwickQ4_11_IMU, false>;
using MadgwickQ4_11_MARGProblem = AttitudeProblem<Q4_11, MadgwickQ4_11_MARG, true>;

// Fixed-Point Problems (Q13.2) - Stress test
using MahonyQ13_2_IMUProblem = AttitudeProblem<Q13_2, MahonyQ13_2_IMU, false>;
using MahonyQ13_2_MARGProblem = AttitudeProblem<Q13_2, MahonyQ13_2_MARG, true>;
using MadgwickQ13_2_IMUProblem = AttitudeProblem<Q13_2, MadgwickQ13_2_IMU, false>;
using MadgwickQ13_2_MARGProblem = AttitudeProblem<Q13_2, MadgwickQ13_2_MARG, true>;

// Fixed-Point Problems (Q11.5) - Stress test
using MahonyQ11_5_IMUProblem = AttitudeProblem<Q11_5, MahonyQ11_5_IMU, false>;
using MahonyQ11_5_MARGProblem = AttitudeProblem<Q11_5, MahonyQ11_5_MARG, true>;
using MadgwickQ11_5_IMUProblem = AttitudeProblem<Q11_5, MadgwickQ11_5_IMU, false>;
using MadgwickQ11_5_MARGProblem = AttitudeProblem<Q11_5, MadgwickQ11_5_MARG, true>;

// Fixed-Point Problems (Q8.7) - Balanced 16-bit
using MahonyQ8_7_IMUProblem = AttitudeProblem<Q8_7, MahonyQ8_7_IMU, false>;
using MahonyQ8_7_MARGProblem = AttitudeProblem<Q8_7, MahonyQ8_7_MARG, true>;
using MadgwickQ8_7_IMUProblem = AttitudeProblem<Q8_7, MadgwickQ8_7_IMU, false>;
using MadgwickQ8_7_MARGProblem = AttitudeProblem<Q8_7, MadgwickQ8_7_MARG, true>;

// Fixed-Point Problems (Q6.10) - Moderate 16-bit
using MahonyQ6_10_IMUProblem = AttitudeProblem<Q6_10, MahonyQ6_10_IMU, false>;
using MahonyQ6_10_MARGProblem = AttitudeProblem<Q6_10, MahonyQ6_10_MARG, true>;
using MadgwickQ6_10_IMUProblem = AttitudeProblem<Q6_10, MadgwickQ6_10_IMU, false>;
using MadgwickQ6_10_MARGProblem = AttitudeProblem<Q6_10, MadgwickQ6_10_MARG, true>;

// Fixed-Point Problems (Q1.14) - Extreme precision, narrow range
using MahonyQ1_14_IMUProblem = AttitudeProblem<Q1_14, MahonyQ1_14_IMU, false>;
using MahonyQ1_14_MARGProblem = AttitudeProblem<Q1_14, MahonyQ1_14_MARG, true>;
using MadgwickQ1_14_IMUProblem = AttitudeProblem<Q1_14, MadgwickQ1_14_IMU, false>;
using MadgwickQ1_14_MARGProblem = AttitudeProblem<Q1_14, MadgwickQ1_14_MARG, true>;

// =============================================================================
// Forward Declarations
// =============================================================================
void analyze_fixed_point_precision();
void test_attitude_filter_benchmark();
void test_gamma_bot_datasets();
void test_refined_q_format_coverage();
void run_all_datasets_float_and_export_csv(std::ofstream& csv_file, const char* format_name, int total_bits);
void run_all_datasets_double_and_export_csv(std::ofstream& csv_file, const char* format_name, int total_bits);
void run_all_datasets_q6_25_and_export_csv(std::ofstream& csv_file);
void run_all_datasets_q7_24_and_export_csv(std::ofstream& csv_file);
void run_all_datasets_q4_27_and_export_csv(std::ofstream& csv_file);
void run_all_datasets_q2_13_and_export_csv(std::ofstream& csv_file);
void run_all_datasets_q4_11_and_export_csv(std::ofstream& csv_file);
void run_all_datasets_q8_7_and_export_csv(std::ofstream& csv_file);
// =============================================================================
// Error Tracking and Results Structure
// =============================================================================
struct BenchmarkResult {
    const char* filter_name;
    const char* precision_type;
    const char* sensor_type;
    const char* dataset_name;  // Add dataset field
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

// Forward declaration for export function (needs BenchmarkResult to be defined)
void export_result_to_csv(std::ofstream& csv_file, const BenchmarkResult& result, 
                         const char* format_name, const char* filter_name, const char* sensor_type,
                         const char* dataset_name, int total_bits, int integer_bits, int fractional_bits,
                         const char* range_min, const char* range_max, const char* resolution);

// Helper function for computing quaternion angle distance with proper type handling

// =============================================================================
// Custom Error-Tracking Harness
// =============================================================================
// Overloaded version with dataset name
template<typename Problem, typename Kernel, typename... GainArgs>
BenchmarkResult run_error_benchmark(const char* filter_name, 
                                   const char* precision_type,
                                   const char* sensor_type,
                                   const char* dataset_name,
                                   const char* input_path,
                                   Kernel kernel,
                                   GainArgs... gains) {
    
    BenchmarkResult result = {};
    result.filter_name = filter_name;
    result.precision_type = precision_type;
    result.sensor_type = sensor_type;
    result.dataset_name = dataset_name;
    result.completed_successfully = false;
    
    std::vector<double> errors;
    size_t valid_count = 0;
    size_t failed_count = 0;
    
    try {
        //ENTO_INFO("Running error benchmark: %s (%s, %s)", filter_name, precision_type, sensor_type);
        
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
            ENTO_INFO("Failed to open input file: %s", input_path);
            return result;
        }
        
        std::string line;
        size_t sample_count = 0;
        
        // Skip the header line "Attitude Estimation Problem"
        if (std::getline(input_file, line) && line.find("Attitude Estimation Problem") != std::string::npos) {
            // Header found and skipped
        } else {
            // No header, rewind to beginning
            input_file.clear();
            input_file.seekg(0);
        }
        
        while (std::getline(input_file, line) && !line.empty()) {
            // Deserialize the line
            if (!problem.deserialize_impl(line.c_str())) {
                // ENTO_INFO("Failed to deserialize line %zu", sample_count);  // Commented out to reduce noise
                failed_count++;
                continue;
            }
            
            // Solve the problem (run the filter)
            problem.solve_impl();
            
            // DEBUG: Print quaternion values for first few samples to diagnose fixed-point issue
            if (sample_count < 3 && (precision_type[0] == 'Q')) {
                //ENTO_INFO("[%s %s] Sample %zu: GT=(%.6f,%.6f,%.6f,%.6f) EST=(%.6f,%.6f,%.6f,%.6f)", 
                //          filter_name, precision_type, sample_count,
                //          problem.q_gt_.w(), problem.q_gt_.x(), problem.q_gt_.y(), problem.q_gt_.z(),
                //          static_cast<float>(problem.q_.w()), static_cast<float>(problem.q_.x()), 
                //          static_cast<float>(problem.q_.y()), static_cast<float>(problem.q_.z()));
            }
            
            // Compute the actual error using float ground truth for precision consistency
            double error_deg = compute_quat_angle_distance(problem.q_, problem.q_gt_);
            errors.push_back(error_deg);
            
            // DEBUG: Print error for first few samples of fixed-point filters
            if (sample_count < 3 && (precision_type[0] == 'Q')) {
                //ENTO_INFO("[%s %s] Sample %zu: Error = %.6f degrees", filter_name, precision_type, sample_count, error_deg);
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
        
        //ENTO_INFO("Completed: %s - %zu samples, %.1f%% success rate, %.2f° mean error", 
        //          filter_name, result.total_samples, result.success_rate, result.mean_error_deg);
        
    } catch (const std::exception& e) {
        ENTO_INFO("Benchmark failed for %s: %s", filter_name, e.what());
        result.completed_successfully = false;
    }
    
    return result;
}

// Original version (backwards compatibility)
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
    result.dataset_name = "Main";  // Default dataset name
    result.completed_successfully = false;
    
    std::vector<double> errors;
    size_t valid_count = 0;
    size_t failed_count = 0;
    
    try {
        //ENTO_INFO("Running error benchmark: %s (%s, %s)", filter_name, precision_type, sensor_type);
        
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
            ENTO_INFO("Failed to open input file: %s", input_path);
            return result;
        }
        
        std::string line;
        size_t sample_count = 0;
        
        // Skip the header line "Attitude Estimation Problem"
        if (std::getline(input_file, line) && line.find("Attitude Estimation Problem") != std::string::npos) {
            // Header found and skipped
        } else {
            // No header, rewind to beginning
            input_file.clear();
            input_file.seekg(0);
        }
        
        while (std::getline(input_file, line) && !line.empty()) {
            // Deserialize the line
            if (!problem.deserialize_impl(line.c_str())) {
                // ENTO_INFO("Failed to deserialize line %zu", sample_count);  // Commented out to reduce noise
                failed_count++;
                continue;
            }
            
            // Solve the problem (run the filter)
            problem.solve_impl();
            
            // DEBUG: Print quaternion values for first few samples to diagnose fixed-point issue
            if (sample_count < 3 && (precision_type[0] == 'Q')) {
                //ENTO_INFO("[%s %s] Sample %zu: GT=(%.6f,%.6f,%.6f,%.6f) EST=(%.6f,%.6f,%.6f,%.6f)", 
                //          filter_name, precision_type, sample_count,
                //          problem.q_gt_.w(), problem.q_gt_.x(), problem.q_gt_.y(), problem.q_gt_.z(),
                //          static_cast<float>(problem.q_.w()), static_cast<float>(problem.q_.x()), 
                //          static_cast<float>(problem.q_.y()), static_cast<float>(problem.q_.z()));
            }
            
            // Compute the actual error using float ground truth for precision consistency
            double error_deg = compute_quat_angle_distance(problem.q_, problem.q_gt_);
            errors.push_back(error_deg);
            
            // DEBUG: Print error for first few samples of fixed-point filters
            if (sample_count < 3 && (precision_type[0] == 'Q')) {
                //ENTO_INFO("[%s %s] Sample %zu: Error = %.6f degrees", filter_name, precision_type, sample_count, error_deg);
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
        
        //ENTO_INFO("Completed: %s - %zu samples, %.1f%% success rate, %.2f° mean error", 
        //          filter_name, result.total_samples, result.success_rate, result.mean_error_deg);
        
    } catch (const std::exception& e) {
        ENTO_INFO("Benchmark failed for %s: %s", filter_name, e.what());
        result.completed_successfully = false;
    }
    
    return result;
}

// =============================================================================
// Individual Benchmark Functions (all filter types)
// =============================================================================

void benchmark_floating_point_filters() {
    ENTO_INFO("=== Floating-Point Filter Error Benchmarks (Baseline) ===");
    
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
    ENTO_INFO("=== Q7.24 Fixed-Point Filter Error Benchmarks ===");
    
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
    ENTO_INFO("=== Q5.26 Fixed-Point Filter Error Benchmarks ===");
    
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
    
    // Madgwick Q5.26 (MARG)
    {
        MadgwickQ5_26_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ5_26_MARGProblem>("Madgwick", "Q5.26", "MARG", 
                                                                    marg_input_path, kernel, Q5_26(0.001f));
        benchmark_results.push_back(result);
    }
}

void benchmark_fixed_point_q3_12_filters() {
    ENTO_INFO("=== Q3.12 Fixed-Point Filter Error Benchmarks (With Float Ground Truth) ===");
    
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
    ENTO_INFO("\n=== ATTITUDE FILTER ERROR BENCHMARK SUMMARY ===");
    ENTO_INFO("%-12s %-8s %-6s %-8s %-7s %-7s %-8s %-8s %-8s %s", 
              "Filter", "Precision", "Sensor", "Dataset", "Samples", "Success", "Mean(°)", "Max(°)", "Std(°)", "Status");
    ENTO_INFO("%-12s %-8s %-6s %-8s %-7s %-7s %-8s %-8s %-8s %s", 
              "--------", "--------", "------", "--------", "-------", "-------", "--------", "-------", "-------", "------");
    
    for (const auto& result : benchmark_results) {
        const char* status = result.completed_successfully ? "OK" : "FAIL";
        
        if (result.completed_successfully) {
            ENTO_INFO("%-12s %-8s %-6s %-8s %7zu %6.1f%% %8.6f %8.6f %8.6f %s", 
                      result.filter_name, result.precision_type, result.sensor_type, result.dataset_name,
                      result.total_samples, result.success_rate,
                      result.mean_error_deg, result.max_error_deg, result.std_error_deg,
                      status);
        } else {
            ENTO_INFO("%-12s %-8s %-6s %-8s %7s %7s %8s %8s %8s %s", 
                      result.filter_name, result.precision_type, result.sensor_type, result.dataset_name,
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
    
    ENTO_INFO("\n=== SUMMARY ===");
    ENTO_INFO("Total tests: %zu", total_tests);
    ENTO_INFO("Successful: %zu", successful_tests);
    ENTO_INFO("Failed: %zu", total_tests - successful_tests);
    ENTO_INFO("Success rate: %.1f%%", (100.0 * successful_tests) / total_tests);
}

// =============================================================================
// Debug Analysis Functions
// =============================================================================

void analyze_fixed_point_precision() {
    ENTO_INFO("=== Fixed-Point Precision Analysis ===");
    
    // Test small values that are typical in our dataset
    std::vector<float> test_values = {
        0.000001f, 0.00001f, 0.0001f, 0.001f, 0.01f, 0.1f, 1.0f,
        -0.002930f, -0.001221f, -0.000488f,  // Actual values from debug output
        10.0f, 50.0f, 100.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f  // Higher gyro rates to test integer range
    };
    
    std::ofstream analysis_file("fixed_point_analysis.txt");
    analysis_file << "Value,Q3_12,Q7_24,Q5_26,Q6_25,Q8_23,Q4_27,Q2_29,Q2_13,Q4_11,Q8_7,Q6_10,Q13_2,Q11_5,Q1_14,";
    analysis_file << "Q3_12_sq,Q7_24_sq,Q5_26_sq,Q6_25_sq,Q8_23_sq,Q4_27_sq,Q2_29_sq,Q2_13_sq,Q4_11_sq,Q8_7_sq,Q6_10_sq,Q13_2_sq,Q11_5_sq,Q1_14_sq\\n";
    
    for (float val : test_values) {
        // Test basic conversion accuracy
        Q3_12 q3_12_val(val);
        Q7_24 q7_24_val(val);
        Q5_26 q5_26_val(val);
        Q6_25 q6_25_val(val);
        Q8_23 q8_23_val(val);
        Q4_27 q4_27_val(val);
        Q2_29 q2_29_val(val);
        Q2_13 q2_13_val(val);
        Q4_11 q4_11_val(val);
        Q8_7 q8_7_val(val);
        Q6_10 q6_10_val(val);
        Q13_2 q13_2_val(val);
        Q11_5 q11_5_val(val);
        Q1_14 q1_14_val(val);
        
        // Test squared values (common in algorithms)
        float val_sq = val * val;
        Q3_12 q3_12_sq(val_sq);
        Q7_24 q7_24_sq(val_sq);
        Q5_26 q5_26_sq(val_sq);
        Q6_25 q6_25_sq(val_sq);
        Q8_23 q8_23_sq(val_sq);
        Q4_27 q4_27_sq(val_sq);
        Q2_29 q2_29_sq(val_sq);
        Q2_13 q2_13_sq(val_sq);
        Q4_11 q4_11_sq(val_sq);
        Q8_7 q8_7_sq(val_sq);
        Q6_10 q6_10_sq(val_sq);
        Q13_2 q13_2_sq(val_sq);
        Q11_5 q11_5_sq(val_sq);
        Q1_14 q1_14_sq(val_sq);
        
        analysis_file << val << ","
                     << (float)q3_12_val << "," << (float)q7_24_val << "," << (float)q5_26_val << ","
                     << (float)q6_25_val << "," << (float)q8_23_val << "," << (float)q4_27_val << ","
                     << (float)q2_29_val << "," << (float)q2_13_val << "," << (float)q4_11_val << ","
                     << (float)q8_7_val << "," << (float)q6_10_val << "," << (float)q13_2_val << ","
                     << (float)q11_5_val << "," << (float)q1_14_val << ","
                     << (float)q3_12_sq << "," << (float)q7_24_sq << "," << (float)q5_26_sq << ","
                     << (float)q6_25_sq << "," << (float)q8_23_sq << "," << (float)q4_27_sq << ","
                     << (float)q2_29_sq << "," << (float)q2_13_sq << "," << (float)q4_11_sq << ","
                     << (float)q8_7_sq << "," << (float)q6_10_sq << "," << (float)q13_2_sq << ","
                     << (float)q11_5_sq << "," << (float)q1_14_sq << "\\n";
    }
    analysis_file.close();
    
    // Create comprehensive Q format summary with all formats
    std::ofstream format_summary("q_format_summary.txt");
    format_summary << "Format,Bits,IntBits,FracBits,Range,Resolution,Notes\\n";
    format_summary << "Float,32,8,23,±3.4e38,1.19e-07,IEEE 754 reference\\n";
    format_summary << "Q7.24,32,7,24,±128,5.96e-08,High precision 32-bit\\n";
    format_summary << "Q5.26,32,5,26,±32,1.49e-08,Very high precision\\n";
    format_summary << "Q6.25,32,6,25,±64,2.98e-08,Good balance\\n";
    format_summary << "Q8.23,32,8,23,±256,1.19e-07,Higher range for gyro\\n";
    format_summary << "Q4.27,32,4,27,±16,7.45e-09,Extreme precision\\n";
    format_summary << "Q2.29,32,2,29,±4,1.86e-09,Ultra precision narrow\\n";
    format_summary << "Q8.7,16,8,7,±256,0.0078125,Balanced 16-bit\\n";
    format_summary << "Q6.10,16,6,10,±64,0.000976563,Moderate 16-bit\\n";
    format_summary << "Q4.11,16,4,11,±16,0.000488281,Low range 16-bit\\n";
    format_summary << "Q2.13,16,2,13,±4,0.000122070,Narrow range 16-bit\\n";
    format_summary << "Q13.2,16,13,2,±8192,0.25,Max range minimal precision\\n";
    format_summary << "Q11.5,16,11,5,±2048,0.03125,High range low precision\\n";
    format_summary << "Q1.14,16,1,14,±2,6.10e-05,Ultra precision ultra narrow\\n";
    format_summary << "Q3.12,16,3,12,±8,0.000244141,Original test format\\n";
    format_summary.close();
    
    ENTO_INFO("Fixed-point precision analysis saved to fixed_point_analysis.txt");
    ENTO_INFO("Q format summary saved to q_format_summary.txt");
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
    
    ENTO_INFO("Benchmark dataset paths:");
    ENTO_INFO("  MARG input: %s", marg_input_path);
    ENTO_INFO("  IMU input: %s", imu_input_path);
    
    // Clear previous results
    benchmark_results.clear();
    
    // COMPREHENSIVE ATTITUDE FILTER BENCHMARK: All Precision Types + Fourati
    ENTO_INFO("=== COMPREHENSIVE ATTITUDE FILTER BENCHMARK ===");
    
    // ========== FLOAT PRECISION TESTS (baseline) ==========
    ENTO_INFO("=== FLOAT PRECISION FILTERS ===");
    
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
                                                                   imu_input_path, kernel, 0.001f);  // gain
        benchmark_results.push_back(result);
    }
    
    // Madgwick Float (MARG)
    {
        MadgwickFloat_MARG kernel;
        auto result = run_error_benchmark<MadgwickFloat_MARGProblem>("Madgwick", "Float", "MARG", 
                                                                    marg_input_path, kernel, 0.001f);  // gain
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
    ENTO_INFO("=== DOUBLE PRECISION FILTERS ===");
    
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
    ENTO_INFO("=== Q7.24 FIXED-POINT FILTERS ===");
    
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
    ENTO_INFO("=== Q5.26 FIXED-POINT FILTERS ===");
    
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
    
    // ========== Q6.25 FIXED-POINT TESTS ==========
    ENTO_INFO("=== Q6.25 FIXED-POINT FILTERS ===");
    
    // Mahony Q6.25 (IMU)
    {
        MahonyQ6_25_IMU kernel;
        auto result = run_error_benchmark<MahonyQ6_25_IMUProblem>("Mahony", "Q6.25", "IMU", 
                                                                 imu_input_path, kernel, Q6_25(0.01f), Q6_25(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q6.25 (MARG)
    {
        MahonyQ6_25_MARG kernel;
        auto result = run_error_benchmark<MahonyQ6_25_MARGProblem>("Mahony", "Q6.25", "MARG", 
                                                                  marg_input_path, kernel, Q6_25(0.01f), Q6_25(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q6.25 (IMU)
    {
        MadgwickQ6_25_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ6_25_IMUProblem>("Madgwick", "Q6.25", "IMU", 
                                                                   imu_input_path, kernel, Q6_25(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q6.25 (MARG)
    {
        MadgwickQ6_25_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ6_25_MARGProblem>("Madgwick", "Q6.25", "MARG", 
                                                                    marg_input_path, kernel, Q6_25(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q8.23 FIXED-POINT TESTS ==========
    ENTO_INFO("=== Q8.23 FIXED-POINT FILTERS ===");
    
    // Mahony Q8.23 (IMU)
    {
        MahonyQ8_23_IMU kernel;
        auto result = run_error_benchmark<MahonyQ8_23_IMUProblem>("Mahony", "Q8.23", "IMU", 
                                                                 imu_input_path, kernel, Q8_23(0.01f), Q8_23(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q8.23 (MARG)
    {
        MahonyQ8_23_MARG kernel;
        auto result = run_error_benchmark<MahonyQ8_23_MARGProblem>("Mahony", "Q8.23", "MARG", 
                                                                  marg_input_path, kernel, Q8_23(0.01f), Q8_23(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q8.23 (IMU)
    {
        MadgwickQ8_23_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ8_23_IMUProblem>("Madgwick", "Q8.23", "IMU", 
                                                                   imu_input_path, kernel, Q8_23(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q8.23 (MARG)
    {
        MadgwickQ8_23_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ8_23_MARGProblem>("Madgwick", "Q8.23", "MARG", 
                                                                    marg_input_path, kernel, Q8_23(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q4.27 FIXED-POINT TESTS ==========
    ENTO_INFO("=== Q4.27 FIXED-POINT FILTERS ===");
    
    // Mahony Q4.27 (IMU)
    {
        MahonyQ4_27_IMU kernel;
        auto result = run_error_benchmark<MahonyQ4_27_IMUProblem>("Mahony", "Q4.27", "IMU", 
                                                                 imu_input_path, kernel, Q4_27(0.01f), Q4_27(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q4.27 (MARG)
    {
        MahonyQ4_27_MARG kernel;
        auto result = run_error_benchmark<MahonyQ4_27_MARGProblem>("Mahony", "Q4.27", "MARG", 
                                                                  marg_input_path, kernel, Q4_27(0.01f), Q4_27(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q4.27 (IMU)
    {
        MadgwickQ4_27_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ4_27_IMUProblem>("Madgwick", "Q4.27", "IMU", 
                                                                   imu_input_path, kernel, Q4_27(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q4.27 (MARG)
    {
        MadgwickQ4_27_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ4_27_MARGProblem>("Madgwick", "Q4.27", "MARG", 
                                                                    marg_input_path, kernel, Q4_27(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q2.29 FIXED-POINT TESTS ==========
    ENTO_INFO("=== Q2.29 FIXED-POINT FILTERS ===");
    
    // Mahony Q2.29 (IMU)
    {
        MahonyQ2_29_IMU kernel;
        auto result = run_error_benchmark<MahonyQ2_29_IMUProblem>("Mahony", "Q2.29", "IMU", 
                                                                 imu_input_path, kernel, Q2_29(0.01f), Q2_29(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q2.29 (MARG)
    {
        MahonyQ2_29_MARG kernel;
        auto result = run_error_benchmark<MahonyQ2_29_MARGProblem>("Mahony", "Q2.29", "MARG", 
                                                                  marg_input_path, kernel, Q2_29(0.01f), Q2_29(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q2.29 (IMU)
    {
        MadgwickQ2_29_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ2_29_IMUProblem>("Madgwick", "Q2.29", "IMU", 
                                                                   imu_input_path, kernel, Q2_29(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q2.29 (MARG)
    {
        MadgwickQ2_29_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ2_29_MARGProblem>("Madgwick", "Q2.29", "MARG", 
                                                                    marg_input_path, kernel, Q2_29(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q2.13 FIXED-POINT TESTS (16-bit) ==========
    ENTO_INFO("=== Q2.13 FIXED-POINT FILTERS ===");
    
    // Mahony Q2.13 (IMU)
    {
        MahonyQ2_13_IMU kernel;
        auto result = run_error_benchmark<MahonyQ2_13_IMUProblem>("Mahony", "Q2.13", "IMU", 
                                                                 imu_input_path, kernel, Q2_13(0.01f), Q2_13(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q2.13 (MARG)
    {
        MahonyQ2_13_MARG kernel;
        auto result = run_error_benchmark<MahonyQ2_13_MARGProblem>("Mahony", "Q2.13", "MARG", 
                                                                  marg_input_path, kernel, Q2_13(0.01f), Q2_13(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q2.13 (IMU)
    {
        MadgwickQ2_13_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ2_13_IMUProblem>("Madgwick", "Q2.13", "IMU", 
                                                                   imu_input_path, kernel, Q2_13(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q2.13 (MARG)
    {
        MadgwickQ2_13_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ2_13_MARGProblem>("Madgwick", "Q2.13", "MARG", 
                                                                    marg_input_path, kernel, Q2_13(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q4.11 FIXED-POINT TESTS (16-bit) ==========
    ENTO_INFO("=== Q4.11 FIXED-POINT FILTERS ===");
    
    // Mahony Q4.11 (IMU)
    {
        MahonyQ4_11_IMU kernel;
        auto result = run_error_benchmark<MahonyQ4_11_IMUProblem>("Mahony", "Q4.11", "IMU", 
                                                                 imu_input_path, kernel, Q4_11(0.01f), Q4_11(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q4.11 (MARG)
    {
        MahonyQ4_11_MARG kernel;
        auto result = run_error_benchmark<MahonyQ4_11_MARGProblem>("Mahony", "Q4.11", "MARG", 
                                                                  marg_input_path, kernel, Q4_11(0.01f), Q4_11(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q4.11 (IMU)
    {
        MadgwickQ4_11_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ4_11_IMUProblem>("Madgwick", "Q4.11", "IMU", 
                                                                   imu_input_path, kernel, Q4_11(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q4.11 (MARG)
    {
        MadgwickQ4_11_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ4_11_MARGProblem>("Madgwick", "Q4.11", "MARG", 
                                                                    marg_input_path, kernel, Q4_11(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q3.12 FIXED-POINT TESTS (16-bit) ==========
    ENTO_INFO("=== Q3.12 FIXED-POINT FILTERS ===");
    
    // Mahony Q3.12 (IMU)
    {
        MahonyQ3_12_IMU kernel;
        auto result = run_error_benchmark<MahonyQ3_12_IMUProblem>("Mahony", "Q3.12", "IMU", 
                                                                 imu_input_path, kernel, Q3_12(0.01f), Q3_12(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Mahony Q3.12 (MARG)
    {
        MahonyQ3_12_MARG kernel;
        auto result = run_error_benchmark<MahonyQ3_12_MARGProblem>("Mahony", "Q3.12", "MARG", 
                                                                  marg_input_path, kernel, Q3_12(0.01f), Q3_12(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q3.12 (IMU)
    {
        MadgwickQ3_12_IMU kernel;
        auto result = run_error_benchmark<MadgwickQ3_12_IMUProblem>("Madgwick", "Q3.12", "IMU", 
                                                                   imu_input_path, kernel, Q3_12(0.001f));
        benchmark_results.push_back(result);
    }
    
    // Madgwick Q3.12 (MARG)
    {
        MadgwickQ3_12_MARG kernel;
        auto result = run_error_benchmark<MadgwickQ3_12_MARGProblem>("Madgwick", "Q3.12", "MARG", 
                                                                    marg_input_path, kernel, Q3_12(0.001f));
        benchmark_results.push_back(result);
    }
    
    // ========== PERFORMANCE ANALYSIS SECTION ==========
    ENTO_INFO("=== MAHONY MARG PERFORMANCE ANALYSIS ===");
    
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
    
    // ========== Q7.24 FIXED-POINT FOURATI TESTS ==========
    ENTO_INFO("=== Q7.24 FIXED-POINT FOURATI FILTERS ===");
    
    // Fourati Q7.24 (MARG)
    {
        FouratiQ7_24_MARG kernel;
        auto result = run_error_benchmark<FouratiQ7_24_MARGProblem>("Fourati", "Q7.24", "MARG", 
                                                                   marg_input_path, kernel, Q7_24(0.1f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q5.26 FIXED-POINT FOURATI TESTS ==========
    ENTO_INFO("=== Q5.26 FIXED-POINT FOURATI FILTERS ===");
    
    // Fourati Q5.26 (MARG)
    {
        FouratiQ5_26_MARG kernel;
        auto result = run_error_benchmark<FouratiQ5_26_MARGProblem>("Fourati", "Q5.26", "MARG", 
                                                                   marg_input_path, kernel, Q5_26(0.1f));
        benchmark_results.push_back(result);
    }
    
    // ========== Q3.12 FIXED-POINT FOURATI TESTS ==========
    ENTO_INFO("=== Q3.12 FIXED-POINT FOURATI FILTERS ===");
    
    // Fourati Q3.12 (MARG)
    {
        FouratiQ3_12_MARG kernel;
        auto result = run_error_benchmark<FouratiQ3_12_MARGProblem>("Fourati", "Q3.12", "MARG", 
                                                                   marg_input_path, kernel, Q3_12(0.1f));
        benchmark_results.push_back(result);
    }
    
    // ========== ADDITIONAL FOURATI FIXED-POINT TESTS ==========
    ENTO_INFO("=== ADDITIONAL FOURATI FIXED-POINT FILTERS ===");
    
    // Fourati Q6.25 (MARG)
    {
        FouratiQ6_25_MARG kernel;
        auto result = run_error_benchmark<FouratiQ6_25_MARGProblem>("Fourati", "Q6.25", "MARG", 
                                                                   marg_input_path, kernel, Q6_25(0.1f));
        benchmark_results.push_back(result);
    }
    
    // Fourati Q8.23 (MARG)
    {
        FouratiQ8_23_MARG kernel;
        auto result = run_error_benchmark<FouratiQ8_23_MARGProblem>("Fourati", "Q8.23", "MARG", 
                                                                   marg_input_path, kernel, Q8_23(0.1f));
        benchmark_results.push_back(result);
    }
    
    // Fourati Q4.27 (MARG)
    {
        FouratiQ4_27_MARG kernel;
        auto result = run_error_benchmark<FouratiQ4_27_MARGProblem>("Fourati", "Q4.27", "MARG", 
                                                                   marg_input_path, kernel, Q4_27(0.1f));
        benchmark_results.push_back(result);
    }
    
    // Fourati Q2.29 (MARG)
    {
        FouratiQ2_29_MARG kernel;
        auto result = run_error_benchmark<FouratiQ2_29_MARGProblem>("Fourati", "Q2.29", "MARG", 
                                                                   marg_input_path, kernel, Q2_29(0.1f));
        benchmark_results.push_back(result);
    }
    
    // Fourati Q2.13 (MARG)
    {
        FouratiQ2_13_MARG kernel;
        auto result = run_error_benchmark<FouratiQ2_13_MARGProblem>("Fourati", "Q2.13", "MARG", 
                                                                   marg_input_path, kernel, Q2_13(0.1f));
        benchmark_results.push_back(result);
    }
    
    // Fourati Q4.11 (MARG)
    {
        FouratiQ4_11_MARG kernel;
        auto result = run_error_benchmark<FouratiQ4_11_MARGProblem>("Fourati", "Q4.11", "MARG", 
                                                                   marg_input_path, kernel, Q4_11(0.1f));
        benchmark_results.push_back(result);
    }
    
    // Print summary
    print_benchmark_summary();
}

// =============================================================================
// Gamma-Bot Dataset Benchmark Function  
// =============================================================================

void test_gamma_bot_datasets() {
    ENTO_INFO("=== GAMMA-BOT DATASETS BENCHMARK ===");
    
    // Dataset paths for gamma-bot data
    const char* base_dir = "../../datasets/state-est/attitude";
    
    char steering_marg_path[FILEPATH_SIZE];
    char steering_imu_path[FILEPATH_SIZE];
    char straight_marg_path[FILEPATH_SIZE];
    char straight_imu_path[FILEPATH_SIZE];
    
    snprintf(steering_marg_path, FILEPATH_SIZE, "%s/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt", base_dir);
    snprintf(steering_imu_path, FILEPATH_SIZE,  "%s/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt", base_dir);
    snprintf(straight_marg_path, FILEPATH_SIZE, "%s/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt", base_dir);
    snprintf(straight_imu_path, FILEPATH_SIZE,  "%s/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt", base_dir);
    
    ENTO_INFO("Gamma-bot dataset paths:");
    ENTO_INFO("  Steering MARG: %s", steering_marg_path);
    ENTO_INFO("  Steering IMU: %s", steering_imu_path);
    ENTO_INFO("  Straight MARG: %s", straight_marg_path);
    ENTO_INFO("  Straight IMU: %s", straight_imu_path);
    
    // Clear previous results for gamma-bot tests
    benchmark_results.clear();
    
    // Test datasets with comprehensive Q formats - ALL ESTIMATOR TYPES (Mahony, Madgwick, Fourati)
    const char* datasets[] = {"Steering", "Straight"};
    const char* marg_paths[] = {steering_marg_path, straight_marg_path};
    const char* imu_paths[] = {steering_imu_path, straight_imu_path};
    
    for (int d = 0; d < 2; d++) {
        const char* dataset_name = datasets[d];
        const char* marg_path = marg_paths[d];
        const char* imu_path = imu_paths[d];
        
        ENTO_INFO("=== Testing %s Dataset (All Estimators: Mahony, Madgwick, Fourati) ===", dataset_name);
        
        // ========== FLOAT PRECISION BASELINE (all estimators) ==========
        ENTO_INFO("--- Float Precision Baseline ---");
        
        // Mahony Float (MARG & IMU)
        {
            MahonyFloat_MARG kernel;
            auto result = run_error_benchmark<MahonyFloat_MARGProblem>("Mahony", "Float", "MARG", 
                                                                      dataset_name, marg_path, kernel, 0.01f, 0.001f);
            benchmark_results.push_back(result);
        }
        {
            MahonyFloat_IMU kernel;
            auto result = run_error_benchmark<MahonyFloat_IMUProblem>("Mahony", "Float", "IMU", 
                                                                     dataset_name, imu_path, kernel, 0.01f, 0.001f);
            benchmark_results.push_back(result);
        }
        
        // Madgwick Float (MARG & IMU)
        {
            MadgwickFloat_MARG kernel;
            auto result = run_error_benchmark<MadgwickFloat_MARGProblem>("Madgwick", "Float", "MARG", 
                                                                        dataset_name, marg_path, kernel, 0.001f);
            benchmark_results.push_back(result);
        }
        {
            MadgwickFloat_IMU kernel;
            auto result = run_error_benchmark<MadgwickFloat_IMUProblem>("Madgwick", "Float", "IMU", 
                                                                       dataset_name, imu_path, kernel, 0.001f);
            benchmark_results.push_back(result);
        }
        
        // Fourati Float (MARG only)
        {
            FouratiFloat_MARG kernel;
            auto result = run_error_benchmark<FouratiFloat_MARGProblem>("Fourati", "Float", "MARG", 
                                                                       dataset_name, marg_path, kernel, 0.1f);
            benchmark_results.push_back(result);
        }
        
        // ========== KEY 32-BIT Q FORMATS (all estimators) ==========
        ENTO_INFO("--- 32-bit Q Formats (expected to work well) ---");
        
        // Q8.23 - Higher integer range for aggressive maneuvers
        {
            MahonyQ8_23_MARG kernel;
            auto result = run_error_benchmark<MahonyQ8_23_MARGProblem>("Mahony", "Q8.23", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q8_23(0.01f), Q8_23(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ8_23_IMU kernel;
            auto result = run_error_benchmark<MahonyQ8_23_IMUProblem>("Mahony", "Q8.23", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q8_23(0.01f), Q8_23(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MadgwickQ8_23_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ8_23_MARGProblem>("Madgwick", "Q8.23", "MARG", 
                                                                        dataset_name, marg_path, kernel, Q8_23(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MadgwickQ8_23_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ8_23_IMUProblem>("Madgwick", "Q8.23", "IMU", 
                                                                       dataset_name, imu_path, kernel, Q8_23(0.001f));
            benchmark_results.push_back(result);
        }
        {
            FouratiQ8_23_MARG kernel;
            auto result = run_error_benchmark<FouratiQ8_23_MARGProblem>("Fourati", "Q8.23", "MARG", 
                                                                       dataset_name, marg_path, kernel, Q8_23(0.1f));
            benchmark_results.push_back(result);
        }
        
        // Q6.25 - Good balance
        {
            MahonyQ6_25_MARG kernel;
            auto result = run_error_benchmark<MahonyQ6_25_MARGProblem>("Mahony", "Q6.25", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q6_25(0.01f), Q6_25(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ6_25_IMU kernel;
            auto result = run_error_benchmark<MahonyQ6_25_IMUProblem>("Mahony", "Q6.25", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q6_25(0.01f), Q6_25(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MadgwickQ6_25_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ6_25_MARGProblem>("Madgwick", "Q6.25", "MARG", 
                                                                        dataset_name, marg_path, kernel, Q6_25(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MadgwickQ6_25_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ6_25_IMUProblem>("Madgwick", "Q6.25", "IMU", 
                                                                       dataset_name, imu_path, kernel, Q6_25(0.001f));
            benchmark_results.push_back(result);
        }
        {
            FouratiQ6_25_MARG kernel;
            auto result = run_error_benchmark<FouratiQ6_25_MARGProblem>("Fourati", "Q6.25", "MARG", 
                                                                       dataset_name, marg_path, kernel, Q6_25(0.1f));
            benchmark_results.push_back(result);
        }
        
        // 16-bit Q formats - expected high failure rates
        ENTO_INFO("--- Testing 16-bit Q formats (high failure rate expected) ---");
        
        // Q8.7 - Balanced 16-bit format (all estimators)
        {
            MahonyQ8_7_MARG kernel;
            auto result = run_error_benchmark<MahonyQ8_7_MARGProblem>("Mahony", "Q8.7", "MARG", 
                                                                     dataset_name, marg_path, kernel, Q8_7(0.01f), Q8_7(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ8_7_IMU kernel;
            auto result = run_error_benchmark<MahonyQ8_7_IMUProblem>("Mahony", "Q8.7", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q8_7(0.01f), Q8_7(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MadgwickQ8_7_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ8_7_MARGProblem>("Madgwick", "Q8.7", "MARG", 
                                                                       dataset_name, marg_path, kernel, Q8_7(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MadgwickQ8_7_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ8_7_IMUProblem>("Madgwick", "Q8.7", "IMU", 
                                                                      dataset_name, imu_path, kernel, Q8_7(0.001f));
            benchmark_results.push_back(result);
        }
        
        // Q6.10 - Moderate range, moderate fractional precision
        {
            MahonyQ6_10_MARG kernel;
            auto result = run_error_benchmark<MahonyQ6_10_MARGProblem>("Mahony", "Q6.10", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q6_10(0.01f), Q6_10(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ6_10_IMU kernel;
            auto result = run_error_benchmark<MahonyQ6_10_IMUProblem>("Mahony", "Q6.10", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q6_10(0.01f), Q6_10(0.001f));
            benchmark_results.push_back(result);
        }
        
        // Q4.11 - Expected high failure rate (16-bit, coarse fractional)
        {
            MahonyQ4_11_MARG kernel;
            auto result = run_error_benchmark<MahonyQ4_11_MARGProblem>("Mahony", "Q4.11", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q4_11(0.01f), Q4_11(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ4_11_IMU kernel;
            auto result = run_error_benchmark<MahonyQ4_11_IMUProblem>("Mahony", "Q4.11", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q4_11(0.01f), Q4_11(0.001f));
            benchmark_results.push_back(result);
        }
        
        // Q2.13 - Expected high failure rate (16-bit, narrow range)
        {
            MahonyQ2_13_MARG kernel;
            auto result = run_error_benchmark<MahonyQ2_13_MARGProblem>("Mahony", "Q2.13", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q2_13(0.01f), Q2_13(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ2_13_IMU kernel;
            auto result = run_error_benchmark<MahonyQ2_13_IMUProblem>("Mahony", "Q2.13", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q2_13(0.01f), Q2_13(0.001f));
            benchmark_results.push_back(result);
        }
        
        // Extreme 16-bit formats - very likely to fail
        ENTO_INFO("--- Testing extreme 16-bit Q formats (failures almost certain) ---");
        
        // Q13.2 - Maximum integer range, minimal fractional precision (almost certainly fails)
        {
            MahonyQ13_2_MARG kernel;
            auto result = run_error_benchmark<MahonyQ13_2_MARGProblem>("Mahony", "Q13.2", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q13_2(0.01f), Q13_2(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ13_2_IMU kernel;
            auto result = run_error_benchmark<MahonyQ13_2_IMUProblem>("Mahony", "Q13.2", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q13_2(0.01f), Q13_2(0.001f));
            benchmark_results.push_back(result);
        }
        
        // Q11.5 - High integer range, very low fractional precision (almost certainly fails)
        {
            MahonyQ11_5_MARG kernel;
            auto result = run_error_benchmark<MahonyQ11_5_MARGProblem>("Mahony", "Q11.5", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q11_5(0.01f), Q11_5(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ11_5_IMU kernel;
            auto result = run_error_benchmark<MahonyQ11_5_IMUProblem>("Mahony", "Q11.5", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q11_5(0.01f), Q11_5(0.001f));
            benchmark_results.push_back(result);
        }
        
        // Q1.14 - Extreme fractional precision, very narrow range ±2 (almost certainly fails for gyro rates)
        {
            MahonyQ1_14_MARG kernel;
            auto result = run_error_benchmark<MahonyQ1_14_MARGProblem>("Mahony", "Q1.14", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q1_14(0.01f), Q1_14(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ1_14_IMU kernel;
            auto result = run_error_benchmark<MahonyQ1_14_IMUProblem>("Mahony", "Q1.14", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q1_14(0.01f), Q1_14(0.001f));
            benchmark_results.push_back(result);
        }
        
        // Q3.12 - Expected high failure rate (small fractional bits)
        {
            MahonyQ3_12_MARG kernel;
            auto result = run_error_benchmark<MahonyQ3_12_MARGProblem>("Mahony", "Q3.12", "MARG", 
                                                                      dataset_name, marg_path, kernel, Q3_12(0.01f), Q3_12(0.001f));
            benchmark_results.push_back(result);
        }
        {
            MahonyQ3_12_IMU kernel;
            auto result = run_error_benchmark<MahonyQ3_12_IMUProblem>("Mahony", "Q3.12", "IMU", 
                                                                     dataset_name, imu_path, kernel, Q3_12(0.01f), Q3_12(0.001f));
            benchmark_results.push_back(result);
        }
    }
    
    // Generate gamma-bot specific results file
    char gamma_results_path[FILEPATH_SIZE];
    snprintf(gamma_results_path, FILEPATH_SIZE, "./gamma_bot_failure_analysis.csv");
    
    std::ofstream results_file(gamma_results_path);
    results_file << "Filter,Format,Sensor,Dataset,AvgError,MaxError,Samples,Failures,FailureRate,FormatBits,IntBits,FracBits,Range\n";
    
    for (const auto& result : benchmark_results) {
        double failure_rate = (result.total_samples > 0) ? 
                             (double)result.failed_samples / result.total_samples * 100.0 : 0.0;
        
        // Add format analysis info
        std::string format_info = "";
        if (result.precision_type == std::string("Q8.23")) format_info = "32,8,23,±256";
        else if (result.precision_type == std::string("Q6.25")) format_info = "32,6,25,±64";
        else if (result.precision_type == std::string("Q4.27")) format_info = "32,4,27,±16";
        else if (result.precision_type == std::string("Q2.29")) format_info = "32,2,29,±4";
        else if (result.precision_type == std::string("Q8.7")) format_info = "16,8,7,±256";
        else if (result.precision_type == std::string("Q6.10")) format_info = "16,6,10,±64";
        else if (result.precision_type == std::string("Q4.11")) format_info = "16,4,11,±16";
        else if (result.precision_type == std::string("Q2.13")) format_info = "16,2,13,±4";
        else if (result.precision_type == std::string("Q13.2")) format_info = "16,13,2,±8192";
        else if (result.precision_type == std::string("Q11.5")) format_info = "16,11,5,±2048";
        else if (result.precision_type == std::string("Q1.14")) format_info = "16,1,14,±2";
        else if (result.precision_type == std::string("Q3.12")) format_info = "16,3,12,±8";
        else if (result.precision_type == std::string("Float")) format_info = "32,8,23,±3.4e38";
        
        results_file << result.filter_name << "," << result.precision_type << "," 
                    << result.sensor_type << "," << result.dataset_name << "," << result.mean_error_deg << "," 
                    << result.max_error_deg << "," << result.total_samples << "," 
                    << result.failed_samples << "," << failure_rate << "," << format_info << "\n";
    }
    results_file.close();
    
    ENTO_INFO("=== GAMMA-BOT BENCHMARK RESULTS ===");
    ENTO_INFO("%-12s %-8s %-6s %-8s %-7s %-7s %-8s %-8s %-8s %-8s %s", 
              "Filter", "Precision", "Sensor", "Dataset", "Samples", "Success", "Failures", "Fail%", "Mean(°)", "Max(°)", "Status");
    ENTO_INFO("%-12s %-8s %-6s %-8s %-7s %-7s %-8s %-8s %-8s %-8s %s", 
              "--------", "--------", "------", "--------", "-------", "-------", "--------", "-------", "--------", "-------", "------");
    
    for (const auto& result : benchmark_results) {
        double failure_rate = (result.total_samples > 0) ? 
                             (double)result.failed_samples / result.total_samples * 100.0 : 0.0;
        const char* status = result.completed_successfully ? "OK" : "FAIL";
        
        if (result.completed_successfully) {
            ENTO_INFO("%-12s %-8s %-6s %-8s %7zu %6.1f%% %8zu %6.1f%% %8.6f %8.6f %s", 
                      result.filter_name, result.precision_type, result.sensor_type, result.dataset_name,
                      result.total_samples, result.success_rate, result.failed_samples, failure_rate,
                      result.mean_error_deg, result.max_error_deg, status);
        } else {
            ENTO_INFO("%-12s %-8s %-6s %-8s %7s %7s %8s %7s %8s %8s %s", 
                      result.filter_name, result.precision_type, result.sensor_type, result.dataset_name,
                      "N/A", "N/A", "N/A", "N/A", "N/A", "N/A", status);
        }
    }
    
    ENTO_INFO("Gamma-bot failure analysis saved to: %s", gamma_results_path);
    ENTO_INFO("=== GAMMA-BOT BENCHMARK COMPLETED ===");
}

// =============================================================================
// Refined Q Format Coverage Analysis (8 Key Formats for Visualization)
// =============================================================================

void test_refined_q_format_coverage() {
    ENTO_INFO("=== REFINED Q FORMAT COVERAGE ANALYSIS (8 Key Formats) ===");
    ENTO_INFO("Testing carefully selected formats that span interesting precision regimes:");
    ENTO_INFO("• Float/Double: Baselines");
    ENTO_INFO("• Q6.25, Q7.24, Q4.27: 32-bit formats with different trade-offs");  
    ENTO_INFO("• Q2.13, Q4.11, Q8.7: 16-bit formats from high precision to stress test");
    ENTO_INFO("");

    // Open CSV file for visualization-ready output
    std::ofstream csv_file("refined_attitude_benchmark_results.csv");
    csv_file << "Format,Filter,Sensor,Dataset,Success_Rate,Total_Samples,Failed_Samples,Mean_Error,Std_Error,Max_Error,Integer_Bits,Fractional_Bits,Total_Bits,Range_Min,Range_Max,Resolution\n";

    // Test Float baseline
    ENTO_INFO("Testing Float (32-bit floating point baseline):");
    run_all_datasets_float_and_export_csv(csv_file, "Float", 32);
    
    // Test Double baseline  
    ENTO_INFO("Testing Double (64-bit floating point baseline):");
    run_all_datasets_double_and_export_csv(csv_file, "Double", 64);

    // Test Q6.25 (32-bit: good balance, more integer bits than Q5.26)
    ENTO_INFO("Testing Q6.25 (32-bit: good balance, ±63 range, high precision):");
    run_all_datasets_q6_25_and_export_csv(csv_file);

    // Test Q7.24 (32-bit: higher integer range for aggressive gyro rates)
    ENTO_INFO("Testing Q7.24 (32-bit: higher integer range, ±127 range):");
    run_all_datasets_q7_24_and_export_csv(csv_file);

    // Test Q4.27 (32-bit: very high fractional precision)
    ENTO_INFO("Testing Q4.27 (32-bit: extreme fractional precision, ±15 range):");
    run_all_datasets_q4_27_and_export_csv(csv_file);

    // Test Q2.13 (16-bit: high fractional precision, should work for low gyro rates)
    ENTO_INFO("Testing Q2.13 (16-bit: high precision, ±3 range):");
    run_all_datasets_q2_13_and_export_csv(csv_file);

    // Test Q4.11 (16-bit: moderate balance, interesting transition zone)
    ENTO_INFO("Testing Q4.11 (16-bit: moderate balance, ±15 range):");
    run_all_datasets_q4_11_and_export_csv(csv_file);

    // Test Q8.7 (16-bit: stress test - high integer range, low precision)
    ENTO_INFO("Testing Q8.7 (16-bit: stress test, ±255 range, coarse precision):");
    run_all_datasets_q8_7_and_export_csv(csv_file);

    csv_file.close();
    
    ENTO_INFO("=== REFINED Q FORMAT COVERAGE ANALYSIS COMPLETE ===");
    ENTO_INFO("Results exported to: refined_attitude_benchmark_results.csv");
    ENTO_INFO("Run visualization: python3 tools/analyze_attitude_heatmap.py refined_attitude_benchmark_results.csv");
}

// Helper functions for CSV export with format metadata

void run_all_datasets_float_and_export_csv(std::ofstream& csv_file, const char* format_name, int total_bits) {
    // Use real datasets that actually exist
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };
    
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyFloat_MARG kernel;
            auto result = run_error_benchmark<MahonyFloat_MARGProblem>("Mahony", format_name, "MARG", 
                                                                      dataset_names[d], datasets[d], kernel, 1.0f, 0.1f);
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Mahony IMU
        {
            MahonyFloat_IMU kernel;
            auto result = run_error_benchmark<MahonyFloat_IMUProblem>("Mahony", format_name, "IMU", 
                                                                     dataset_names[d], datasets[d], kernel, 1.0f, 0.1f);
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Madgwick MARG
        {
            MadgwickFloat_MARG kernel;
            auto result = run_error_benchmark<MadgwickFloat_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                        dataset_names[d], datasets[d], kernel, 0.1f);
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Madgwick IMU
        {
            MadgwickFloat_IMU kernel;
            auto result = run_error_benchmark<MadgwickFloat_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                       dataset_names[d], datasets[d], kernel, 0.1f);
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Fourati MARG only
        {
            FouratiFloat_MARG kernel;
            auto result = run_error_benchmark<FouratiFloat_MARGProblem>("Fourati", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, 0.1f);
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }
    }
}

void run_all_datasets_double_and_export_csv(std::ofstream& csv_file, const char* format_name, int total_bits) {
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };
    
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyDouble_MARG kernel;
            auto result = run_error_benchmark<MahonyDouble_MARGProblem>("Mahony", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, 1.0, 0.1);
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Mahony IMU
        {
            MahonyDouble_IMU kernel;
            auto result = run_error_benchmark<MahonyDouble_IMUProblem>("Mahony", format_name, "IMU", 
                                                                      dataset_names[d], datasets[d], kernel, 1.0, 0.1);
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Madgwick MARG
        {
            MadgwickDouble_MARG kernel;
            auto result = run_error_benchmark<MadgwickDouble_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                         dataset_names[d], datasets[d], kernel, 0.1);
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Madgwick IMU
        {
            MadgwickDouble_IMU kernel;
            auto result = run_error_benchmark<MadgwickDouble_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                        dataset_names[d], datasets[d], kernel, 0.1);
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }

        // Fourati MARG only
        {
            FouratiDouble_MARG kernel;
            auto result = run_error_benchmark<FouratiDouble_MARGProblem>("Fourati", format_name, "MARG", 
                                                                        dataset_names[d], datasets[d], kernel, 0.1);
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                total_bits, -1, -1, "N/A", "N/A", "N/A");
        }
    }
}

void run_all_datasets_q6_25_and_export_csv(std::ofstream& csv_file) {
    const char* format_name = "Q6.25";
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };
    
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyQ6_25_MARG kernel;
            auto result = run_error_benchmark<MahonyQ6_25_MARGProblem>("Mahony", format_name, "MARG", 
                                                                      dataset_names[d], datasets[d], kernel, Q6_25(1.0f), Q6_25(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                32, 6, 25, "-64", "63.999969", "2.98e-08");
        }

        // Mahony IMU
        {
            MahonyQ6_25_IMU kernel;
            auto result = run_error_benchmark<MahonyQ6_25_IMUProblem>("Mahony", format_name, "IMU", 
                                                                     dataset_names[d], datasets[d], kernel, Q6_25(1.0f), Q6_25(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                32, 6, 25, "-64", "63.999969", "2.98e-08");
        }

        // Madgwick MARG
        {
            MadgwickQ6_25_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ6_25_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                        dataset_names[d], datasets[d], kernel, Q6_25(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                32, 6, 25, "-64", "63.999969", "2.98e-08");
        }

        // Madgwick IMU
        {
            MadgwickQ6_25_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ6_25_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                       dataset_names[d], datasets[d], kernel, Q6_25(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                32, 6, 25, "-64", "63.999969", "2.98e-08");
        }

        // Fourati MARG only
        {
            FouratiQ6_25_MARG kernel;
            auto result = run_error_benchmark<FouratiQ6_25_MARGProblem>("Fourati", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, Q6_25(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                32, 6, 25, "-64", "63.999969", "2.98e-08");
        }
    }
}

void run_all_datasets_q7_24_and_export_csv(std::ofstream& csv_file) {
    const char* format_name = "Q7.24";
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };

    // Yes, using more descriptive generic names for better readability in CSV output
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyQ7_24_MARG kernel;
            auto result = run_error_benchmark<MahonyQ7_24_MARGProblem>("Mahony", format_name, "MARG", 
                                                                      dataset_names[d], datasets[d], kernel, Q7_24(1.0f), Q7_24(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                32, 7, 24, "-128", "127.999994", "5.96e-08");
        }

        // Mahony IMU
        {
            MahonyQ7_24_IMU kernel;
            auto result = run_error_benchmark<MahonyQ7_24_IMUProblem>("Mahony", format_name, "IMU", 
                                                                     dataset_names[d], datasets[d], kernel, Q7_24(1.0f), Q7_24(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                32, 7, 24, "-128", "127.999994", "5.96e-08");
        }

        // Madgwick MARG
        {
            MadgwickQ7_24_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ7_24_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                        dataset_names[d], datasets[d], kernel, Q7_24(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                32, 7, 24, "-128", "127.999994", "5.96e-08");
        }

        // Madgwick IMU
        {
            MadgwickQ7_24_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ7_24_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                       dataset_names[d], datasets[d], kernel, Q7_24(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                32, 7, 24, "-128", "127.999994", "5.96e-08");
        }

        // Fourati MARG only
        {
            FouratiQ7_24_MARG kernel;
            auto result = run_error_benchmark<FouratiQ7_24_MARGProblem>("Fourati", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, Q7_24(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                32, 7, 24, "-128", "127.999994", "5.96e-08");
        }
    }
}

void run_all_datasets_q4_27_and_export_csv(std::ofstream& csv_file) {
    const char* format_name = "Q4.27";
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };

    // Yes, using more descriptive generic names for better readability in CSV output
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyQ4_27_MARG kernel;
            auto result = run_error_benchmark<MahonyQ4_27_MARGProblem>("Mahony", format_name, "MARG", 
                                                                      dataset_names[d], datasets[d], kernel, Q4_27(1.0f), Q4_27(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                32, 4, 27, "-16", "15.999999", "7.45e-09");
        }

        // Mahony IMU
        {
            MahonyQ4_27_IMU kernel;
            auto result = run_error_benchmark<MahonyQ4_27_IMUProblem>("Mahony", format_name, "IMU", 
                                                                     dataset_names[d], datasets[d], kernel, Q4_27(1.0f), Q4_27(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                32, 4, 27, "-16", "15.999999", "7.45e-09");
        }

        // Madgwick MARG
        {
            MadgwickQ4_27_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ4_27_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                        dataset_names[d], datasets[d], kernel, Q4_27(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                32, 4, 27, "-16", "15.999999", "7.45e-09");
        }

        // Madgwick IMU
        {
            MadgwickQ4_27_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ4_27_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                       dataset_names[d], datasets[d], kernel, Q4_27(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                32, 4, 27, "-16", "15.999999", "7.45e-09");
        }

        // Fourati MARG only
        {
            FouratiQ4_27_MARG kernel;
            auto result = run_error_benchmark<FouratiQ4_27_MARGProblem>("Fourati", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, Q4_27(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                32, 4, 27, "-16", "15.999999", "7.45e-09");
        }
    }
}

void run_all_datasets_q2_13_and_export_csv(std::ofstream& csv_file) {
    const char* format_name = "Q2.13";
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };
    
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyQ2_13_MARG kernel;
            auto result = run_error_benchmark<MahonyQ2_13_MARGProblem>("Mahony", format_name, "MARG", 
                                                                      dataset_names[d], datasets[d], kernel, Q2_13(1.0f), Q2_13(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                16, 2, 13, "-4", "3.999878", "0.000122");
        }

        // Mahony IMU
        {
            MahonyQ2_13_IMU kernel;
            auto result = run_error_benchmark<MahonyQ2_13_IMUProblem>("Mahony", format_name, "IMU", 
                                                                     dataset_names[d], datasets[d], kernel, Q2_13(1.0f), Q2_13(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                16, 2, 13, "-4", "3.999878", "0.000122");
        }

        // Madgwick MARG
        {
            MadgwickQ2_13_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ2_13_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                        dataset_names[d], datasets[d], kernel, Q2_13(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                16, 2, 13, "-4", "3.999878", "0.000122");
        }

        // Madgwick IMU
        {
            MadgwickQ2_13_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ2_13_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                       dataset_names[d], datasets[d], kernel, Q2_13(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                16, 2, 13, "-4", "3.999878", "0.000122");
        }

        // Fourati MARG only
        {
            FouratiQ2_13_MARG kernel;
            auto result = run_error_benchmark<FouratiQ2_13_MARGProblem>("Fourati", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, Q2_13(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                16, 2, 13, "-4", "3.999878", "0.000122");
        }
    }
}

void run_all_datasets_q4_11_and_export_csv(std::ofstream& csv_file) {
    const char* format_name = "Q4.11";
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };
    
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyQ4_11_MARG kernel;
            auto result = run_error_benchmark<MahonyQ4_11_MARGProblem>("Mahony", format_name, "MARG", 
                                                                      dataset_names[d], datasets[d], kernel, Q4_11(1.0f), Q4_11(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                16, 4, 11, "-16", "15.999512", "0.000488");
        }

        // Mahony IMU
        {
            MahonyQ4_11_IMU kernel;
            auto result = run_error_benchmark<MahonyQ4_11_IMUProblem>("Mahony", format_name, "IMU", 
                                                                     dataset_names[d], datasets[d], kernel, Q4_11(1.0f), Q4_11(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                16, 4, 11, "-16", "15.999512", "0.000488");
        }

        // Madgwick MARG
        {
            MadgwickQ4_11_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ4_11_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                        dataset_names[d], datasets[d], kernel, Q4_11(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                16, 4, 11, "-16", "15.999512", "0.000488");
        }

        // Madgwick IMU
        {
            MadgwickQ4_11_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ4_11_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                       dataset_names[d], datasets[d], kernel, Q4_11(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                16, 4, 11, "-16", "15.999512", "0.000488");
        }

        // Fourati MARG only
        {
            FouratiQ4_11_MARG kernel;
            auto result = run_error_benchmark<FouratiQ4_11_MARGProblem>("Fourati", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, Q4_11(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                16, 4, 11, "-16", "15.999512", "0.000488");
        }
    }
}

void run_all_datasets_q8_7_and_export_csv(std::ofstream& csv_file) {
    const char* format_name = "Q8.7";
    const char* datasets[] = {
        "../../datasets/state-est/tuned_icm42688_1khz_marg_dataset.txt",
        "../../datasets/state-est/tuned_icm42688_1khz_imu_dataset.txt", 
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1637steering_L0R182__1k_800hz__sensor1_imu_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_marg_dataset.txt",
        "../../datasets/state-est/attitude/gamma-bot-imu-data-iiswc25_-_1647straig_L182R127__1k_800hz__sensor1_imu_dataset.txt"
    };
    
    const char* dataset_names[] = {
        "RobobeeHover-1kHz-MARG",
        "RobobeeHover-1kHz-IMU",
        "GammaBotSteering-MARG",
        "GammaBotSteering-IMU",
        "GammaBotStraight-MARG",
        "GammaBotStraight-IMU"
    };

    for (int d = 0; d < 6; d++) {
        // Mahony MARG
        {
            MahonyQ8_7_MARG kernel;
            auto result = run_error_benchmark<MahonyQ8_7_MARGProblem>("Mahony", format_name, "MARG", 
                                                                     dataset_names[d], datasets[d], kernel, Q8_7(1.0f), Q8_7(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "MARG", dataset_names[d], 
                                16, 8, 7, "-256", "255.992188", "0.007812");
        }

        // Mahony IMU
        {
            MahonyQ8_7_IMU kernel;
            auto result = run_error_benchmark<MahonyQ8_7_IMUProblem>("Mahony", format_name, "IMU", 
                                                                    dataset_names[d], datasets[d], kernel, Q8_7(1.0f), Q8_7(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Mahony", "IMU", dataset_names[d], 
                                16, 8, 7, "-256", "255.992188", "0.007812");
        }

        // Madgwick MARG
        {
            MadgwickQ8_7_MARG kernel;
            auto result = run_error_benchmark<MadgwickQ8_7_MARGProblem>("Madgwick", format_name, "MARG", 
                                                                       dataset_names[d], datasets[d], kernel, Q8_7(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "MARG", dataset_names[d], 
                                16, 8, 7, "-256", "255.992188", "0.007812");
        }

        // Madgwick IMU
        {
            MadgwickQ8_7_IMU kernel;
            auto result = run_error_benchmark<MadgwickQ8_7_IMUProblem>("Madgwick", format_name, "IMU", 
                                                                      dataset_names[d], datasets[d], kernel, Q8_7(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Madgwick", "IMU", dataset_names[d], 
                                16, 8, 7, "-256", "255.992188", "0.007812");
        }

        // Fourati MARG only
        {
            FouratiQ8_7_MARG kernel;
            auto result = run_error_benchmark<FouratiQ8_7_MARGProblem>("Fourati", format_name, "MARG", 
                                                                      dataset_names[d], datasets[d], kernel, Q8_7(0.1f));
            export_result_to_csv(csv_file, result, format_name, "Fourati", "MARG", dataset_names[d], 
                                16, 8, 7, "-256", "255.992188", "0.007812");
        }
    }
}

// Enhanced CSV export function
void export_result_to_csv(std::ofstream& csv_file, const BenchmarkResult& result, 
                         const char* format_name, const char* filter_name, const char* sensor_type,
                         const char* dataset_name, int total_bits, int integer_bits, int fractional_bits,
                         const char* range_min, const char* range_max, const char* resolution) {
    csv_file << format_name << ","
             << filter_name << ","
             << sensor_type << ","
             << dataset_name << ","
             << result.success_rate << ","
             << result.total_samples << ","
             << result.failed_samples << ","
             << result.mean_error_deg << ","
             << result.std_error_deg << ","
             << result.max_error_deg << ","
             << integer_bits << ","
             << fractional_bits << ","
             << total_bits << ","
             << range_min << ","
             << range_max << ","
             << resolution << "\n";
}

// =============================================================================
// Test Entry Point
// =============================================================================

int main() {
    ENTO_INFO("Starting Attitude Filter Error Benchmark Suite...");
    
    try {
        // Run precision analysis first
        analyze_fixed_point_precision();
        
        // Run the main benchmark test
        test_attitude_filter_benchmark();
        
        // Run gamma-bot dataset benchmark
        test_gamma_bot_datasets();
        
        // Run refined Q format coverage analysis
        test_refined_q_format_coverage();
        
        ENTO_INFO("Benchmark suite completed successfully!");
        return 0;
    } catch (const std::exception& e) {
        ENTO_INFO("Benchmark suite failed with exception: %s", e.what());
        return 1;
    }
} 