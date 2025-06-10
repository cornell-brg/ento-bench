#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <cstring>

#include <ento-util/containers.h>
#include <ento-util/debug.h>
#include <ento-util/random.h>
#include <ento-math/core.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/data_gen.h>
#include <ento-pose/prob_gen.h>


// Include the solver interfaces from data_gen.h
#include <ento-pose/data_gen.h>

using namespace EntoPose;
using namespace EntoUtil;

// Command line options structure
struct Options {
    int num_problems = 100;
    int num_points = 100;
    bool generate_8pt = true;
    bool generate_5pt = true;
    bool generate_upright_3pt = true;
    bool generate_upright_planar_3pt = true;
    bool generate_upright_planar_2pt = true;
    bool use_double = false;
    std::vector<double> noise_levels = {0.0, 0.01}; // Default noise levels
    bool use_realistic_generation = false; // NEW: Enable realistic data generation
    double pixel_noise_std = 0.1;         // NEW: Pixel noise standard deviation
    double focal_length = 500.0;          // NEW: Camera focal length
    double baseline_magnitude = 1.0;      // NEW: Fixed baseline magnitude
    double rotation_magnitude_deg = 16.0; // NEW: Controlled rotation magnitude
    
    void print_help() {
        std::cout << "Usage: generate_minimal_solver_data_rel_enhanced [options]\n";
        std::cout << "Options:\n";
        std::cout << "  --problems N          Number of problems to generate (default: 100)\n";
        std::cout << "  --points N            Number of points per problem (default: 100)\n";
        std::cout << "  --noise-levels \"x,y,z\" Comma-separated noise levels (default: \"0.0,0.01\")\n";
        std::cout << "  --8pt-only            Generate only 8pt data\n";
        std::cout << "  --5pt-only            Generate only 5pt data\n";
        std::cout << "  --upright-3pt-only    Generate only upright 3pt data\n";
        std::cout << "  --upright-planar-3pt-only Generate only upright planar 3pt data\n";
        std::cout << "  --upright-planar-2pt-only Generate only upright planar 2pt data\n";
        std::cout << "  --all-types           Run all solver types\n";
        std::cout << "  --double              Use double precision (default: float)\n";
        std::cout << "  --realistic           Use realistic data generation (paper methodology)\n";
        std::cout << "  --pixel-noise N       Pixel noise std dev for realistic mode (default: 0.1)\n";
        std::cout << "  --focal-length N      Camera focal length for realistic mode (default: 500)\n";
        std::cout << "  --baseline N          Fixed baseline magnitude (default: 1.0)\n";
        std::cout << "  --rotation-deg N      Rotation magnitude in degrees (default: 16.0)\n";
        std::cout << "  --help                Show this help message\n";
        std::cout << "\nExamples:\n";
        std::cout << "  # Use realistic data generation with 0.1 pixel noise:\n";
        std::cout << "  ./generate_minimal_solver_data_rel_enhanced --realistic --pixel-noise 0.1\n";
        std::cout << "  # Compare realistic vs traditional with custom noise levels:\n";
        std::cout << "  ./generate_minimal_solver_data_rel_enhanced --noise-levels \"0.0,0.05,0.1,0.2\"\n";
        std::cout << "  # Test 5pt solver with realistic 8° and 16° rotations:\n";
        std::cout << "  ./generate_minimal_solver_data_rel_enhanced --5pt-only --realistic --rotation-deg 8\n";
    }
};

Options parse_args(int argc, char** argv) {
    Options opts;
    
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            opts.print_help();
            exit(0);
        } else if (strcmp(argv[i], "--problems") == 0 && i + 1 < argc) {
            opts.num_problems = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--points") == 0 && i + 1 < argc) {
            opts.num_points = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--noise-levels") == 0 && i + 1 < argc) {
            std::string noise_str = argv[++i];
            opts.noise_levels.clear();
            std::stringstream ss(noise_str);
            std::string item;
            while (std::getline(ss, item, ',')) {
                opts.noise_levels.push_back(std::stod(item));
            }
        } else if (strcmp(argv[i], "--8pt-only") == 0) {
            opts.generate_8pt = true;
            opts.generate_5pt = false;
            opts.generate_upright_3pt = false;
            opts.generate_upright_planar_3pt = false;
            opts.generate_upright_planar_2pt = false;
        } else if (strcmp(argv[i], "--5pt-only") == 0) {
            opts.generate_8pt = false;
            opts.generate_5pt = true;
            opts.generate_upright_3pt = false;
            opts.generate_upright_planar_3pt = false;
            opts.generate_upright_planar_2pt = false;
        } else if (strcmp(argv[i], "--upright-3pt-only") == 0) {
            opts.generate_8pt = false;
            opts.generate_5pt = false;
            opts.generate_upright_3pt = true;
            opts.generate_upright_planar_3pt = false;
            opts.generate_upright_planar_2pt = false;
        } else if (strcmp(argv[i], "--upright-planar-3pt-only") == 0) {
            opts.generate_8pt = false;
            opts.generate_5pt = false;
            opts.generate_upright_3pt = false;
            opts.generate_upright_planar_3pt = true;
            opts.generate_upright_planar_2pt = false;
        } else if (strcmp(argv[i], "--upright-planar-2pt-only") == 0) {
            opts.generate_8pt = false;
            opts.generate_5pt = false;
            opts.generate_upright_3pt = false;
            opts.generate_upright_planar_3pt = false;
            opts.generate_upright_planar_2pt = true;
        } else if (strcmp(argv[i], "--all-types") == 0) {
            opts.generate_8pt = true;
            opts.generate_5pt = true;
            opts.generate_upright_3pt = true;
            opts.generate_upright_planar_3pt = true;
            opts.generate_upright_planar_2pt = true;
        } else if (strcmp(argv[i], "--double") == 0) {
            opts.use_double = true;
        } else if (strcmp(argv[i], "--realistic") == 0) {
            opts.use_realistic_generation = true;
        } else if (strcmp(argv[i], "--pixel-noise") == 0 && i + 1 < argc) {
            opts.pixel_noise_std = std::stod(argv[++i]);
        } else if (strcmp(argv[i], "--focal-length") == 0 && i + 1 < argc) {
            opts.focal_length = std::stod(argv[++i]);
        } else if (strcmp(argv[i], "--baseline") == 0 && i + 1 < argc) {
            opts.baseline_magnitude = std::stod(argv[++i]);
        } else if (strcmp(argv[i], "--rotation-deg") == 0 && i + 1 < argc) {
            opts.rotation_magnitude_deg = std::stod(argv[++i]);
        } else {
            std::cout << "Unknown option: " << argv[i] << std::endl;
            opts.print_help();
            exit(1);
        }
    }
    
    // Validation
    if (opts.num_problems <= 0 || opts.num_points <= 0) {
        std::cout << "Error: Number of problems and points must be positive\n";
        exit(1);
    }
    
    return opts;
}

// Enhanced pose error computation with separate rotation and translation tracking
template<typename Scalar>
struct PoseErrors {
    Scalar rotation_error_deg;
    Scalar translation_error_deg;    // Translation direction error (degrees)
    Scalar reprojection_error;       // Sampson distance to fundamental matrix
    
    // Default constructor for no-solution cases
    PoseErrors() : rotation_error_deg(std::numeric_limits<Scalar>::max()),
                   translation_error_deg(std::numeric_limits<Scalar>::max()),
                   reprojection_error(std::numeric_limits<Scalar>::max()) {}
    
    // Constructor with values
    PoseErrors(Scalar rot_err, Scalar trans_err, Scalar reproj_err)
        : rotation_error_deg(rot_err), translation_error_deg(trans_err), reprojection_error(reproj_err) {}
    
    Scalar max_error() const {
        return std::max(rotation_error_deg, translation_error_deg);
    }
};

// Compute Sampson distance (reprojection error for relative pose)
template<typename Scalar, size_t N>
Scalar compute_sampson_distance(
    const CameraPose<Scalar>& pose,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2)
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Mat3 = Eigen::Matrix<Scalar,3,3>;
    
    // Build fundamental matrix from pose: F = [t]_× * R
    Mat3 t_skew;
    t_skew << 0, -pose.t(2), pose.t(1),
              pose.t(2), 0, -pose.t(0),
              -pose.t(1), pose.t(0), 0;
    Mat3 F = t_skew * pose.R();
    
    Scalar total_error = 0.0;
    size_t valid_points = 0;
    
    for (size_t i = 0; i < x1.size(); ++i) {
        const Vec3& p1 = x1[i];
        const Vec3& p2 = x2[i];
        
        // Sampson distance: (p2^T F p1)^2 / (||F^T p2||^2 + ||F p1||^2)
        Scalar numerator = p2.transpose() * F * p1;
        numerator = numerator * numerator;
        
        Vec3 Fp1 = F * p1;
        Vec3 FTp2 = F.transpose() * p2;
        Scalar denominator = Fp1.squaredNorm() + FTp2.squaredNorm();
        
        if (denominator > Scalar(1e-12)) {
            Scalar sampson_dist = numerator / denominator;
            total_error += sampson_dist;
            valid_points++;
        }
    }
    
    if (valid_points == 0) return std::numeric_limits<Scalar>::max();
    
    // Return RMS Sampson distance
    return std::sqrt(total_error / valid_points);
}

template<typename Scalar, size_t N>
PoseErrors<Scalar> compute_pose_errors(
    const CameraPose<Scalar>& true_pose, 
    const CameraPose<Scalar>& estimated_pose,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2)
{
    PoseErrors<Scalar> errors;
    
    // FIXED: Always use double precision for error computation to get true accuracy metrics
    // Cast poses to double precision for error computation
    CameraPose<double> true_pose_double;
    true_pose_double.q(0) = static_cast<double>(true_pose.q(0));
    true_pose_double.q(1) = static_cast<double>(true_pose.q(1));
    true_pose_double.q(2) = static_cast<double>(true_pose.q(2));
    true_pose_double.q(3) = static_cast<double>(true_pose.q(3));
    true_pose_double.t(0) = static_cast<double>(true_pose.t(0));
    true_pose_double.t(1) = static_cast<double>(true_pose.t(1));
    true_pose_double.t(2) = static_cast<double>(true_pose.t(2));
    
    CameraPose<double> estimated_pose_double;
    estimated_pose_double.q(0) = static_cast<double>(estimated_pose.q(0));
    estimated_pose_double.q(1) = static_cast<double>(estimated_pose.q(1));
    estimated_pose_double.q(2) = static_cast<double>(estimated_pose.q(2));
    estimated_pose_double.q(3) = static_cast<double>(estimated_pose.q(3));
    estimated_pose_double.t(0) = static_cast<double>(estimated_pose.t(0));
    estimated_pose_double.t(1) = static_cast<double>(estimated_pose.t(1));
    estimated_pose_double.t(2) = static_cast<double>(estimated_pose.t(2));
    
    // Rotation error (angular difference between rotation matrices) - ALWAYS use double precision
    double trace_val = (estimated_pose_double.R().transpose() * true_pose_double.R()).trace();
    double angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
    errors.rotation_error_deg = static_cast<Scalar>(angle_rad * 180.0 / M_PI);
    
    // Translation direction error (angular difference between translation directions) - ALWAYS use double precision
    Eigen::Matrix<double,3,1> t_est = estimated_pose_double.t.normalized();
    Eigen::Matrix<double,3,1> t_true = true_pose_double.t.normalized();
    
    // Handle both +t and -t directions (relative pose translation is up to scale)
    double dot_pos = std::clamp(t_est.dot(t_true), -1.0, 1.0);
    double dot_neg = std::clamp(t_est.dot(-t_true), -1.0, 1.0);
    double abs_dot = std::max(std::abs(dot_pos), std::abs(dot_neg));
    errors.translation_error_deg = static_cast<Scalar>(std::acos(abs_dot) * 180.0 / M_PI);
    
    // Reprojection error (Sampson distance) - cast bearing vectors to double for accurate computation
    EntoUtil::EntoContainer<Eigen::Matrix<double,3,1>, N> x1_double, x2_double;
    for (size_t i = 0; i < x1.size(); ++i) {
        Eigen::Matrix<double,3,1> x1_d(static_cast<double>(x1[i](0)), 
                                       static_cast<double>(x1[i](1)), 
                                       static_cast<double>(x1[i](2)));
        Eigen::Matrix<double,3,1> x2_d(static_cast<double>(x2[i](0)), 
                                       static_cast<double>(x2[i](1)), 
                                       static_cast<double>(x2[i](2)));
        x1_double.push_back(x1_d);
        x2_double.push_back(x2_d);
    }
    
    double reprojection_error_double = compute_sampson_distance<double, N>(estimated_pose_double, x1_double, x2_double);
    errors.reprojection_error = static_cast<Scalar>(reprojection_error_double);
    
    return errors;
}

// Enhanced statistics tracking with separate rotation/translation errors and standard deviations
template<typename Scalar>
struct SolverStats {
    int total_problems = 0;
    int successful_solves = 0;
    int failed_solves = 0;
    
    // Error tracking - SUCCESSFUL CASES ONLY (for threshold-based analysis)
    std::vector<Scalar> successful_rotation_errors;
    std::vector<Scalar> successful_translation_errors;
    std::vector<Scalar> successful_reprojection_errors;
    std::vector<Scalar> successful_combined_errors;
    
    // Error tracking - ALL SOLUTIONS (regardless of success threshold)
    std::vector<Scalar> all_rotation_errors;
    std::vector<Scalar> all_translation_errors;
    std::vector<Scalar> all_reprojection_errors;
    std::vector<Scalar> all_combined_errors;
    
    // Solution tracking
    int total_solutions_found = 0;
    int problems_with_multiple_solutions = 0;
    
    void add_success(const PoseErrors<Scalar>& errors, int num_solutions) {
        successful_solves++;
        
        // Add to successful-only statistics
        successful_rotation_errors.push_back(errors.rotation_error_deg);
        successful_translation_errors.push_back(errors.translation_error_deg);
        successful_reprojection_errors.push_back(errors.reprojection_error);
        successful_combined_errors.push_back(errors.max_error());
        
        // Also add to all-solutions statistics
        all_rotation_errors.push_back(errors.rotation_error_deg);
        all_translation_errors.push_back(errors.translation_error_deg);
        all_reprojection_errors.push_back(errors.reprojection_error);
        all_combined_errors.push_back(errors.max_error());
        
        total_solutions_found += num_solutions;
        if (num_solutions > 1) {
            problems_with_multiple_solutions++;
        }
    }
    
    void add_failure() {
        failed_solves++;
    }
    
    void add_failure_with_solution(const PoseErrors<Scalar>& errors) {
        failed_solves++;
        
        // Failed cases still go into all-solutions statistics
        all_rotation_errors.push_back(errors.rotation_error_deg);
        all_translation_errors.push_back(errors.translation_error_deg);
        all_reprojection_errors.push_back(errors.reprojection_error);
        all_combined_errors.push_back(errors.max_error());
    }
    
    void add_problem() {
        total_problems++;
    }
    
    // Traditional statistics
    Scalar mean(const std::vector<Scalar>& values) const {
        if (values.empty()) return Scalar(0);
        return std::accumulate(values.begin(), values.end(), Scalar(0)) / values.size();
    }
    
    Scalar stddev(const std::vector<Scalar>& values) const {
        if (values.size() < 2) return Scalar(0);
        Scalar m = mean(values);
        Scalar sum_sq_diff = 0;
        for (const auto& val : values) {
            Scalar diff = val - m;
            sum_sq_diff += diff * diff;
        }
        return std::sqrt(sum_sq_diff / (values.size() - 1));
    }
    
    // NEW: Robust statistics
    Scalar median(const std::vector<Scalar>& values) const {
        if (values.empty()) return Scalar(0);
        
        std::vector<Scalar> sorted_values = values;
        std::sort(sorted_values.begin(), sorted_values.end());
        
        size_t n = sorted_values.size();
        if (n % 2 == 0) {
            return (sorted_values[n/2-1] + sorted_values[n/2]) / Scalar(2);
        } else {
            return sorted_values[n/2];
        }
    }
    
    Scalar percentile(const std::vector<Scalar>& values, Scalar p) const {
        if (values.empty()) return Scalar(0);
        
        std::vector<Scalar> sorted_values = values;
        std::sort(sorted_values.begin(), sorted_values.end());
        
        size_t n = sorted_values.size();
        Scalar index = p * (n - 1);
        size_t lower = static_cast<size_t>(std::floor(index));
        size_t upper = static_cast<size_t>(std::ceil(index));
        
        if (lower == upper) {
            return sorted_values[lower];
        } else {
            Scalar weight = index - lower;
            return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight;
        }
    }
    
    Scalar q1(const std::vector<Scalar>& values) const { return percentile(values, 0.25); }
    Scalar q3(const std::vector<Scalar>& values) const { return percentile(values, 0.75); }
    Scalar iqr(const std::vector<Scalar>& values) const { return q3(values) - q1(values); }
    
    // NEW: Additional percentiles for better distribution understanding
    Scalar p5(const std::vector<Scalar>& values) const { return percentile(values, 0.05); }
    Scalar p10(const std::vector<Scalar>& values) const { return percentile(values, 0.10); }
    Scalar p90(const std::vector<Scalar>& values) const { return percentile(values, 0.90); }
    Scalar p95(const std::vector<Scalar>& values) const { return percentile(values, 0.95); }
    
    // Helper function to count zeros
    int count_zeros(const std::vector<Scalar>& values) const {
        return std::count_if(values.begin(), values.end(), [](Scalar val) { return std::abs(val) < 1e-10; });
    }
    
    void print_stats(const std::string& solver_name, double noise_level) const {
        std::cout << "\n=== " << solver_name << " Statistics (noise=" << noise_level << ") ===" << std::endl;
        std::cout << "  Total problems: " << total_problems << std::endl;
        std::cout << "  Successful solves: " << successful_solves << " (" 
                  << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
        std::cout << "  Failed solves: " << failed_solves << " (" 
                  << (100.0 * failed_solves / total_problems) << "%)" << std::endl;
        
        if (successful_solves > 0) {
            std::cout << "  \n=== SUCCESSFUL CASES ONLY (threshold-based) ===" << std::endl;
            
            // Rotation Error Statistics
            std::cout << "  Rotation Error Statistics:" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(successful_rotation_errors) << " ± " << stddev(successful_rotation_errors) << " degrees" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(successful_rotation_errors) 
                      << " [" << q1(successful_rotation_errors) << ", " << q3(successful_rotation_errors) << "] degrees" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(successful_rotation_errors) << ", P10=" << p10(successful_rotation_errors) 
                      << ", P90=" << p90(successful_rotation_errors) << ", P95=" << p95(successful_rotation_errors) << " degrees" << std::endl;
            std::cout << "    Range: [" << *std::min_element(successful_rotation_errors.begin(), successful_rotation_errors.end()) 
                      << ", " << *std::max_element(successful_rotation_errors.begin(), successful_rotation_errors.end()) << "] degrees" << std::endl;
            std::cout << "    Zero values: " << count_zeros(successful_rotation_errors) << "/" << successful_rotation_errors.size() 
                      << " (" << (100.0 * count_zeros(successful_rotation_errors) / successful_rotation_errors.size()) << "%)" << std::endl;
            
            // Translation Direction Error Statistics
            std::cout << "  \nTranslation Direction Error Statistics:" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(successful_translation_errors) << " ± " << stddev(successful_translation_errors) << " degrees" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(successful_translation_errors) 
                      << " [" << q1(successful_translation_errors) << ", " << q3(successful_translation_errors) << "] degrees" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(successful_translation_errors) << ", P10=" << p10(successful_translation_errors) 
                      << ", P90=" << p90(successful_translation_errors) << ", P95=" << p95(successful_translation_errors) << " degrees" << std::endl;
            std::cout << "    Range: [" << *std::min_element(successful_translation_errors.begin(), successful_translation_errors.end()) 
                      << ", " << *std::max_element(successful_translation_errors.begin(), successful_translation_errors.end()) << "] degrees" << std::endl;
            std::cout << "    Zero values: " << count_zeros(successful_translation_errors) << "/" << successful_translation_errors.size() 
                      << " (" << (100.0 * count_zeros(successful_translation_errors) / successful_translation_errors.size()) << "%)" << std::endl;
            
            // Reprojection Error Statistics
            std::cout << "  \nReprojection Error Statistics (Sampson distance):" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(successful_reprojection_errors) << " ± " << stddev(successful_reprojection_errors) << " (normalized)" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(successful_reprojection_errors) 
                      << " [" << q1(successful_reprojection_errors) << ", " << q3(successful_reprojection_errors) << "] (normalized)" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(successful_reprojection_errors) << ", P10=" << p10(successful_reprojection_errors) 
                      << ", P90=" << p90(successful_reprojection_errors) << ", P95=" << p95(successful_reprojection_errors) << " (normalized)" << std::endl;
            std::cout << "    Range: [" << *std::min_element(successful_reprojection_errors.begin(), successful_reprojection_errors.end()) 
                      << ", " << *std::max_element(successful_reprojection_errors.begin(), successful_reprojection_errors.end()) << "] (normalized)" << std::endl;
            
            // Combined Error Statistics
            std::cout << "  \nCombined Error Statistics:" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(successful_combined_errors) << " ± " << stddev(successful_combined_errors) << " degrees (max of rot and trans direction)" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(successful_combined_errors) 
                      << " [" << q1(successful_combined_errors) << ", " << q3(successful_combined_errors) << "] degrees" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(successful_combined_errors) << ", P10=" << p10(successful_combined_errors) 
                      << ", P90=" << p90(successful_combined_errors) << ", P95=" << p95(successful_combined_errors) << " degrees" << std::endl;
            std::cout << "    Range: [" << *std::min_element(successful_combined_errors.begin(), successful_combined_errors.end()) 
                      << ", " << *std::max_element(successful_combined_errors.begin(), successful_combined_errors.end()) << "] degrees" << std::endl;
            std::cout << "    Zero values: " << count_zeros(successful_combined_errors) << "/" << successful_combined_errors.size() 
                      << " (" << (100.0 * count_zeros(successful_combined_errors) / successful_combined_errors.size()) << "%)" << std::endl;
            
            std::cout << "  \nSolution Statistics:" << std::endl;
            std::cout << "    Average solutions per problem: " 
                      << (float(total_solutions_found) / successful_solves) << std::endl;
            std::cout << "    Problems with multiple solutions: " << problems_with_multiple_solutions 
                      << " (" << (100.0 * problems_with_multiple_solutions / successful_solves) << "%)" << std::endl;
        }
        
        // NEW: All-solutions statistics (complete performance picture)
        if (!all_rotation_errors.empty()) {
            std::cout << "  \n=== ALL SOLUTIONS (complete performance) ===" << std::endl;
            
            // Rotation Error Statistics
            std::cout << "  Rotation Error Statistics:" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(all_rotation_errors) << " ± " << stddev(all_rotation_errors) << " degrees" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(all_rotation_errors) 
                      << " [" << q1(all_rotation_errors) << ", " << q3(all_rotation_errors) << "] degrees" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(all_rotation_errors) << ", P10=" << p10(all_rotation_errors) 
                      << ", P90=" << p90(all_rotation_errors) << ", P95=" << p95(all_rotation_errors) << " degrees" << std::endl;
            std::cout << "    Range: [" << *std::min_element(all_rotation_errors.begin(), all_rotation_errors.end()) 
                      << ", " << *std::max_element(all_rotation_errors.begin(), all_rotation_errors.end()) << "] degrees" << std::endl;
            std::cout << "    Zero values: " << count_zeros(all_rotation_errors) << "/" << all_rotation_errors.size() 
                      << " (" << (100.0 * count_zeros(all_rotation_errors) / all_rotation_errors.size()) << "%)" << std::endl;
            
            // Translation Direction Error Statistics
            std::cout << "  \nTranslation Direction Error Statistics:" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(all_translation_errors) << " ± " << stddev(all_translation_errors) << " degrees" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(all_translation_errors) 
                      << " [" << q1(all_translation_errors) << ", " << q3(all_translation_errors) << "] degrees" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(all_translation_errors) << ", P10=" << p10(all_translation_errors) 
                      << ", P90=" << p90(all_translation_errors) << ", P95=" << p95(all_translation_errors) << " degrees" << std::endl;
            std::cout << "    Range: [" << *std::min_element(all_translation_errors.begin(), all_translation_errors.end()) 
                      << ", " << *std::max_element(all_translation_errors.begin(), all_translation_errors.end()) << "] degrees" << std::endl;
            std::cout << "    Zero values: " << count_zeros(all_translation_errors) << "/" << all_translation_errors.size() 
                      << " (" << (100.0 * count_zeros(all_translation_errors) / all_translation_errors.size()) << "%)" << std::endl;
            
            // Reprojection Error Statistics
            std::cout << "  \nReprojection Error Statistics (Sampson distance):" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(all_reprojection_errors) << " ± " << stddev(all_reprojection_errors) << " (normalized)" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(all_reprojection_errors) 
                      << " [" << q1(all_reprojection_errors) << ", " << q3(all_reprojection_errors) << "] (normalized)" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(all_reprojection_errors) << ", P10=" << p10(all_reprojection_errors) 
                      << ", P90=" << p90(all_reprojection_errors) << ", P95=" << p95(all_reprojection_errors) << " (normalized)" << std::endl;
            std::cout << "    Range: [" << *std::min_element(all_reprojection_errors.begin(), all_reprojection_errors.end()) 
                      << ", " << *std::max_element(all_reprojection_errors.begin(), all_reprojection_errors.end()) << "] (normalized)" << std::endl;
            
            // Combined Error Statistics
            std::cout << "  \nCombined Error Statistics:" << std::endl;
            std::cout << "    Traditional: Mean ± Std Dev: " << mean(all_combined_errors) << " ± " << stddev(all_combined_errors) << " degrees (max of rot and trans direction)" << std::endl;
            std::cout << "    Robust: Median [Q1, Q3]: " << median(all_combined_errors) 
                      << " [" << q1(all_combined_errors) << ", " << q3(all_combined_errors) << "] degrees" << std::endl;
            std::cout << "    Percentiles: P5=" << p5(all_combined_errors) << ", P10=" << p10(all_combined_errors) 
                      << ", P90=" << p90(all_combined_errors) << ", P95=" << p95(all_combined_errors) << " degrees" << std::endl;
            std::cout << "    Range: [" << *std::min_element(all_combined_errors.begin(), all_combined_errors.end()) 
                      << ", " << *std::max_element(all_combined_errors.begin(), all_combined_errors.end()) << "] degrees" << std::endl;
            std::cout << "    Zero values: " << count_zeros(all_combined_errors) << "/" << all_combined_errors.size() 
                      << " (" << (100.0 * count_zeros(all_combined_errors) / all_combined_errors.size()) << "%)" << std::endl;
        }
        std::cout << "================================" << std::endl;
    }
};

// CSV line generation function
template <typename Scalar, size_t N>
std::string make_csv_line_relative_pose(
    const CameraPose<Scalar>& pose,
    const EntoContainer<Vec3<Scalar>, N>& x1,
    const EntoContainer<Vec3<Scalar>, N>& x2,
    Scalar scale = Scalar{1},
    Scalar focal = Scalar{1})
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    std::ostringstream oss;

    // 1) problem_type (3 for relative pose), N
    oss << 3 << ',' << N << ',';

    // 2) quaternion (w, x, y, z)
    oss << pose.q.w() << ','
        << pose.q.x() << ','
        << pose.q.y() << ','
        << pose.q.z() << ',';

    // 3) translation (tx, ty, tz)
    oss << pose.t.x() << ','
        << pose.t.y() << ','
        << pose.t.z() << ',';

    // 4) camera parameters (scale, focal)
    oss << scale << ',' << focal << ',';

    // 5) x1: dump N bearing vectors (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3& pt = x1[i];
        oss << pt(0) << ',' << pt(1) << ',' << pt(2) << ',';
    }

    // 6) x2: dump N bearing vectors (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3& pt = x2[i];
        oss << pt(0) << ',' << pt(1) << ',' << pt(2);
        if (i < N - 1) oss << ',';
    }

    return oss.str();
}

// Generate unified relative pose data with realistic camera model and degeneracy checks
// N can be 0 (dynamic) or >0 (fixed size)
// noise_level: standard deviation of Gaussian pixel noise added to image coordinates
template<typename Scalar, size_t N>
void generate_unified_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, noise_level);  // PIXEL noise, not radians!
    
    // Add proper nested retry logic like abs-pose
    int max_pose_attempts = 100;  // Try different poses
    int max_point_attempts = 1000; // Try generating points for each pose
    
    for (int pose_attempt = 0; pose_attempt < max_pose_attempts; ++pose_attempt) {
        // FIXED: Generate controlled realistic poses (not pathological random poses)
        if (upright_only) {
            // Controlled yaw rotation like realistic mode (not random 0-360°)
            Scalar yaw_deg = 15.0 * coord_gen(rng);  // ±16° controlled rotation
            Scalar yaw_rad = yaw_deg * M_PI / 180.0;
            Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
            true_pose.q(0) = q.w();
            true_pose.q(1) = q.x();
            true_pose.q(2) = q.y();
            true_pose.q(3) = q.z();
            
            if (planar_only) {
                // Controlled baseline in XZ plane with fixed magnitude
                Scalar tx = coord_gen(rng);  // ±1 normalized direction
                Scalar tz = coord_gen(rng);
                Vec3 t_dir = Vec3(tx, 0, tz).normalized();
                true_pose.t = t_dir;  // Fixed baseline magnitude = 1.0
            } else {
                // Controlled 3D translation with fixed magnitude
                Vec3 t_dir = Vec3::Random().normalized();
                true_pose.t = t_dir;  // Fixed baseline magnitude = 1.0
            }
        } else {
            // FIXED: Controlled 6-DoF pose (not completely random)
            // Rotation: controlled magnitude around random axis (like realistic mode)
            Vec3 rot_axis = Vec3::Random().normalized();
            Scalar rot_angle_deg = 16.0 * coord_gen(rng);  // ±16° controlled rotation
            Scalar rot_angle_rad = rot_angle_deg * M_PI / 180.0;
            Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(rot_angle_rad, rot_axis));
            true_pose.q(0) = q.w();
            true_pose.q(1) = q.x();
            true_pose.q(2) = q.y();
            true_pose.q(3) = q.z();
            
            // Translation: fixed baseline magnitude (not random [0-4])
            Vec3 t_dir = Vec3::Random().normalized();
            true_pose.t = t_dir;  // Fixed baseline magnitude = 1.0
        }
        
        // Try to generate points with this pose using degeneracy checks
        for (int point_attempt = 0; point_attempt < max_point_attempts; ++point_attempt) {
            // Clear containers for this attempt
            if constexpr (N == 0) {
                x1_bear.clear();
                x2_bear.clear();
                x1_bear.reserve(num_points);
                x2_bear.reserve(num_points);
            } else {
                x1_bear.clear();
                x2_bear.clear();
            }
            
            // Temporary storage for degeneracy checks
            std::vector<Vec3> points3D_temp;
            std::vector<Vec2> points2D_cam1_temp, points2D_cam2_temp;
            points3D_temp.reserve(num_points);
            points2D_cam1_temp.reserve(num_points);
            points2D_cam2_temp.reserve(num_points);
            
            // Generate points
            size_t valid_points = 0;
            int safety_counter = 0;
            const int max_safety = 10000;
            
            while (valid_points < num_points && safety_counter < max_safety) {
                Vec3 X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
                Vec3 x1_cam = X;
                Vec3 x2_cam = true_pose.R() * X + true_pose.t;
                
                if (x1_cam(2) > Scalar(0.1) && x2_cam(2) > Scalar(0.1)) {
                    points3D_temp.push_back(X);
                    points2D_cam1_temp.push_back(Vec2(x1_cam(0) / x1_cam(2), x1_cam(1) / x1_cam(2)));
                    points2D_cam2_temp.push_back(Vec2(x2_cam(0) / x2_cam(2), x2_cam(1) / x2_cam(2)));
                    
                    // FIXED: Use proper image point projection and pixel noise (like abs-pose)
                    const Scalar focal_length = Scalar(500.0);
                    
                    // Project to image coordinates (pinhole camera model)
                    Vec2 x1_img(focal_length * x1_cam(0) / x1_cam(2), focal_length * x1_cam(1) / x1_cam(2));
                    Vec2 x2_img(focal_length * x2_cam(0) / x2_cam(2), focal_length * x2_cam(1) / x2_cam(2));
                    
                    // Add pixel noise (NOT tangent space noise!)
                    if (noise_level > Scalar(0.0)) {
                        std::normal_distribution<Scalar> pixel_noise_gen(0.0, noise_level * focal_length);
                        x1_img(0) += pixel_noise_gen(rng);
                        x1_img(1) += pixel_noise_gen(rng);
                        x2_img(0) += pixel_noise_gen(rng);
                        x2_img(1) += pixel_noise_gen(rng);
                    }
                    
                    // Convert back to bearing vectors
                    Vec3 f1 = Vec3(x1_img(0) / focal_length, x1_img(1) / focal_length, Scalar(1.0)).normalized();
                    Vec3 f2 = Vec3(x2_img(0) / focal_length, x2_img(1) / focal_length, Scalar(1.0)).normalized();
                    
                    // Degeneracy checks (like abs-pose)
                    Scalar dot_product = std::abs(f1.dot(f2));
                    if (dot_product > Scalar(0.999)) {  // Skip near-parallel rays
                        continue;
                    }
                    
                    x1_bear.push_back(f1);
                    x2_bear.push_back(f2);
                    ++valid_points;
                }
                safety_counter++;
            }
            
            if (safety_counter >= max_safety) continue; // Try again with same pose
            
            // DEGENERACY CHECKS
            bool is_degenerate = false;
            
            // Check 1: Collinearity test for 3D points
            if (num_points >= 3) {
                Vec3 v1 = points3D_temp[1] - points3D_temp[0];
                Vec3 v2 = points3D_temp[2] - points3D_temp[0];
                Vec3 cross = v1.cross(v2);
                if (cross.norm() < Scalar(1e-6)) {
                    is_degenerate = true;
                }
            }
            
            // Check 2: Coplanarity test for 4+ points
            if (num_points >= 4 && !is_degenerate) {
                Vec3 v1 = points3D_temp[1] - points3D_temp[0];
                Vec3 v2 = points3D_temp[2] - points3D_temp[0];
                Vec3 v3 = points3D_temp[3] - points3D_temp[0];
                Vec3 normal = v1.cross(v2);
                if (normal.norm() > Scalar(1e-8)) {
                    Scalar distance = std::abs(normal.dot(v3)) / normal.norm();
                    if (distance < Scalar(1e-4)) {
                        is_degenerate = true;
                    }
                }
            }
            
            // Check 3: Depth variation test
            if (!is_degenerate) {
                Scalar min_depth1 = std::numeric_limits<Scalar>::max();
                Scalar max_depth1 = std::numeric_limits<Scalar>::lowest();
                Scalar min_depth2 = std::numeric_limits<Scalar>::max();
                Scalar max_depth2 = std::numeric_limits<Scalar>::lowest();
                
                for (size_t i = 0; i < num_points; ++i) {
                    Vec3 x1_cam = points3D_temp[i];
                    Vec3 x2_cam = true_pose.R() * points3D_temp[i] + true_pose.t;
                    
                    min_depth1 = std::min(min_depth1, x1_cam(2));
                    max_depth1 = std::max(max_depth1, x1_cam(2));
                    min_depth2 = std::min(min_depth2, x2_cam(2));
                    max_depth2 = std::max(max_depth2, x2_cam(2));
                }
                
                Scalar depth_ratio1 = max_depth1 / min_depth1;
                Scalar depth_ratio2 = max_depth2 / min_depth2;
                if (depth_ratio1 < Scalar(1.5) || depth_ratio2 < Scalar(1.5)) {
                    is_degenerate = true;
                }
            }
            
            // Check 4: 2D point spread test
            if (!is_degenerate) {
                Scalar min_x1 = std::numeric_limits<Scalar>::max(), max_x1 = std::numeric_limits<Scalar>::lowest();
                Scalar min_y1 = std::numeric_limits<Scalar>::max(), max_y1 = std::numeric_limits<Scalar>::lowest();
                Scalar min_x2 = std::numeric_limits<Scalar>::max(), max_x2 = std::numeric_limits<Scalar>::lowest();
                Scalar min_y2 = std::numeric_limits<Scalar>::max(), max_y2 = std::numeric_limits<Scalar>::lowest();
                
                for (size_t i = 0; i < num_points; ++i) {
                    min_x1 = std::min(min_x1, points2D_cam1_temp[i](0));
                    max_x1 = std::max(max_x1, points2D_cam1_temp[i](0));
                    min_y1 = std::min(min_y1, points2D_cam1_temp[i](1));
                    max_y1 = std::max(max_y1, points2D_cam1_temp[i](1));
                    
                    min_x2 = std::min(min_x2, points2D_cam2_temp[i](0));
                    max_x2 = std::max(max_x2, points2D_cam2_temp[i](0));
                    min_y2 = std::min(min_y2, points2D_cam2_temp[i](1));
                    max_y2 = std::max(max_y2, points2D_cam2_temp[i](1));
                }
                
                Scalar x_spread1 = max_x1 - min_x1, y_spread1 = max_y1 - min_y1;
                Scalar x_spread2 = max_x2 - min_x2, y_spread2 = max_y2 - min_y2;
                
                if (x_spread1 < Scalar(0.5) || y_spread1 < Scalar(0.5) || 
                    x_spread2 < Scalar(0.5) || y_spread2 < Scalar(0.5)) {
                    is_degenerate = true;
                }
            }
            
            // Check 5: Baseline and rotation magnitude
            if (!is_degenerate) {
                Scalar baseline_length = true_pose.t.norm();
                if (baseline_length < Scalar(0.1)) {
                    is_degenerate = true;
                }
                
                Scalar trace_val = true_pose.R().trace();
                Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
                Scalar angle_deg = angle_rad * 180.0 / M_PI;
                if (angle_deg < Scalar(2.0)) {
                    is_degenerate = true;
                }
            }
            
            if (!is_degenerate) {
                return; // SUCCESS! We found good data
            }
        } // end point_attempts loop
    } // end pose_attempts loop
    
    // If we get here, we couldn't generate a good configuration - REJECT this experiment
    std::cerr << "ERROR: Could not generate well-conditioned relative pose data after " << max_pose_attempts 
              << " pose attempts for problem " << problem_id << ". This experiment should be discarded!" << std::endl;
    throw std::runtime_error("Failed to generate valid relative pose data - experiment should be discarded");
}

// Realistic data generation based on research paper methodology
template<typename Scalar, size_t N>
void generate_realistic_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar pixel_noise_std = Scalar(0.1),  // Standard deviation in pixels
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false,
    Scalar focal_length = Scalar(500.0),    // Realistic focal length
    Scalar baseline_magnitude = Scalar(1.0), // Fixed baseline magnitude
    Scalar rotation_magnitude_deg = Scalar(16.0)) // Controlled rotation magnitude
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> uniform_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(2.0, 10.0); // Realistic depth range
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, pixel_noise_std);
    
    // Generate controlled pose (similar to paper methodology)
    if (upright_only) {
        // ROBOTICS: More realistic yaw rotation variation
        // Mix small challenging rotations with larger easier ones
        std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
        Scalar rot_selector = rot_scenario(rng);
        
        Scalar yaw_deg;
        if (rot_selector < 0.3) {
            // Small challenging rotations: ±2°
            std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
            yaw_deg = small_rot(rng);
        } else if (rot_selector < 0.7) {
            // Medium typical rotations: ±10°  
            std::uniform_real_distribution<Scalar> medium_rot(-10.0, 10.0);
            yaw_deg = medium_rot(rng);
        } else {
            // Large easier rotations: ±45°
            std::uniform_real_distribution<Scalar> large_rot(-45.0, 45.0);
            yaw_deg = large_rot(rng);
        }
        
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        if (planar_only) {
            // ROBOTICS: Realistic baseline variation for planar motion
            // Generate baselines from 5cm to 5m to cover robotics scenarios:
            // - 5-20cm: Close-up manipulation, very challenging
            // - 20cm-1m: Typical robot arm/mobile robot motion  
            // - 1-5m: Larger robot motion, easier for algorithms
            std::uniform_real_distribution<Scalar> baseline_gen(0.05, 5.0);  // 5cm to 5m
            Scalar baseline_length = baseline_gen(rng);
            
            // Random direction in XZ plane
            Scalar tx = uniform_gen(rng);  // ±1 normalized direction
            Scalar tz = uniform_gen(rng);
            Vec3 t_dir = Vec3(tx, 0, tz).normalized();
            true_pose.t = baseline_length * t_dir;
        } else {
            // ROBOTICS: Realistic 3D baseline variation
            std::uniform_real_distribution<Scalar> baseline_gen(0.05, 5.0);  // 5cm to 5m
            Scalar baseline_length = baseline_gen(rng);
            
            Vec3 t_dir = Vec3::Random().normalized();
            true_pose.t = baseline_length * t_dir;
        }
    } else {
        // ROBOTICS: More realistic 6-DoF rotation variation
        // Mix small challenging rotations with larger easier ones
        std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
        Scalar rot_selector = rot_scenario(rng);
        
        // Rotation: controlled magnitude around random axis
        Vec3 rot_axis = Vec3::Random().normalized();
        Scalar rot_angle_deg;
        if (rot_selector < 0.3) {
            // Small challenging rotations: ±2°
            std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
            rot_angle_deg = small_rot(rng);
        } else if (rot_selector < 0.7) {
            // Medium typical rotations: ±15°  
            std::uniform_real_distribution<Scalar> medium_rot(-15.0, 15.0);
            rot_angle_deg = medium_rot(rng);
        } else {
            // Large easier rotations: ±60°
            std::uniform_real_distribution<Scalar> large_rot(-60.0, 60.0);
            rot_angle_deg = large_rot(rng);
        }
        
        Scalar rot_angle_rad = rot_angle_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(rot_angle_rad, rot_axis));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // ROBOTICS: Realistic 3D baseline variation (same as above)
        std::uniform_real_distribution<Scalar> baseline_gen(0.05, 5.0);  // 5cm to 5m  
        Scalar baseline_length = baseline_gen(rng);
        
        Vec3 t_dir = Vec3::Random().normalized();
        true_pose.t = baseline_length * t_dir;
    }
    
    if constexpr (N == 0) {
        x1_bear.clear();
        x2_bear.clear();
        x1_bear.reserve(num_points);
        x2_bear.reserve(num_points);
    }
    
    size_t valid_points = 0;
    int safety_counter = 0;
    const int max_safety_attempts = 50000;
    
    while (valid_points < num_points && safety_counter < max_safety_attempts) {
        safety_counter++;
        
        // ROBOTICS: More realistic world coordinates and depth variation
        // Mix of scenarios: close manipulation (0.2-2m), mid-range (1-10m), far-field (5-50m)
        std::uniform_real_distribution<Scalar> scenario_selector(0.0, 1.0);
        Scalar scenario = scenario_selector(rng);
        
        Vec3 X;
        if (scenario < 0.4) {
            // Close manipulation: 0.2-2m depth, tight lateral spread (challenging)
            std::uniform_real_distribution<Scalar> close_lateral(-0.5, 0.5);  // ±50cm
            std::uniform_real_distribution<Scalar> close_depth(0.2, 2.0);     // 20cm-2m
            X = Vec3(close_lateral(rng), close_lateral(rng), close_depth(rng));
        } else if (scenario < 0.8) {
            // Mid-range robotics: 1-10m depth, moderate spread (typical)
            std::uniform_real_distribution<Scalar> mid_lateral(-2.0, 2.0);    // ±2m  
            std::uniform_real_distribution<Scalar> mid_depth(1.0, 10.0);      // 1-10m
            X = Vec3(mid_lateral(rng), mid_lateral(rng), mid_depth(rng));
        } else {
            // Far-field navigation: 5-50m depth, wide spread (easier)
            std::uniform_real_distribution<Scalar> far_lateral(-10.0, 10.0);  // ±10m
            std::uniform_real_distribution<Scalar> far_depth(5.0, 50.0);      // 5-50m  
            X = Vec3(far_lateral(rng), far_lateral(rng), far_depth(rng));
        }
        
        // Project to first camera (identity pose)
        Vec3 x1_cam = X;
        if (x1_cam(2) <= Scalar(0.1)) continue; // Skip if behind camera
        
        // Project to second camera
        Vec3 x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue; // Skip if behind camera
        
        // Project to image coordinates (pinhole camera model)
        Vec2 x1_img = Vec2(
            focal_length * x1_cam(0) / x1_cam(2),
            focal_length * x1_cam(1) / x1_cam(2)
        );
        Vec2 x2_img = Vec2(
            focal_length * x2_cam(0) / x2_cam(2),
            focal_length * x2_cam(1) / x2_cam(2)
        );
        
        // Add realistic pixel noise (this is the key improvement!)
        x1_img(0) += pixel_noise_gen(rng);
        x1_img(1) += pixel_noise_gen(rng);
        x2_img(0) += pixel_noise_gen(rng);
        x2_img(1) += pixel_noise_gen(rng);
        
        // Convert back to bearing vectors (normalized camera coordinates)
        Vec3 f1 = Vec3(x1_img(0) / focal_length, x1_img(1) / focal_length, Scalar(1.0)).normalized();
        Vec3 f2 = Vec3(x2_img(0) / focal_length, x2_img(1) / focal_length, Scalar(1.0)).normalized();
        
        x1_bear.push_back(f1);
        x2_bear.push_back(f2);
        ++valid_points;
    }
    
    if (safety_counter >= max_safety_attempts) {
        std::cout << "    Warning: Hit safety limit generating realistic data for problem " << problem_id << std::endl;
    }
}

// Solver factory functions
template<typename Scalar, size_t N>
auto make_8pt_solver() {
    return [](const EntoContainer<Vec3<Scalar>, N>& x1, 
              const EntoContainer<Vec3<Scalar>, N>& x2,
              std::vector<CameraPose<Scalar>>* solutions) -> int {
        // Use first 8 points for 8pt solver
        EntoContainer<Vec3<Scalar>, N> x1_8pt, x2_8pt;
        for (size_t j = 0; j < N; ++j) {
            x1_8pt.push_back(x1[j]);
            x2_8pt.push_back(x2[j]);
        }
        
        return SolverRel8pt<Scalar>::template solve<N>(x1_8pt, x2_8pt, solutions);
    };
}

template<typename Scalar, size_t N>
auto make_5pt_solver() {
    return [](const EntoContainer<Vec3<Scalar>, N>& x1, 
              const EntoContainer<Vec3<Scalar>, N>& x2,
              std::vector<CameraPose<Scalar>>* solutions) -> int {
        // Use first 5 points for 5pt solver
        EntoContainer<Vec3<Scalar>, 5> x1_5pt, x2_5pt;
        for (size_t j = 0; j < 5; ++j) {
            x1_5pt.push_back(x1[j]);
            x2_5pt.push_back(x2[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 40> solutions_array;
        int num_solutions = SolverRel5pt<Scalar>::template solve<5>(x1_5pt, x2_5pt, &solutions_array);
        
        // Convert to vector
        solutions->clear();
        for (int i = 0; i < num_solutions; ++i) {
            solutions->push_back(solutions_array[i]);
        }
        
        return num_solutions;
    };
}

template<typename Scalar, size_t N>
auto make_upright_3pt_solver() {
    return [](const EntoContainer<Vec3<Scalar>, N>& x1, 
              const EntoContainer<Vec3<Scalar>, N>& x2,
              std::vector<CameraPose<Scalar>>* solutions) -> int {
        // Use first 3 points for upright 3pt solver
        EntoContainer<Vec3<Scalar>, N> x1_3pt, x2_3pt;
        for (size_t j = 0; j < N; ++j) {
            x1_3pt.push_back(x1[j]);
            x2_3pt.push_back(x2[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions_array;
        int num_solutions = SolverRelUpright3pt<Scalar>::template solve<N>(x1_3pt, x2_3pt, &solutions_array);
        
        // Convert to vector
        solutions->clear();
        for (int i = 0; i < num_solutions; ++i) {
            solutions->push_back(solutions_array[i]);
        }
        
        return num_solutions;
    };
}

template<typename Scalar, size_t N>
auto make_upright_planar_3pt_solver() {
    return [](const EntoContainer<Vec3<Scalar>, N>& x1, 
              const EntoContainer<Vec3<Scalar>, N>& x2,
              std::vector<CameraPose<Scalar>>* solutions) -> int {
        // Use first 3 points for upright planar 3pt solver
        EntoContainer<Vec3<Scalar>, N> x1_3pt, x2_3pt;
        for (size_t j = 0; j < N; ++j) {
            x1_3pt.push_back(x1[j]);
            x2_3pt.push_back(x2[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 2> solutions_array;
        int num_solutions = SolverRelUprightPlanar3pt<Scalar>::template solve<N>(x1_3pt, x2_3pt, &solutions_array);
        
        // Convert to vector
        solutions->clear();
        for (int i = 0; i < num_solutions; ++i) {
            solutions->push_back(solutions_array[i]);
        }
        
        return num_solutions;
    };
}

template<typename Scalar, size_t N>
auto make_upright_planar_2pt_solver() {
    return [](const EntoContainer<Vec3<Scalar>, N>& x1, 
              const EntoContainer<Vec3<Scalar>, N>& x2,
              std::vector<CameraPose<Scalar>>* solutions) -> int {
        // Use first 2 points for upright planar 2pt solver
        EntoContainer<Vec3<Scalar>, 2> x1_2pt, x2_2pt;
        for (size_t j = 0; j < 2; ++j) {
            x1_2pt.push_back(x1[j]);
            x2_2pt.push_back(x2[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions_array;
        int num_solutions = SolverRelUprightPlanar2pt<Scalar>::template solve<2>(x1_2pt, x2_2pt, &solutions_array);
        
        // Convert to vector
        solutions->clear();
        for (int i = 0; i < num_solutions; ++i) {
            solutions->push_back(solutions_array[i]);
        }
        
        return num_solutions;
    };
}

// Generic solver data generation function with parameterized noise levels
template<typename Scalar, size_t N, typename SolverFunc>
void generate_solver_data_for_noise_level(
    const Options& opts,
    const std::string& solver_name,
    SolverFunc solver_func,
    double noise_level)
{
    std::cout << "  Generating " << solver_name << " data (noise=" << noise_level << ")..." << std::endl;
    
    // Convert noise level based on generation method
    Scalar actual_noise_level;
    if (opts.use_realistic_generation) {
        // For realistic generation, noise_level is already in pixels
        actual_noise_level = static_cast<Scalar>(noise_level);
    } else {
        // For traditional generation, convert from "relative" to radians
        // 0.01 traditional ≈ 0.1 pixels, so scale accordingly
        actual_noise_level = static_cast<Scalar>(noise_level);
    }
    
    std::string filename = solver_name + "_" + 
                          (noise_level == 0.0 ? "clean" : 
                           "noise_" + std::to_string(noise_level).substr(0, 6)) + ".csv";
    
    std::ofstream file(filename);
    SolverStats<Scalar> stats;
    
    for (int i = 0; i < opts.num_problems; ++i) {
        std::cout << "    Generating " << solver_name << " problem " << i 
                  << " (noise=" << noise_level << ")..." << std::endl;
        stats.add_problem();
        
        EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
        CameraPose<Scalar> true_pose;
        
        // Choose data generation method
        if (opts.use_realistic_generation) {
            // Use realistic paper-based methodology
            generate_realistic_relpose_data<Scalar, N>(
                x1_bear, x2_bear, true_pose, opts.num_points,
                actual_noise_level,  // pixel noise std dev
                i,  // problem_id
                solver_name.find("upright") != std::string::npos,  // upright_only
                solver_name.find("planar") != std::string::npos,   // planar_only
                static_cast<Scalar>(opts.focal_length),
                static_cast<Scalar>(opts.baseline_magnitude),
                static_cast<Scalar>(opts.rotation_magnitude_deg)
            );
        } else {
            // Use traditional methodology
            generate_unified_relpose_data<Scalar, N>(
                x1_bear, x2_bear, true_pose, opts.num_points,
                actual_noise_level,  // bearing vector noise
                i,  // problem_id
                solver_name.find("upright") != std::string::npos,  // upright_only
                solver_name.find("planar") != std::string::npos    // planar_only
            );
        }
        
        // Test solver
        std::vector<CameraPose<Scalar>> solutions;
        int num_solutions = solver_func(x1_bear, x2_bear, &solutions);
        
        if (num_solutions > 0) {
            // Find best solution (closest to ground truth)
            PoseErrors<Scalar> best_error = compute_pose_errors<Scalar, N>(true_pose, solutions[0], x1_bear, x2_bear);
            for (size_t s = 1; s < solutions.size(); ++s) {
                PoseErrors<Scalar> error = compute_pose_errors<Scalar, N>(true_pose, solutions[s], x1_bear, x2_bear);
                if (error.max_error() < best_error.max_error()) {
                    best_error = error;
                }
            }
            
            // Use adaptive error threshold based on noise level and generation method
            Scalar error_threshold;
            if (opts.use_realistic_generation) {
                // More lenient thresholds for realistic data (pixel noise is challenging)
                error_threshold = (noise_level == 0.0) ? Scalar(5.0) : Scalar(20.0);
            } else {
                // Traditional thresholds
                error_threshold = (noise_level == 0.0) ? Scalar(10.0) : Scalar(15.0);
            }
            
            if (best_error.max_error() < error_threshold) {
                stats.add_success(best_error, num_solutions);
                std::cout << "      " << solver_name << " solved! Error: " << best_error.max_error() 
                          << "° (rot:" << best_error.rotation_error_deg 
                          << "°, trans:" << best_error.translation_error_deg 
                          << "°), Solutions: " << num_solutions << std::endl;
            } else {
                stats.add_failure_with_solution(best_error);
                std::cout << "      " << solver_name << " solution too inaccurate: " 
                          << best_error.max_error() << "°" << std::endl;
            }
        } else {
            stats.add_failure();
            std::cout << "      " << solver_name << " failed to find solution" << std::endl;
        }
        
        std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
            true_pose, x1_bear, x2_bear);
        
        file << csv_line << std::endl;
        std::cout << "    " << solver_name << " problem " << i << " complete." << std::endl;
    }
    
    file.close();
    std::cout << "  " << filename << " complete." << std::endl;
    
    // Print statistics for this noise level
    std::string stats_name = solver_name + " (noise=" + std::to_string(noise_level) + 
                             " pixels)";
    stats.print_stats(stats_name, noise_level);
}

// Generic function to generate data for any solver across all noise levels (parameterized)
template<typename Scalar, size_t N, typename SolverFunc>
void generate_solver_data_parameterized(
    const Options& opts,
    const std::string& solver_name,
    SolverFunc solver_func)
{
    std::cout << "Generating " << solver_name << " data..." << std::endl;
    
    // Generate data for each noise level
    for (double noise_level : opts.noise_levels) {
        generate_solver_data_for_noise_level<Scalar, N>(opts, solver_name, solver_func, noise_level);
    }
}

// Template dispatch functions with comprehensive size support
template<typename Scalar, size_t N>
void run_generation_specialized(const Options& opts) {
    std::cout << "Starting data generation with options:" << std::endl;
    std::cout << "  Scalar type: " << (sizeof(Scalar) == 8 ? "double" : "float") << std::endl;
    std::cout << "  Problems: " << opts.num_problems << std::endl;
    std::cout << "  Points per problem: " << opts.num_points << std::endl;
    std::cout << "  Container size N: " << N << std::endl;
    std::cout << "  Generate 8pt: " << (opts.generate_8pt ? "yes" : "no") << std::endl;
    std::cout << "  Generate 5pt: " << (opts.generate_5pt ? "yes" : "no") << std::endl;
    std::cout << "  Generate upright 3pt: " << (opts.generate_upright_3pt ? "yes" : "no") << std::endl;
    std::cout << "  Generate upright planar 3pt: " << (opts.generate_upright_planar_3pt ? "yes" : "no") << std::endl;
    std::cout << "  Generate upright planar 2pt: " << (opts.generate_upright_planar_2pt ? "yes" : "no") << std::endl;
    std::cout << std::endl;
    
    // Linear solvers: support overdetermined systems
    if (opts.generate_8pt && N >= 8) {
        generate_solver_data_parameterized<Scalar, N>(opts, "8pt", make_8pt_solver<Scalar, N>());
    }
    
    if (opts.generate_upright_planar_3pt && N >= 3) {
        generate_solver_data_parameterized<Scalar, N>(opts, "upright_planar_3pt", make_upright_planar_3pt_solver<Scalar, N>());
    }
    
    // Non-linear solvers: only exact minimal sizes
    if (opts.generate_5pt && N == 5) {
        generate_solver_data_parameterized<Scalar, N>(opts, "5pt", make_5pt_solver<Scalar, N>());
    }
    
    if (opts.generate_upright_3pt && N == 3) {
        generate_solver_data_parameterized<Scalar, N>(opts, "upright_3pt", make_upright_3pt_solver<Scalar, N>());
    }
    
    if (opts.generate_upright_planar_2pt && N == 2) {
        generate_solver_data_parameterized<Scalar, N>(opts, "upright_planar_2pt", make_upright_planar_2pt_solver<Scalar, N>());
    }
}

template<typename Scalar>
void run_generation_with_size(const Options& opts, int N) {
    // Dispatch to template specializations based on N
    switch (N) {
        case 2: run_generation_specialized<Scalar, 2>(opts); break;
        case 3: run_generation_specialized<Scalar, 3>(opts); break;
        case 4: run_generation_specialized<Scalar, 4>(opts); break;
        case 5: run_generation_specialized<Scalar, 5>(opts); break;
        case 8: run_generation_specialized<Scalar, 8>(opts); break;
        case 10: run_generation_specialized<Scalar, 10>(opts); break;
        case 16: run_generation_specialized<Scalar, 16>(opts); break;
        case 20: run_generation_specialized<Scalar, 20>(opts); break;
        case 30: run_generation_specialized<Scalar, 30>(opts); break;
        case 32: run_generation_specialized<Scalar, 32>(opts); break;
        case 40: run_generation_specialized<Scalar, 40>(opts); break;
        case 50: run_generation_specialized<Scalar, 50>(opts); break;
        case 64: run_generation_specialized<Scalar, 64>(opts); break;
        case 80: run_generation_specialized<Scalar, 80>(opts); break;
        case 100: run_generation_specialized<Scalar, 100>(opts); break;
        case 128: run_generation_specialized<Scalar, 128>(opts); break;
        default:
            std::cout << "Error: Unsupported container size " << N << std::endl;
            std::cout << "Supported sizes: 2, 3, 4, 5, 8, 10, 16, 20, 30, 32, 40, 50, 64, 80, 100, 128" << std::endl;
            std::cout << "Linear solvers (8pt, upright planar 3pt): support overdetermined systems" << std::endl;
            std::cout << "Non-linear solvers (5pt, upright 3pt, upright planar 2pt): require exact minimal sizes" << std::endl;
            exit(1);
    }
}

template<typename Scalar>
void run_generation(const Options& opts) {
    run_generation_with_size<Scalar>(opts, opts.num_points);
}

int main(int argc, char** argv)
{
    Options opts = parse_args(argc, argv);
    
    std::cout << "Enhanced Relative Pose Data Generation Tool" << std::endl;
    std::cout << "Generation method: " << (opts.use_realistic_generation ? "Realistic (paper-based)" : "Traditional") << std::endl;
    if (opts.use_realistic_generation) {
        std::cout << "  Pixel noise std: " << opts.pixel_noise_std << " pixels" << std::endl;
        std::cout << "  Focal length: " << opts.focal_length << std::endl;
        std::cout << "  Baseline magnitude: " << opts.baseline_magnitude << std::endl;
        std::cout << "  Rotation magnitude: " << opts.rotation_magnitude_deg << "°" << std::endl;
    }
    std::cout << "Noise levels: ";
    for (size_t i = 0; i < opts.noise_levels.size(); ++i) {
        std::cout << opts.noise_levels[i];
        if (i < opts.noise_levels.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    
    if (opts.use_double) {
        run_generation<double>(opts);
    } else {
        run_generation<float>(opts);
    }
    
    std::cout << "\nData generation complete!" << std::endl;
    std::cout << "Generated files with suffixes: _clean.csv, _noise_*.csv" << std::endl;
    
    return 0;
} 