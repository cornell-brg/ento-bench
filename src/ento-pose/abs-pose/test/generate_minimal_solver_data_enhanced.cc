#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <numeric>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ento-util/containers.h>
#include <ento-util/debug.h>  // Add debug support
#include <ento-pose/pose_util.h>
#include <ento-pose/synthetic_abspose.h>

// Include the solver interfaces from data_gen.h
#include <ento-pose/data_gen.h>

#ifndef ENTO_DEBUG_ARRAY
#define ENTO_DEBUG_ARRAY(...)
#endif

#ifndef ENTO_DEBUG
#define ENTO_DEBUG(...)
#endif

using namespace EntoPose;
using namespace EntoUtil;

// Command line options structure
struct Options {
    int num_problems = 100;
    int num_points = 100;
    bool generate_p3p = true;
    bool generate_up2p = true;
    bool generate_dlt = true;
    bool generate_gold_standard_abs = true;
    bool use_double = false;
    bool realistic = false;
    
    // Noise level specification
    std::vector<double> noise_levels = {0.0, 0.01};
    
    // Outlier support for LO-RANSAC benchmarks
    bool generate_outliers = false;
    std::vector<double> outlier_ratios = {0.25}; // Default 25% outliers
    
    // Realistic camera parameters
    double focal_length = 500.0;
    double baseline = 1.0;
    double rotation_deg = 16.0;
    double pixel_noise = 0.1;  // pixels
    int image_width = 640;
    int image_height = 480;
    
    // DLT specialization options - REMOVED dlt_min_points
    static const std::vector<int>& get_supported_dlt_sizes() {
        static const std::vector<int> sizes = {6, 8, 16, 32, 64, 128};
        return sizes;
    }
    
    bool is_valid_dlt_size(int size) const {
        const auto& sizes = get_supported_dlt_sizes();
        return std::find(sizes.begin(), sizes.end(), size) != sizes.end();
    }
    
    void print_help() {
        std::cout << "Enhanced Absolute Pose Data Generation Tool\n";
        std::cout << "===========================================\n\n";
        std::cout << "Usage: generate_minimal_solver_data_abs_enhanced [options]\n\n";
        std::cout << "General Options:\n";
        std::cout << "  --problems N           Number of problems to generate (default: 100)\n";
        std::cout << "  --points N             Number of points per problem (default: 100)\n";
        std::cout << "  --noise-levels \"X,Y,Z\" Comma-separated noise levels (default: \"0.0,0.01\")\n";
        std::cout << "  --double               Use double precision (default: float)\n";
        std::cout << "  --help                 Show this help message\n\n";
        
        std::cout << "Solver Selection:\n";
        std::cout << "  --p3p-only             Generate only P3P data\n";
        std::cout << "  --up2p-only            Generate only UP2P data\n";
        std::cout << "  --dlt-only             Generate only DLT data\n";
        std::cout << "  --gold-standard-abs-only Generate only gold standard absolute pose data\n";
        std::cout << "                         (DLT will use all --points specified)\n\n";
        
        std::cout << "LO-RANSAC Outlier Support:\n";
        std::cout << "  --with-outliers        Generate datasets with outliers for robust estimation\n";
        std::cout << "  --outlier-ratios \"X,Y\" Comma-separated outlier ratios (default: \"0.25\")\n\n";
        
        std::cout << "Realistic Data Generation:\n";
        std::cout << "  --realistic            Use realistic camera parameters and methodology\n";
        std::cout << "  --focal-length F       Camera focal length in pixels (default: 500)\n";
        std::cout << "  --baseline B           Camera baseline in meters (default: 1.0)\n";
        std::cout << "  --rotation-deg R       Maximum rotation in degrees (default: 16)\n";
        std::cout << "  --pixel-noise P        Pixel noise standard deviation (default: 0.1)\n";
        std::cout << "  --image-width W        Image width in pixels (default: 640)\n";
        std::cout << "  --image-height H       Image height in pixels (default: 480)\n\n";
        
        std::cout << "Examples:\n";
        std::cout << "  # Basic usage with custom noise levels:\n";
        std::cout << "  ./generate_minimal_solver_data_abs_enhanced --noise-levels \"0.0,0.005,0.01,0.02\"\n\n";
        std::cout << "  # Generate realistic data with research paper methodology:\n";
        std::cout << "  ./generate_minimal_solver_data_abs_enhanced --realistic --focal-length 500 --pixel-noise 0.1\n\n";
        std::cout << "  # Generate P3P data with 25% outliers for LO-RANSAC:\n";
        std::cout << "  ./generate_minimal_solver_data_abs_enhanced --p3p-only --with-outliers --outlier-ratios \"0.25\"\n\n";
        std::cout << "  # Double precision with all solvers:\n";
        std::cout << "  ./generate_minimal_solver_data_abs_enhanced --double --problems 200\n\n";
    }
};

// Parse noise levels from comma-separated string
std::vector<double> parse_noise_levels(const std::string& noise_str) {
    std::vector<double> noise_levels;
    std::stringstream ss(noise_str);
    std::string item;
    
    while (std::getline(ss, item, ',')) {
        try {
            double noise = std::stod(item);
            if (noise < 0.0) {
                std::cout << "Error: Noise levels must be non-negative\n";
                exit(1);
            }
            noise_levels.push_back(noise);
        } catch (const std::exception& e) {
            std::cout << "Error: Invalid noise level '" << item << "'\n";
            exit(1);
        }
    }
    
    if (noise_levels.empty()) {
        std::cout << "Error: No valid noise levels specified\n";
        exit(1);
    }
    
    return noise_levels;
}

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
            opts.noise_levels = parse_noise_levels(argv[++i]);
        } else if (strcmp(argv[i], "--focal-length") == 0 && i + 1 < argc) {
            opts.focal_length = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--baseline") == 0 && i + 1 < argc) {
            opts.baseline = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--rotation-deg") == 0 && i + 1 < argc) {
            opts.rotation_deg = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--pixel-noise") == 0 && i + 1 < argc) {
            opts.pixel_noise = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--image-width") == 0 && i + 1 < argc) {
            opts.image_width = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--image-height") == 0 && i + 1 < argc) {
            opts.image_height = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--p3p-only") == 0) {
            opts.generate_p3p = true;
            opts.generate_up2p = false;
            opts.generate_dlt = false;
        } else if (strcmp(argv[i], "--up2p-only") == 0) {
            opts.generate_p3p = false;
            opts.generate_up2p = true;
            opts.generate_dlt = false;
        } else if (strcmp(argv[i], "--dlt-only") == 0) {
            opts.generate_p3p = false;
            opts.generate_up2p = false;
            opts.generate_dlt = true;
            opts.generate_gold_standard_abs = false;
        } else if (strcmp(argv[i], "--gold-standard-abs-only") == 0) {
            opts.generate_p3p = false;
            opts.generate_up2p = false;
            opts.generate_dlt = false;
            opts.generate_gold_standard_abs = true;
        } else if (strcmp(argv[i], "--all-types") == 0) {
            opts.generate_p3p = true;
            opts.generate_up2p = true;
            opts.generate_dlt = true;
            opts.generate_gold_standard_abs = true;
        } else if (strcmp(argv[i], "--double") == 0) {
            opts.use_double = true;
        } else if (strcmp(argv[i], "--realistic") == 0) {
            opts.realistic = true;
        } else if (strcmp(argv[i], "--with-outliers") == 0) {
            opts.generate_outliers = true;
        } else if (strcmp(argv[i], "--outlier-ratios") == 0 && i + 1 < argc) {
            opts.outlier_ratios = parse_noise_levels(argv[++i]);
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
    if (opts.focal_length <= 0 || opts.baseline <= 0 || opts.rotation_deg <= 0) {
        std::cout << "Error: Realistic parameters must be positive\n";
        exit(1);
    }
    
    return opts;
}

// Outlier injection function for absolute pose LO-RANSAC
template<typename Scalar, size_t N>
void inject_outliers(EntoContainer<Vec2<Scalar>, N>& points2D,
                     EntoContainer<Vec3<Scalar>, N>& points3D,
                     EntoContainer<bool, N>& inlier_mask,
                     double outlier_ratio,
                     std::default_random_engine& rng) {
    const size_t num_points = points2D.size();
    const size_t num_outliers = static_cast<size_t>(num_points * outlier_ratio);
    
    ENTO_DEBUG("Injecting %zu outliers out of %zu points (ratio=%.2f)", 
               num_outliers, num_points, outlier_ratio);
    
    // Initialize inlier mask - everyone starts as inlier
    if constexpr (N == 0) {
        inlier_mask.clear();
        inlier_mask.resize(num_points, true);
    } else {
        std::fill(inlier_mask.begin(), inlier_mask.end(), true);
    }
    
    // Generate random indices to replace with outliers
    std::vector<size_t> indices(num_points);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    
    // Replace first num_outliers correspondences with random ones
    std::uniform_real_distribution<Scalar> coord_dist(-2.0, 2.0);
    std::uniform_real_distribution<Scalar> depth_dist(0.5, 8.0);
    
    for (size_t i = 0; i < num_outliers; ++i) {
        size_t idx = indices[i];
        
        // Generate random 2D point (normalized coordinates)
        points2D[idx] = Vec2<Scalar>(coord_dist(rng), coord_dist(rng));
        
        // Generate random 3D point 
        points3D[idx] = Vec3<Scalar>(coord_dist(rng), coord_dist(rng), depth_dist(rng));
        
        // Mark as outlier
        inlier_mask[idx] = false;
        
        ENTO_DEBUG("Outlier %zu: idx=%zu, 2D=(%f,%f), 3D=(%f,%f,%f)", 
                   i, idx, points2D[idx](0), points2D[idx](1),
                   points3D[idx](0), points3D[idx](1), points3D[idx](2));
    }
    
    ENTO_DEBUG("Outlier injection complete: %zu outliers, %zu inliers", 
               num_outliers, num_points - num_outliers);
}

// Enhanced pose error computation with separate rotation and translation tracking
template<typename Scalar>
struct PoseErrors {
    Scalar rotation_error_deg;
    Scalar translation_error_m;    // Change to L2 norm in meters
    Scalar reprojection_error;  // Add reprojection error
    
    Scalar max_error() const {
        return std::max(rotation_error_deg, translation_error_m);
    }
};

// Compute reprojection error for absolute pose
// For DLT: points2D are normalized image coordinates
// For P3P/UP2P: need to convert bearing vectors back to image coordinates for comparison
template<typename Scalar, size_t N>
Scalar compute_reprojection_error(
    const CameraPose<Scalar>& pose,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    bool use_bearing_vectors = false)  // Flag to indicate coordinate system
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    
    Scalar total_error = 0.0;
    size_t valid_points = 0;
    
    for (size_t i = 0; i < points2D.size(); ++i) {
        // Project 3D point using estimated pose
        Vec3 Z = pose.R() * points3D[i] + pose.t;
        
        // Skip points behind camera
        if (Z(2) <= Scalar(1e-6)) continue;
        
        if (use_bearing_vectors) {
            // For P3P/UP2P: Compare bearing vectors (normalized 3D directions)
            Vec3 estimated_bearing = Z.normalized();
            Vec3 true_bearing(points2D[i](0), points2D[i](1), Scalar(1.0));
            true_bearing.normalize();
            
            // Angular error between bearing vectors (in radians, then convert to "pixels")
            Scalar dot_product = std::clamp(estimated_bearing.dot(true_bearing), Scalar(-1), Scalar(1));
            Scalar angular_error = std::acos(std::abs(dot_product));  // Use abs to handle ±direction ambiguity
            
            // Convert angular error to approximate pixel error (assuming ~500 pixel focal length)
            Scalar pixel_error = angular_error * Scalar(500.0);
            total_error += pixel_error * pixel_error;
        } else {
            // For DLT: Compare normalized image coordinates directly
            Vec2 proj = Vec2(Z(0) / Z(2), Z(1) / Z(2));
            Scalar error = (proj - points2D[i]).squaredNorm();
            total_error += error;
        }
        
        valid_points++;
    }
    
    if (valid_points == 0) return std::numeric_limits<Scalar>::max();
    
    // Return RMS reprojection error
    return std::sqrt(total_error / valid_points);
}

template<typename Scalar, size_t N>
PoseErrors<Scalar> compute_pose_errors(
    const CameraPose<Scalar>& true_pose, 
    const CameraPose<Scalar>& estimated_pose,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    bool use_bearing_vectors = false)  // Flag to indicate coordinate system
{
    PoseErrors<Scalar> errors;
    
    // Rotation error (angular difference between rotation matrices)
    double trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
    double angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
    errors.rotation_error_deg = Scalar(angle_rad * 180.0 / M_PI);
    
    // Translation error (L2 norm)
    errors.translation_error_m = (estimated_pose.t - true_pose.t).norm();
    
    // Reprojection error (coordinate-system aware)
    errors.reprojection_error = compute_reprojection_error<Scalar, N>(estimated_pose, points2D, points3D, use_bearing_vectors);
    
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
        successful_translation_errors.push_back(errors.translation_error_m);
        successful_reprojection_errors.push_back(errors.reprojection_error);
        successful_combined_errors.push_back(errors.max_error());
        
        // Also add to all-solutions statistics
        all_rotation_errors.push_back(errors.rotation_error_deg);
        all_translation_errors.push_back(errors.translation_error_m);
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
        all_translation_errors.push_back(errors.translation_error_m);
        all_reprojection_errors.push_back(errors.reprojection_error);
        all_combined_errors.push_back(errors.max_error());
    }
    
    void add_problem() {
        total_problems++;
    }
    
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
    
    void print_stats(const std::string& solver_name, double noise_level) const {
        std::cout << "\n=== " << solver_name << " Statistics (noise=" << noise_level << ") ===" << std::endl;
        std::cout << "  Total problems: " << total_problems << std::endl;
        std::cout << "  Successful solves: " << successful_solves << " (" 
                  << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
        std::cout << "  Failed solves: " << failed_solves << " (" 
                  << (100.0 * failed_solves / total_problems) << "%)" << std::endl;
        
        if (successful_solves > 0) {
            std::cout << "  \n=== SUCCESSFUL CASES ONLY (threshold-based) ===" << std::endl;
            std::cout << "  Rotation Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(successful_rotation_errors) << " ± " << stddev(successful_rotation_errors) << " degrees" << std::endl;
            std::cout << "    Min: " << *std::min_element(successful_rotation_errors.begin(), successful_rotation_errors.end()) << " degrees" << std::endl;
            std::cout << "    Max: " << *std::max_element(successful_rotation_errors.begin(), successful_rotation_errors.end()) << " degrees" << std::endl;
            
            std::cout << "  \nTranslation Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(successful_translation_errors) << " ± " << stddev(successful_translation_errors) << " meters" << std::endl;
            std::cout << "    Min: " << *std::min_element(successful_translation_errors.begin(), successful_translation_errors.end()) << " meters" << std::endl;
            std::cout << "    Max: " << *std::max_element(successful_translation_errors.begin(), successful_translation_errors.end()) << " meters" << std::endl;
            
            std::cout << "  \nReprojection Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(successful_reprojection_errors) << " ± " << stddev(successful_reprojection_errors) << " pixels" << std::endl;
            std::cout << "    Min: " << *std::min_element(successful_reprojection_errors.begin(), successful_reprojection_errors.end()) << " pixels" << std::endl;
            std::cout << "    Max: " << *std::max_element(successful_reprojection_errors.begin(), successful_reprojection_errors.end()) << " pixels" << std::endl;
            
            std::cout << "  \nCombined Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(successful_combined_errors) << " ± " << stddev(successful_combined_errors) << " (mixed units: max of rot° and trans m)" << std::endl;
            std::cout << "    Min: " << *std::min_element(successful_combined_errors.begin(), successful_combined_errors.end()) << " (mixed units)" << std::endl;
            std::cout << "    Max: " << *std::max_element(successful_combined_errors.begin(), successful_combined_errors.end()) << " (mixed units)" << std::endl;
            
            std::cout << "  \nSolution Statistics:" << std::endl;
            std::cout << "    Average solutions per problem: " 
                      << (float(total_solutions_found) / successful_solves) << std::endl;
            std::cout << "    Problems with multiple solutions: " << problems_with_multiple_solutions 
                      << " (" << (100.0 * problems_with_multiple_solutions / successful_solves) << "%)" << std::endl;
        }
        
        // NEW: All-solutions statistics (complete performance picture)
        if (!all_rotation_errors.empty()) {
            std::cout << "  \n=== ALL SOLUTIONS (complete performance) ===" << std::endl;
            std::cout << "  Rotation Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(all_rotation_errors) << " ± " << stddev(all_rotation_errors) << " degrees" << std::endl;
            std::cout << "    Min: " << *std::min_element(all_rotation_errors.begin(), all_rotation_errors.end()) << " degrees" << std::endl;
            std::cout << "    Max: " << *std::max_element(all_rotation_errors.begin(), all_rotation_errors.end()) << " degrees" << std::endl;
            
            std::cout << "  \nTranslation Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(all_translation_errors) << " ± " << stddev(all_translation_errors) << " meters" << std::endl;
            std::cout << "    Min: " << *std::min_element(all_translation_errors.begin(), all_translation_errors.end()) << " meters" << std::endl;
            std::cout << "    Max: " << *std::max_element(all_translation_errors.begin(), all_translation_errors.end()) << " meters" << std::endl;
            
            std::cout << "  \nReprojection Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(all_reprojection_errors) << " ± " << stddev(all_reprojection_errors) << " pixels" << std::endl;
            std::cout << "    Min: " << *std::min_element(all_reprojection_errors.begin(), all_reprojection_errors.end()) << " pixels" << std::endl;
            std::cout << "    Max: " << *std::max_element(all_reprojection_errors.begin(), all_reprojection_errors.end()) << " pixels" << std::endl;
            
            std::cout << "  \nCombined Error Statistics:" << std::endl;
            std::cout << "    Mean ± Std Dev: " << mean(all_combined_errors) << " ± " << stddev(all_combined_errors) << " (mixed units: max of rot° and trans m)" << std::endl;
            std::cout << "    Min: " << *std::min_element(all_combined_errors.begin(), all_combined_errors.end()) << " (mixed units)" << std::endl;
            std::cout << "    Max: " << *std::max_element(all_combined_errors.begin(), all_combined_errors.end()) << " (mixed units)" << std::endl;
        }
        std::cout << "================================" << std::endl;
    }
};

// CSV line generation for absolute pose problems
template <typename Scalar, size_t N>
std::string make_csv_line_absolute_pose(
    const CameraPose<Scalar>& pose,
    const EntoContainer<Vec2<Scalar>, N>& points2D,
    const EntoContainer<Vec3<Scalar>, N>& points3D,
    Scalar scale = Scalar{1},
    Scalar focal = Scalar{1})
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    std::ostringstream oss;

    // 1) problem_type (1 for absolute pose), N
    oss << 1 << ',' << N << ',';

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

    // 5) x: dump N points (x, y)
    for (size_t i = 0; i < N; ++i) {
        const Vec2& pt = points2D[i];
        oss << pt(0) << ',' << pt(1) << ',';
    }

    // 6) X: dump N points (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3& pt = points3D[i];
        oss << pt(0) << ',' << pt(1) << ',' << pt(2);
        if (i < N - 1) oss << ',';
    }

    return oss.str();
}

// CSV line generation for robust absolute pose problems (with outlier masks)
template <typename Scalar, size_t N>
std::string make_csv_line_robust_absolute_pose(
    const CameraPose<Scalar>& pose,
    const EntoContainer<Vec2<Scalar>, N>& points2D,
    const EntoContainer<Vec3<Scalar>, N>& points3D,
    const EntoContainer<bool, N>& inlier_mask,
    Scalar scale = Scalar{1},
    Scalar focal = Scalar{1})
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    std::ostringstream oss;
    
    const size_t num_points = points2D.size();

    // 1) problem_type (1 for absolute pose), N
    oss << 1 << ',' << num_points << ',';

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

    // 5) x: dump N points (x, y)
    for (size_t i = 0; i < num_points; ++i) {
        const Vec2& pt = points2D[i];
        oss << pt(0) << ',' << pt(1) << ',';
    }

    // 6) X: dump N points (x, y, z)
    for (size_t i = 0; i < num_points; ++i) {
        const Vec3& pt = points3D[i];
        oss << pt(0) << ',' << pt(1) << ',' << pt(2) << ',';
    }

    // 7) pack "inliers" (bool flags) into bytes
    constexpr size_t kNumBytes = (N > 0) ? ((N + 7) / 8) : 0;
    size_t dyn_num_bytes = (N == 0) ? ((num_points + 7) / 8) : 0;
    size_t actual_num_bytes = (N > 0) ? kNumBytes : dyn_num_bytes;
    
    std::vector<uint8_t> mask_bytes(actual_num_bytes, 0);
    for (size_t i = 0; i < num_points; ++i) {
        if (inlier_mask[i]) {
            size_t byte_idx = i / 8;
            size_t bit_idx = i % 8; // LSB = point-0, next bit = point-1, etc.
            mask_bytes[byte_idx] |= static_cast<uint8_t>(1u << bit_idx);
        }
    }

    // 8) append each byte in decimal. If N<=8, that's one byte; otherwise multiple.
    for (size_t b = 0; b < actual_num_bytes; ++b) {
        oss << static_cast<uint32_t>(mask_bytes[b]);
        if (b + 1 < actual_num_bytes) {
            oss << ','; // comma-separate multiple inlier-bytes
        }
    }

    return oss.str();
}

// Realistic data generation with proper retry logic
template<typename Scalar, size_t N>
void generate_realistic_abspose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    const Options& opts,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0,
    bool upright_only = false)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    if constexpr (N == 0) {
        points2D.clear();
        points3D.clear();
        points2D.reserve(num_points);
        points3D.reserve(num_points);
    }
    
    std::default_random_engine rng(123 + problem_id);
    std::uniform_real_distribution<Scalar> coord_gen(-5.0, 5.0);
    std::uniform_real_distribution<Scalar> depth_gen(2.0, 20.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    
    // Keep trying until we get enough valid points
    int pose_attempts = 0;
    const int max_pose_attempts = 100;
    
    while (pose_attempts < max_pose_attempts) {
        // Generate realistic pose
        if (upright_only) {
            // UP2P: only rotation around Y axis
            Scalar yaw = std::uniform_real_distribution<Scalar>(-M_PI, M_PI)(rng);
            Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
            true_pose.q(0) = q.w();
            true_pose.q(1) = q.x();
            true_pose.q(2) = q.y();
            true_pose.q(3) = q.z();
        } else {
            // General 6DOF pose
            true_pose.q = Quaternion::UnitRandom().coeffs();
        }
        
        // Realistic translation (camera movement)
        true_pose.t = Vec3(
            std::uniform_real_distribution<Scalar>(-3.0, 3.0)(rng),
            std::uniform_real_distribution<Scalar>(-1.0, 1.0)(rng),
            std::uniform_real_distribution<Scalar>(-2.0, 2.0)(rng)
        );
        
        // Try to generate points with this pose
        size_t valid_points = 0;
        int point_attempts = 0;
        const int max_point_attempts = 1000;
        
        if constexpr (N == 0) {
            points2D.clear();
            points3D.clear();
        }
        
        while (valid_points < num_points && point_attempts < max_point_attempts) {
            // Generate 3D point in world coordinates
            Vec3 X(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            
            // Project to camera coordinates
            Vec3 x_cam = true_pose.R() * X + true_pose.t;
            
            if (x_cam(2) > Scalar(0.5)) { // Reasonable depth
                // Project to normalized image coordinates
                Vec2 x_norm(x_cam(0) / x_cam(2), x_cam(1) / x_cam(2));
                
                // Check if point is within reasonable image bounds
                if (std::abs(x_norm(0)) < 2.0 && std::abs(x_norm(1)) < 2.0) {
                    // Add pixel noise
                    x_norm(0) += noise_gen(rng);
                    x_norm(1) += noise_gen(rng);
                    
                    points2D.push_back(x_norm);
                    points3D.push_back(X);
                    valid_points++;
                }
            }
            point_attempts++;
        }
        
        if (valid_points >= num_points) {
            return; // Success!
        }
        
        pose_attempts++;
    }
    
    // If we get here, just use what we have
    if (points2D.size() < num_points) {
        std::cout << "    Warning: Could not generate enough valid points, got " 
                  << points2D.size() << "/" << num_points << std::endl;
    }
}

// Enhanced data generation with proper retry logic and degeneracy checks
template<typename Scalar, size_t N>
void generate_unified_abspose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0,
    bool upright_only = false)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem to avoid problematic poses
    std::default_random_engine rng(42 + problem_id);
    
    // Set random true pose
    if (upright_only) {
        // Set random true pose with only y-axis rotation (for UP2P)
        Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
        // Store quaternion with real part first (w, x, y, z)
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
    } else {
        // General pose (for P3P, DLT)
        true_pose.q = Quaternion::UnitRandom().coeffs();
    }
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0); // Scale translation
    
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    
    if constexpr (N == 0) {
        points2D.clear();
        points3D.clear();
        points2D.reserve(num_points);
        points3D.reserve(num_points);
    }
    
    // Generate points with degeneracy checks
    int max_attempts = 1000;
    int attempt = 0;
    
    while (attempt < max_attempts) {
        // Clear containers for this attempt - both dynamic AND fixed-size containers
        if constexpr (N == 0) {
            points2D.clear();
            points3D.clear();
        } else {
            // For fixed-size containers, we need to clear them too!
            points2D.clear();
            points3D.clear();
        }
        
        size_t valid_points = 0;
        while (valid_points < num_points) {
            Vec3 X;
            Vec3 x_cam;
            Vec2 x_norm;
            
            // Add safety counter to prevent infinite loops
            int point_attempts = 0;
            const int max_point_attempts = 100;
            while (true) {
                // Generate 3D point
                X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
                // Project to camera coordinates
                x_cam = true_pose.R() * X + true_pose.t;
                if (x_cam(2) > Scalar(0.1)) break; // Accept only if in front of camera
                
                ++point_attempts;
                if (point_attempts >= max_point_attempts) {
                    // Regenerate pose and try again
                    if (upright_only) {
                        Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
                        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
                        true_pose.q(0) = q.w();
                        true_pose.q(1) = q.x();
                        true_pose.q(2) = q.y();
                        true_pose.q(3) = q.z();
                    } else {
                        true_pose.q = Quaternion::UnitRandom().coeffs();
                    }
                    true_pose.t.setRandom();
                    true_pose.t *= Scalar(2.0);
                    point_attempts = 0;
                    points2D.clear();
                    points3D.clear();
                }
            }
            
            // Project to normalized image coordinates
            x_norm = Vec2(x_cam(0) / x_cam(2), x_cam(1) / x_cam(2));
            
            // Add pixel noise in image space (UNIFIED NOISE MODEL)
            x_norm(0) += noise_gen(rng);
            x_norm(1) += noise_gen(rng);

            points2D.push_back(x_norm);
            points3D.push_back(X);
            ++valid_points;
        }
        
        // DEGENERACY CHECKS: Verify point configuration is well-conditioned
        bool is_degenerate = false;
        
        // Check 1: Collinearity test (for 3D points)
        if (num_points >= 3) {
            // Check if first 3 points are collinear
            Vec3 v1 = points3D[1] - points3D[0];
            Vec3 v2 = points3D[2] - points3D[0];
            Vec3 cross = v1.cross(v2);
            if (cross.norm() < Scalar(1e-6)) {
                ENTO_DEBUG("[generate_unified_abspose_data] Detected collinear 3D points, retrying...");
                is_degenerate = true;
            }
        }
        
        // Check 2: Coplanarity test (for 4+ points, critical for DLT)
        if (num_points >= 4 && !is_degenerate) {
            // Check if first 4 points are coplanar
            Vec3 v1 = points3D[1] - points3D[0];
            Vec3 v2 = points3D[2] - points3D[0];
            Vec3 v3 = points3D[3] - points3D[0];
            Vec3 normal = v1.cross(v2);
            Scalar distance = std::abs(normal.dot(v3)) / normal.norm();
            if (distance < Scalar(1e-4)) {
                ENTO_DEBUG("[generate_unified_abspose_data] Detected coplanar 3D points, retrying...");
                is_degenerate = true;
            }
        }
        
        // Check 3: Depth variation test (ensure good depth distribution)
        if (!is_degenerate) {
            Scalar min_depth = std::numeric_limits<Scalar>::max();
            Scalar max_depth = std::numeric_limits<Scalar>::lowest();
            for (size_t i = 0; i < num_points; ++i) {
                Vec3 x_cam = true_pose.R() * points3D[i] + true_pose.t;
                min_depth = std::min(min_depth, x_cam(2));
                max_depth = std::max(max_depth, x_cam(2));
            }
            Scalar depth_ratio = max_depth / min_depth;
            if (depth_ratio < Scalar(1.5)) {  // Require at least 1.5x depth variation
                ENTO_DEBUG("[generate_unified_abspose_data] Poor depth variation (ratio=%f), retrying...", depth_ratio);
                is_degenerate = true;
            }
        }
        
        // Check 4: 2D point spread test (ensure good image coverage)
        if (!is_degenerate) {
            Scalar min_x = std::numeric_limits<Scalar>::max();
            Scalar max_x = std::numeric_limits<Scalar>::lowest();
            Scalar min_y = std::numeric_limits<Scalar>::max();
            Scalar max_y = std::numeric_limits<Scalar>::lowest();
            for (size_t i = 0; i < num_points; ++i) {
                min_x = std::min(min_x, points2D[i](0));
                max_x = std::max(max_x, points2D[i](0));
                min_y = std::min(min_y, points2D[i](1));
                max_y = std::max(max_y, points2D[i](1));
            }
            Scalar x_spread = max_x - min_x;
            Scalar y_spread = max_y - min_y;
            if (x_spread < Scalar(0.5) || y_spread < Scalar(0.5)) {
                ENTO_DEBUG("[generate_unified_abspose_data] Poor 2D point spread (x=%f, y=%f), retrying...", x_spread, y_spread);
                is_degenerate = true;
            }
        }
        
        if (!is_degenerate) {
            ENTO_DEBUG("[generate_unified_abspose_data] Generated well-conditioned point set after %d attempts", attempt + 1);
            return; // Success! Points are well-conditioned
        }
        
        attempt++;
    }
    
    // If we get here, we couldn't generate a good configuration
    ENTO_DEBUG("[generate_unified_abspose_data] Warning: Could not generate well-conditioned points after %d attempts, using current set", max_attempts);
}

// Convert 2D points to bearing vectors for P3P/UP2P solvers
template<typename Scalar, size_t N>
void convert_2d_to_bearing_vectors(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& bearing_vectors)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    
    if constexpr (N == 0) {
        bearing_vectors.clear();
        bearing_vectors.reserve(points2D.size());
    }
    
    for (size_t i = 0; i < points2D.size(); ++i) {
        const Vec2& pt2d = points2D[i];
        Vec3 bearing(pt2d(0), pt2d(1), Scalar(1.0));
        bearing.normalize();
        bearing_vectors.push_back(bearing);
    }
}

// Convert 2D points to homogeneous coordinates for DLT solver
template<typename Scalar, size_t N>
void convert_2d_to_homogeneous(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points2D_homo)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    
    if constexpr (N == 0) {
        points2D_homo.clear();
        points2D_homo.reserve(points2D.size());
    }
    
    for (size_t i = 0; i < points2D.size(); ++i) {
        const Vec2& pt2d = points2D[i];
        // Convert to homogeneous coordinates [x, y, 1] - NO NORMALIZATION for DLT!
        Vec3 homo(pt2d(0), pt2d(1), Scalar(1.0));
        // Note: P3P/UP2P use normalized bearing vectors, but DLT needs true homogeneous coords
        points2D_homo.push_back(homo);
    }
}

// Solver result structure
template<typename Scalar>
struct SolverResult {
    int num_solutions = 0;
    PoseErrors<Scalar> errors;
    Scalar best_error = std::numeric_limits<Scalar>::max();
};

// P3P solver wrapper
template<typename Scalar, size_t N>
auto make_p3p_solver() {
    return [](const EntoContainer<Vec2<Scalar>, N>& points2D,
              const EntoContainer<Vec3<Scalar>, N>& points3D,
              const CameraPose<Scalar>& true_pose) -> SolverResult<Scalar> {
        
        SolverResult<Scalar> result;
        
        // Convert to bearing vectors
        EntoContainer<Vec3<Scalar>, N> x_bear;
        convert_2d_to_bearing_vectors<Scalar, N>(points2D, x_bear);
        
        // Use first 3 points
        EntoContainer<Vec3<Scalar>, 3> x_bear_3;
        EntoContainer<Vec3<Scalar>, 3> points3D_3;
        for (int j = 0; j < 3; ++j) {
            x_bear_3.push_back(x_bear[j]);
            points3D_3.push_back(points3D[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions;
        result.num_solutions = SolverP3P<Scalar>::template solve<3>(x_bear_3, points3D_3, &solutions);
        
        if (result.num_solutions > 0) {
            // Find best solution
            for (int s = 0; s < result.num_solutions; ++s) {
                // Use bearing vector flag for P3P reprojection error calculation
                auto errors = compute_pose_errors<Scalar, N>(true_pose, solutions[s], points2D, points3D, true);
                Scalar error = errors.max_error();
                if (error < result.best_error) {
                    result.best_error = error;
                    result.errors = errors;
                }
            }
        }
        
        return result;
    };
}

// UP2P solver wrapper
template<typename Scalar, size_t N>
auto make_up2p_solver() {
    return [](const EntoContainer<Vec2<Scalar>, N>& points2D,
              const EntoContainer<Vec3<Scalar>, N>& points3D,
              const CameraPose<Scalar>& true_pose) -> SolverResult<Scalar> {
        
        SolverResult<Scalar> result;
        
        // Convert to bearing vectors
        EntoContainer<Vec3<Scalar>, N> x_bear;
        convert_2d_to_bearing_vectors<Scalar, N>(points2D, x_bear);
        
        // Use first 2 points
        EntoContainer<Vec3<Scalar>, 2> x_up2p;
        EntoContainer<Vec3<Scalar>, 2> X_up2p;
        for (int j = 0; j < 2; ++j) {
            x_up2p.push_back(x_bear[j]);
            X_up2p.push_back(points3D[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions;
        result.num_solutions = SolverUP2P<Scalar>::template solve<2>(x_up2p, X_up2p, &solutions);
        
        if (result.num_solutions > 0) {
            // Find best solution
            for (int s = 0; s < result.num_solutions; ++s) {
                // Use bearing vector flag for UP2P reprojection error calculation
                auto errors = compute_pose_errors<Scalar, N>(true_pose, solutions[s], points2D, points3D, true);
                Scalar error = errors.max_error();
                if (error < result.best_error) {
                    result.best_error = error;
                    result.errors = errors;
                }
            }
        }
        
        return result;
    };
}

// DLT solver wrapper with proper template specialization
template<typename Scalar, size_t N>
auto make_dlt_solver() {
    return [](const EntoContainer<Vec2<Scalar>, N>& points2D,
              const EntoContainer<Vec3<Scalar>, N>& points3D,
              const CameraPose<Scalar>& true_pose) -> SolverResult<Scalar> {
        
        SolverResult<Scalar> result;
        
        // Convert to homogeneous coordinates - make copies for non-const solver interface
        EntoContainer<Vec3<Scalar>, N> points2D_homo;
        EntoContainer<Vec3<Scalar>, N> points3D_copy;
        convert_2d_to_homogeneous<Scalar, N>(points2D, points2D_homo);
        
        // Copy points3D since solver requires non-const reference
        if constexpr (N == 0) {
            points3D_copy.clear();
            points3D_copy.reserve(points3D.size());
        }
        for (size_t i = 0; i < points3D.size(); ++i) {
            points3D_copy.push_back(points3D[i]);
        }
        
        ENTO_DEBUG("[DLT] Using all %zu points for DLT system", points2D.size());
        
        // Debug: Print input data for DLT (first few points)
        ENTO_DEBUG("[DLT] Input 3D points (first 6):");
        for (size_t i = 0; i < std::min(size_t(6), points3D_copy.size()); ++i) {
            ENTO_DEBUG("  X[%zu] = (%f, %f, %f)", i, points3D_copy[i](0), points3D_copy[i](1), points3D_copy[i](2));
        }
        ENTO_DEBUG("[DLT] Input 2D homogeneous points (first 6):");
        for (size_t i = 0; i < std::min(size_t(6), points2D_homo.size()); ++i) {
            ENTO_DEBUG("  x[%zu] = (%f, %f, %f)", i, points2D_homo[i](0), points2D_homo[i](1), points2D_homo[i](2));
        }
        
        // Debug: Print true pose
        ENTO_DEBUG("[DLT] True pose: R = [%f %f %f; %f %f %f; %f %f %f], t = [%f, %f, %f]",
                   true_pose.R()(0,0), true_pose.R()(0,1), true_pose.R()(0,2),
                   true_pose.R()(1,0), true_pose.R()(1,1), true_pose.R()(1,2),
                   true_pose.R()(2,0), true_pose.R()(2,1), true_pose.R()(2,2),
                   true_pose.t(0), true_pose.t(1), true_pose.t(2));
        
        // Build true projection matrix for reference
        Eigen::Matrix<Scalar, 3, 4> P_true;
        P_true.template block<3,3>(0,0) = true_pose.R();
        P_true.col(3) = true_pose.t;
        ENTO_DEBUG("[DLT] True projection matrix P_true:");
        ENTO_DEBUG("  [%f %f %f %f]", P_true(0,0), P_true(0,1), P_true(0,2), P_true(0,3));
        ENTO_DEBUG("  [%f %f %f %f]", P_true(1,0), P_true(1,1), P_true(1,2), P_true(1,3));
        ENTO_DEBUG("  [%f %f %f %f]", P_true(2,0), P_true(2,1), P_true(2,2), P_true(2,3));
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 1> solutions;
        ENTO_DEBUG("[DLT] Calling SolverDLT::solve<%zu>...", N);
        int num_solutions = SolverDLT<Scalar>::template solve<N>(points2D_homo, points3D_copy, &solutions);
        ENTO_DEBUG("[DLT] Solver returned %d solutions", num_solutions);

        if (num_solutions > 0) {
            const CameraPose<Scalar>& estimated_pose = solutions[0];
            ENTO_DEBUG("[DLT] Estimated pose: R = [%f %f %f; %f %f %f; %f %f %f], t = [%f, %f, %f]",
                       estimated_pose.R()(0,0), estimated_pose.R()(0,1), estimated_pose.R()(0,2),
                       estimated_pose.R()(1,0), estimated_pose.R()(1,1), estimated_pose.R()(1,2),
                       estimated_pose.R()(2,0), estimated_pose.R()(2,1), estimated_pose.R()(2,2),
                       estimated_pose.t(0), estimated_pose.t(1), estimated_pose.t(2));
            
            // Compute reprojection errors
            Scalar max_reproj_error = Scalar(0);
            Scalar avg_reproj_error = Scalar(0);
            for (size_t i = 0; i < points3D.size(); ++i) {
                Vec3<Scalar> X_cam = estimated_pose.R() * points3D[i] + estimated_pose.t;
                if (X_cam(2) > Scalar(1e-6)) {
                    Vec2<Scalar> x_proj(X_cam(0) / X_cam(2), X_cam(1) / X_cam(2));
                    Scalar reproj_err = (x_proj - points2D[i]).norm();
                    max_reproj_error = std::max(max_reproj_error, reproj_err);
                    avg_reproj_error += reproj_err;
                    ENTO_DEBUG("[DLT] Point %zu: 3D=(%f,%f,%f) -> proj=(%f,%f), true=(%f,%f), error=%f", 
                               i, points3D[i](0), points3D[i](1), points3D[i](2),
                               x_proj(0), x_proj(1), points2D[i](0), points2D[i](1), reproj_err);
                } else {
                    ENTO_DEBUG("[DLT] Point %zu: behind camera (depth=%f)", i, X_cam(2));
                }
            }
            avg_reproj_error /= Scalar(points3D.size());
            ENTO_DEBUG("[DLT] Reprojection errors: avg=%f, max=%f", avg_reproj_error, max_reproj_error);
            
            result.num_solutions = num_solutions;
            result.best_error = avg_reproj_error;
            
            // Compute proper pose errors for success evaluation
            result.errors = compute_pose_errors<Scalar, N>(true_pose, estimated_pose, points2D, points3D, false);
            ENTO_DEBUG("[DLT] Computed pose errors: rot=%f°, trans=%f m, reproj=%f", 
                       result.errors.rotation_error_deg, result.errors.translation_error_m, result.errors.reprojection_error);
            
            // SURGICAL DEBUG: Investigate K matrix / pose extraction issues
            // If reprojection is good but pose errors are high, this suggests projection matrix P is correct
            // but pose extraction from P is wrong (likely due to K matrix assumptions)
            if (result.errors.reprojection_error < Scalar(0.1) && 
                (result.errors.rotation_error_deg > Scalar(10.0) || result.errors.translation_error_m > Scalar(0.5))) {
                ENTO_DEBUG("=== POSE EXTRACTION ISSUE DETECTED ===");
                ENTO_DEBUG("Good reprojection (%.4f) but bad pose (rot=%.2f°, trans=%.2f m)", 
                           result.errors.reprojection_error, result.errors.rotation_error_deg, result.errors.translation_error_m);
                
                // Build projection matrices for comparison
                Eigen::Matrix<Scalar, 3, 4> P_true, P_estimated;
                P_true.template block<3,3>(0,0) = true_pose.R();
                P_true.col(3) = true_pose.t;
                P_estimated.template block<3,3>(0,0) = estimated_pose.R();
                P_estimated.col(3) = estimated_pose.t;
                
                ENTO_DEBUG("True P matrix:      [%.3f %.3f %.3f %.3f; %.3f %.3f %.3f %.3f; %.3f %.3f %.3f %.3f]",
                           P_true(0,0), P_true(0,1), P_true(0,2), P_true(0,3),
                           P_true(1,0), P_true(1,1), P_true(1,2), P_true(1,3),
                           P_true(2,0), P_true(2,1), P_true(2,2), P_true(2,3));
                ENTO_DEBUG("Estimated P matrix: [%.3f %.3f %.3f %.3f; %.3f %.3f %.3f %.3f; %.3f %.3f %.3f %.3f]",
                           P_estimated(0,0), P_estimated(0,1), P_estimated(0,2), P_estimated(0,3),
                           P_estimated(1,0), P_estimated(1,1), P_estimated(1,2), P_estimated(1,3),
                           P_estimated(2,0), P_estimated(2,1), P_estimated(2,2), P_estimated(2,3));
                
                // Check if they're equivalent up to scale
                Scalar scale_factor = P_estimated(2,3) / P_true(2,3);
                ENTO_DEBUG("Scale factor P_est(2,3)/P_true(2,3) = %.6f", scale_factor);
                
                // Check K matrix assumption: if DLT assumes K=I but true K≠I, pose extraction fails
                // Let's see if there's an effective K matrix that makes them match
                Eigen::Matrix<Scalar, 3, 3> K_candidate = P_estimated.template block<3,3>(0,0) * P_true.template block<3,3>(0,0).transpose();
                ENTO_DEBUG("Candidate K matrix: [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]",
                           K_candidate(0,0), K_candidate(0,1), K_candidate(0,2),
                           K_candidate(1,0), K_candidate(1,1), K_candidate(1,2),
                           K_candidate(2,0), K_candidate(2,1), K_candidate(2,2));
                ENTO_DEBUG("=========================================");
            }
        } else {
            std::cout << " ✗ (no solutions)" << std::endl;
        }
        
        return result;
    };
}

// Gold Standard Absolute Pose solver wrapper
template<typename Scalar, size_t N>
auto make_gold_standard_abs_solver() {
    return [](const EntoContainer<Vec2<Scalar>, N>& points2D,
              const EntoContainer<Vec3<Scalar>, N>& points3D,
              const CameraPose<Scalar>& true_pose) -> SolverResult<Scalar> {
        
        SolverResult<Scalar> result;
        
        // Convert to homogeneous coordinates for gold standard
        EntoContainer<Vec3<Scalar>, N> points2D_homo;
        EntoContainer<Vec3<Scalar>, N> points3D_copy;
        convert_2d_to_homogeneous<Scalar, N>(points2D, points2D_homo);
        
        // Copy points3D since solver requires non-const reference
        if constexpr (N == 0) {
            points3D_copy.clear();
            points3D_copy.reserve(points3D.size());
        }
        for (size_t i = 0; i < points3D.size(); ++i) {
            points3D_copy.push_back(points3D[i]);
        }
        
        ENTO_DEBUG("[GoldStandardAbs] Using all %zu points for gold standard system", points2D.size());
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 1> solutions;
        result.num_solutions = SolverGoldStandardAbs<Scalar>::template solve<N>(
            points2D_homo, points3D_copy, &solutions);
        
        if (result.num_solutions > 0) {
            result.errors = compute_pose_errors<Scalar, N>(
                true_pose, solutions[0], points2D, points3D, false);
            result.best_error = result.errors.max_error();
            
            ENTO_DEBUG("[GoldStandardAbs] Computed pose errors: rot=%f°, trans=%f m, reproj=%f", 
                       result.errors.rotation_error_deg, result.errors.translation_error_m, result.errors.reprojection_error);
        } else {
            ENTO_DEBUG("[GoldStandardAbs] No solutions found");
        }
        
        return result;
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
    
    SolverStats<Scalar> stats;
    
    // Create filename
    std::ostringstream filename;
    filename << solver_name << "_noise_" << std::fixed << std::setprecision(3) << noise_level << ".csv";
    
    std::cout << "    Creating " << filename.str() << "..." << std::endl;
    std::ofstream file(filename.str());
    
    for (int i = 0; i < opts.num_problems; ++i) {
        std::cout << "      Problem " << i << "..." << std::flush;
        stats.add_problem();
        
        // Generate data using appropriate method
        EntoContainer<Vec2<Scalar>, N> points2D;
        EntoContainer<Vec3<Scalar>, N> points3D;
        CameraPose<Scalar> true_pose;
        
        if (opts.realistic) {
            generate_realistic_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts, 
                                                      opts.num_points, Scalar(noise_level), i, 
                                                      solver_name.find("up2p") != std::string::npos);
        } else {
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, 
                                                    opts.num_points, Scalar(noise_level), i,
                                                    solver_name.find("up2p") != std::string::npos);
        }
        
        // Test solver and collect statistics
        auto result = solver_func(points2D, points3D, true_pose);
        
        if (result.num_solutions > 0) {
            // Use adaptive pose error thresholds based on solver type and noise level
            Scalar rotation_threshold, translation_threshold;
            
            if (noise_level == 0.0) {
                // Clean data - expect very good results
                rotation_threshold = Scalar(1.0);   // 1 degree
                translation_threshold = Scalar(0.1); // 0.1 meters
            } else {
                // Noisy data - more lenient thresholds
                if (solver_name.find("dlt") != std::string::npos) {
                    // DLT is noise sensitive, allow higher errors
                    rotation_threshold = Scalar(15.0);  // 15 degrees
                    translation_threshold = Scalar(1.0); // 1 meter
                } else {
                    // P3P/UP2P are generally more robust
                    rotation_threshold = Scalar(10.0);  // 10 degrees
                    translation_threshold = Scalar(0.5); // 0.5 meters
                }
            }
            
            // Success if both rotation and translation errors are within thresholds
            bool pose_success = (result.errors.rotation_error_deg < rotation_threshold && 
                                result.errors.translation_error_m < translation_threshold);
            
            if (pose_success) { 
                stats.add_success(result.errors, result.num_solutions);
                std::cout << " ✓ (rot: " << std::fixed << std::setprecision(2) 
                          << result.errors.rotation_error_deg << "°, trans: " 
                          << std::setprecision(3) << result.errors.translation_error_m 
                          << "m, reproj: " << std::setprecision(4) 
                          << result.errors.reprojection_error << ")" << std::endl;
            } else {
                stats.add_failure_with_solution(result.errors);
                // Show which threshold was exceeded
                std::string reason = "";
                if (result.errors.rotation_error_deg >= rotation_threshold) 
                    reason += "rot>" + std::to_string(rotation_threshold) + "° ";
                if (result.errors.translation_error_m >= translation_threshold)
                    reason += "trans>" + std::to_string(translation_threshold) + "m ";
                
                std::cout << " ✗ (" << reason << "| rot: " << std::fixed << std::setprecision(2) 
                          << result.errors.rotation_error_deg << "°, trans: " 
                          << std::setprecision(3) << result.errors.translation_error_m 
                          << "m, reproj: " << std::setprecision(4) 
                          << result.errors.reprojection_error << ")" << std::endl;
            }
        } else {
            stats.add_failure();
            std::cout << " ✗ (no solutions)" << std::endl;
        }
        
        // Write CSV line
        std::string csv_line = make_csv_line_absolute_pose<Scalar, N>(
            true_pose, points2D, points3D, Scalar(1.0), Scalar(opts.focal_length));
        file << csv_line << std::endl;
    }
    
    file.close();
    std::cout << "    " << filename.str() << " complete." << std::endl;
    
    // Print statistics
    stats.print_stats(solver_name, noise_level);
}

// Generic solver data generation function WITH OUTLIER INJECTION for LO-RANSAC
template<typename Scalar, size_t N, typename SolverFunc>
void generate_solver_data_for_noise_level_robust(
    const Options& opts,
    const std::string& solver_name,
    SolverFunc solver_func,
    double noise_level,
    double outlier_ratio)
{
    std::cout << "  Generating " << solver_name << " ROBUST data (noise=" << noise_level 
              << ", outliers=" << outlier_ratio << ")..." << std::endl;
    
    SolverStats<Scalar> stats;
    
    // Create filename with outlier ratio
    std::ostringstream filename;
    filename << solver_name << "_noise_" << std::fixed << std::setprecision(3) << noise_level 
             << "_outliers_" << std::setprecision(3) << outlier_ratio << ".csv";
    
    std::cout << "    Creating " << filename.str() << "..." << std::endl;
    std::ofstream file(filename.str());
    
    // Fixed seed for reproducible outlier injection
    std::default_random_engine outlier_rng(42);
    
    for (int i = 0; i < opts.num_problems; ++i) {
        std::cout << "      Problem " << i << "..." << std::flush;
        stats.add_problem();
        
        // Generate clean data first
        EntoContainer<Vec2<Scalar>, N> points2D;
        EntoContainer<Vec3<Scalar>, N> points3D;
        CameraPose<Scalar> true_pose;
        
        if (opts.realistic) {
            generate_realistic_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts, 
                                                      opts.num_points, Scalar(noise_level), i, 
                                                      solver_name.find("up2p") != std::string::npos);
        } else {
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, 
                                                    opts.num_points, Scalar(noise_level), i,
                                                    solver_name.find("up2p") != std::string::npos);
        }
        
        // Inject outliers 
        EntoContainer<bool, N> inlier_mask;
        inject_outliers<Scalar, N>(points2D, points3D, inlier_mask, outlier_ratio, outlier_rng);
        
        // For robust benchmarks, we don't run the solver here - just generate data
        // The benchmark will run LO-RANSAC during execution
        std::cout << " ✓ (data generated with " << static_cast<int>(outlier_ratio * 100) << "% outliers)" << std::endl;
        
        // Write robust CSV line with inlier mask
        std::string csv_line = make_csv_line_robust_absolute_pose<Scalar, N>(
            true_pose, points2D, points3D, inlier_mask, Scalar(1.0), Scalar(opts.focal_length));
        file << csv_line << std::endl;
    }
    
    file.close();
    std::cout << "    " << filename.str() << " complete." << std::endl;
    
    // Note: No solver statistics for robust data generation - that happens in benchmarks
    std::cout << "    Generated " << opts.num_problems << " problems with " 
              << static_cast<int>(outlier_ratio * 100) << "% outliers." << std::endl;
}

// Generic function to generate data for any solver across all noise levels (parameterized)
template<typename Scalar, size_t N, typename SolverFunc>
void generate_solver_data_parameterized(
    const Options& opts,
    const std::string& solver_name,
    SolverFunc solver_func)
{
    std::cout << "Generating " << solver_name << " data..." << std::endl;
    
    if (opts.generate_outliers) {
        // Generate robust data with outliers for each noise level AND outlier ratio
        for (double noise_level : opts.noise_levels) {
            for (double outlier_ratio : opts.outlier_ratios) {
                generate_solver_data_for_noise_level_robust<Scalar, N>(opts, solver_name, solver_func, noise_level, outlier_ratio);
            }
        }
    } else {
        // Generate clean data for each noise level
    for (double noise_level : opts.noise_levels) {
        generate_solver_data_for_noise_level<Scalar, N>(opts, solver_name, solver_func, noise_level);
        }
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
    std::cout << "  Generate P3P: " << (opts.generate_p3p ? "yes" : "no") << std::endl;
    std::cout << "  Generate UP2P: " << (opts.generate_up2p ? "yes" : "no") << std::endl;
    std::cout << "  Generate DLT: " << (opts.generate_dlt ? "yes" : "no") << std::endl;
    std::cout << "  Generate Gold Standard Abs: " << (opts.generate_gold_standard_abs ? "yes" : "no") << std::endl;
    std::cout << std::endl;
    
    // Linear solver: DLT supports overdetermined systems
    if constexpr (N >= 6) {
        if (opts.generate_dlt) {
            generate_solver_data_parameterized<Scalar, N>(opts, "dlt", make_dlt_solver<Scalar, N>());
        }
        
        if (opts.generate_gold_standard_abs) {
            generate_solver_data_parameterized<Scalar, N>(opts, "gold_standard_abs", make_gold_standard_abs_solver<Scalar, N>());
        }
    } else {
        if (opts.generate_dlt) {
            std::cout << "Skipping DLT for N=" << N << " (requires N >= 6)" << std::endl;
        }
        if (opts.generate_gold_standard_abs) {
            std::cout << "Skipping Gold Standard Abs for N=" << N << " (requires N >= 6)" << std::endl;
        }
    }
    
    // Non-linear solvers: only exact minimal sizes
    if (opts.generate_p3p && N == 3) {
        generate_solver_data_parameterized<Scalar, N>(opts, "p3p", make_p3p_solver<Scalar, N>());
    }
    
    if (opts.generate_up2p && N == 2) {
        generate_solver_data_parameterized<Scalar, N>(opts, "up2p", make_up2p_solver<Scalar, N>());
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
        case 6: run_generation_specialized<Scalar, 6>(opts); break;
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
            std::cout << "Supported sizes: 2, 3, 4, 5, 6, 8, 10, 16, 20, 30, 32, 40, 50, 64, 80, 100, 128" << std::endl;
            std::cout << "Linear solver (DLT): supports overdetermined systems (N >= 6)" << std::endl;
            std::cout << "Non-linear solvers (P3P, UP2P): require exact minimal sizes" << std::endl;
            exit(1);
    }
}

// Main generation function with template dispatch
template<typename Scalar>
void run_generation(const Options& opts) {
    run_generation_with_size<Scalar>(opts, opts.num_points);
}

int main(int argc, char** argv)
{
    Options opts = parse_args(argc, argv);
    
    std::cout << "Enhanced Absolute Pose Minimal Solver Data Generator" << std::endl;
    std::cout << "====================================================" << std::endl;
    
    if (opts.use_double) {
        run_generation<double>(opts);
    } else {
        run_generation<float>(opts);
    }
    
    std::cout << "\n=== Data Generation Complete! ===" << std::endl;
    
    if (opts.generate_outliers) {
        std::cout << "Generated ROBUST files with outliers:" << std::endl;
        for (double noise_level : opts.noise_levels) {
            for (double outlier_ratio : opts.outlier_ratios) {
                std::cout << "\nNoise level " << noise_level << ", Outlier ratio " << outlier_ratio << ":" << std::endl;
                if (opts.generate_p3p) {
                    std::cout << "  - p3p_noise_" << std::fixed << std::setprecision(3) << noise_level 
                              << "_outliers_" << std::setprecision(3) << outlier_ratio << ".csv" << std::endl;
                }
                if (opts.generate_up2p) {
                    std::cout << "  - up2p_noise_" << std::fixed << std::setprecision(3) << noise_level 
                              << "_outliers_" << std::setprecision(3) << outlier_ratio << ".csv" << std::endl;
                }
                if (opts.generate_dlt) {
                    std::cout << "  - dlt_noise_" << std::fixed << std::setprecision(3) << noise_level 
                              << "_outliers_" << std::setprecision(3) << outlier_ratio << ".csv" << std::endl;
                }
                if (opts.generate_gold_standard_abs) {
                    std::cout << "  - gold_standard_abs_noise_" << std::fixed << std::setprecision(3) << noise_level 
                              << "_outliers_" << std::setprecision(3) << outlier_ratio << ".csv" << std::endl;
                }
            }
        }
        std::cout << "\nAll robust files are ready for LO-RANSAC benchmarking!" << std::endl;
    } else {
        std::cout << "Generated files for each noise level:" << std::endl;
    for (double noise_level : opts.noise_levels) {
        std::cout << "\nNoise level " << noise_level << ":" << std::endl;
        if (opts.generate_p3p) {
            std::cout << "  - p3p_noise_" << std::fixed << std::setprecision(3) << noise_level << ".csv" << std::endl;
        }
        if (opts.generate_up2p) {
            std::cout << "  - up2p_noise_" << std::fixed << std::setprecision(3) << noise_level << ".csv" << std::endl;
        }
        if (opts.generate_dlt) {
            std::cout << "  - dlt_" << opts.num_points << "pt_noise_" << std::fixed << std::setprecision(3) << noise_level << ".csv" << std::endl;
        }
        if (opts.generate_gold_standard_abs) {
            std::cout << "  - gold_standard_abs_noise_" << std::fixed << std::setprecision(3) << noise_level << ".csv" << std::endl;
        }
    }
    std::cout << "\nAll files are ready for microcontroller benchmarking!" << std::endl;
    }
    return 0;
} 