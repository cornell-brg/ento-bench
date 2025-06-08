#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>
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

// Helper structure to hold both rotation and translation errors
template<typename Scalar>
struct PoseErrors {
    Scalar rotation_error_deg;
    Scalar translation_error_deg;
    
    PoseErrors(Scalar rot_err, Scalar trans_err) 
        : rotation_error_deg(rot_err), translation_error_deg(trans_err) {}
    
    // Get the maximum error for backward compatibility
    Scalar max_error() const {
        return std::max(rotation_error_deg, translation_error_deg);
    }
};

// Helper function to compute pose error for relative pose
template<typename Scalar>
PoseErrors<Scalar> compute_pose_error(const CameraPose<Scalar>& true_pose, const CameraPose<Scalar>& estimated_pose)
{
    // Rotation error (angular difference between rotation matrices)
    Eigen::Matrix<Scalar,3,3> R_diff = estimated_pose.R().transpose() * true_pose.R();
    Scalar trace_val = R_diff.trace();
    Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
    Scalar rotation_error_deg = angle_rad * Scalar(180.0 / M_PI);
    
    // Translation direction error (angular difference between translation directions)
    Vec3<Scalar> t_est = estimated_pose.t.normalized();
    Vec3<Scalar> t_true = true_pose.t.normalized();
    
    // Handle both +t and -t directions (relative pose translation is up to scale)
    Scalar dot_pos = std::clamp(t_est.dot(t_true), Scalar(-1), Scalar(1));
    Scalar dot_neg = std::clamp(t_est.dot(-t_true), Scalar(-1), Scalar(1));
    Scalar abs_dot = std::max(std::abs(dot_pos), std::abs(dot_neg));
    Scalar translation_error_deg = std::acos(abs_dot) * Scalar(180.0 / M_PI);
    
    return PoseErrors<Scalar>(rotation_error_deg, translation_error_deg);
}

// Statistics tracking structure with separate rotation/translation errors
template<typename Scalar>
struct SolverStats {
    int total_problems = 0;
    int successful_solves = 0;
    int failed_solves = 0;
    
    // Rotation error statistics
    Scalar total_rotation_error = 0.0;
    Scalar min_rotation_error = std::numeric_limits<Scalar>::max();
    Scalar max_rotation_error = 0.0;
    std::vector<Scalar> rotation_errors; // Store all values for std dev
    
    // Translation error statistics  
    Scalar total_translation_error = 0.0;
    Scalar min_translation_error = std::numeric_limits<Scalar>::max();
    Scalar max_translation_error = 0.0;
    std::vector<Scalar> translation_errors; // Store all values for std dev
    
    // Combined error statistics
    Scalar total_combined_error = 0.0;
    Scalar min_combined_error = std::numeric_limits<Scalar>::max();
    Scalar max_combined_error = 0.0;
    std::vector<Scalar> combined_errors; // Store all values for std dev
    
    // Solution count statistics
    int total_solutions_found = 0;
    int problems_with_multiple_solutions = 0;
    
    void add_success(const PoseErrors<Scalar>& errors, int num_solutions) {
        successful_solves++;
        
        // Rotation errors
        total_rotation_error += errors.rotation_error_deg;
        min_rotation_error = std::min(min_rotation_error, errors.rotation_error_deg);
        max_rotation_error = std::max(max_rotation_error, errors.rotation_error_deg);
        rotation_errors.push_back(errors.rotation_error_deg);
        
        // Translation errors
        total_translation_error += errors.translation_error_deg;
        min_translation_error = std::min(min_translation_error, errors.translation_error_deg);
        max_translation_error = std::max(max_translation_error, errors.translation_error_deg);
        translation_errors.push_back(errors.translation_error_deg);
        
        // Combined errors
        Scalar combined = errors.max_error();
        total_combined_error += combined;
        min_combined_error = std::min(min_combined_error, combined);
        max_combined_error = std::max(max_combined_error, combined);
        combined_errors.push_back(combined);
        
        // Solution count
        total_solutions_found += num_solutions;
        if (num_solutions > 1) {
            problems_with_multiple_solutions++;
        }
    }
    
    void add_failure() {
        failed_solves++;
    }
    
    void add_problem() {
        total_problems++;
    }
    
    // Helper function to compute standard deviation
    Scalar compute_std_dev(const std::vector<Scalar>& values, Scalar mean) const {
        if (values.size() <= 1) return 0.0;
        
        Scalar sum_sq_diff = 0.0;
        for (Scalar val : values) {
            Scalar diff = val - mean;
            sum_sq_diff += diff * diff;
        }
        return std::sqrt(sum_sq_diff / (values.size() - 1)); // Sample std dev
    }
    
    void print_stats(const std::string& solver_name) const {
        std::cout << "\n=== " << solver_name << " Statistics ===" << std::endl;
        std::cout << "  Total problems: " << total_problems << std::endl;
        std::cout << "  Successful solves: " << successful_solves << " (" 
                  << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
        std::cout << "  Failed solves: " << failed_solves << " (" 
                  << (100.0 * failed_solves / total_problems) << "%)" << std::endl;
        
        if (successful_solves > 0) {
            // Rotation error statistics
            Scalar avg_rot = total_rotation_error / successful_solves;
            Scalar std_rot = compute_std_dev(rotation_errors, avg_rot);
            std::cout << "  --- Rotation Errors ---" << std::endl;
            std::cout << "    Average: " << avg_rot << " degrees" << std::endl;
            std::cout << "    Std Dev: " << std_rot << " degrees" << std::endl;
            std::cout << "    Min: " << min_rotation_error << " degrees" << std::endl;
            std::cout << "    Max: " << max_rotation_error << " degrees" << std::endl;
            
            // Translation error statistics
            Scalar avg_trans = total_translation_error / successful_solves;
            Scalar std_trans = compute_std_dev(translation_errors, avg_trans);
            std::cout << "  --- Translation Errors ---" << std::endl;
            std::cout << "    Average: " << avg_trans << " degrees" << std::endl;
            std::cout << "    Std Dev: " << std_trans << " degrees" << std::endl;
            std::cout << "    Min: " << min_translation_error << " degrees" << std::endl;
            std::cout << "    Max: " << max_translation_error << " degrees" << std::endl;
            
            // Combined error statistics
            Scalar avg_combined = total_combined_error / successful_solves;
            Scalar std_combined = compute_std_dev(combined_errors, avg_combined);
            std::cout << "  --- Combined Errors (max of rot/trans) ---" << std::endl;
            std::cout << "    Average: " << avg_combined << " degrees" << std::endl;
            std::cout << "    Std Dev: " << std_combined << " degrees" << std::endl;
            std::cout << "    Min: " << min_combined_error << " degrees" << std::endl;
            std::cout << "    Max: " << max_combined_error << " degrees" << std::endl;
            
            // Solution count statistics
            std::cout << "  --- Solution Statistics ---" << std::endl;
            std::cout << "    Average solutions per problem: " 
                      << (float(total_solutions_found) / successful_solves) << std::endl;
            std::cout << "    Problems with multiple solutions: " << problems_with_multiple_solutions 
                      << " (" << (100.0 * problems_with_multiple_solutions / successful_solves) << "%)" << std::endl;
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

// Unified data generation with pixel noise for fair comparison
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
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem to avoid problematic poses
    std::default_random_engine rng(42 + problem_id);
    
    // Set random true pose
    if (upright_only) {
        // Set random upright pose (only y-axis rotation)
        Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        if (planar_only) {
            // Planar motion: only x and z translation
            true_pose.t = Vec3(
                static_cast<Scalar>(rand()) / RAND_MAX * 4 - 2,  // x: [-2, 2]
                0,  // y: 0 (planar)
                static_cast<Scalar>(rand()) / RAND_MAX * 4 - 2   // z: [-2, 2]
            );
        } else {
            true_pose.t.setRandom();
            true_pose.t *= Scalar(2.0);
        }
    } else {
        // General pose
        true_pose.q = Quaternion::UnitRandom().coeffs();
        true_pose.t.setRandom();
        true_pose.t *= Scalar(2.0); // Scale translation
    }
    
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    
    if constexpr (N == 0) {
        x1_bear.clear();
        x2_bear.clear();
        x1_bear.reserve(num_points);
        x2_bear.reserve(num_points);
    }
    
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x1_cam, x2_cam;
        
        // Add safety counter to prevent infinite loops (same as working version)
        int attempts = 0;
        const int max_attempts = 10000;
        while (true) {
            // Generate 3D point
            X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            
            // Project to first camera (identity)
            x1_cam = X;
            // Project to second camera
            x2_cam = true_pose.R() * X + true_pose.t;
            
            if (x1_cam(2) > Scalar(0.1) && x2_cam(2) > Scalar(0.1)) break; // Accept only if in front of both cameras
            
            ++attempts;
            if (attempts >= max_attempts) {
                std::cout << "    Warning: Could not find point in front of both cameras after " << max_attempts << " attempts for problem " << problem_id << std::endl;
                std::cout << "    Regenerating pose..." << std::endl;
                // Regenerate pose and try again (same as working version)
                if (upright_only) {
                    Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
                    Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
                    true_pose.q(0) = q.w();
                    true_pose.q(1) = q.x();
                    true_pose.q(2) = q.y();
                    true_pose.q(3) = q.z();
                    
                    if (planar_only) {
                        true_pose.t = Vec3(
                            static_cast<Scalar>(rand()) / RAND_MAX * 4 - 2,  // x: [-2, 2]
                            0,  // y: 0 (planar)
                            static_cast<Scalar>(rand()) / RAND_MAX * 4 - 2   // z: [-2, 2]
                        );
                    } else {
                        true_pose.t.setRandom();
                        true_pose.t *= Scalar(2.0);
                    }
                } else {
                    true_pose.q = Quaternion::UnitRandom().coeffs();
                    true_pose.t.setRandom();
                    true_pose.t *= Scalar(2.0);
                }
                attempts = 0;
            }
        }
        
        // Bearing vectors (unit vectors in camera frames)
        Vec3 f1 = x1_cam.normalized();
        Vec3 f2 = x2_cam.normalized();
        
        // Add noise in tangent space (small angle approx)
        if (noise_level > Scalar(0.0)) {
            // Add noise to f1
            Vec3 n1 = Vec3::Random().normalized();
            n1 -= n1.dot(f1) * f1;
            n1.normalize();
            Vec3 n2 = f1.cross(n1).normalized();
            Scalar theta = noise_gen(rng);
            Scalar phi = noise_gen(rng);
            f1 = (f1 + theta * n1 + phi * n2).normalized();
            
            // Add noise to f2
            n1 = Vec3::Random().normalized();
            n1 -= n1.dot(f2) * f2;
            n1.normalize();
            n2 = f2.cross(n1).normalized();
            theta = noise_gen(rng);
            phi = noise_gen(rng);
            f2 = (f2 + theta * n1 + phi * n2).normalized();
        }
        
        x1_bear.push_back(f1);
        x2_bear.push_back(f2);
        ++valid_points;
    }
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
        // Upright pose: only y-axis rotation with controlled magnitude
        Scalar yaw_deg = rotation_magnitude_deg * uniform_gen(rng); // ±rotation_magnitude
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        if (planar_only) {
            // Planar motion: controlled baseline in XZ plane
            Scalar tx = baseline_magnitude * uniform_gen(rng);
            Scalar tz = baseline_magnitude * uniform_gen(rng);
            true_pose.t = Vec3(tx, 0, tz);
        } else {
            // Controlled 3D translation with fixed magnitude
            Vec3 t_dir = Vec3::Random().normalized();
            true_pose.t = baseline_magnitude * t_dir;
        }
    } else {
        // General 6-DoF pose with controlled magnitudes
        // Rotation: controlled magnitude around random axis
        Vec3 rot_axis = Vec3::Random().normalized();
        Scalar rot_angle_deg = rotation_magnitude_deg * uniform_gen(rng);
        Scalar rot_angle_rad = rot_angle_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(rot_angle_rad, rot_axis));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // Translation: fixed baseline magnitude
        Vec3 t_dir = Vec3::Random().normalized();
        true_pose.t = baseline_magnitude * t_dir;
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
        
        // Generate 3D point in realistic world coordinates
        Vec3 X = Vec3(
            uniform_gen(rng) * 5.0,  // ±5m world coordinates
            uniform_gen(rng) * 5.0,
            depth_gen(rng)           // 2-10m depth
        );
        
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
        EntoContainer<Vec3<Scalar>, 0> x1_8pt, x2_8pt;
        for (int j = 0; j < 8; ++j) {
            x1_8pt.push_back(x1[j]);
            x2_8pt.push_back(x2[j]);
        }
        
        return SolverRel8pt<Scalar>::solve(x1_8pt, x2_8pt, solutions);
    };
}

template<typename Scalar, size_t N>
auto make_5pt_solver() {
    return [](const EntoContainer<Vec3<Scalar>, N>& x1, 
              const EntoContainer<Vec3<Scalar>, N>& x2,
              std::vector<CameraPose<Scalar>>* solutions) -> int {
        // Use first 5 points for 5pt solver
        EntoContainer<Vec3<Scalar>, 5> x1_5pt, x2_5pt;
        for (int j = 0; j < 5; ++j) {
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
        EntoContainer<Vec3<Scalar>, 3> x1_3pt, x2_3pt;
        for (int j = 0; j < 3; ++j) {
            x1_3pt.push_back(x1[j]);
            x2_3pt.push_back(x2[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions_array;
        int num_solutions = SolverRelUpright3pt<Scalar>::template solve<3>(x1_3pt, x2_3pt, &solutions_array);
        
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
        EntoContainer<Vec3<Scalar>, 3> x1_3pt, x2_3pt;
        for (int j = 0; j < 3; ++j) {
            x1_3pt.push_back(x1[j]);
            x2_3pt.push_back(x2[j]);
        }
        
        EntoUtil::EntoArray<CameraPose<Scalar>, 2> solutions_array;
        int num_solutions = SolverRelUprightPlanar3pt<Scalar>::template solve<3>(x1_3pt, x2_3pt, &solutions_array);
        
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
        for (int j = 0; j < 2; ++j) {
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
            PoseErrors<Scalar> best_error = compute_pose_error(true_pose, solutions[0]);
            for (size_t s = 1; s < solutions.size(); ++s) {
                PoseErrors<Scalar> error = compute_pose_error(true_pose, solutions[s]);
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
                stats.add_failure();
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
                            (opts.use_realistic_generation ? " pixels)" : " radians)");
    stats.print_stats(stats_name);
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