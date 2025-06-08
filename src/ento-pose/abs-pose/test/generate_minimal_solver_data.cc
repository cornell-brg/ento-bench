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
#include <ento-pose/pose_util.h>
#include <ento-pose/synthetic_abspose.h>

// Include the solver interfaces from data_gen.h
#include <ento-pose/data_gen.h>

using namespace EntoPose;
using namespace EntoUtil;

// Command line options structure
struct Options {
    int num_problems = 100;
    int num_points = 100;
    bool generate_p3p = true;
    bool generate_up2p = true;
    bool generate_dlt = true;
    bool use_double = false;
    int dlt_min_points = 6;
    bool run_all_types = false;
    
    // Supported DLT specialization sizes
    static const std::vector<int>& get_supported_dlt_sizes() {
        static const std::vector<int> sizes = {6, 8, 16, 32, 64, 128};
        return sizes;
    }
    
    bool is_valid_dlt_size(int size) const {
        const auto& sizes = get_supported_dlt_sizes();
        return std::find(sizes.begin(), sizes.end(), size) != sizes.end();
    }
    
    void print_help() {
        std::cout << "Usage: generate_minimal_solver_data_abs [options]\n";
        std::cout << "Options:\n";
        std::cout << "  --problems N      Number of problems to generate (default: 100)\n";
        std::cout << "  --points N        Number of points per problem (default: 100)\n";
        std::cout << "  --dlt-min N       Minimum points for DLT solver (default: 6)\n";
        std::cout << "                    Supported values: ";
        const auto& sizes = get_supported_dlt_sizes();
        for (size_t i = 0; i < sizes.size(); ++i) {
            std::cout << sizes[i];
            if (i < sizes.size() - 1) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "  --p3p-only        Generate only P3P data\n";
        std::cout << "  --up2p-only       Generate only UP2P data\n";
        std::cout << "  --dlt-only        Generate only DLT data\n";
        std::cout << "  --all-types       Run all solver types with both float and double\n";
        std::cout << "  --double          Use double precision (default: float)\n";
        std::cout << "  --help            Show this help message\n";
        std::cout << "\nExamples:\n";
        std::cout << "  # Generate DLT data with 16 minimum points:\n";
        std::cout << "  ./generate_minimal_solver_data_abs --dlt-only --dlt-min 16\n";
        std::cout << "  # Generate 50 problems with 200 points each:\n";
        std::cout << "  ./generate_minimal_solver_data_abs --problems 50 --points 200\n";
        std::cout << "  # Test all combinations (float/double, different DLT sizes):\n";
        std::cout << "  ./generate_minimal_solver_data_abs --all-types\n";
        std::cout << "  # Use double precision:\n";
        std::cout << "  ./generate_minimal_solver_data_abs --double\n";
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
        } else if (strcmp(argv[i], "--dlt-min") == 0 && i + 1 < argc) {
            opts.dlt_min_points = std::atoi(argv[++i]);
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
        } else if (strcmp(argv[i], "--all-types") == 0) {
            opts.run_all_types = true;
        } else if (strcmp(argv[i], "--double") == 0) {
            opts.use_double = true;
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
    if (opts.dlt_min_points < 6) {
        std::cout << "Error: DLT requires at least 6 points\n";
        exit(1);
    }
    if (!opts.is_valid_dlt_size(opts.dlt_min_points)) {
        std::cout << "Error: DLT minimum points must be one of the supported values: ";
        const auto& sizes = opts.get_supported_dlt_sizes();
        for (size_t i = 0; i < sizes.size(); ++i) {
            std::cout << sizes[i];
            if (i < sizes.size() - 1) std::cout << ", ";
        }
        std::cout << "\n";
        exit(1);
    }
    if (opts.dlt_min_points > opts.num_points) {
        std::cout << "Error: DLT minimum points cannot exceed total points\n";
        exit(1);
    }
    
    return opts;
}

// Helper function to compute pose error
template<typename Scalar>
Scalar compute_pose_error(const CameraPose<Scalar>& true_pose, const CameraPose<Scalar>& estimated_pose)
{
    // Rotation error (angular)
    double trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
    double angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
    double angle_deg = angle_rad * 180.0 / M_PI;
    
    // Translation error (normalized direction)
    auto t_est = estimated_pose.t.normalized();
    auto t_true = true_pose.t.normalized();
    Scalar dot = std::clamp(t_est.dot(t_true), Scalar(-1), Scalar(1));
    Scalar trans_angle = std::acos(dot) * Scalar(180.0 / M_PI);
    
    return std::max(Scalar(angle_deg), trans_angle);
}

// Statistics tracking structure
template<typename Scalar>
struct SolverStats {
    int total_problems = 0;
    int successful_solves = 0;
    int failed_solves = 0;
    Scalar total_error = 0.0;
    Scalar min_error = std::numeric_limits<Scalar>::max();
    Scalar max_error = 0.0;
    int total_solutions_found = 0;
    int problems_with_multiple_solutions = 0;
    
    void add_success(Scalar error, int num_solutions) {
        successful_solves++;
        total_error += error;
        min_error = std::min(min_error, error);
        max_error = std::max(max_error, error);
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
    
    void print_stats(const std::string& solver_name) const {
        std::cout << "\n=== " << solver_name << " Statistics ===" << std::endl;
        std::cout << "  Total problems: " << total_problems << std::endl;
        std::cout << "  Successful solves: " << successful_solves << " (" 
                  << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
        std::cout << "  Failed solves: " << failed_solves << " (" 
                  << (100.0 * failed_solves / total_problems) << "%)" << std::endl;
        
        if (successful_solves > 0) {
            std::cout << "  Average error: " << (total_error / successful_solves) << " degrees" << std::endl;
            std::cout << "  Min error: " << min_error << " degrees" << std::endl;
            std::cout << "  Max error: " << max_error << " degrees" << std::endl;
            std::cout << "  Average solutions per problem: " 
                      << (float(total_solutions_found) / successful_solves) << std::endl;
            std::cout << "  Problems with multiple solutions: " << problems_with_multiple_solutions 
                      << " (" << (100.0 * problems_with_multiple_solutions / successful_solves) << "%)" << std::endl;
        }
        std::cout << "================================" << std::endl;
    }
};

template <typename Scalar, size_t N>
std::string make_csv_line_absolute_pose(
    const CameraPose<Scalar>& pose,
    const EntoContainer<Vec2<Scalar>, N>& x,
    const EntoContainer<Vec3<Scalar>, N>& X,
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
        const Vec2& pt = x[i];
        oss << pt(0) << ',' << pt(1) << ',';
    }

    // 6) X: dump N points (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3& pt = X[i];
        oss << pt(0) << ',' << pt(1) << ',' << pt(2);
        if (i < N - 1) oss << ',';
    }

    return oss.str();
}

template <typename Scalar, size_t N>
std::string make_csv_line_absolute_pose_bearing(
    const CameraPose<Scalar>& pose,
    const EntoContainer<Vec3<Scalar>, N>& x_bear,
    const EntoContainer<Vec3<Scalar>, N>& X)
{
    std::ostringstream oss;
    
    // 1) problem_type, N
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
    oss << 1.0 << ',' << 1.0 << ',';
    
    // 5) bearing vectors: dump N vectors (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3<Scalar>& bear = x_bear[i];
        oss << bear(0) << ',' << bear(1) << ',' << bear(2) << ',';
    }
    
    // 6) X: dump N points (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3<Scalar>& pt = X[i];
        oss << pt(0) << ',' << pt(1) << ',' << pt(2) << ',';
    }
    
    // Remove trailing comma
    std::string result = oss.str();
    if (!result.empty() && result.back() == ',') {
        result.pop_back();
    }
    
    return result;
}

template <typename Scalar, size_t N>
std::string make_csv_line_absolute_pose_homogeneous(
    const CameraPose<Scalar>& pose,
    const EntoContainer<Vec3<Scalar>, N>& x_homo,
    const EntoContainer<Vec3<Scalar>, N>& X)
{
    std::ostringstream oss;
    
    // 1) problem_type, N
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
    oss << 1.0 << ',' << 1.0 << ',';
    
    // 5) homogeneous 2D points: dump N vectors (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3<Scalar>& homo = x_homo[i];
        oss << homo(0) << ',' << homo(1) << ',' << homo(2) << ',';
    }
    
    // 6) X: dump N points (x, y, z)
    for (size_t i = 0; i < N; ++i) {
        const Vec3<Scalar>& pt = X[i];
        oss << pt(0) << ',' << pt(1) << ',' << pt(2) << ',';
    }
    
    // Remove trailing comma
    std::string result = oss.str();
    if (!result.empty() && result.back() == ',') {
        result.pop_back();
    }
    
    return result;
}

// Unified data generation with pixel noise for fair comparison
// All solvers get the same underlying pixel noise, converted to their preferred format
template<typename Scalar, size_t N>
void generate_unified_abspose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0,
    bool upright_only = false)  // For UP2P constraint
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
    
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X;
        Vec3 x_cam;
        Vec2 x_norm;
        
        // Add safety counter to prevent infinite loops
        int attempts = 0;
        const int max_attempts = 10000;
        while (true) {
            // Generate 3D point
            X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            // Project to camera coordinates
            x_cam = true_pose.R() * X + true_pose.t;
            if (x_cam(2) > Scalar(0.1)) break; // Accept only if in front of camera
            
            ++attempts;
            if (attempts >= max_attempts) {
                std::cout << "    Warning: Could not find point in front of camera after " << max_attempts << " attempts for problem " << problem_id << std::endl;
                std::cout << "    Regenerating pose..." << std::endl;
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
                attempts = 0;
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
        // Convert normalized 2D point to bearing vector
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
        // Convert to homogeneous coordinates [x, y, 1]
        Vec3 homo(pt2d(0), pt2d(1), Scalar(1.0));
        points2D_homo.push_back(homo);
    }
}

template<typename Scalar, size_t N>
void generate_p3p_data(const Options& opts)
{
    std::cout << "Generating P3P data (Scalar=" << (sizeof(Scalar) == 8 ? "double" : "float") 
              << ", N=" << N << ")..." << std::endl;
    
    SolverStats<Scalar> clean_stats, noisy_stats;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating p3p_clean.csv..." << std::endl;
        std::ofstream file("p3p_clean.csv");
        for (int i = 0; i < opts.num_problems; ++i) {
            std::cout << "    Generating P3P problem " << i << "..." << std::endl;
            clean_stats.add_problem();
            
            // Generate unified data with pixel noise
            EntoContainer<Vec2<Scalar>, N> points2D;
            EntoContainer<Vec3<Scalar>, N> points3D;
            CameraPose<Scalar> true_pose;
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts.num_points, 0.0, i);
            
            // Convert 2D points to bearing vectors for P3P
            EntoContainer<Vec3<Scalar>, N> x_bear;
            convert_2d_to_bearing_vectors<Scalar, N>(points2D, x_bear);
            
            // Test P3P solver with first 3 bearing vectors
            EntoContainer<Vec3<Scalar>, 3> x_bear_3;
            EntoContainer<Vec3<Scalar>, 3> points3D_3;
            for (int j = 0; j < 3; ++j) {
                x_bear_3.push_back(x_bear[j]);
                points3D_3.push_back(points3D[j]);
            }
            
            EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions;
            int num_solutions = SolverP3P<Scalar>::template solve<3>(x_bear_3, points3D_3, &solutions);
            
            if (num_solutions > 0) {
                // Find best solution by checking reprojection error
                Scalar best_error = std::numeric_limits<Scalar>::max();
                for (int s = 0; s < num_solutions; ++s) {
                    Scalar error = compute_pose_error(true_pose, solutions[s]);
                    if (error < best_error) {
                        best_error = error;
                    }
                }
                
                if (best_error < Scalar(10.0)) { // 10 degree threshold for clean data
                    clean_stats.add_success(best_error, num_solutions);
                    std::cout << "      P3P solved! Error: " << best_error << " degrees, Solutions: " << num_solutions << std::endl;
                } else {
                    clean_stats.add_failure();
                    std::cout << "      P3P solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                clean_stats.add_failure();
                std::cout << "      P3P failed to find solution" << std::endl;
            }
            
            std::string csv_line = make_csv_line_absolute_pose_bearing<Scalar, N>(
                true_pose, x_bear, points3D);
            
            file << csv_line << std::endl;
            std::cout << "    P3P problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  p3p_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating p3p_noisy.csv..." << std::endl;
        std::ofstream file("p3p_noisy.csv");
        for (int i = 0; i < opts.num_problems; ++i) {
            std::cout << "    Generating P3P noisy problem " << i << "..." << std::endl;
            noisy_stats.add_problem();
            
            // Generate unified data with pixel noise
            EntoContainer<Vec2<Scalar>, N> points2D;
            EntoContainer<Vec3<Scalar>, N> points3D;
            CameraPose<Scalar> true_pose;
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts.num_points, 0.01, i);
            
            // Convert 2D points to bearing vectors for P3P
            EntoContainer<Vec3<Scalar>, N> x_bear;
            convert_2d_to_bearing_vectors<Scalar, N>(points2D, x_bear);
            
            // Test P3P solver with first 3 bearing vectors
            EntoContainer<Vec3<Scalar>, 3> x_bear_3;
            EntoContainer<Vec3<Scalar>, 3> points3D_3;
            for (int j = 0; j < 3; ++j) {
                x_bear_3.push_back(x_bear[j]);
                points3D_3.push_back(points3D[j]);
            }
            
            EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions;
            int num_solutions = SolverP3P<Scalar>::template solve<3>(x_bear_3, points3D_3, &solutions);
            
            if (num_solutions > 0) {
                // Find best solution by checking reprojection error
                Scalar best_error = std::numeric_limits<Scalar>::max();
                for (int s = 0; s < num_solutions; ++s) {
                    Scalar error = compute_pose_error(true_pose, solutions[s]);
                    if (error < best_error) {
                        best_error = error;
                    }
                }
                
                if (best_error < Scalar(15.0)) { // 15 degree threshold for noisy data
                    noisy_stats.add_success(best_error, num_solutions);
                    std::cout << "      P3P solved! Error: " << best_error << " degrees, Solutions: " << num_solutions << std::endl;
                } else {
                    noisy_stats.add_failure();
                    std::cout << "      P3P solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                noisy_stats.add_failure();
                std::cout << "      P3P failed to find solution" << std::endl;
            }
            
            std::string csv_line = make_csv_line_absolute_pose_bearing<Scalar, N>(
                true_pose, x_bear, points3D);
            
            file << csv_line << std::endl;
            std::cout << "    P3P noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  p3p_noisy.csv complete." << std::endl;
    }
    
    std::cout << "P3P data generation complete!" << std::endl;
    clean_stats.print_stats("P3P Clean");
    noisy_stats.print_stats("P3P Noisy");
}

template<typename Scalar, size_t N>
void generate_up2p_data(const Options& opts)
{
    std::cout << "Generating UP2P data (Scalar=" << (sizeof(Scalar) == 8 ? "double" : "float") 
              << ", N=" << N << ")..." << std::endl;
    
    SolverStats<Scalar> clean_stats, noisy_stats;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating up2p_clean.csv..." << std::endl;
        std::ofstream file("up2p_clean.csv");
        for (int i = 0; i < opts.num_problems; ++i) {
            std::cout << "    Generating UP2P problem " << i << "..." << std::endl;
            clean_stats.add_problem();
            
            // Generate unified data with pixel noise (upright constraint)
            EntoContainer<Vec2<Scalar>, N> points2D;
            EntoContainer<Vec3<Scalar>, N> points3D;
            CameraPose<Scalar> true_pose;
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts.num_points, 0.0, i, true);
            
            // Convert 2D points to bearing vectors for UP2P
            EntoContainer<Vec3<Scalar>, N> x_bear;
            convert_2d_to_bearing_vectors<Scalar, N>(points2D, x_bear);
            
            // Test UP2P solver with first 2 points (already bearing vectors)
            EntoContainer<Vec3<Scalar>, 2> x_up2p;
            EntoContainer<Vec3<Scalar>, 2> X_up2p;
            for (int j = 0; j < 2; ++j) {
                x_up2p.push_back(x_bear[j]);
                X_up2p.push_back(points3D[j]);
            }
            
            EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions;
            int num_solutions = SolverUP2P<Scalar>::template solve<2>(x_up2p, X_up2p, &solutions);
            
            if (num_solutions > 0) {
                // Find best solution (closest to ground truth)
                Scalar best_error = std::numeric_limits<Scalar>::max();
                for (int s = 0; s < num_solutions; ++s) {
                    Scalar error = compute_pose_error(true_pose, solutions[s]);
                    if (error < best_error) {
                        best_error = error;
                    }
                }
                
                if (best_error < 10.0) { // Accept if error < 10 degrees
                    clean_stats.add_success(best_error, num_solutions);
                    std::cout << "      UP2P solved! Error: " << best_error << " degrees, Solutions: " << num_solutions << std::endl;
                } else {
                    clean_stats.add_failure();
                    std::cout << "      UP2P solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                clean_stats.add_failure();
                std::cout << "      UP2P failed to find solution" << std::endl;
            }
            
            std::string csv_line = make_csv_line_absolute_pose_bearing<Scalar, N>(
                true_pose, x_bear, points3D);
            
            file << csv_line << std::endl;
            std::cout << "    UP2P problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  up2p_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating up2p_noisy.csv..." << std::endl;
        std::ofstream file("up2p_noisy.csv");
        for (int i = 0; i < opts.num_problems; ++i) {
            std::cout << "    Generating UP2P noisy problem " << i << "..." << std::endl;
            noisy_stats.add_problem();
            
            // Generate unified data with pixel noise (upright constraint)
            EntoContainer<Vec2<Scalar>, N> points2D;
            EntoContainer<Vec3<Scalar>, N> points3D;
            CameraPose<Scalar> true_pose;
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts.num_points, 0.01, i, true);
            
            // Convert 2D points to bearing vectors for UP2P
            EntoContainer<Vec3<Scalar>, N> x_bear;
            convert_2d_to_bearing_vectors<Scalar, N>(points2D, x_bear);
            
            // Test UP2P solver with first 2 points (already bearing vectors)
            EntoContainer<Vec3<Scalar>, 2> x_up2p;
            EntoContainer<Vec3<Scalar>, 2> X_up2p;
            for (int j = 0; j < 2; ++j) {
                x_up2p.push_back(x_bear[j]);
                X_up2p.push_back(points3D[j]);
            }
            
            EntoUtil::EntoArray<CameraPose<Scalar>, 4> solutions;
            int num_solutions = SolverUP2P<Scalar>::template solve<2>(x_up2p, X_up2p, &solutions);
            
            if (num_solutions > 0) {
                // Find best solution (closest to ground truth)
                Scalar best_error = std::numeric_limits<Scalar>::max();
                for (int s = 0; s < num_solutions; ++s) {
                    Scalar error = compute_pose_error(true_pose, solutions[s]);
                    if (error < best_error) {
                        best_error = error;
                    }
                }
                
                if (best_error < 15.0) { // More lenient for noisy data
                    noisy_stats.add_success(best_error, num_solutions);
                    std::cout << "      UP2P solved! Error: " << best_error << " degrees, Solutions: " << num_solutions << std::endl;
                } else {
                    noisy_stats.add_failure();
                    std::cout << "      UP2P solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                noisy_stats.add_failure();
                std::cout << "      UP2P failed to find solution" << std::endl;
            }
            
            std::string csv_line = make_csv_line_absolute_pose_bearing<Scalar, N>(
                true_pose, x_bear, points3D);
            
            file << csv_line << std::endl;
            std::cout << "    UP2P noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  up2p_noisy.csv complete." << std::endl;
    }
    
    std::cout << "UP2P data generation complete!" << std::endl;
    clean_stats.print_stats("UP2P Clean");
    noisy_stats.print_stats("UP2P Noisy");
}

template<typename Scalar, size_t N>
void generate_dlt_data(const Options& opts)
{
    std::cout << "Generating DLT data (Scalar=" << (sizeof(Scalar) == 8 ? "double" : "float") 
              << ", N=" << N << ", min_points=" << opts.dlt_min_points << ")..." << std::endl;
    
    SolverStats<Scalar> clean_stats, noisy_stats;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating dlt_clean.csv..." << std::endl;
        std::ofstream file("dlt_clean.csv");
        for (int i = 0; i < opts.num_problems; ++i) {
            std::cout << "    Generating DLT problem " << i << "..." << std::endl;
            clean_stats.add_problem();
            
            // Generate 2D/3D correspondences using the original 2D point generation
            EntoContainer<Vec2<Scalar>, N> points2D;
            EntoContainer<Vec3<Scalar>, N> points3D;
            CameraPose<Scalar> true_pose;
            
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts.num_points, 0.0, i);
            
            // Convert 2D points to homogeneous coordinates for DLT
            EntoContainer<Vec3<Scalar>, N> points2D_homo;
            convert_2d_to_homogeneous<Scalar, N>(points2D, points2D_homo);
            
            // Test DLT solver with specified minimum points
            EntoContainer<Vec3<Scalar>, 0> x_dlt;
            EntoContainer<Vec3<Scalar>, 0> X_dlt;
            for (int j = 0; j < opts.dlt_min_points; ++j) {
                x_dlt.push_back(points2D_homo[j]);
                X_dlt.push_back(points3D[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverDLT<Scalar>::template solve<0>(x_dlt, X_dlt, &solutions);
            
            if (num_solutions > 0) {
                Scalar error = compute_pose_error(true_pose, solutions[0]);
                
                if (error < 10.0) { // Accept if error < 10 degrees
                    clean_stats.add_success(error, num_solutions);
                    std::cout << "      DLT solved! Error: " << error << " degrees" << std::endl;
                } else {
                    clean_stats.add_failure();
                    std::cout << "      DLT solution too inaccurate: " << error << " degrees" << std::endl;
                }
            } else {
                clean_stats.add_failure();
                std::cout << "      DLT failed to find solution" << std::endl;
            }
            
            std::string csv_line = make_csv_line_absolute_pose_homogeneous<Scalar, N>(
                true_pose, points2D_homo, points3D);
            
            file << csv_line << std::endl;
            std::cout << "    DLT problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  dlt_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating dlt_noisy.csv..." << std::endl;
        std::ofstream file("dlt_noisy.csv");
        for (int i = 0; i < opts.num_problems; ++i) {
            std::cout << "    Generating DLT noisy problem " << i << "..." << std::endl;
            noisy_stats.add_problem();
            
            // Generate 2D/3D correspondences with noise using the original 2D point generation
            EntoContainer<Vec2<Scalar>, N> points2D;
            EntoContainer<Vec3<Scalar>, N> points3D;
            CameraPose<Scalar> true_pose;
            
            generate_unified_abspose_data<Scalar, N>(points2D, points3D, true_pose, opts.num_points, 0.01, i);
            
            // Convert 2D points to homogeneous coordinates for DLT
            EntoContainer<Vec3<Scalar>, N> points2D_homo;
            convert_2d_to_homogeneous<Scalar, N>(points2D, points2D_homo);
            
            // Test DLT solver with specified minimum points
            EntoContainer<Vec3<Scalar>, 0> x_dlt;
            EntoContainer<Vec3<Scalar>, 0> X_dlt;
            for (int j = 0; j < opts.dlt_min_points; ++j) {
                x_dlt.push_back(points2D_homo[j]);
                X_dlt.push_back(points3D[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverDLT<Scalar>::template solve<0>(x_dlt, X_dlt, &solutions);
            
            if (num_solutions > 0) {
                Scalar error = compute_pose_error(true_pose, solutions[0]);
                
                if (error < 15.0) { // More lenient for noisy data
                    noisy_stats.add_success(error, num_solutions);
                    std::cout << "      DLT solved! Error: " << error << " degrees" << std::endl;
                } else {
                    noisy_stats.add_failure();
                    std::cout << "      DLT solution too inaccurate: " << error << " degrees" << std::endl;
                }
            } else {
                noisy_stats.add_failure();
                std::cout << "      DLT failed to find solution" << std::endl;
            }
            
            std::string csv_line = make_csv_line_absolute_pose_homogeneous<Scalar, N>(
                true_pose, points2D_homo, points3D);
            
            file << csv_line << std::endl;
            std::cout << "    DLT noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  dlt_noisy.csv complete." << std::endl;
    }
    
    std::cout << "DLT data generation complete!" << std::endl;
    clean_stats.print_stats("DLT Clean");
    noisy_stats.print_stats("DLT Noisy");
}

// Explicit template specializations for common DLT sizes
template<typename Scalar>
void generate_dlt_data_specialized(const Options& opts, int N) {
    switch (N) {
        case 6:
            generate_dlt_data<Scalar, 6>(opts);
            break;
        case 8:
            generate_dlt_data<Scalar, 8>(opts);
            break;
        case 16:
            generate_dlt_data<Scalar, 16>(opts);
            break;
        case 32:
            generate_dlt_data<Scalar, 32>(opts);
            break;
        case 64:
            generate_dlt_data<Scalar, 64>(opts);
            break;
        case 128:
            generate_dlt_data<Scalar, 128>(opts);
            break;
        default:
            std::cout << "Error: Unsupported DLT size " << N << std::endl;
            exit(1);
    }
}

// Template dispatch functions
template<typename Scalar>
void run_generation(const Options& opts) {
    constexpr size_t N = 0; // Use dynamic sizing
    
    std::cout << "Starting data generation with options:" << std::endl;
    std::cout << "  Scalar type: " << (sizeof(Scalar) == 8 ? "double" : "float") << std::endl;
    std::cout << "  Problems: " << opts.num_problems << std::endl;
    std::cout << "  Points per problem: " << opts.num_points << std::endl;
    std::cout << "  DLT minimum points: " << opts.dlt_min_points << std::endl;
    std::cout << "  Generate P3P: " << (opts.generate_p3p ? "yes" : "no") << std::endl;
    std::cout << "  Generate UP2P: " << (opts.generate_up2p ? "yes" : "no") << std::endl;
    std::cout << "  Generate DLT: " << (opts.generate_dlt ? "yes" : "no") << std::endl;
    std::cout << std::endl;
    
    if (opts.generate_p3p) {
        generate_p3p_data<Scalar, N>(opts);
    }
    
    if (opts.generate_up2p) {
        generate_up2p_data<Scalar, N>(opts);
    }
    
    if (opts.generate_dlt) {
        generate_dlt_data_specialized<Scalar>(opts, opts.dlt_min_points);
    }
}

int main(int argc, char** argv)
{
    Options opts = parse_args(argc, argv);
    
    std::cout << "Generating minimal solver data for absolute pose..." << std::endl;
    
    if (opts.run_all_types) {
        std::cout << "Running all types with different configurations..." << std::endl;
        
        // Test both float and double
        for (bool use_double : {false, true}) {
            std::cout << "\n=== Testing with " << (use_double ? "double" : "float") << " precision ===" << std::endl;
            
            // Test P3P and UP2P (these don't depend on N)
            Options test_opts = opts;
            test_opts.use_double = use_double;
            test_opts.generate_dlt = false;
            
            if (use_double) {
                run_generation<double>(test_opts);
            } else {
                run_generation<float>(test_opts);
            }
            
            // Test all DLT sizes
            for (int dlt_size : opts.get_supported_dlt_sizes()) {
                std::cout << "\n--- Testing DLT with " << dlt_size << " points ---" << std::endl;
                Options dlt_opts = opts;
                dlt_opts.use_double = use_double;
                dlt_opts.generate_p3p = false;
                dlt_opts.generate_up2p = false;
                dlt_opts.generate_dlt = true;
                dlt_opts.dlt_min_points = dlt_size;
                
                if (use_double) {
                    run_generation<double>(dlt_opts);
                } else {
                    run_generation<float>(dlt_opts);
                }
            }
        }
    } else {
        if (opts.use_double) {
            run_generation<double>(opts);
        } else {
            run_generation<float>(opts);
        }
    }
    
    std::cout << "\nData generation complete!" << std::endl;
    std::cout << "Generated files:" << std::endl;
    
    if (opts.generate_p3p) {
        std::cout << "  - p3p_clean.csv" << std::endl;
        std::cout << "  - p3p_noisy.csv" << std::endl;
    }
    if (opts.generate_up2p) {
        std::cout << "  - up2p_clean.csv" << std::endl;
        std::cout << "  - up2p_noisy.csv" << std::endl;
    }
    if (opts.generate_dlt) {
        std::cout << "  - dlt_clean.csv" << std::endl;
        std::cout << "  - dlt_noisy.csv" << std::endl;
    }
    
    return 0;
} 