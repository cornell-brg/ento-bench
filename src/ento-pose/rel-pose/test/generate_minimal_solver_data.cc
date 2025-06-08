#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdint>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

#include <ento-util/containers.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/synthetic_relpose.h>

// Include the solver interfaces from data_gen.h
#include <ento-pose/data_gen.h>

using namespace EntoPose;
using namespace EntoUtil;

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

// Custom safe version of generate_synthetic_relpose_bearing_vectors that won't hang
template<typename Scalar, size_t N>
void generate_synthetic_relpose_bearing_vectors_safe(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0)  // Add problem_id to vary the seed
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem to avoid problematic poses
    std::default_random_engine rng(42 + problem_id);
    
    // Set random true pose
    true_pose.q = Quaternion::UnitRandom().coeffs();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0); // Scale translation
    
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
        
        // Add safety counter to prevent infinite loops
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
                // Regenerate pose and try again
                true_pose.q = Quaternion::UnitRandom().coeffs();
                true_pose.t.setRandom();
                true_pose.t *= Scalar(2.0);
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

// Custom safe version for upright relative pose
template<typename Scalar, size_t N>
void generate_synthetic_relpose_upright_bearing_vectors_safe(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0)
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem
    std::default_random_engine rng(42 + problem_id);
    
    // Set random upright pose (only y-axis rotation)
    Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
    Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
    true_pose.q(0) = q.w();
    true_pose.q(1) = q.x();
    true_pose.q(2) = q.y();
    true_pose.q(3) = q.z();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0);
    
    // Use the general function with the upright pose
    generate_synthetic_relpose_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, num_points, noise_level, problem_id);
}

// Custom safe version for upright planar relative pose
template<typename Scalar, size_t N>
void generate_synthetic_relpose_upright_planar_bearing_vectors_safe(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0)
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem
    std::default_random_engine rng(42 + problem_id);
    
    // Set random upright pose (only y-axis rotation) with planar motion (no y translation)
    Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
    Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
    true_pose.q(0) = q.w();
    true_pose.q(1) = q.x();
    true_pose.q(2) = q.y();
    true_pose.q(3) = q.z();
    
    // Planar motion: only x and z translation
    true_pose.t = Vec3(
        static_cast<Scalar>(rand()) / RAND_MAX * 4 - 2,  // x: [-2, 2]
        0,  // y: 0 (planar)
        static_cast<Scalar>(rand()) / RAND_MAX * 4 - 2   // z: [-2, 2]
    );
    
    // Use the general function with the upright planar pose
    generate_synthetic_relpose_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, num_points, noise_level, problem_id);
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

void generate_8pt_data()
{
    using Scalar = float;
    constexpr size_t N = 100;
    
    std::cout << "Generating 8pt data..." << std::endl;
    
    int successful_solves = 0;
    int total_problems = 0;
    Scalar total_error = 0.0;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating 8pt_clean.csv..." << std::endl;
        std::ofstream file("8pt_clean.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating 8pt problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.0, i);
            
            // Test 8pt solver with first 8 points
            EntoContainer<Vec3<Scalar>, 0> x1_8pt, x2_8pt;
            for (int j = 0; j < 8; ++j) {
                x1_8pt.push_back(x1_bear[j]);
                x2_8pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRel8pt<Scalar>::solve(x1_8pt, x2_8pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      8pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      8pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      8pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    8pt problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  8pt_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating 8pt_noisy.csv..." << std::endl;
        std::ofstream file("8pt_noisy.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating 8pt noisy problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.01, i);
            
            // Test 8pt solver with first 8 points
            EntoContainer<Vec3<Scalar>, 0> x1_8pt, x2_8pt;
            for (int j = 0; j < 8; ++j) {
                x1_8pt.push_back(x1_bear[j]);
                x2_8pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRel8pt<Scalar>::solve(x1_8pt, x2_8pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      8pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      8pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      8pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    8pt noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  8pt_noisy.csv complete." << std::endl;
    }
    
    std::cout << "8pt data generation complete!" << std::endl;
    std::cout << "  Success rate: " << successful_solves << "/" << total_problems 
              << " (" << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
    if (successful_solves > 0) {
        std::cout << "  Average error: " << (total_error / successful_solves) << " degrees" << std::endl;
    }
}

void generate_5pt_data()
{
    using Scalar = float;
    constexpr size_t N = 100;
    
    std::cout << "Generating 5pt data..." << std::endl;
    
    int successful_solves = 0;
    int total_problems = 0;
    Scalar total_error = 0.0;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating 5pt_clean.csv..." << std::endl;
        std::ofstream file("5pt_clean.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating 5pt problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.0, i);
            
            // Test 5pt solver with first 5 points
            EntoContainer<Vec3<Scalar>, 0> x1_5pt, x2_5pt;
            for (int j = 0; j < 5; ++j) {
                x1_5pt.push_back(x1_bear[j]);
                x2_5pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRel5pt<Scalar>::solve(x1_5pt, x2_5pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      5pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      5pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      5pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    5pt problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  5pt_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating 5pt_noisy.csv..." << std::endl;
        std::ofstream file("5pt_noisy.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating 5pt noisy problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.01, i);
            
            // Test 5pt solver with first 5 points
            EntoContainer<Vec3<Scalar>, 0> x1_5pt, x2_5pt;
            for (int j = 0; j < 5; ++j) {
                x1_5pt.push_back(x1_bear[j]);
                x2_5pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRel5pt<Scalar>::solve(x1_5pt, x2_5pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      5pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      5pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      5pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    5pt noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  5pt_noisy.csv complete." << std::endl;
    }
    
    std::cout << "5pt data generation complete!" << std::endl;
    std::cout << "  Success rate: " << successful_solves << "/" << total_problems 
              << " (" << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
    if (successful_solves > 0) {
        std::cout << "  Average error: " << (total_error / successful_solves) << " degrees" << std::endl;
    }
}

void generate_upright_3pt_data()
{
    using Scalar = float;
    constexpr size_t N = 100;
    
    std::cout << "Generating upright 3pt data..." << std::endl;
    
    int successful_solves = 0;
    int total_problems = 0;
    Scalar total_error = 0.0;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating upright_3pt_clean.csv..." << std::endl;
        std::ofstream file("upright_3pt_clean.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating upright 3pt problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_upright_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.0, i);
            
            // Test upright 3pt solver with first 3 points
            EntoContainer<Vec3<Scalar>, 0> x1_3pt, x2_3pt;
            for (int j = 0; j < 3; ++j) {
                x1_3pt.push_back(x1_bear[j]);
                x2_3pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRelUpright3pt<Scalar>::solve(x1_3pt, x2_3pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      upright 3pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      upright 3pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      upright 3pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    upright 3pt problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  upright_3pt_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating upright_3pt_noisy.csv..." << std::endl;
        std::ofstream file("upright_3pt_noisy.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating upright 3pt noisy problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_upright_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.01, i);
            
            // Test upright 3pt solver with first 3 points
            EntoContainer<Vec3<Scalar>, 0> x1_3pt, x2_3pt;
            for (int j = 0; j < 3; ++j) {
                x1_3pt.push_back(x1_bear[j]);
                x2_3pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRelUpright3pt<Scalar>::solve(x1_3pt, x2_3pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      upright 3pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      upright 3pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      upright 3pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    upright 3pt noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  upright_3pt_noisy.csv complete." << std::endl;
    }
    
    std::cout << "upright 3pt data generation complete!" << std::endl;
    std::cout << "  Success rate: " << successful_solves << "/" << total_problems 
              << " (" << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
    if (successful_solves > 0) {
        std::cout << "  Average error: " << (total_error / successful_solves) << " degrees" << std::endl;
    }
}

void generate_upright_planar_3pt_data()
{
    using Scalar = float;
    constexpr size_t N = 100;
    
    std::cout << "Generating upright planar 3pt data..." << std::endl;
    
    int successful_solves = 0;
    int total_problems = 0;
    Scalar total_error = 0.0;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating upright_planar_3pt_clean.csv..." << std::endl;
        std::ofstream file("upright_planar_3pt_clean.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating upright planar 3pt problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_upright_planar_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.0, i);
            
            // Test upright planar 3pt solver with first 3 points
            EntoContainer<Vec3<Scalar>, 0> x1_3pt, x2_3pt;
            for (int j = 0; j < 3; ++j) {
                x1_3pt.push_back(x1_bear[j]);
                x2_3pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRelUprightPlanar3pt<Scalar>::solve(x1_3pt, x2_3pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      upright planar 3pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      upright planar 3pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      upright planar 3pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    upright planar 3pt problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  upright_planar_3pt_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating upright_planar_3pt_noisy.csv..." << std::endl;
        std::ofstream file("upright_planar_3pt_noisy.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating upright planar 3pt noisy problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_upright_planar_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.01, i);
            
            // Test upright planar 3pt solver with first 3 points
            EntoContainer<Vec3<Scalar>, 0> x1_3pt, x2_3pt;
            for (int j = 0; j < 3; ++j) {
                x1_3pt.push_back(x1_bear[j]);
                x2_3pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRelUprightPlanar3pt<Scalar>::solve(x1_3pt, x2_3pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      upright planar 3pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      upright planar 3pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      upright planar 3pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    upright planar 3pt noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  upright_planar_3pt_noisy.csv complete." << std::endl;
    }
    
    std::cout << "upright planar 3pt data generation complete!" << std::endl;
    std::cout << "  Success rate: " << successful_solves << "/" << total_problems 
              << " (" << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
    if (successful_solves > 0) {
        std::cout << "  Average error: " << (total_error / successful_solves) << " degrees" << std::endl;
    }
}

void generate_upright_planar_2pt_data()
{
    using Scalar = float;
    constexpr size_t N = 100;
    
    std::cout << "Generating upright planar 2pt data..." << std::endl;
    
    int successful_solves = 0;
    int total_problems = 0;
    Scalar total_error = 0.0;
    
    // Clean data (0.00 noise)
    {
        std::cout << "  Creating upright_planar_2pt_clean.csv..." << std::endl;
        std::ofstream file("upright_planar_2pt_clean.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating upright planar 2pt problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_upright_planar_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.0, i);
            
            // Test upright planar 2pt solver with first 2 points
            EntoContainer<Vec3<Scalar>, 0> x1_2pt, x2_2pt;
            for (int j = 0; j < 2; ++j) {
                x1_2pt.push_back(x1_bear[j]);
                x2_2pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRelUprightPlanar2pt<Scalar>::solve(x1_2pt, x2_2pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      upright planar 2pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      upright planar 2pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      upright planar 2pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    upright planar 2pt problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  upright_planar_2pt_clean.csv complete." << std::endl;
    }
    
    // Noisy data (0.01 noise)
    {
        std::cout << "  Creating upright_planar_2pt_noisy.csv..." << std::endl;
        std::ofstream file("upright_planar_2pt_noisy.csv");
        for (int i = 0; i < 100; ++i) {
            std::cout << "    Generating upright planar 2pt noisy problem " << i << "..." << std::endl;
            EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
            CameraPose<Scalar> true_pose;
            
            generate_synthetic_relpose_upright_planar_bearing_vectors_safe<Scalar, N>(x1_bear, x2_bear, true_pose, N, 0.01, i);
            
            // Test upright planar 2pt solver with first 2 points
            EntoContainer<Vec3<Scalar>, 0> x1_2pt, x2_2pt;
            for (int j = 0; j < 2; ++j) {
                x1_2pt.push_back(x1_bear[j]);
                x2_2pt.push_back(x2_bear[j]);
            }
            
            std::vector<CameraPose<Scalar>> solutions;
            int num_solutions = SolverRelUprightPlanar2pt<Scalar>::solve(x1_2pt, x2_2pt, &solutions);
            
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
                    successful_solves++;
                    total_error += best_error;
                    std::cout << "      upright planar 2pt solved! Error: " << best_error << " degrees" << std::endl;
                } else {
                    std::cout << "      upright planar 2pt solution too inaccurate: " << best_error << " degrees" << std::endl;
                }
            } else {
                std::cout << "      upright planar 2pt failed to find solution" << std::endl;
            }
            total_problems++;
            
            std::string csv_line = make_csv_line_relative_pose<Scalar, N>(
                true_pose, x1_bear, x2_bear);
            
            file << csv_line << std::endl;
            std::cout << "    upright planar 2pt noisy problem " << i << " complete." << std::endl;
        }
        file.close();
        std::cout << "  upright_planar_2pt_noisy.csv complete." << std::endl;
    }
    
    std::cout << "upright planar 2pt data generation complete!" << std::endl;
    std::cout << "  Success rate: " << successful_solves << "/" << total_problems 
              << " (" << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
    if (successful_solves > 0) {
        std::cout << "  Average error: " << (total_error / successful_solves) << " degrees" << std::endl;
    }
}

int main()
{
    std::cout << "Generating minimal solver data for relative pose..." << std::endl;
    
    generate_8pt_data();
    generate_5pt_data();
    generate_upright_3pt_data();
    generate_upright_planar_3pt_data();
    generate_upright_planar_2pt_data();
    
    std::cout << "Data generation complete!" << std::endl;
    std::cout << "Generated files:" << std::endl;
    std::cout << "  - 8pt_clean.csv" << std::endl;
    std::cout << "  - 8pt_noisy.csv" << std::endl;
    std::cout << "  - 5pt_clean.csv" << std::endl;
    std::cout << "  - 5pt_noisy.csv" << std::endl;
    std::cout << "  - upright_3pt_clean.csv" << std::endl;
    std::cout << "  - upright_3pt_noisy.csv" << std::endl;
    std::cout << "  - upright_planar_3pt_clean.csv" << std::endl;
    std::cout << "  - upright_planar_3pt_noisy.csv" << std::endl;
    std::cout << "  - upright_planar_2pt_clean.csv" << std::endl;
    std::cout << "  - upright_planar_2pt_noisy.csv" << std::endl;
    
    return 0;
} 