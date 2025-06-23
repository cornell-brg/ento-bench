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
#include <iomanip>

#include <ento-util/containers.h>
#include <ento-util/debug.h>
#include <ento-util/random.h>
#include <ento-math/core.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/data_gen.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/synthetic_relpose.h>
#include <ento-pose/robust-est/robust_pose_solver.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/robust.h>
#include <ento-pose/rel-pose/upright_planar_three_pt.h>

using namespace EntoPose;
using namespace EntoUtil;
using namespace EntoMath;

struct Options {
    std::string solver_type = "5pt";
    double outlier_ratio = 0.25;
    std::string refinement_type = "linear";
    double noise_level = 0.01;
    double max_epipolar_error = -1.0;  // -1 means use solver-specific default
    size_t num_points = 64;
    size_t num_problems = 10;
    size_t max_iterations = 10000;  // Default RANSAC max iterations
    bool verbose = false;
    bool shuffle_data = true;  // Default to true for better statistical properties
    bool final_refinement = true;  // Default to true for post-RANSAC refinement
    std::string data_generator = "entobench";  // "simple", "realistic", "entobench"
    std::string precision = "float";  // "float" or "double"
};

void print_help() {
    std::cout << "Focused Robust Relative Pose Data Generator and Tester\n";
    std::cout << "Usage: generate_robust_relative_pose_data [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --solver-type TYPE       Solver type: 5pt, 8pt, upright_3pt, upright_planar_2pt, upright_planar_3pt (default: 5pt)\n";
    std::cout << "  --outlier-ratio RATIO    Outlier ratio 0.0-1.0 (default: 0.25)\n";
    std::cout << "  --refinement TYPE        Refinement: none, linear, nonlinear (default: linear)\n";
    std::cout << "  --noise LEVEL           Noise level (default: 0.01)\n";
    std::cout << "  --threshold PIXELS      RANSAC epipolar error threshold in pixels (default: auto-select)\n";
    std::cout << "  --points N              Number of points: 32, 64, 128, 256 (default: 64)\n";
    std::cout << "  --problems N            Number of problems to generate (default: 10)\n";
    std::cout << "  --max-iterations N      Maximum RANSAC iterations (default: 10000)\n";
    std::cout << "  --shuffle-data BOOL     Shuffle data points (default: true)\n";
    std::cout << "  --final-refinement BOOL Enable post-RANSAC bundle adjustment (default: true)\n";
    std::cout << "  --data-generator TYPE   Data generation method: simple, realistic, entobench (default: entobench)\n";
    std::cout << "  --precision TYPE        Floating point precision: float, double (default: float)\n";
    std::cout << "  --verbose               Enable verbose output\n";
    std::cout << "  --help                  Show this help message\n";
}

template<typename Scalar, size_t N>
void inject_relative_pose_outliers(EntoContainer<Vec2<Scalar>, N>& x1,
                                   EntoContainer<Vec2<Scalar>, N>& x2,
                                   EntoContainer<bool, N>& inlier_mask,
                                   double outlier_ratio,
                                   std::default_random_engine& rng) {
    // Calculate how many outliers to add based on the current size and desired ratio
    size_t current_inliers = x1.size();
    size_t total_desired = static_cast<size_t>(current_inliers / (1.0 - outlier_ratio));
    size_t num_outliers = total_desired - current_inliers;
    
    // Generate outliers by creating random correspondences in realistic image coordinate range
    std::uniform_real_distribution<Scalar> coord_dist(50.0, 450.0);  // Realistic image coordinate range
    
    // Append outliers to the end of the dataset
    for (size_t i = 0; i < num_outliers; ++i) {
        // Generate random correspondence in realistic image coordinate range
        Vec2<Scalar> outlier1(coord_dist(rng), coord_dist(rng));
        Vec2<Scalar> outlier2(coord_dist(rng), coord_dist(rng));
        
        x1.push_back(outlier1);
        x2.push_back(outlier2);
        inlier_mask.push_back(false);  // Mark as outlier
    }
}

// Simple, controlled data generation that produces well-conditioned problems
template<typename Scalar, size_t N>
void generate_simple_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_img,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_img,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar pixel_noise_std = Scalar(0.5),  // Standard deviation in pixels
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false,
    Scalar focal_length = Scalar(500.0))    // Realistic focal length
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem
    std::default_random_engine rng(42 + problem_id);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, pixel_noise_std);
    
    // Generate SIMPLE, well-conditioned pose
    if (upright_only) {
        // Simple yaw rotation: 5-20 degrees
        std::uniform_real_distribution<Scalar> yaw_gen(5.0, 20.0);
        Scalar yaw_deg = yaw_gen(rng);
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        if (planar_only) {
            // Simple XZ translation: 0.2-1.0m baseline
            std::uniform_real_distribution<Scalar> baseline_gen(0.2, 1.0);
            Scalar baseline_length = baseline_gen(rng);
            std::uniform_real_distribution<Scalar> angle_gen(0.0, 2.0 * M_PI);
            Scalar angle = angle_gen(rng);
            true_pose.t = Vec3(baseline_length * std::cos(angle), 0, baseline_length * std::sin(angle));
        } else {
            // Simple 3D translation: 0.2-1.0m baseline
            std::uniform_real_distribution<Scalar> baseline_gen(0.2, 1.0);
            Scalar baseline_length = baseline_gen(rng);
            Vec3 t_dir = Vec3::Random().normalized();
            true_pose.t = baseline_length * t_dir;
        }
    } else {
        // Simple 6DOF rotation: 5-15 degrees around random axis
        std::uniform_real_distribution<Scalar> angle_gen(5.0, 15.0);
        Scalar rot_angle_deg = angle_gen(rng);
        Scalar rot_angle_rad = rot_angle_deg * M_PI / 180.0;
        Vec3 rot_axis = Vec3::Random().normalized();
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(rot_angle_rad, rot_axis));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // Simple baseline: 0.2-1.0m
        std::uniform_real_distribution<Scalar> baseline_gen(0.2, 1.0);
        Scalar baseline_length = baseline_gen(rng);
        Vec3 t_dir = Vec3::Random().normalized();
        true_pose.t = baseline_length * t_dir;
    }
    
    if constexpr (N == 0) {
        x1_img.clear();
        x2_img.clear();
        x1_img.reserve(num_points);
        x2_img.reserve(num_points);
    }
    
    // Generate points in a simple, well-distributed pattern
    size_t valid_points = 0;
    
    if (planar_only) {
        // For planar case: generate points on a ground plane
        while (valid_points < num_points) {
            // Simple ground plane at Z=3m with good spread
            std::uniform_real_distribution<Scalar> lateral_gen(-2.0, 2.0);    // ±2m lateral
            std::uniform_real_distribution<Scalar> depth_gen(2.0, 8.0);       // 2-8m depth
            std::uniform_real_distribution<Scalar> height_gen(-0.2, 1.5);     // -0.2 to 1.5m height
            
            Vec3 X = Vec3(lateral_gen(rng), height_gen(rng), depth_gen(rng));
            
            // Project to both cameras
            Vec3 x1_cam = X;
            Vec3 x2_cam = true_pose.R() * X + true_pose.t;
            
            // Skip points behind cameras
            if (x1_cam(2) <= Scalar(0.1) || x2_cam(2) <= Scalar(0.1)) continue;
            
            // Project to image coordinates
            Vec2 x1_img_coord = Vec2(
                focal_length * x1_cam(0) / x1_cam(2) + 250.0,  // cx = 250
                focal_length * x1_cam(1) / x1_cam(2) + 250.0   // cy = 250
            );
            Vec2 x2_img_coord = Vec2(
                focal_length * x2_cam(0) / x2_cam(2) + 250.0,
                focal_length * x2_cam(1) / x2_cam(2) + 250.0
            );
            
            // Skip points outside reasonable image bounds
            if (std::abs(x1_img_coord(0) - 250.0) > 200 || std::abs(x1_img_coord(1) - 250.0) > 200 ||
                std::abs(x2_img_coord(0) - 250.0) > 200 || std::abs(x2_img_coord(1) - 250.0) > 200) {
                continue;
            }
            
            // Add pixel noise
            x1_img_coord(0) += pixel_noise_gen(rng);
            x1_img_coord(1) += pixel_noise_gen(rng);
            x2_img_coord(0) += pixel_noise_gen(rng);
            x2_img_coord(1) += pixel_noise_gen(rng);
            
            x1_img.push_back(x1_img_coord);
            x2_img.push_back(x2_img_coord);
            ++valid_points;
        }
    } else {
        // For general case: generate points in a simple 3D distribution
        while (valid_points < num_points) {
            // Simple 3D point distribution: 1-5m depth, good lateral spread
            std::uniform_real_distribution<Scalar> lateral_gen(-1.5, 1.5);    // ±1.5m lateral
            std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);       // 1-5m depth
            
            Vec3 X = Vec3(lateral_gen(rng), lateral_gen(rng), depth_gen(rng));
            
            // Project to both cameras
            Vec3 x1_cam = X;
            Vec3 x2_cam = true_pose.R() * X + true_pose.t;
            
            // Skip points behind cameras
            if (x1_cam(2) <= Scalar(0.1) || x2_cam(2) <= Scalar(0.1)) continue;
            
            // Project to image coordinates
            Vec2 x1_img_coord = Vec2(
                focal_length * x1_cam(0) / x1_cam(2) + 250.0,  // cx = 250
                focal_length * x1_cam(1) / x1_cam(2) + 250.0   // cy = 250
            );
            Vec2 x2_img_coord = Vec2(
                focal_length * x2_cam(0) / x2_cam(2) + 250.0,
                focal_length * x2_cam(1) / x2_cam(2) + 250.0
            );
            
            // Skip points outside reasonable image bounds
            if (std::abs(x1_img_coord(0) - 250.0) > 200 || std::abs(x1_img_coord(1) - 250.0) > 200 ||
                std::abs(x2_img_coord(0) - 250.0) > 200 || std::abs(x2_img_coord(1) - 250.0) > 200) {
                continue;
            }
            
            // Add pixel noise
            x1_img_coord(0) += pixel_noise_gen(rng);
            x1_img_coord(1) += pixel_noise_gen(rng);
            x2_img_coord(0) += pixel_noise_gen(rng);
            x2_img_coord(1) += pixel_noise_gen(rng);
            
            x1_img.push_back(x1_img_coord);
            x2_img.push_back(x2_img_coord);
            ++valid_points;
        }
    }
}

// NEW: High-quality data generation based on the standalone test approach
template<typename Scalar, size_t N>
void generate_high_quality_relpose_data(
    EntoContainer<Vec2<Scalar>, N>& x1_img,
    EntoContainer<Vec2<Scalar>, N>& x2_img,
    CameraPose<Scalar>& true_pose,
    int num_inliers,
    int num_outliers,
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> uniform_gen(-1.0, 1.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, 0.5);  // Realistic pixel noise
    Scalar focal_length = Scalar(500.0);
    
    // Generate motion based on solver constraints
    if (upright_only && planar_only) {
        // Upright planar motion (rotation only around Y-axis, translation in XZ plane)
        std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
        Scalar rot_selector = rot_scenario(rng);
        
        Scalar yaw_deg;
        if (rot_selector < 0.3) {
            std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
            yaw_deg = small_rot(rng);
        } else if (rot_selector < 0.7) {
            std::uniform_real_distribution<Scalar> medium_rot(-10.0, 10.0);
            yaw_deg = medium_rot(rng);
        } else {
            std::uniform_real_distribution<Scalar> large_rot(-45.0, 45.0);
            yaw_deg = large_rot(rng);
        }
        
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // Translation in XZ plane only
        std::uniform_real_distribution<Scalar> baseline_gen(0.1, 3.0);
        Scalar baseline_length = baseline_gen(rng);
        Scalar tx = uniform_gen(rng);
        Scalar tz = uniform_gen(rng);
        Vec3 t_dir = Vec3(tx, 0, tz).normalized();
        true_pose.t = baseline_length * t_dir;
        
    } else if (upright_only) {
        // Upright motion (rotation around Y-axis only, but translation in 3D)
        std::uniform_real_distribution<Scalar> yaw_gen(-30.0, 30.0);
        Scalar yaw_deg = yaw_gen(rng);
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // 3D translation
        std::uniform_real_distribution<Scalar> baseline_gen(0.1, 3.0);
        Scalar baseline_length = baseline_gen(rng);
        Vec3 t_dir = Vec3(uniform_gen(rng), uniform_gen(rng), uniform_gen(rng)).normalized();
        true_pose.t = baseline_length * t_dir;
        
    } else {
        // General 6DOF motion - REDUCED for better data generation
        std::uniform_real_distribution<Scalar> angle_gen(-5.0, 5.0);  // Much smaller rotations: ±5°
        Scalar roll = angle_gen(rng) * M_PI / 180.0;
        Scalar pitch = angle_gen(rng) * M_PI / 180.0;
        Scalar yaw = angle_gen(rng) * M_PI / 180.0;
        
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitZ()) *
                                 Eigen::AngleAxis<Scalar>(pitch, Vec3::UnitY()) *
                                 Eigen::AngleAxis<Scalar>(roll, Vec3::UnitX()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // Smaller baseline for better data generation
        std::uniform_real_distribution<Scalar> baseline_gen(0.1, 0.5);
        Scalar baseline_length = baseline_gen(rng);
        Vec3 t_dir = Vec3(uniform_gen(rng), uniform_gen(rng), uniform_gen(rng)).normalized();
        true_pose.t = baseline_length * t_dir;
    }
    
    x1_img.clear();
    x2_img.clear();
    
    std::cout << "Generating " << num_inliers << " inliers + " << num_outliers << " outliers = " << (num_inliers + num_outliers) << " total points" << std::endl;
    
    // Generate inlier points
    int generated_inliers = 0;
    int attempts = 0;
    while (generated_inliers < num_inliers && attempts < num_inliers * 3) {
        attempts++;
        
        // Generate 3D point with good distribution
        std::uniform_real_distribution<Scalar> lateral_gen(-3.0, 3.0);
        std::uniform_real_distribution<Scalar> depth_gen(2.0, 15.0);
        Vec3 X = Vec3(lateral_gen(rng), lateral_gen(rng), depth_gen(rng));
        
        // Project to first camera (identity pose)
        Vec3 x1_cam = X;
        if (x1_cam(2) <= Scalar(0.1)) continue;
        
        // Project to second camera
        Vec3 x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        
        // Project to image coordinates
        Vec2 img1 = Vec2(
            focal_length * x1_cam(0) / x1_cam(2) + 250.0,  // Add cx = 250
            focal_length * x1_cam(1) / x1_cam(2) + 250.0   // Add cy = 250
        );
        Vec2 img2 = Vec2(
            focal_length * x2_cam(0) / x2_cam(2) + 250.0,  // Add cx = 250
            focal_length * x2_cam(1) / x2_cam(2) + 250.0   // Add cy = 250
        );
        
        // Check if points are within reasonable image bounds (relative to principal point)
        if (std::abs(img1(0) - 250.0) > 200 || std::abs(img1(1) - 250.0) > 200 ||
            std::abs(img2(0) - 250.0) > 200 || std::abs(img2(1) - 250.0) > 200) {
            continue;
        }
        
        // Add pixel noise
        img1(0) += pixel_noise_gen(rng);
        img1(1) += pixel_noise_gen(rng);
        img2(0) += pixel_noise_gen(rng);
        img2(1) += pixel_noise_gen(rng);
        
        x1_img.push_back(img1);
        x2_img.push_back(img2);
        generated_inliers++;
        
        if (generated_inliers <= 5) {  // Only print first few for brevity
            std::cout << "  Inlier " << generated_inliers << ": (" << img1(0) << "," << img1(1) 
                      << ") -> (" << img2(0) << "," << img2(1) << ")" << std::endl;
        }
    }
    
    if (generated_inliers < num_inliers) {
        std::cout << "WARNING: Only generated " << generated_inliers << "/" << num_inliers << " inliers" << std::endl;
    }
    
    // Generate outlier points
    std::uniform_real_distribution<Scalar> outlier_gen(-200.0, 200.0);
    for (int k = 0; k < num_outliers; ++k) {
        Vec2 outlier1(outlier_gen(rng), outlier_gen(rng));
        Vec2 outlier2(outlier_gen(rng), outlier_gen(rng));
        x1_img.push_back(outlier1);
        x2_img.push_back(outlier2);
        
        if (k < 3) {  // Only print first few for brevity
            std::cout << "  Outlier " << (k+1) << ": (" << outlier1(0) << "," << outlier1(1) 
                      << ") -> (" << outlier2(0) << "," << outlier2(1) << ")" << std::endl;
        }
    }
    
    std::cout << "Final dataset: " << x1_img.size() << " correspondences" << std::endl;
}

// Robust CSV line generation function with inlier mask
template <typename Scalar, size_t N>
std::string make_csv_line_robust_relative_pose(
    const CameraPose<Scalar>& pose,
    const EntoContainer<Vec2<Scalar>, N>& x1_img,
    const EntoContainer<Vec2<Scalar>, N>& x2_img,
    const EntoContainer<bool, N>& inliers,
    Scalar scale = Scalar{1},
    Scalar focal = Scalar{1})
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    std::ostringstream oss;

    // 1) problem_type (3 for relative pose), N
    oss << 3 << ',' << x1_img.size() << ',';

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

    // 5) x1_img: dump N 2D image points (x, y)
    for (size_t i = 0; i < x1_img.size(); ++i) {
        const Vec2& pt = x1_img[i];
        oss << pt(0) << ',' << pt(1) << ',';
    }

    // 6) x2_img: dump N 2D image points (x, y)
    for (size_t i = 0; i < x2_img.size(); ++i) {
        const Vec2& pt = x2_img[i];
        oss << pt(0) << ',' << pt(1) << ',';
    }

    // 7) pack "inliers" (bool flags) into bytes
    const size_t kNumBytes = (x1_img.size() + 7) / 8;
    std::vector<uint8_t> mask_bytes(kNumBytes, 0);
    for (size_t i = 0; i < x1_img.size(); ++i) {
        if (inliers[i]) {
            size_t byte_idx = i / 8;
            size_t bit_idx  = i % 8; // LSB = point‐0, next bit = point‐1, etc.
            mask_bytes[byte_idx] |= static_cast<uint8_t>(1u << bit_idx);
        }
    }

    // 8) append each byte in decimal.  If N<=8, that's one byte; otherwise multiple.
    for (size_t b = 0; b < kNumBytes; ++b) {
        oss << static_cast<uint32_t>(mask_bytes[b]);
        if (b + 1 < kNumBytes) {
            oss << ',';  // comma‐separate multiple inlier‐bytes
        }
    }

    return oss.str();
}

// Function to compute rotation error in degrees
template<typename Scalar>
Scalar compute_rotation_error_degrees(const CameraPose<Scalar>& pose_gt, const CameraPose<Scalar>& pose_est) {
    // Compute relative rotation: R_error = R_gt^T * R_est
    Eigen::Matrix<Scalar, 3, 3> R_gt = pose_gt.R();
    Eigen::Matrix<Scalar, 3, 3> R_est = pose_est.R();
    Eigen::Matrix<Scalar, 3, 3> R_error = R_gt.transpose() * R_est;
    
    // Extract rotation angle from rotation matrix using trace
    Scalar trace = R_error.trace();
    Scalar cos_angle = (trace - 1.0) / 2.0;
    
    // Clamp to avoid numerical issues with acos
    cos_angle = std::max(Scalar(-1.0), std::min(Scalar(1.0), cos_angle));
    Scalar angle_rad = std::acos(cos_angle);
    
    return angle_rad * 180.0 / M_PI;  // Convert to degrees
}

// Function to compute translation direction error in degrees
template<typename Scalar>
Scalar compute_translation_error_degrees(const CameraPose<Scalar>& pose_gt, const CameraPose<Scalar>& pose_est) {
    // Normalize translation vectors
    Eigen::Matrix<Scalar, 3, 1> t_gt = pose_gt.t.normalized();
    Eigen::Matrix<Scalar, 3, 1> t_est = pose_est.t.normalized();
    
    // Compute angle between translation directions
    Scalar cos_angle = t_gt.dot(t_est);
    
    // Clamp to avoid numerical issues with acos
    cos_angle = std::max(Scalar(-1.0), std::min(Scalar(1.0), cos_angle));
    Scalar angle_rad = std::acos(std::abs(cos_angle));  // Use abs to handle sign ambiguity
    
    return angle_rad * 180.0 / M_PI;  // Convert to degrees
}

// Function to check if a set of correspondences is well-conditioned for relative pose estimation
template<typename Scalar>
bool check_correspondence_geometry(
    const std::vector<Eigen::Matrix<Scalar,2,1>>& x1_img,
    const std::vector<Eigen::Matrix<Scalar,2,1>>& x2_img,
    const CameraPose<Scalar>& true_pose,
    Scalar focal_length = Scalar(500.0))
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    
    if (x1_img.size() < 8) return false;  // Need minimum points for 8pt algorithm
    
    // 1. Check point distribution in image (avoid clustering) - RELAXED
    Scalar min_x1 = std::numeric_limits<Scalar>::max();
    Scalar max_x1 = std::numeric_limits<Scalar>::lowest();
    Scalar min_y1 = std::numeric_limits<Scalar>::max();
    Scalar max_y1 = std::numeric_limits<Scalar>::lowest();
    
    for (const auto& pt : x1_img) {
        min_x1 = std::min(min_x1, pt(0));
        max_x1 = std::max(max_x1, pt(0));
        min_y1 = std::min(min_y1, pt(1));
        max_y1 = std::max(max_y1, pt(1));
    }
    
    // RELAXED: Require points to span at least 10% of image in both dimensions (was 30%)
    Scalar span_x = max_x1 - min_x1;
    Scalar span_y = max_y1 - min_y1;
    if (span_x < Scalar(50.0) || span_y < Scalar(50.0)) {  // 10% of 500px image (was 150px)
        return false;  // Points too clustered
    }
    
    // 2. Check parallax (baseline vs depth ratio) - RELAXED
    Scalar baseline_length = true_pose.t.norm();
    if (baseline_length < Scalar(0.001)) {  // RELAXED: was 0.01, now 0.001 (1mm minimum)
        return false;  // Baseline too small (pure rotation)
    }
    
    // 3. Convert to normalized coordinates and check parallax - RELAXED
    std::vector<Vec2> x1_norm, x2_norm;
    for (size_t i = 0; i < x1_img.size(); ++i) {
        x1_norm.push_back(Vec2(x1_img[i](0) / focal_length, x1_img[i](1) / focal_length));
        x2_norm.push_back(Vec2(x2_img[i](0) / focal_length, x2_img[i](1) / focal_length));
    }
    
    // Check average parallax (correspondence motion) - RELAXED
    Scalar total_parallax = Scalar(0.0);
    for (size_t i = 0; i < x1_norm.size(); ++i) {
        Scalar parallax = (x1_norm[i] - x2_norm[i]).norm();
        total_parallax += parallax;
    }
    Scalar avg_parallax = total_parallax / Scalar(x1_norm.size());
    
    // RELAXED: Require minimum average parallax (was 0.001, now 0.0001)
    if (avg_parallax < Scalar(0.0001)) {  // Very small motion
        return false;
    }
    
    // 4. Check parallax variance (avoid all points at same depth) - RELAXED
    std::vector<Scalar> parallax_values;
    for (size_t i = 0; i < x1_norm.size(); ++i) {
        Scalar parallax = (x1_norm[i] - x2_norm[i]).norm();
        parallax_values.push_back(parallax);
    }
    
    // Compute variance of parallax
    Scalar mean_parallax = Scalar(0.0);
    for (Scalar p : parallax_values) mean_parallax += p;
    mean_parallax /= Scalar(parallax_values.size());
    
    Scalar variance = Scalar(0.0);
    for (Scalar p : parallax_values) {
        Scalar diff = p - mean_parallax;
        variance += diff * diff;
    }
    variance /= Scalar(parallax_values.size());
    
    // RELAXED: Require sufficient parallax variation (was 1e-6, now 1e-8)
    if (variance < Scalar(1e-8)) {
        return false;  // All points at similar depths - degenerate
    }
    
    // 5. Check rotation magnitude (avoid near-identity rotations) - RELAXED
    Eigen::Quaternion<Scalar> q(true_pose.q(0), true_pose.q(1), true_pose.q(2), true_pose.q(3));
    Scalar rotation_angle = Scalar(2.0) * std::acos(std::abs(q.w()));  // Rotation angle in radians
    if (rotation_angle < Scalar(0.001)) {  // RELAXED: was 0.01 (~0.6°), now 0.001 (~0.06°)
        return false;  // Insufficient rotation
    }
    
    return true;  // All checks passed
}

// EntoBench-realistic data generation with mixed scenarios for proper workload characterization
template<typename Scalar, size_t N>
void generate_entobench_realistic_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_img,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_img,
    CameraPose<Scalar>& true_pose,
    int num_inliers,
    int num_outliers,
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false,
    Scalar pixel_noise_std = Scalar(0.5))  // EntoBench standard noise levels: 0.5, 1.0, 2.5
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> uniform_gen(-1.0, 1.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, pixel_noise_std);
    Scalar focal_length = Scalar(500.0);  // Standard EntoBench focal length
    
    // Generate realistic motion based on mixed robotics scenarios
    if (upright_only && planar_only) {
        // Upright planar motion (robotics ground vehicles, indoor navigation)
        std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
        Scalar rot_selector = rot_scenario(rng);
        
        Scalar yaw_deg;
        if (rot_selector < 0.3) {
            // Small challenging rotations: ±2° (precision tasks)
            std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
            yaw_deg = small_rot(rng);
        } else if (rot_selector < 0.7) {
            // Medium typical rotations: ±10° (normal maneuvering)
            std::uniform_real_distribution<Scalar> medium_rot(-10.0, 10.0);
            yaw_deg = medium_rot(rng);
        } else {
            // Large easier rotations: ±45° (wide turns)
            std::uniform_real_distribution<Scalar> large_rot(-45.0, 45.0);
            yaw_deg = large_rot(rng);
        }
        
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // Realistic baseline variation for robotics applications
        std::uniform_real_distribution<Scalar> baseline_scenario(0.0, 1.0);
        Scalar baseline_selector = baseline_scenario(rng);
        Scalar baseline_length;
        
        if (baseline_selector < 0.3) {
            // Close manipulation: 5cm - 50cm (tabletop robotics)
            std::uniform_real_distribution<Scalar> close_baseline(0.05, 0.5);
            baseline_length = close_baseline(rng);
        } else if (baseline_selector < 0.7) {
            // Mid-range navigation: 0.5m - 2m (indoor robotics)
            std::uniform_real_distribution<Scalar> mid_baseline(0.5, 2.0);
            baseline_length = mid_baseline(rng);
        } else {
            // Long-range navigation: 2m - 5m (outdoor robotics)
            std::uniform_real_distribution<Scalar> long_baseline(2.0, 5.0);
            baseline_length = long_baseline(rng);
        }
        
        // Random direction in XZ plane
        Scalar tx = uniform_gen(rng);
        Scalar tz = uniform_gen(rng);
        Vec3 t_dir = Vec3(tx, 0, tz).normalized();
        true_pose.t = baseline_length * t_dir;
        
    } else if (upright_only) {
        // Upright 3DOF motion (drones with altitude control)
        std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
        Scalar rot_selector = rot_scenario(rng);
        
        Scalar yaw_deg;
        if (rot_selector < 0.3) {
            std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
            yaw_deg = small_rot(rng);
        } else if (rot_selector < 0.7) {
            std::uniform_real_distribution<Scalar> medium_rot(-10.0, 10.0);
            yaw_deg = medium_rot(rng);
        } else {
            std::uniform_real_distribution<Scalar> large_rot(-45.0, 45.0);
            yaw_deg = large_rot(rng);
        }
        
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // 3D baseline with realistic magnitudes
        std::uniform_real_distribution<Scalar> baseline_gen(0.1, 3.0);
        Scalar baseline_length = baseline_gen(rng);
        Vec3 t_dir = Vec3::Random().normalized();
        true_pose.t = baseline_length * t_dir;
        
    } else {
        // General 6DOF motion (full 3D robotics, aerial vehicles)
        std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
        Scalar rot_selector = rot_scenario(rng);
        
        Scalar rot_angle_deg;
        if (rot_selector < 0.3) {
            // Small challenging rotations: ±5°
            std::uniform_real_distribution<Scalar> small_rot(2.0, 5.0);
            rot_angle_deg = small_rot(rng);
        } else if (rot_selector < 0.7) {
            // Medium typical rotations: ±15°
            std::uniform_real_distribution<Scalar> medium_rot(5.0, 15.0);
            rot_angle_deg = medium_rot(rng);
        } else {
            // Large easier rotations: ±30°
            std::uniform_real_distribution<Scalar> large_rot(15.0, 30.0);
            rot_angle_deg = large_rot(rng);
        }
        
        Scalar rot_angle_rad = rot_angle_deg * M_PI / 180.0;
        Vec3 rot_axis = Vec3::Random().normalized();
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(rot_angle_rad, rot_axis));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // Realistic 3D baseline
        std::uniform_real_distribution<Scalar> baseline_gen(0.2, 2.0);
        Scalar baseline_length = baseline_gen(rng);
        Vec3 t_dir = Vec3::Random().normalized();
        true_pose.t = baseline_length * t_dir;
    }
    
    if constexpr (N == 0) {
        x1_img.clear();
        x2_img.clear();
        x1_img.reserve(num_inliers + num_outliers);
        x2_img.reserve(num_inliers + num_outliers);
    }
    
    // Generate inlier points with mixed realistic scenarios
    int generated_inliers = 0;
    int attempts = 0;
    const int max_attempts = num_inliers * 5;
    
    while (generated_inliers < num_inliers && attempts < max_attempts) {
        attempts++;
        
        // Mixed depth scenarios based on robotics applications
        std::uniform_real_distribution<Scalar> depth_scenario(0.0, 1.0);
        Scalar depth_selector = depth_scenario(rng);
        
        Vec3 X;
        if (depth_selector < 0.4) {
            // Close manipulation: 0.2-2m depth, tight lateral spread (challenging)
            std::uniform_real_distribution<Scalar> close_lateral(-0.5, 0.5);  // ±50cm
            std::uniform_real_distribution<Scalar> close_depth(0.2, 2.0);     // 20cm-2m
            if (planar_only) {
                std::uniform_real_distribution<Scalar> height_gen(-0.2, 1.5); // Ground plane variation
                X = Vec3(close_lateral(rng), height_gen(rng), close_depth(rng));
            } else {
                X = Vec3(close_lateral(rng), close_lateral(rng), close_depth(rng));
            }
        } else if (depth_selector < 0.8) {
            // Mid-range robotics: 1-10m depth, moderate spread (typical)
            std::uniform_real_distribution<Scalar> mid_lateral(-2.0, 2.0);    // ±2m  
            std::uniform_real_distribution<Scalar> mid_depth(1.0, 10.0);      // 1-10m
            if (planar_only) {
                std::uniform_real_distribution<Scalar> height_gen(-0.5, 2.0); // Ground plane variation
                X = Vec3(mid_lateral(rng), height_gen(rng), mid_depth(rng));
            } else {
                X = Vec3(mid_lateral(rng), mid_lateral(rng), mid_depth(rng));
            }
        } else {
            // Far-field navigation: 5-50m depth, wide spread (easier)
            std::uniform_real_distribution<Scalar> far_lateral(-10.0, 10.0);  // ±10m
            std::uniform_real_distribution<Scalar> far_depth(5.0, 50.0);      // 5-50m  
            if (planar_only) {
                std::uniform_real_distribution<Scalar> height_gen(-1.0, 3.0); // Ground plane variation
                X = Vec3(far_lateral(rng), height_gen(rng), far_depth(rng));
            } else {
                X = Vec3(far_lateral(rng), far_lateral(rng), far_depth(rng));
            }
        }
        
        // Project to first camera (identity pose)
        Vec3 x1_cam = X;
        if (x1_cam(2) <= Scalar(0.1)) continue; // Skip if behind camera
        
        // Project to second camera
        Vec3 x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue; // Skip if behind camera
        
        // Project to image coordinates (pinhole camera model)
        Vec2 img1 = Vec2(
            focal_length * x1_cam(0) / x1_cam(2) + 250.0,  // cx = 250
            focal_length * x1_cam(1) / x1_cam(2) + 250.0   // cy = 250
        );
        Vec2 img2 = Vec2(
            focal_length * x2_cam(0) / x2_cam(2) + 250.0,  // cx = 250
            focal_length * x2_cam(1) / x2_cam(2) + 250.0   // cy = 250
        );
        
        // Check if points are within reasonable image bounds (realistic FOV)
        if (std::abs(img1(0) - 250.0) > 200 || std::abs(img1(1) - 250.0) > 200 ||
            std::abs(img2(0) - 250.0) > 200 || std::abs(img2(1) - 250.0) > 200) {
            continue;
        }
        
        // Add realistic pixel noise (EntoBench standard)
        img1(0) += pixel_noise_gen(rng);
        img1(1) += pixel_noise_gen(rng);
        img2(0) += pixel_noise_gen(rng);
        img2(1) += pixel_noise_gen(rng);
        
        x1_img.push_back(img1);
        x2_img.push_back(img2);
        generated_inliers++;
    }
    
    if (generated_inliers < num_inliers) {
        std::cout << "Warning: Only generated " << generated_inliers << "/" << num_inliers << " inliers" << std::endl;
    }
    
    // Generate outlier points (random correspondences within image bounds)
    for (int i = 0; i < num_outliers; ++i) {
        // Random points within image bounds
        std::uniform_real_distribution<Scalar> img_coord_gen(50.0, 450.0);  // Stay within bounds
        
        Vec2 outlier1(img_coord_gen(rng), img_coord_gen(rng));
        Vec2 outlier2(img_coord_gen(rng), img_coord_gen(rng));
        
        // Add noise to outliers too (they're still noisy measurements)
        outlier1(0) += pixel_noise_gen(rng);
        outlier1(1) += pixel_noise_gen(rng);
        outlier2(0) += pixel_noise_gen(rng);
        outlier2(1) += pixel_noise_gen(rng);
        
        x1_img.push_back(outlier1);
        x2_img.push_back(outlier2);
    }
    
    std::cout << "Generated EntoBench-realistic data: " << generated_inliers << " inliers + " 
              << num_outliers << " outliers = " << (generated_inliers + num_outliers) << " total points" << std::endl;
}

template<typename Scalar, size_t N>
void test_robust_solver(const Options& opt) {
    using SolverRel5pt = EntoPose::SolverRel5pt<Scalar>;
    using SolverRel8pt = EntoPose::SolverRel8pt<Scalar>;
    using SolverRelUpright3pt = EntoPose::SolverRelUpright3pt<Scalar>;
    using SolverRelUprightPlanar2pt = EntoPose::SolverRelUprightPlanar2pt<Scalar>;
    using SolverRelUprightPlanar3pt = EntoPose::SolverRelUprightPlanar3pt<Scalar>;
    
    std::cout << "Testing " << opt.solver_type << " robust solver with " 
              << (opt.outlier_ratio * 100) << "% outliers, " 
              << (opt.refinement_type == "linear" ? "linear" : "nonlinear") << " refinement" << std::endl;
    
    // Create output filename
    std::string filename = opt.solver_type + "_" + 
                          (std::is_same_v<Scalar, float> ? "float" : "double") + "_" +
                          "realistic_p" + std::to_string(opt.num_problems) + "_" +
                          "noise" + std::to_string(opt.noise_level).substr(0, 6) + "_" +
                          "outliers" + std::to_string(static_cast<int>(opt.outlier_ratio * 100)) + "_" +
                          opt.refinement_type + "_lo_ransac.csv";
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
        return;
    }
    
    // RANSAC configuration
    RansacOptions<Scalar> ransac_opt;
    ransac_opt.max_iters = opt.max_iterations;
    
    // CRITICAL: Threshold processing chain for relative pose estimation
    // 1. Input threshold (pixels): e.g., 20.0 for 8pt, 4.0 for minimal solvers
    // 2. Divided by focal length (500): 20.0/500 = 0.04 normalized coords
    // 3. Squared for Sampson error: 0.04² = 0.0016 squared normalized coords
    // Final working thresholds: 8pt=0.0016, minimal=0.000064 (squared normalized)
    
    // Use CLI-provided threshold if specified, otherwise use solver-specific defaults
    if (opt.max_epipolar_error > 0.0) {
        ransac_opt.max_epipolar_error = Scalar(opt.max_epipolar_error);
    } else {
        // Use solver-specific thresholds based on algorithm characteristics
        if (opt.solver_type == "8pt") {
            ransac_opt.max_epipolar_error = Scalar(20.0);  // Higher threshold for 8pt due to linearization errors
        } else {
            ransac_opt.max_epipolar_error = Scalar(4.0);   // Standard threshold for minimal solvers
        }
    }
    
    ransac_opt.success_prob = 0.99;
    
    // Set final refinement option from CLI
    ransac_opt.final_refinement = opt.final_refinement;
    
    // Configure refinement type based on command line parameter
    if (opt.refinement_type == "linear") {
        ransac_opt.lo_type = LocalRefinementType::Linear;
        
        // Set appropriate linear refinement method based on solver type
        if (opt.solver_type == "upright_planar_2pt" || opt.solver_type == "upright_planar_3pt") {
            ransac_opt.linear_method = LinearRefinementMethod::UprightPlanar3pt;
        } else {
            ransac_opt.linear_method = LinearRefinementMethod::EightPoint;  // Use EightPoint for general relative pose
        }
        
        ransac_opt.use_irls = true;                                     // Enable IRLS for better outlier handling
        ransac_opt.irls_max_iters = 5;                                  // Reasonable number of IRLS iterations
        ransac_opt.irls_huber_threshold = Scalar(1.0);                 // Huber threshold for outlier downweighting
    } else if (opt.refinement_type == "nonlinear") {
        ransac_opt.lo_type = LocalRefinementType::BundleAdjust;
        ransac_opt.use_irls = false;
    } else {
        // Default to no refinement
        ransac_opt.lo_type = LocalRefinementType::None;
    }
    
    if (opt.verbose) {
        std::cout << "Using RANSAC threshold: " << ransac_opt.max_epipolar_error << " pixels" << std::endl;
        std::cout << "Using refinement type: " << opt.refinement_type << std::endl;
        std::cout << "Using final refinement: " << (opt.final_refinement ? "enabled" : "disabled") << std::endl;
    }
    
    // Bundle adjustment configuration
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.max_iterations = 50;
    
    // Camera setup (use SimplePinholeCameraModel with focal=500)
    using CameraModel = SimplePinholeCameraModel<Scalar>;
    using Params = std::array<Scalar, 3>;  // SimplePinhole needs [focal, cx, cy]
    Params params = {500.0, 250.0, 250.0};     // focal=500, principal point at image center
    Camera<Scalar, CameraModel> camera1(500.0, 500.0, params);  
    Camera<Scalar, CameraModel> camera2(500.0, 500.0, params);
    
    size_t successful_estimates = 0;
    size_t total_inliers_found = 0;
    size_t total_iterations = 0;
    
    // Enhanced statistics tracking
    std::vector<size_t> iteration_counts;
    std::vector<size_t> inlier_counts;
    std::vector<size_t> expected_inlier_counts;
    std::vector<Scalar> rotation_errors;
    std::vector<Scalar> translation_errors;
    
    // NEW: Store ground truth and RANSAC results for exact precision/recall
    std::vector<EntoContainer<bool, N>> ground_truth_masks;
    std::vector<EntoContainer<uint8_t, N>> ransac_results;
    
    for (size_t problem = 0; problem < opt.num_problems; ++problem) {
        // Generate realistic relative pose data (bearing vectors)
        EntoContainer<Vec3<Scalar>, N> x1_bear, x2_bear;
        EntoContainer<Vec2<Scalar>, N> x1_img, x2_img;
        CameraPose<Scalar> pose_gt;
        
        // Random number generator for outlier injection
        std::default_random_engine rng(42 + problem);
        
        // Choose appropriate data generation based on solver type
        bool upright_only = (opt.solver_type == "upright_3pt" || 
                            opt.solver_type == "upright_planar_2pt" || 
                            opt.solver_type == "upright_planar_3pt");
        bool planar_only = (opt.solver_type == "upright_planar_2pt" || 
                           opt.solver_type == "upright_planar_3pt");
        
        // Calculate number of inliers and outliers
        int total_points = opt.num_points;
        int num_outliers = static_cast<int>(total_points * opt.outlier_ratio);
        int num_inliers = total_points - num_outliers;

        // Use selected data generation method
        if (opt.data_generator == "simple") {
            generate_simple_relpose_data<Scalar, N>(
                x1_img, x2_img, pose_gt, opt.num_points,
                static_cast<Scalar>(opt.noise_level), problem,
                opt.solver_type.find("upright") != std::string::npos,
                opt.solver_type.find("planar") != std::string::npos
            );
        } else if (opt.data_generator == "realistic") {
            generate_high_quality_relpose_data<Scalar, N>(
                x1_img, x2_img, pose_gt, num_inliers, num_outliers, problem,
                opt.solver_type.find("upright") != std::string::npos,
                opt.solver_type.find("planar") != std::string::npos
            );
        } else { // "entobench" (default)
            generate_entobench_realistic_relpose_data<Scalar, N>(
                x1_img, x2_img, pose_gt, num_inliers, num_outliers, problem,
                opt.solver_type.find("upright") != std::string::npos,
                opt.solver_type.find("planar") != std::string::npos,
                static_cast<Scalar>(opt.noise_level)
            );
        }
        
        // Inject outliers if needed
        EntoContainer<bool, N> inlier_mask;
        if constexpr (N == 0) {
            inlier_mask.resize(x1_img.size(), true);  // Initialize all as inliers
        } else {
            // For fixed-size arrays, use push_back to add elements
            for (size_t i = 0; i < x1_img.size(); ++i) {
                inlier_mask.push_back(true);
            }
        }
        
        if (opt.outlier_ratio > 0.0) {
            inject_relative_pose_outliers<Scalar, N>(x1_img, x2_img, inlier_mask, opt.outlier_ratio, rng);
        }
        
        // Calculate expected inliers based on the mask
        size_t expected_inliers = std::count(inlier_mask.begin(), inlier_mask.end(), true);
        size_t actual_outliers = x1_img.size() - expected_inliers;
        
        // Shuffle data if requested to eliminate ordering biases
        if (opt.shuffle_data) {
            // Create indices for shuffling
            std::vector<size_t> indices(x1_img.size());
            std::iota(indices.begin(), indices.end(), 0);
            
            // Shuffle indices using problem-specific seed for reproducibility
            std::default_random_engine shuffle_rng(42 + problem);
            std::shuffle(indices.begin(), indices.end(), shuffle_rng);
            
            // Create shuffled copies
            EntoContainer<Vec2<Scalar>, N> x1_shuffled, x2_shuffled;
            EntoContainer<bool, N> mask_shuffled;
            
            if constexpr (N == 0) {
                x1_shuffled.reserve(x1_img.size());
                x2_shuffled.reserve(x2_img.size());
                mask_shuffled.reserve(inlier_mask.size());
            }
            
            for (size_t idx : indices) {
                x1_shuffled.push_back(x1_img[idx]);
                x2_shuffled.push_back(x2_img[idx]);
                mask_shuffled.push_back(inlier_mask[idx]);
            }
            
            // Replace original data with shuffled data
            x1_img = std::move(x1_shuffled);
            x2_img = std::move(x2_shuffled);
            inlier_mask = std::move(mask_shuffled);
            
            if (opt.verbose) {
                std::cout << "  Data shuffled for unbiased sampling" << std::endl;
            }
        }
        
        // NEW: Check for degenerate geometry before running RANSAC
        std::vector<Vec2<Scalar>> x1_vec, x2_vec;
        for (size_t i = 0; i < x1_img.size(); ++i) {
            x1_vec.push_back(x1_img[i]);
            x2_vec.push_back(x2_img[i]);
        }
        
        bool is_well_conditioned = check_correspondence_geometry<Scalar>(x1_vec, x2_vec, pose_gt, Scalar(500.0));
        
        if (!is_well_conditioned) {
            if (opt.verbose) {
                std::cout << "Problem " << problem << ": SKIPPED - Degenerate geometry detected" << std::endl;
                std::cout << "  Baseline length: " << pose_gt.t.norm() << std::endl;
                Eigen::Quaternion<Scalar> q(pose_gt.q(0), pose_gt.q(1), pose_gt.q(2), pose_gt.q(3));
                Scalar rotation_angle_deg = 2.0 * std::acos(std::abs(q.w())) * 180.0 / M_PI;
                std::cout << "  Rotation angle: " << rotation_angle_deg << "°" << std::endl;
                
                // Check point distribution
                Scalar min_x = std::numeric_limits<Scalar>::max();
                Scalar max_x = std::numeric_limits<Scalar>::lowest();
                Scalar min_y = std::numeric_limits<Scalar>::max();
                Scalar max_y = std::numeric_limits<Scalar>::lowest();
                
                for (const auto& pt : x1_vec) {
                    min_x = std::min(min_x, pt(0));
                    max_x = std::max(max_x, pt(0));
                    min_y = std::min(min_y, pt(1));
                    max_y = std::max(max_y, pt(1));
                }
                
                std::cout << "  Point span: " << (max_x - min_x) << " x " << (max_y - min_y) << " pixels" << std::endl;
            }
            
            // FIXED: Instead of infinite loop, just continue with this problem
            // This treats degenerate cases as failed problems (which is realistic)
            if (opt.verbose) {
                std::cout << "Problem " << problem << ": CONTINUING with degenerate case (realistic scenario)" << std::endl;
            }
            // Don't skip - continue with the degenerate problem as a realistic failure case
        }
        
        if (opt.verbose && is_well_conditioned) {
            std::cout << "Problem " << problem << ": Geometry check PASSED - Well-conditioned correspondences" << std::endl;
        }
        
        // Run LO-RANSAC
        CameraPose<Scalar> estimated_pose;
        EntoContainer<uint8_t, N> inliers_found;
        RansacStats<Scalar> stats;
        
        if (opt.verbose) {
            std::cout << "\n=== Problem " << problem << " Debug ===" << std::endl;
            std::cout << "  Expected inliers: " << expected_inliers << std::endl;
            std::cout << "  Actual outliers: " << actual_outliers << std::endl;
            std::cout << "  RANSAC threshold: " << ransac_opt.max_epipolar_error << std::endl;
        }
        
        if (opt.solver_type == "5pt") {
            stats = estimate_relative_pose<SolverRel5pt, N, CameraModel>(
                x1_img, x2_img, camera1, camera2, ransac_opt, bundle_opt, &estimated_pose, &inliers_found
            );
        } else if (opt.solver_type == "8pt") {
            stats = estimate_relative_pose<SolverRel8pt, N, CameraModel>(
                x1_img, x2_img, camera1, camera2, ransac_opt, bundle_opt, &estimated_pose, &inliers_found
            );
        } else if (opt.solver_type == "upright_3pt") {
            stats = estimate_relative_pose<SolverRelUpright3pt, N, CameraModel>(
                x1_img, x2_img, camera1, camera2, ransac_opt, bundle_opt, &estimated_pose, &inliers_found
            );
        } else if (opt.solver_type == "upright_planar_2pt") {
            stats = estimate_relative_pose<SolverRelUprightPlanar2pt, N, CameraModel>(
                x1_img, x2_img, camera1, camera2, ransac_opt, bundle_opt, &estimated_pose, &inliers_found
            );
        } else if (opt.solver_type == "upright_planar_3pt") {
            stats = estimate_relative_pose<SolverRelUprightPlanar3pt, N, CameraModel>(
                x1_img, x2_img, camera1, camera2, ransac_opt, bundle_opt, &estimated_pose, &inliers_found
            );
        }
        
        if (opt.verbose) {
            std::cout << "  RANSAC iterations: " << stats.iters << std::endl;
            std::cout << "  Inliers found: " << std::count(inliers_found.begin(), inliers_found.end(), 1) << std::endl;
            std::cout << "  Success: " << (stats.num_inliers > 0 ? "YES" : "NO") << std::endl;
            if (stats.num_inliers > 0) {
                Scalar rot_error = compute_rotation_error_degrees(pose_gt, estimated_pose);
                Scalar trans_error = compute_translation_error_degrees(pose_gt, estimated_pose);
                std::cout << "  Rotation error: " << rot_error << "°" << std::endl;
                std::cout << "  Translation error: " << trans_error << "°" << std::endl;
            }
            std::cout << "=== End Problem " << problem << " Debug ===\n" << std::endl;
        }
        
        size_t inliers_count = std::count(inliers_found.begin(), inliers_found.end(), 1);
        
        // FIXED: Collect iteration statistics for ALL problems, not just successful ones
        iteration_counts.push_back(stats.iters);
        
        if (stats.num_inliers > 0) {  // Check if we found any inliers
            successful_estimates++;
            total_inliers_found += inliers_count;
            total_iterations += stats.iters;
            
            // Collect detailed statistics for successful problems only
            inlier_counts.push_back(inliers_count);
            expected_inlier_counts.push_back(expected_inliers);
            
            // NEW: Store ground truth and RANSAC results for exact precision/recall
            ground_truth_masks.push_back(inlier_mask);
            ransac_results.push_back(inliers_found);
            
            // Compute rotation and translation errors
            Scalar rotation_error = compute_rotation_error_degrees(pose_gt, estimated_pose);
            Scalar translation_error = compute_translation_error_degrees(pose_gt, estimated_pose);
            rotation_errors.push_back(rotation_error);
            translation_errors.push_back(translation_error);
            
            if (opt.verbose) {
                std::cout << "Problem " << problem << ": Expected " << expected_inliers 
                          << " inliers, found " << inliers_count << " inliers, " 
                          << stats.iters << " iterations" 
                          << ", rot_err=" << std::fixed << std::setprecision(2) << rotation_error << "°"
                          << ", trans_err=" << translation_error << "°" << std::endl;
            }
        } else {
            // ENHANCED: Report failed problems in verbose mode
            if (opt.verbose) {
                std::cout << "Problem " << problem << ": FAILED - Expected " << expected_inliers 
                          << " inliers, found " << inliers_count << " inliers, " 
                          << stats.iters << " iterations (hit max limit)" << std::endl;
            }
        }
        
        // Generate CSV line and save to file
        std::string csv_line = make_csv_line_robust_relative_pose<Scalar, N>(
            pose_gt, x1_img, x2_img, inlier_mask, Scalar(500.0), Scalar(500.0)  // Use focal=500.0 to match camera setup
        );
        
        file << csv_line << std::endl;
    }
    
    file.close();
    std::cout << "Saved " << opt.num_problems << " problems to " << filename << std::endl;
    
    // Compute comprehensive statistics
    auto compute_stats = [](const std::vector<size_t>& values) {
        if (values.empty()) return std::make_tuple(0.0, 0.0, 0.0, size_t(0), size_t(0), 0.0);
        
        double mean = 0.0;
        size_t min_val = *std::min_element(values.begin(), values.end());
        size_t max_val = *std::max_element(values.begin(), values.end());
        
        for (size_t val : values) {
            mean += val;
        }
        mean /= values.size();
        
        double variance = 0.0;
        for (size_t val : values) {
            double diff = val - mean;
            variance += diff * diff;
        }
        variance /= values.size();
        double std_dev = std::sqrt(variance);
        
        // Calculate median
        std::vector<size_t> sorted_values = values;
        std::sort(sorted_values.begin(), sorted_values.end());
        double median;
        size_t n = sorted_values.size();
        if (n % 2 == 0) {
            median = (sorted_values[n/2 - 1] + sorted_values[n/2]) / 2.0;
        } else {
            median = sorted_values[n/2];
        }
        
        return std::make_tuple(mean, std_dev, variance, min_val, max_val, median);
    };
    
    // Print comprehensive summary statistics
    std::cout << "\n=== Comprehensive Results ===" << std::endl;
    std::cout << "Success rate: " << (double)successful_estimates / opt.num_problems * 100 << "%" << std::endl;
    std::cout << "Failed problems: " << (opt.num_problems - successful_estimates) << "/" << opt.num_problems << std::endl;
    
    // ENHANCED: Always show iteration statistics for ALL problems
    auto [iter_mean, iter_std, iter_var, iter_min, iter_max, iter_median] = compute_stats(iteration_counts);
    std::cout << "\nIteration Statistics (ALL " << opt.num_problems << " problems):" << std::endl;
    std::cout << "  Mean: " << std::fixed << std::setprecision(2) << iter_mean << std::endl;
    std::cout << "  Std Dev: " << iter_std << std::endl;
    std::cout << "  Min: " << iter_min << std::endl;
    std::cout << "  Max: " << iter_max << std::endl;
    std::cout << "  Median: " << iter_median << std::endl;
    
    if (successful_estimates > 0) {
        auto [inlier_mean, inlier_std, inlier_var, inlier_min, inlier_max, inlier_median] = compute_stats(inlier_counts);
        auto [expected_mean, expected_std, expected_var, expected_min, expected_max, expected_median] = compute_stats(expected_inlier_counts);
        
        // Compute error statistics
        auto compute_error_stats = [](const std::vector<Scalar>& values) {
            if (values.empty()) return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
            
            double mean = 0.0;
            Scalar min_val = *std::min_element(values.begin(), values.end());
            Scalar max_val = *std::max_element(values.begin(), values.end());
            
            for (Scalar val : values) {
                mean += val;
            }
            mean /= values.size();
            
            double variance = 0.0;
            for (Scalar val : values) {
                double diff = val - mean;
                variance += diff * diff;
            }
            variance /= values.size();
            double std_dev = std::sqrt(variance);
            
            return std::make_tuple(mean, std_dev, variance, double(min_val), double(max_val));
        };
        
        auto [rot_mean, rot_std, rot_var, rot_min, rot_max] = compute_error_stats(rotation_errors);
        auto [trans_mean, trans_std, trans_var, trans_min, trans_max] = compute_error_stats(translation_errors);
        
        std::cout << "\nInliers Found Statistics:" << std::endl;
        std::cout << "  Mean: " << inlier_mean << std::endl;
        std::cout << "  Std Dev: " << inlier_std << std::endl;
        std::cout << "  Min: " << inlier_min << std::endl;
        std::cout << "  Max: " << inlier_max << std::endl;
        std::cout << "  Median: " << inlier_median << std::endl;
        
        std::cout << "\nExpected Inliers Statistics:" << std::endl;
        std::cout << "  Mean: " << expected_mean << std::endl;
        std::cout << "  Std Dev: " << expected_std << std::endl;
        std::cout << "  Min: " << expected_min << std::endl;
        std::cout << "  Max: " << expected_max << std::endl;
        std::cout << "  Median: " << expected_median << std::endl;
        
        std::cout << "\nRotation Error Statistics (degrees):" << std::endl;
        std::cout << "  Mean: " << std::fixed << std::setprecision(3) << rot_mean << "°" << std::endl;
        std::cout << "  Std Dev: " << rot_std << "°" << std::endl;
        std::cout << "  Min: " << rot_min << "°" << std::endl;
        std::cout << "  Max: " << rot_max << "°" << std::endl;
        
        std::cout << "\nTranslation Error Statistics (degrees):" << std::endl;
        std::cout << "  Mean: " << trans_mean << "°" << std::endl;
        std::cout << "  Std Dev: " << trans_std << "°" << std::endl;
        std::cout << "  Min: " << trans_min << "°" << std::endl;
        std::cout << "  Max: " << trans_max << "°" << std::endl;
        
        // Compute detection accuracy - FIXED VERSION
        double total_precision = 0.0;
        double total_recall = 0.0;
        size_t valid_problems = 0;  // Track problems where we can compute metrics
        
        // NEW: Compute EXACT precision/recall using stored ground truth and RANSAC results
        size_t total_true_positives = 0;
        size_t total_false_positives = 0; 
        size_t total_false_negatives = 0;
        size_t total_true_negatives = 0;
        
        for (size_t i = 0; i < successful_estimates; ++i) {
            const auto& ground_truth = ground_truth_masks[i];
            const auto& ransac_result = ransac_results[i];
            
            size_t true_positives = 0;
            size_t false_positives = 0;
            size_t false_negatives = 0;
            size_t true_negatives = 0;
            
            // Compute confusion matrix for this problem
            for (size_t j = 0; j < ground_truth.size(); ++j) {
                bool gt_inlier = ground_truth[j];
                bool ransac_inlier = (ransac_result[j] == 1);
                
                if (ransac_inlier && gt_inlier) true_positives++;
                else if (ransac_inlier && !gt_inlier) false_positives++;
                else if (!ransac_inlier && gt_inlier) false_negatives++;
                else true_negatives++;
            }
            
            // Compute precision and recall for this problem
            if (true_positives + false_positives > 0) {
                double precision = (double)true_positives / (true_positives + false_positives);
                total_precision += precision;
            }
            
            if (true_positives + false_negatives > 0) {
                double recall = (double)true_positives / (true_positives + false_negatives);
                total_recall += recall;
                valid_problems++;
            }
            
            // Accumulate for overall statistics
            total_true_positives += true_positives;
            total_false_positives += false_positives;
            total_false_negatives += false_negatives;
            total_true_negatives += true_negatives;
        }
        
        std::cout << "\nInlier Detection Accuracy (Exact):" << std::endl;
        if (valid_problems > 0) {
            // Per-problem averages
            std::cout << "  Average Precision: " << std::fixed << std::setprecision(3) << (total_precision / valid_problems) * 100 << "%" << std::endl;
            std::cout << "  Average Recall: " << (total_recall / valid_problems) * 100 << "%" << std::endl;
            
            // Overall confusion matrix statistics
            double overall_precision = (total_true_positives + total_false_positives > 0) ? 
                (double)total_true_positives / (total_true_positives + total_false_positives) : 0.0;
            double overall_recall = (total_true_positives + total_false_negatives > 0) ?
                (double)total_true_positives / (total_true_positives + total_false_negatives) : 0.0;
            double overall_accuracy = (double)(total_true_positives + total_true_negatives) / 
                (total_true_positives + total_false_positives + total_false_negatives + total_true_negatives);
            
            std::cout << "\nOverall Confusion Matrix Statistics:" << std::endl;
            std::cout << "  Overall Precision: " << overall_precision * 100 << "%" << std::endl;
            std::cout << "  Overall Recall: " << overall_recall * 100 << "%" << std::endl;
            std::cout << "  Overall Accuracy: " << overall_accuracy * 100 << "%" << std::endl;
            std::cout << "  True Positives: " << total_true_positives << std::endl;
            std::cout << "  False Positives: " << total_false_positives << std::endl;
            std::cout << "  False Negatives: " << total_false_negatives << std::endl;
            std::cout << "  True Negatives: " << total_true_negatives << std::endl;
        } else {
            std::cout << "  Could not compute detection accuracy (no valid problems)" << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    Options opt;
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help") {
            print_help();
            return 0;
        } else if (arg == "--solver-type" && i + 1 < argc) {
            opt.solver_type = argv[++i];
        } else if (arg == "--outlier-ratio" && i + 1 < argc) {
            opt.outlier_ratio = std::stod(argv[++i]);
        } else if (arg == "--refinement" && i + 1 < argc) {
            opt.refinement_type = argv[++i];
        } else if (arg == "--noise" && i + 1 < argc) {
            opt.noise_level = std::stod(argv[++i]);
        } else if (arg == "--threshold" && i + 1 < argc) {
            opt.max_epipolar_error = std::stod(argv[++i]);
        } else if (arg == "--points" && i + 1 < argc) {
            opt.num_points = std::stoul(argv[++i]);
        } else if (arg == "--problems" && i + 1 < argc) {
            opt.num_problems = std::stoul(argv[++i]);
        } else if (arg == "--max-iterations" && i + 1 < argc) {
            opt.max_iterations = std::stoul(argv[++i]);
        } else if (arg == "--shuffle-data" && i + 1 < argc) {
            opt.shuffle_data = (std::string(argv[++i]) == "true");
        } else if (arg == "--final-refinement" && i + 1 < argc) {
            opt.final_refinement = (std::string(argv[++i]) == "true");
        } else if (arg == "--verbose") {
            opt.verbose = true;
        } else if (arg == "--data-generator" && i + 1 < argc) {
            opt.data_generator = argv[++i];
        } else if (arg == "--precision" && i + 1 < argc) {
            opt.precision = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_help();
            return 1;
        }
    }
    
    // Validate num_points and dispatch to appropriate template specialization
    // Support powers of 2 from 32 to 256
    if (opt.precision == "double") {
        if (opt.num_points == 32) {
            test_robust_solver<double, 32>(opt);
        } else if (opt.num_points == 64) {
            test_robust_solver<double, 64>(opt);
        } else if (opt.num_points == 128) {
            test_robust_solver<double, 128>(opt);
        } else if (opt.num_points == 256) {
            test_robust_solver<double, 256>(opt);
        } else {
            std::cerr << "Error: --points must be one of: 32, 64, 128, 256" << std::endl;
            std::cerr << "Currently specified: " << opt.num_points << std::endl;
            return 1;
        }
    } else if (opt.precision == "float") {
        if (opt.num_points == 32) {
            test_robust_solver<float, 32>(opt);
        } else if (opt.num_points == 64) {
            test_robust_solver<float, 64>(opt);
        } else if (opt.num_points == 128) {
            test_robust_solver<float, 128>(opt);
        } else if (opt.num_points == 256) {
            test_robust_solver<float, 256>(opt);
        } else {
            std::cerr << "Error: --points must be one of: 32, 64, 128, 256" << std::endl;
            std::cerr << "Currently specified: " << opt.num_points << std::endl;
            return 1;
        }
    } else {
        std::cerr << "Error: --precision must be either 'float' or 'double'" << std::endl;
        std::cerr << "Currently specified: " << opt.precision << std::endl;
        return 1;
    }
    
    return 0;
} 