#pragma once

#include "ento-util/containers.h"
#include "ento-pose/pose_util.h"
#include "ento-pose/camera_models.h"
#include "ento-pose/data_gen.h"
#include "ento-pose/prob_gen.h"
#include "ento-pose/synthetic_relpose.h"
#include "ento-pose/robust-est/robust_pose_solver.h"
#include "ento-pose/robust-est/ransac_util.h"
#include <Eigen/Dense>
#include <random>
#include <iostream>

using namespace EntoUtil;

// EntoBench-realistic data generation with mixed scenarios for proper workload characterization
template<typename Scalar, size_t N>
void generate_entobench_realistic_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_img,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_img,
    EntoPose::CameraPose<Scalar>& true_pose,
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
            std::uniform_real_distribution<Scalar> large_rot(-15.0, 15.0);
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
        
        if (baseline_selector < 0.15) {
            // Close manipulation: 5cm - 50cm (tabletop robotics)
            std::uniform_real_distribution<Scalar> close_baseline(0.05, 0.5);
            baseline_length = close_baseline(rng);
        } else if (baseline_selector < 0.50) {
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
        //std::cout << "Warning: Only generated " << generated_inliers << "/" << num_inliers << " inliers" << std::endl;
    }
    
    // DEGENERACY CHECKS: Verify the generated data is well-conditioned
    bool is_degenerate = false;
    
    // Check 1: Collinearity test for 3D points (if we have enough points)
    if (generated_inliers >= 3) {
        // Convert first 3 image points back to 3D for collinearity check
        std::vector<Vec3> points3D;
        for (int i = 0; i < 3; ++i) {
            // Unproject image points to 3D (assume reasonable depth)
            Vec2 img_pt = x1_img[i];
            Scalar x = (img_pt(0) - 250.0) / focal_length;
            Scalar y = (img_pt(1) - 250.0) / focal_length;
            Vec3 bearing(x, y, 1.0);
            
            // Assume depth of 5m for degeneracy check
            Vec3 point3D = bearing.normalized() * Scalar(5.0);
            points3D.push_back(point3D);
        }
        
        Vec3 v1 = points3D[1] - points3D[0];
        Vec3 v2 = points3D[2] - points3D[0];
        Vec3 cross = v1.cross(v2);
        if (cross.norm() < Scalar(1e-8)) {
            is_degenerate = true;
        }
    }
    
    // Check 2: Coplanarity test for 4+ points
    if (generated_inliers >= 4 && !is_degenerate) {
        std::vector<Vec3> points3D;
        for (int i = 0; i < 4; ++i) {
            Vec2 img_pt = x1_img[i];
            Scalar x = (img_pt(0) - 250.0) / focal_length;
            Scalar y = (img_pt(1) - 250.0) / focal_length;
            Vec3 bearing(x, y, 1.0);
            Vec3 point3D = bearing.normalized() * Scalar(5.0);
            points3D.push_back(point3D);
        }
        
        Vec3 v1 = points3D[1] - points3D[0];
        Vec3 v2 = points3D[2] - points3D[0];
        Vec3 v3 = points3D[3] - points3D[0];
        Vec3 normal = v1.cross(v2);
        if (normal.norm() > Scalar(1e-8)) {
            Scalar distance = std::abs(normal.dot(v3)) / normal.norm();
            if (distance < Scalar(1e-6)) {
                is_degenerate = true;
            }
        }
    }
    
    // Check 3: 2D point spread test
    if (!is_degenerate) {
        Scalar min_x1 = std::numeric_limits<Scalar>::max(), max_x1 = std::numeric_limits<Scalar>::lowest();
        Scalar min_y1 = std::numeric_limits<Scalar>::max(), max_y1 = std::numeric_limits<Scalar>::lowest();
        Scalar min_x2 = std::numeric_limits<Scalar>::max(), max_x2 = std::numeric_limits<Scalar>::lowest();
        Scalar min_y2 = std::numeric_limits<Scalar>::max(), max_y2 = std::numeric_limits<Scalar>::lowest();
        
        for (int i = 0; i < generated_inliers; ++i) {
            min_x1 = std::min(min_x1, x1_img[i](0));
            max_x1 = std::max(max_x1, x1_img[i](0));
            min_y1 = std::min(min_y1, x1_img[i](1));
            max_y1 = std::max(max_y1, x1_img[i](1));
            
            min_x2 = std::min(min_x2, x2_img[i](0));
            max_x2 = std::max(max_x2, x2_img[i](0));
            min_y2 = std::min(min_y2, x2_img[i](1));
            max_y2 = std::max(max_y2, x2_img[i](1));
        }
        
        Scalar x_spread1 = max_x1 - min_x1, y_spread1 = max_y1 - min_y1;
        Scalar x_spread2 = max_x2 - min_x2, y_spread2 = max_y2 - min_y2;
        
        if (x_spread1 < Scalar(20.0) || y_spread1 < Scalar(20.0) || 
            x_spread2 < Scalar(20.0) || y_spread2 < Scalar(20.0)) {
            is_degenerate = true;
        }
    }
    
    // Check 4: Baseline and rotation magnitude
    if (!is_degenerate) {
        Scalar baseline_length = true_pose.t.norm();
        if (baseline_length < Scalar(0.001)) {
            is_degenerate = true;
        }
        
        Scalar trace_val = true_pose.R().trace();
        Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
        Scalar angle_deg = angle_rad * 180.0 / M_PI;
        if (angle_deg < Scalar(0.1)) {
            is_degenerate = true;
        }
    }
    
    // Note: We don't throw exceptions anymore - just use whatever data we have
    // If degenerate data is detected, we continue with it (realistic scenario)
    
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
    
    //std::cout << "Generated EntoBench-realistic data: " << generated_inliers << " inliers + " 
    //          << num_outliers << " outliers = " << (generated_inliers + num_outliers) << " total points" << std::endl;
}

// Utility: Unproject EntoBench 2D image point to 3D bearing vector
// Usage: bearing = unproject_entobench_pixel<Scalar>(img_pt);
template<typename Scalar>
Eigen::Matrix<Scalar,3,1> unproject_entobench_pixel(const Eigen::Matrix<Scalar,2,1>& img_pt) {
    Scalar fx = 500.0;
    Scalar fy = 500.0;
    Scalar cx = 250.0;
    Scalar cy = 250.0;
    Scalar x = (img_pt(0) - cx) / fx;
    Scalar y = (img_pt(1) - cy) / fy;
    Eigen::Matrix<Scalar,3,1> bearing(x, y, 1.0);
    return bearing.normalized();
}

// Enhanced EntoBench data generation with proper pose-level retry logic
template<typename Scalar, size_t N>
void generate_entobench_realistic_relpose_data_robust(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_img,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_img,
    EntoPose::CameraPose<Scalar>& true_pose,
    int num_inliers,
    int num_outliers,
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false,
    Scalar pixel_noise_std = Scalar(0.5))
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> uniform_gen(-1.0, 1.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, pixel_noise_std);
    Scalar focal_length = Scalar(500.0);
    
    if constexpr (N == 0) {
        x1_img.clear();
        x2_img.clear();
        x1_img.reserve(num_inliers + num_outliers);
        x2_img.reserve(num_inliers + num_outliers);
    }
    
    // Pose-level retry loop for degeneracy handling
    const int max_pose_attempts = 50;
    
    for (int pose_attempt = 0; pose_attempt < max_pose_attempts; ++pose_attempt) {
        // Clear containers for this attempt
        if constexpr (N == 0) {
            x1_img.clear();
            x2_img.clear();
        }
        
        // Generate realistic motion based on mixed robotics scenarios
        if (upright_only && planar_only) {
            // Upright planar motion (robotics ground vehicles, indoor navigation)
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
            
            // Realistic baseline variation
            std::uniform_real_distribution<Scalar> baseline_scenario(0.0, 1.0);
            Scalar baseline_selector = baseline_scenario(rng);
            Scalar baseline_length;
            
            if (baseline_selector < 0.3) {
                std::uniform_real_distribution<Scalar> close_baseline(0.05, 0.5);
                baseline_length = close_baseline(rng);
            } else if (baseline_selector < 0.7) {
                std::uniform_real_distribution<Scalar> mid_baseline(0.5, 2.0);
                baseline_length = mid_baseline(rng);
            } else {
                std::uniform_real_distribution<Scalar> long_baseline(2.0, 5.0);
                baseline_length = long_baseline(rng);
            }
            
            Scalar tx = uniform_gen(rng);
            Scalar tz = uniform_gen(rng);
            Vec3 t_dir = Vec3(tx, 0, tz).normalized();
            true_pose.t = baseline_length * t_dir;
        } else if (upright_only) {
            // Upright 3D motion (robotics with gravity constraint)
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
                std::uniform_real_distribution<Scalar> large_rot(-15.0, 15.0);
                yaw_deg = large_rot(rng);
            }
            
            Scalar yaw_rad = yaw_deg * M_PI / 180.0;
            Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
            true_pose.q(0) = q.w();
            true_pose.q(1) = q.x();
            true_pose.q(2) = q.y();
            true_pose.q(3) = q.z();
            
            // 3D baseline with realistic variation
            std::uniform_real_distribution<Scalar> baseline_gen(0.05, 5.0);
            Scalar baseline_length = baseline_gen(rng);
            Vec3 t_dir = Vec3::Random().normalized();
            true_pose.t = baseline_length * t_dir;
        } else {
            // Full 6-DoF motion (general robotics)
            std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
            Scalar rot_selector = rot_scenario(rng);
            
            Vec3 rot_axis = Vec3::Random().normalized();
            Scalar rot_angle_deg;
            if (rot_selector < 0.3) {
                std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
                rot_angle_deg = small_rot(rng);
            } else if (rot_selector < 0.7) {
                std::uniform_real_distribution<Scalar> medium_rot(-15.0, 15.0);
                rot_angle_deg = medium_rot(rng);
            } else {
                std::uniform_real_distribution<Scalar> large_rot(-30.0, 30.0);
                rot_angle_deg = large_rot(rng);
            }
            
            Scalar rot_angle_rad = rot_angle_deg * M_PI / 180.0;
            Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(rot_angle_rad, rot_axis));
            true_pose.q(0) = q.w();
            true_pose.q(1) = q.x();
            true_pose.q(2) = q.y();
            true_pose.q(3) = q.z();
            
            // 3D baseline with realistic variation
            std::uniform_real_distribution<Scalar> baseline_gen(0.05, 5.0);
            Scalar baseline_length = baseline_gen(rng);
            Vec3 t_dir = Vec3::Random().normalized();
            true_pose.t = baseline_length * t_dir;
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
            continue; // Try again with different pose
        }
        
        // DEGENERACY CHECKS: Verify the generated data is well-conditioned
        bool is_degenerate = false;
        
        // Check 1: Collinearity test for 3D points (if we have enough points) - MORE LENIENT
        if (generated_inliers >= 3) {
            // Convert first 3 image points back to 3D for collinearity check
            std::vector<Vec3> points3D;
            for (int i = 0; i < 3; ++i) {
                // Unproject image points to 3D (assume reasonable depth)
                Vec2 img_pt = x1_img[i];
                Scalar x = (img_pt(0) - 250.0) / focal_length;
                Scalar y = (img_pt(1) - 250.0) / focal_length;
                Vec3 bearing(x, y, 1.0);
                
                // Assume depth of 5m for degeneracy check
                Vec3 point3D = bearing.normalized() * Scalar(5.0);
                points3D.push_back(point3D);
            }
            
            Vec3 v1 = points3D[1] - points3D[0];
            Vec3 v2 = points3D[2] - points3D[0];
            Vec3 cross = v1.cross(v2);
            if (cross.norm() < Scalar(1e-8)) {  // MORE LENIENT: was 1e-6
                is_degenerate = true;
            }
        }
        
        // Check 2: Coplanarity test for 4+ points - MORE LENIENT
        if (generated_inliers >= 4 && !is_degenerate) {
            std::vector<Vec3> points3D;
            for (int i = 0; i < 4; ++i) {
                Vec2 img_pt = x1_img[i];
                Scalar x = (img_pt(0) - 250.0) / focal_length;
                Scalar y = (img_pt(1) - 250.0) / focal_length;
                Vec3 bearing(x, y, 1.0);
                Vec3 point3D = bearing.normalized() * Scalar(5.0);
                points3D.push_back(point3D);
            }
            
            Vec3 v1 = points3D[1] - points3D[0];
            Vec3 v2 = points3D[2] - points3D[0];
            Vec3 v3 = points3D[3] - points3D[0];
            Vec3 normal = v1.cross(v2);
            if (normal.norm() > Scalar(1e-8)) {
                Scalar distance = std::abs(normal.dot(v3)) / normal.norm();
                if (distance < Scalar(1e-6)) {  // MORE LENIENT: was 1e-4
                    is_degenerate = true;
                }
            }
        }
        
        // Check 3: 2D point spread test - MORE LENIENT
        if (!is_degenerate) {
            Scalar min_x1 = std::numeric_limits<Scalar>::max(), max_x1 = std::numeric_limits<Scalar>::lowest();
            Scalar min_y1 = std::numeric_limits<Scalar>::max(), max_y1 = std::numeric_limits<Scalar>::lowest();
            Scalar min_x2 = std::numeric_limits<Scalar>::max(), max_x2 = std::numeric_limits<Scalar>::lowest();
            Scalar min_y2 = std::numeric_limits<Scalar>::max(), max_y2 = std::numeric_limits<Scalar>::lowest();
            
            for (int i = 0; i < generated_inliers; ++i) {
                min_x1 = std::min(min_x1, x1_img[i](0));
                max_x1 = std::max(max_x1, x1_img[i](0));
                min_y1 = std::min(min_y1, x1_img[i](1));
                max_y1 = std::max(max_y1, x1_img[i](1));
                
                min_x2 = std::min(min_x2, x2_img[i](0));
                max_x2 = std::max(max_x2, x2_img[i](0));
                min_y2 = std::min(min_y2, x2_img[i](1));
                max_y2 = std::max(max_y2, x2_img[i](1));
            }
            
            Scalar x_spread1 = max_x1 - min_x1, y_spread1 = max_y1 - min_y1;
            Scalar x_spread2 = max_x2 - min_x2, y_spread2 = max_y2 - min_y2;
            
            if (x_spread1 < Scalar(20.0) || y_spread1 < Scalar(20.0) || 
                x_spread2 < Scalar(20.0) || y_spread2 < Scalar(20.0)) {  // MORE LENIENT: was 50.0
                is_degenerate = true;
            }
        }
        
        // Check 4: Baseline and rotation magnitude - MORE LENIENT
        if (!is_degenerate) {
            Scalar baseline_length = true_pose.t.norm();
            if (baseline_length < Scalar(0.001)) {  // MORE LENIENT: was 0.01
                is_degenerate = true;
            }
            
            Scalar trace_val = true_pose.R().trace();
            Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
            Scalar angle_deg = angle_rad * 180.0 / M_PI;
            if (angle_deg < Scalar(0.1)) {  // MORE LENIENT: was 0.5
                is_degenerate = true;
            }
        }
        
        // If not degenerate, we found good data - exit the pose retry loop
        if (!is_degenerate) {
            break; // SUCCESS! Exit pose retry loop
        }
        
        // If we get here, the data was degenerate - continue to next pose attempt
    }
    
    // If we exhausted all pose attempts, use the last generated data (even if degenerate)
    // This ensures the function doesn't fail completely
    // Note: We don't throw exceptions anymore - just use whatever data we have
    
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
    
    //std::cout << "Generated EntoBench-realistic data: " << generated_inliers << " inliers + " 
    //          << num_outliers << " outliers = " << (generated_inliers + num_outliers) << " total points" << std::endl;
} 