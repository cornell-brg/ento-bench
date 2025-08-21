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

// High-quality data generation based on the standalone test approach
template<typename Scalar, size_t N>
void generate_high_quality_relpose_data(
    EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_img,
    EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_img,
    EntoPose::CameraPose<Scalar>& true_pose,
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