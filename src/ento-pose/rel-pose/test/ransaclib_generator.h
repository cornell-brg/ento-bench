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

// RansacLib-inspired data generation that mimics their successful approach
// Based on successful RansacLib parameters that achieve high success rates
template<typename Scalar, size_t N>
void generate_ransaclib_inspired_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_img,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_img,
    EntoPose::CameraPose<Scalar>& true_pose,
    int num_inliers,
    int num_outliers,
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false,
    Scalar pixel_noise_std = Scalar(1.15))  // RansacLib standard: ~1.15 pixels
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> uniform_gen(-1.0, 1.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, pixel_noise_std);
    Scalar focal_length = Scalar(520.0);  // RansacLib-like focal length
    
    // Generate simple, well-conditioned motion like RansacLib
    // They use modest rotations and reasonable baselines
    if (upright_only && planar_only) {
        // Simple planar motion - modest yaw rotations
        std::uniform_real_distribution<Scalar> yaw_gen(-15.0, 15.0);  // ±15 degrees
        Scalar yaw_deg = yaw_gen(rng);
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // Simple baseline in XZ plane
        std::uniform_real_distribution<Scalar> baseline_gen(0.3, 1.5);  // 0.3-1.5m
        Scalar baseline_length = baseline_gen(rng);
        Scalar tx = uniform_gen(rng);
        Scalar tz = uniform_gen(rng);
        Vec3 t_dir = Vec3(tx, 0, tz).normalized();
        true_pose.t = baseline_length * t_dir;
        
    } else if (upright_only) {
        // Upright motion with modest yaw
        std::uniform_real_distribution<Scalar> yaw_gen(-20.0, 20.0);  // ±20 degrees
        Scalar yaw_deg = yaw_gen(rng);
        Scalar yaw_rad = yaw_deg * M_PI / 180.0;
        Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
        true_pose.q(0) = q.w();
        true_pose.q(1) = q.x();
        true_pose.q(2) = q.y();
        true_pose.q(3) = q.z();
        
        // 3D baseline
        std::uniform_real_distribution<Scalar> baseline_gen(0.3, 1.5);
        Scalar baseline_length = baseline_gen(rng);
        Vec3 t_dir = Vec3(uniform_gen(rng), uniform_gen(rng), uniform_gen(rng)).normalized();
        true_pose.t = baseline_length * t_dir;
        
    } else {
        // General 6DOF motion - keep it modest like RansacLib
        std::uniform_real_distribution<Scalar> angle_gen(-10.0, 10.0);  // ±10 degrees
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
        
        // Modest baseline
        std::uniform_real_distribution<Scalar> baseline_gen(0.3, 1.2);
        Scalar baseline_length = baseline_gen(rng);
        Vec3 t_dir = Vec3(uniform_gen(rng), uniform_gen(rng), uniform_gen(rng)).normalized();
        true_pose.t = baseline_length * t_dir;
    }
    
    if constexpr (N == 0) {
        x1_img.clear();
        x2_img.clear();
        x1_img.reserve(num_inliers + num_outliers);
        x2_img.reserve(num_inliers + num_outliers);
    }
    
    std::cout << "Generating RansacLib-inspired data: " << num_inliers << " inliers + " 
              << num_outliers << " outliers = " << (num_inliers + num_outliers) << " total points" << std::endl;
    
    // Generate inlier points following RansacLib approach
    // They use simple depth ranges: 2-8m with good distribution
    int generated_inliers = 0;
    int attempts = 0;
    const int max_attempts = num_inliers * 3;
    
    while (generated_inliers < num_inliers && attempts < max_attempts) {
        attempts++;
        
        // RansacLib-like point distribution: simple and well-conditioned
        // Depth range: 2-8m (their successful range)
        std::uniform_real_distribution<Scalar> lateral_gen(-2.0, 2.0);    // ±2m lateral
        std::uniform_real_distribution<Scalar> depth_gen(2.0, 8.0);       // 2-8m depth (RansacLib range)
        
        Vec3 X;
        if (planar_only) {
            // Ground plane with small height variation
            std::uniform_real_distribution<Scalar> height_gen(-0.3, 1.0);
            X = Vec3(lateral_gen(rng), height_gen(rng), depth_gen(rng));
        } else {
            // Full 3D distribution
            X = Vec3(lateral_gen(rng), lateral_gen(rng), depth_gen(rng));
        }
        
        // Project to first camera (identity pose)
        Vec3 x1_cam = X;
        if (x1_cam(2) <= Scalar(0.1)) continue;
        
        // Project to second camera
        Vec3 x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        
        // Project to image coordinates using RansacLib-like parameters
        Vec2 img1 = Vec2(
            focal_length * x1_cam(0) / x1_cam(2) + 320.0,  // cx = 320 (640x480 center)
            focal_length * x1_cam(1) / x1_cam(2) + 240.0   // cy = 240 (640x480 center)
        );
        Vec2 img2 = Vec2(
            focal_length * x2_cam(0) / x2_cam(2) + 320.0,
            focal_length * x2_cam(1) / x2_cam(2) + 240.0
        );
        
        // Check if points are within image bounds (640x480 like RansacLib tests)
        if (img1(0) < 50 || img1(0) > 590 || img1(1) < 50 || img1(1) > 430 ||
            img2(0) < 50 || img2(0) > 590 || img2(1) < 50 || img2(1) > 430) {
            continue;
        }
        
        // Add RansacLib-level pixel noise (~1.15 pixels std)
        img1(0) += pixel_noise_gen(rng);
        img1(1) += pixel_noise_gen(rng);
        img2(0) += pixel_noise_gen(rng);
        img2(1) += pixel_noise_gen(rng);
        
        x1_img.push_back(img1);
        x2_img.push_back(img2);
        generated_inliers++;
        
        if (generated_inliers <= 3) {  // Print first few
            std::cout << "  Inlier " << generated_inliers << ": (" << img1(0) << "," << img1(1) 
                      << ") -> (" << img2(0) << "," << img2(1) << ")" << std::endl;
        }
    }
    
    if (generated_inliers < num_inliers) {
        std::cout << "WARNING: Only generated " << generated_inliers << "/" << num_inliers << " inliers" << std::endl;
    }
    
    // Generate outlier points within image bounds like RansacLib
    std::uniform_real_distribution<Scalar> outlier_gen_x(50.0, 590.0);   // Within 640px width
    std::uniform_real_distribution<Scalar> outlier_gen_y(50.0, 430.0);   // Within 480px height
    
    for (int i = 0; i < num_outliers; ++i) {
        Vec2 outlier1(outlier_gen_x(rng), outlier_gen_y(rng));
        Vec2 outlier2(outlier_gen_x(rng), outlier_gen_y(rng));
        
        // Add noise to outliers too
        outlier1(0) += pixel_noise_gen(rng);
        outlier1(1) += pixel_noise_gen(rng);
        outlier2(0) += pixel_noise_gen(rng);
        outlier2(1) += pixel_noise_gen(rng);
        
        x1_img.push_back(outlier1);
        x2_img.push_back(outlier2);
        
        if (i < 2) {  // Print first few
            std::cout << "  Outlier " << (i+1) << ": (" << outlier1(0) << "," << outlier1(1) 
                      << ") -> (" << outlier2(0) << "," << outlier2(1) << ")" << std::endl;
        }
    }
    
    std::cout << "Final RansacLib-inspired dataset: " << x1_img.size() << " correspondences" << std::endl;
    std::cout << "Motion: rotation=" << std::sqrt(true_pose.q(1)*true_pose.q(1) + true_pose.q(2)*true_pose.q(2) + true_pose.q(3)*true_pose.q(3)) * 2.0 * 180.0/M_PI 
              << "°, baseline=" << true_pose.t.norm() << "m" << std::endl;
} 