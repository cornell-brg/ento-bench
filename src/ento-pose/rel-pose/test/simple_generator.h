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

using namespace EntoUtil;

// Simple, controlled data generation that produces well-conditioned problems
template<typename Scalar, size_t N>
void generate_simple_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_img,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_img,
    EntoPose::CameraPose<Scalar>& true_pose,
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