#ifndef ENTO_POSE_SYNTHETIC_ABSPOSE_H
#define ENTO_POSE_SYNTHETIC_ABSPOSE_H

#include <random>
#include <Eigen/Dense>
#include <ento-util/containers.h>
#include <ento-pose/pose_util.h>

namespace EntoPose {

// General absolute pose (arbitrary rotation/translation)
template<typename Scalar, size_t N>
void generate_synthetic_abspose_general(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Set random true pose
    true_pose.q = Quaternion::UnitRandom().coeffs();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0); // Scale translation
    
    std::default_random_engine rng(42); // Fixed seed for reproducibility
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
        while (true) {
            // Generate 3D point
            X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            // Project to camera coordinates
            x_cam = true_pose.R() * X + true_pose.t;
            if (x_cam(2) > Scalar(0.1)) break; // Accept only if in front of camera
        }
        x_norm = Vec2(x_cam(0) / x_cam(2), x_cam(1) / x_cam(2));
        // Add small amount of noise
        x_norm(0) += noise_gen(rng);
        x_norm(1) += noise_gen(rng);

        points2D.push_back(x_norm);
        points3D.push_back(X);
        ++valid_points;
    }
}

// Upright absolute pose (yaw-only, y-up)
template<typename Scalar, size_t N>
void generate_synthetic_abspose_upright(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Set random true pose with only y-axis rotation
    Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
    Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
    // Store quaternion with real part first (w, x, y, z)
    true_pose.q(0) = q.w();
    true_pose.q(1) = q.x();
    true_pose.q(2) = q.y();
    true_pose.q(3) = q.z();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0); // Scale translation
    
    std::default_random_engine rng(42); // Fixed seed for reproducibility
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
        while (true) {
            // Generate 3D point
            X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            // Project to camera coordinates
            x_cam = true_pose.R() * X + true_pose.t;
            if (x_cam(2) > Scalar(0.1)) break; // Accept only if in front of camera
        }
        x_norm = Vec2(x_cam(0) / x_cam(2), x_cam(1) / x_cam(2));
        // Add small amount of noise
        x_norm(0) += noise_gen(rng);
        x_norm(1) += noise_gen(rng);

        points2D.push_back(x_norm);
        points3D.push_back(X);
        ++valid_points;
    }
}

// OpenGV-style: Generate synthetic bearing vector correspondences for absolute pose
// Each correspondence is a 3D unit vector (bearing vector) in the camera frame
// true_pose is the absolute pose from world to camera (R, t)
// points3D: 3D points in world frame
// x_bear: output container for bearing vectors in camera frame
// N = 0 for dynamic, N > 0 for fixed-size
// noise_level: standard deviation of Gaussian noise (radians) added in tangent space
template<typename Scalar, size_t N>
void generate_synthetic_abspose_bearing_vectors(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Set random true pose
    true_pose.q = Quaternion::UnitRandom().coeffs();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0);
    
    std::default_random_engine rng(42);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    
    if constexpr (N == 0) {
        x_bear.clear();
        points3D.clear();
        x_bear.reserve(num_points);
        points3D.reserve(num_points);
    }
    
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x_cam;
        while (true) {
            // Generate 3D point
            X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            // Project to camera coordinates
            x_cam = true_pose.R() * X + true_pose.t;
            if (x_cam(2) > Scalar(0.1)) break; // Accept only if in front of camera
        }
        
        // Bearing vector (unit vector in camera frame)
        Vec3 f = x_cam.normalized();
        
        // Add noise in tangent space (small angle approx)
        if (noise_level > Scalar(0.0)) {
            // Generate random 3D vector orthogonal to f
            Vec3 n1 = Vec3::Random().normalized();
            n1 -= n1.dot(f) * f;
            n1.normalize();
            Vec3 n2 = f.cross(n1).normalized();
            Scalar theta = noise_gen(rng);
            Scalar phi = noise_gen(rng);
            f = (f + theta * n1 + phi * n2).normalized();
        }
        
        x_bear.push_back(f);
        points3D.push_back(X);
        ++valid_points;
    }
}

// Upright (yaw-only, y-up) OpenGV-style: Generate synthetic bearing vector correspondences for upright absolute pose
// Each correspondence is a 3D unit vector (bearing vector) in the camera frame
// true_pose is the upright absolute pose from world to camera (yaw-only R, t)
// points3D: 3D points in world frame
// x_bear: output container for bearing vectors in camera frame
// N = 0 for dynamic, N > 0 for fixed-size
// noise_level: standard deviation of Gaussian noise (radians) added in tangent space
template<typename Scalar, size_t N>
void generate_synthetic_abspose_upright_bearing_vectors(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Set random true pose with only y-axis rotation
    std::default_random_engine rng(42);
    std::uniform_real_distribution<Scalar> yaw_gen(0, Scalar(2.0 * M_PI));
    Scalar yaw = yaw_gen(rng);
    Quaternion q(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
    true_pose.q(0) = q.w();
    true_pose.q(1) = q.x();
    true_pose.q(2) = q.y();
    true_pose.q(3) = q.z();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0);
    
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    
    if constexpr (N == 0) {
        x_bear.clear();
        points3D.clear();
        x_bear.reserve(num_points);
        points3D.reserve(num_points);
    }
    
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x_cam;
        while (true) {
            // Generate 3D point
            X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            // Project to camera coordinates
            x_cam = true_pose.R() * X + true_pose.t;
            if (x_cam(2) > Scalar(0.1)) break; // Accept only if in front of camera
        }
        
        // Bearing vector (unit vector in camera frame)
        Vec3 f = x_cam.normalized();
        
        // Add noise in tangent space (small angle approx)
        if (noise_level > Scalar(0.0)) {
            // Generate random 3D vector orthogonal to f
            Vec3 n1 = Vec3::Random().normalized();
            n1 -= n1.dot(f) * f;
            n1.normalize();
            Vec3 n2 = f.cross(n1).normalized();
            Scalar theta = noise_gen(rng);
            Scalar phi = noise_gen(rng);
            f = (f + theta * n1 + phi * n2).normalized();
        }
        
        x_bear.push_back(f);
        points3D.push_back(X);
        ++valid_points;
    }
}

} // namespace EntoPose 

#endif // ENTO_POSE_SYNTHETIC_ABSPOSE_H