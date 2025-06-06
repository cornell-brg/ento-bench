#pragma once
#include <random>
#include <Eigen/Dense>
#include <ento-util/containers.h>
#include <ento-pose/pose_util.h>

namespace EntoPose {

// General relative pose (arbitrary rotation/translation)
template<typename Scalar, size_t N>
void generate_synthetic_relpose_general(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_2d,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_2d,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    true_pose.q = Quaternion::UnitRandom().coeffs();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0);
    std::default_random_engine rng(42);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    if constexpr (N == 0) {
        x1_2d.clear(); x2_2d.clear(); x1_2d.reserve(num_points); x2_2d.reserve(num_points);
    }
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x1_cam, x2_cam;
        while (true) {
            Scalar x = coord_gen(rng), y = coord_gen(rng), z = depth_gen(rng);
            X = Vec3(x, y, z); x1_cam = X;
            if (x1_cam(2) > Scalar(0.1)) break;
        }
        x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        Vec2 x1_norm(x1_cam(0)/x1_cam(2), x1_cam(1)/x1_cam(2));
        Vec2 x2_norm(x2_cam(0)/x2_cam(2), x2_cam(1)/x2_cam(2));
        x1_norm(0) += noise_gen(rng); x1_norm(1) += noise_gen(rng);
        x2_norm(0) += noise_gen(rng); x2_norm(1) += noise_gen(rng);
        x1_2d.push_back(x1_norm); x2_2d.push_back(x2_norm); ++valid_points;
    }
}

// Upright (yaw-only, y-up)
template<typename Scalar, size_t N>
void generate_synthetic_relpose_upright(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_2d,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_2d,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    std::default_random_engine rng(42);
    std::uniform_real_distribution<Scalar> yaw_gen(0, Scalar(2.0 * M_PI));
    Scalar yaw = yaw_gen(rng);
    Quaternion q(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
    true_pose.q(0) = q.w(); true_pose.q(1) = q.x(); true_pose.q(2) = q.y(); true_pose.q(3) = q.z();
    true_pose.t.setRandom(); true_pose.t *= Scalar(2.0);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    if constexpr (N == 0) {
        x1_2d.clear(); x2_2d.clear(); x1_2d.reserve(num_points); x2_2d.reserve(num_points);
    }
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x1_cam, x2_cam;
        while (true) {
            Scalar x = coord_gen(rng), y = coord_gen(rng), z = depth_gen(rng);
            X = Vec3(x, y, z); x1_cam = X;
            if (x1_cam(2) > Scalar(0.1)) break;
        }
        x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        Vec2 x1_norm(x1_cam(0)/x1_cam(2), x1_cam(1)/x1_cam(2));
        Vec2 x2_norm(x2_cam(0)/x2_cam(2), x2_cam(1)/x2_cam(2));
        x1_norm(0) += noise_gen(rng); x1_norm(1) += noise_gen(rng);
        x2_norm(0) += noise_gen(rng); x2_norm(1) += noise_gen(rng);
        x1_2d.push_back(x1_norm); x2_2d.push_back(x2_norm); ++valid_points;
    }
}

// Upright planar (planar motion, y-up)
template<typename Scalar, size_t N>
void generate_synthetic_relpose_upright_planar(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1_2d,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2_2d,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    std::default_random_engine rng(42);
    std::uniform_real_distribution<Scalar> yaw_gen(0, Scalar(2.0 * M_PI));
    std::uniform_real_distribution<Scalar> trans_gen(-2.0, 2.0);
    Scalar yaw = yaw_gen(rng);
    Quaternion q(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
    true_pose.q(0) = q.w(); true_pose.q(1) = q.x(); true_pose.q(2) = q.y(); true_pose.q(3) = q.z();
    Scalar tx = trans_gen(rng); Scalar tz = trans_gen(rng);
    true_pose.t = Vec3(tx, Scalar(0), tz);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    if constexpr (N == 0) {
        x1_2d.clear(); x2_2d.clear(); x1_2d.reserve(num_points); x2_2d.reserve(num_points);
    }
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x1_cam, x2_cam;
        while (true) {
            Scalar x = coord_gen(rng), y = coord_gen(rng), z = depth_gen(rng);
            X = Vec3(x, y, z); x1_cam = X;
            if (x1_cam(2) > Scalar(0.1)) break;
        }
        x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        Vec2 x1_norm(x1_cam(0)/x1_cam(2), x1_cam(1)/x1_cam(2));
        Vec2 x2_norm(x2_cam(0)/x2_cam(2), x2_cam(1)/x2_cam(2));
        x1_norm(0) += noise_gen(rng); x1_norm(1) += noise_gen(rng);
        x2_norm(0) += noise_gen(rng); x2_norm(1) += noise_gen(rng);
        x1_2d.push_back(x1_norm); x2_2d.push_back(x2_norm); ++valid_points;
    }
}

// OpenGV-style: Generate synthetic bearing vector correspondences for relative pose
// Each correspondence is a pair of 3D unit vectors (bearing vectors) in each camera frame
// true_pose is the relative pose from camera 1 to camera 2 (R, t)
// x1_bear, x2_bear: output containers for bearing vectors in each camera
// N = 0 for dynamic, N > 0 for fixed-size
// noise_level: standard deviation of Gaussian noise (radians) added in tangent space
//
template<typename Scalar, size_t N>
void generate_synthetic_relpose_bearing_vectors(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    true_pose.q = Quaternion::UnitRandom().coeffs();
    true_pose.t.setRandom();
    true_pose.t *= Scalar(2.0);
    std::default_random_engine rng(42);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    if constexpr (N == 0) {
        x1_bear.clear(); x2_bear.clear(); x1_bear.reserve(num_points); x2_bear.reserve(num_points);
    }
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x1_cam, x2_cam;
        while (true) {
            Scalar x = coord_gen(rng), y = coord_gen(rng), z = depth_gen(rng);
            X = Vec3(x, y, z); x1_cam = X;
            if (x1_cam(2) > Scalar(0.1)) break;
        }
        x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        // Bearing vectors (unit vectors in each camera frame)
        Vec3 f1 = x1_cam.normalized();
        Vec3 f2 = x2_cam.normalized();
        // Add noise in tangent space (small angle approx)
        if (noise_level > Scalar(0.0)) {
            // Generate random 3D vector orthogonal to f1
            Vec3 n1 = Vec3::Random().normalized();
            n1 -= n1.dot(f1) * f1; n1.normalize();
            Vec3 n2 = f1.cross(n1).normalized();
            Scalar theta1 = noise_gen(rng);
            Scalar phi1 = noise_gen(rng);
            f1 = (f1 + theta1 * n1 + phi1 * n2).normalized();
            // Same for f2
            Vec3 m1 = Vec3::Random().normalized();
            m1 -= m1.dot(f2) * f2; m1.normalize();
            Vec3 m2 = f2.cross(m1).normalized();
            Scalar theta2 = noise_gen(rng);
            Scalar phi2 = noise_gen(rng);
            f2 = (f2 + theta2 * m1 + phi2 * m2).normalized();
        }
        x1_bear.push_back(f1); x2_bear.push_back(f2); ++valid_points;
    }
}

// Upright (yaw-only, y-up) OpenGV-style: Generate synthetic bearing vector correspondences for upright relative pose
// Each correspondence is a pair of 3D unit vectors (bearing vectors) in each camera frame
// true_pose is the upright relative pose from camera 1 to camera 2 (yaw-only R, t)
// x1_bear, x2_bear: output containers for bearing vectors in each camera
// N = 0 for dynamic, N > 0 for fixed-size
// noise_level: standard deviation of Gaussian noise (radians) added in tangent space
//
template<typename Scalar, size_t N>
void generate_synthetic_relpose_upright_bearing_vectors(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0))
{
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    std::default_random_engine rng(42);
    std::uniform_real_distribution<Scalar> yaw_gen(0, Scalar(2.0 * M_PI));
    std::uniform_real_distribution<Scalar> trans_gen(-2.0, 2.0);
    Scalar yaw = yaw_gen(rng);
    Quaternion q(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitY()));
    true_pose.q(0) = q.w(); true_pose.q(1) = q.x(); true_pose.q(2) = q.y(); true_pose.q(3) = q.z();
    Scalar tx = trans_gen(rng); Scalar tz = trans_gen(rng);
    true_pose.t = Vec3(tx, Scalar(0), tz);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
    if constexpr (N == 0) {
        x1_bear.clear(); x2_bear.clear(); x1_bear.reserve(num_points); x2_bear.reserve(num_points);
    }
    size_t valid_points = 0;
    while (valid_points < num_points) {
        Vec3 X, x1_cam, x2_cam;
        while (true) {
            Scalar x = coord_gen(rng), y = coord_gen(rng), z = depth_gen(rng);
            X = Vec3(x, y, z); x1_cam = X;
            if (x1_cam(2) > Scalar(0.1)) break;
        }
        x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        // Bearing vectors (unit vectors in each camera frame)
        Vec3 f1 = x1_cam.normalized();
        Vec3 f2 = x2_cam.normalized();
        // Add noise in tangent space (small angle approx)
        if (noise_level > Scalar(0.0)) {
            // Generate random 3D vector orthogonal to f1
            Vec3 n1 = Vec3::Random().normalized();
            n1 -= n1.dot(f1) * f1; n1.normalize();
            Vec3 n2 = f1.cross(n1).normalized();
            Scalar theta1 = noise_gen(rng);
            Scalar phi1 = noise_gen(rng);
            f1 = (f1 + theta1 * n1 + phi1 * n2).normalized();
            // Same for f2
            Vec3 m1 = Vec3::Random().normalized();
            m1 -= m1.dot(f2) * f2; m1.normalize();
            Vec3 m2 = f2.cross(m1).normalized();
            Scalar theta2 = noise_gen(rng);
            Scalar phi2 = noise_gen(rng);
            f2 = (f2 + theta2 * m1 + phi2 * m2).normalized();
        }
        x1_bear.push_back(f1); x2_bear.push_back(f2); ++valid_points;
    }
}

// Utility: Convert a container of 3D bearing vectors to normalized 2D points (Vec2) using hnormalized()
// x_bear: input container of 3D unit vectors
// x_2d: output container of 2D normalized points (cleared and filled)
template<typename Scalar, size_t N>
void bearing_vectors_to_normalized_points(const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x_bear,
                                          EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x_2d)
{
    x_2d.clear();
    for (size_t i = 0; i < x_bear.size(); ++i) {
        x_2d.push_back(x_bear[i].hnormalized());
    }
}

// Utility: Convert a container of 3D bearing vectors to 2D image points using a camera model (default: IdentityCameraModel)
// x_bear: input container of 3D unit vectors
// x_2d: output container of 2D image points (cleared and filled)
// camera: camera model to use for projection (default: identity)
template<typename Scalar, size_t N>
void bearing_vectors_to_image_points(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x_2d,
    const Camera<Scalar, IdentityCameraModel<Scalar>>& camera = Camera<Scalar, IdentityCameraModel<Scalar>>(1, 1, {}))
{
    x_2d.clear();
    for (size_t i = 0; i < x_bear.size(); ++i) {
        Eigen::Matrix<Scalar,2,1> norm_pt = x_bear[i].hnormalized();
        Eigen::Matrix<Scalar,2,1> img_pt;
        camera.project(norm_pt, &img_pt);
        x_2d.push_back(img_pt);
    }
}

} // namespace EntoPose 
