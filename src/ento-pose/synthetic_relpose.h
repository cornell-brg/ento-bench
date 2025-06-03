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

} // namespace EntoPose 