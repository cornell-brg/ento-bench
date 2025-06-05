#ifndef ROBUST_POSE_SOLVER_H
#define ROBUST_POSE_SOLVER_H

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/ransac.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/robust.h>
#include <ento-util/containers.h>

namespace EntoPose
{

template <typename Solver, size_t N = 0>
class RobustRelativePoseSolver {
public:
    using Scalar = typename Solver::scalar_type;
    
    RobustRelativePoseSolver(const RansacOptions<Scalar>& options, 
                            const BundleOptions<Scalar>& bundle_options,
                            const Camera<Scalar>& camera1,
                            const Camera<Scalar>& camera2)
        : options_(options)
        , bundle_options_(bundle_options)
        , camera1_(camera1)
        , camera2_(camera2) {}

    RansacStats<Scalar> solve(const EntoUtil::EntoContainer<Vec2<Scalar>, N>& x1,
               const EntoUtil::EntoContainer<Vec2<Scalar>, N>& x2,
               CameraPose<Scalar>* best_pose,
               EntoUtil::EntoContainer<uint8_t, N>* inliers = nullptr) {
        
        RansacStats<Scalar> stats;
        if (inliers) {

            ENTO_DEBUG("Calling estimate_relative_pose in RobustRelativePoseSolver.");
            stats = estimate_relative_pose<Solver, N>(x1, x2, camera1_, camera2_, 
                                                     options_, bundle_options_,
                                                     best_pose, inliers);
        } else {
            EntoUtil::EntoContainer<uint8_t, N> temp_inliers;
            stats = estimate_relative_pose<Solver, N>(x1, x2, camera1_, camera2_,
                                                     options_, bundle_options_,
                                                     best_pose, &temp_inliers);
        }

        return stats;
    }

private:
    const RansacOptions<Scalar>& options_;
    const BundleOptions<Scalar>& bundle_options_;
    const Camera<Scalar>& camera1_;
    const Camera<Scalar>& camera2_;
};

template <typename Solver, size_t N = 0>
class RobustAbsolutePoseSolver {
public:
    using Scalar = typename Solver::scalar_type;
    
    RobustAbsolutePoseSolver(const RansacOptions<Scalar>& options,
                            const BundleOptions<Scalar>& bundle_options,
                            const Camera<Scalar>& camera)
        : options_(options)
        , bundle_options_(bundle_options)
        , camera_(camera) {}

    bool solve(const EntoUtil::EntoContainer<Vec2<Scalar>, N>& points2D,
               const EntoUtil::EntoContainer<Vec3<Scalar>, N>& points3D,
               CameraPose<Scalar>* best_pose,
               EntoUtil::EntoContainer<uint8_t, N>* inliers = nullptr) {
        
        RansacStats<Scalar> stats;
        if (inliers) {
            stats = estimate_absolute_pose<Solver, N>(points2D, points3D, camera_,
                                                     options_, bundle_options_,
                                                     best_pose, inliers);
        } else {
            EntoUtil::EntoContainer<uint8_t, N> temp_inliers;
            stats = estimate_absolute_pose<Solver, N>(points2D, points3D, camera_,
                                                     options_, bundle_options_,
                                                     best_pose, &temp_inliers);
        }

        return stats.num_inliers >= options_.min_iters;
    }

private:
    const RansacOptions<Scalar>& options_;
    const BundleOptions<Scalar>& bundle_options_;
    const Camera<Scalar>& camera_;
};

} // namespace EntoPose

#endif // ROBUST_POSE_SOLVER_H 
