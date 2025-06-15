#ifndef ENTO_POSE_GOLD_STANDARD_ABS_H
#define ENTO_POSE_GOLD_STANDARD_ABS_H

#include <ento-pose/abs-pose/dlt.h>
#include <ento-pose/bundle_adjustment.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/robust-est/loss.h>
#include <ento-util/containers.h>
#include <vector>

namespace EntoPose {

// Gold Standard Absolute Pose (H&Z): DLT initialization + Bundle Adjustment refinement
// This implements the Hartley & Zisserman gold standard method:
// 1. Linear initialization using DLT
// 2. Nonlinear refinement using bundle adjustment (minimizing reprojection error)

template <typename Scalar, size_t N>
int gold_standard_abs(const EntoArray<Vec3<Scalar>, N>& x,
                      const EntoArray<Vec3<Scalar>, N>& X,
                      EntoArray<CameraPose<Scalar>, 1>* solutions)
{
    static_assert(N >= 6, "Gold standard absolute pose requires at least 6 points");
    
    // Step 1: Linear initialization using DLT
    EntoArray<CameraPose<Scalar>, 1> dlt_solutions;
    int num_dlt_solutions = dlt<Scalar, N>(x, X, &dlt_solutions);
    
    if (num_dlt_solutions == 0) {
        ENTO_DEBUG("[Gold Standard Abs] DLT initialization failed");
        return 0;
    }
    
    CameraPose<Scalar> pose = dlt_solutions[0];
    ENTO_DEBUG("[Gold Standard Abs] DLT initialization successful");
    
    // Step 2: Nonlinear refinement using bundle adjustment
    // Convert homogeneous 2D points to Euclidean for bundle adjustment
    EntoArray<Vec2<Scalar>, N> points2D;
    EntoArray<Vec3<Scalar>, N> points3D;
    
    for (size_t i = 0; i < N; ++i) {
        // Convert from homogeneous to Euclidean coordinates
        Vec2<Scalar> pt2d(x[i].x() / x[i].z(), x[i].y() / x[i].z());
        points2D.push_back(pt2d);
        points3D.push_back(X[i]);
    }
    
    // Setup bundle adjustment options
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.max_iterations = 1;  // Reduce to just 1 iteration for debugging
    bundle_opt.loss_scale = Scalar(1e-3);
    bundle_opt.verbose = true;  // Enable verbose output for debugging
    
    ENTO_DEBUG("[Gold Standard Abs] Starting bundle adjustment with %zu points", N);
    
    // Use identity camera model (normalized coordinates)
    using WeightType = UniformWeightVector<Scalar>;
    using LossFunction = TruncatedLoss<Scalar>;
    
    BundleStats<Scalar> stats = bundle_adjust<Scalar, WeightType, LossFunction, N>(
        points2D, points3D, &pose, bundle_opt);
    
    ENTO_DEBUG("[Gold Standard Abs] Bundle adjustment: %d iterations, final cost: %f", 
               stats.iterations, stats.cost);
    
    // Return refined solution
    solutions->clear();
    solutions->push_back(pose);
    return 1;
}

#if defined(NATIVE)
// std::vector overload for NATIVE builds
template <typename Scalar>
int gold_standard_abs(const std::vector<Vec3<Scalar>>& x,
                      const std::vector<Vec3<Scalar>>& X,
                      std::vector<CameraPose<Scalar>>* solutions)
{
    size_t N = x.size();
    if (N < 6 || X.size() != N) {
        ENTO_DEBUG("[Gold Standard Abs] Insufficient points: need >= 6, got %zu", N);
        return 0;
    }
    
    // Step 1: Linear initialization using DLT
    std::vector<CameraPose<Scalar>> dlt_solutions;
    int num_dlt_solutions = dlt<Scalar>(x, X, &dlt_solutions);
    
    if (num_dlt_solutions == 0) {
        ENTO_DEBUG("[Gold Standard Abs] DLT initialization failed");
        return 0;
    }
    
    CameraPose<Scalar> pose = dlt_solutions[0];
    ENTO_DEBUG("[Gold Standard Abs] DLT initialization successful");
    
    // Step 2: Nonlinear refinement using bundle adjustment
    // Convert homogeneous 2D points to Euclidean for bundle adjustment
    EntoUtil::EntoContainer<Vec2<Scalar>, 0> points2D;
    EntoUtil::EntoContainer<Vec3<Scalar>, 0> points3D;
    
    points2D.reserve(N);
    points3D.reserve(N);
    
    for (size_t i = 0; i < N; ++i) {
        // Convert from homogeneous to Euclidean coordinates
        Vec2<Scalar> pt2d(x[i].x() / x[i].z(), x[i].y() / x[i].z());
        points2D.push_back(pt2d);
        points3D.push_back(X[i]);
    }
    
    // Setup bundle adjustment options
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.max_iterations = 1;  // Reduce to just 1 iteration for debugging
    bundle_opt.loss_scale = Scalar(1e-3);
    bundle_opt.verbose = true;  // Enable verbose output for debugging
    
    ENTO_DEBUG("[Gold Standard Abs] Starting bundle adjustment with %zu points", N);
    
    // Use identity camera model (normalized coordinates)
    using WeightType = UniformWeightVector<Scalar>;
    using LossFunction = TruncatedLoss<Scalar>;
    
    BundleStats<Scalar> stats = bundle_adjust<Scalar, WeightType, LossFunction, 0>(
        points2D, points3D, &pose, bundle_opt);
    
    ENTO_DEBUG("[Gold Standard Abs] Bundle adjustment: %d iterations, final cost: %f", 
               stats.iterations, stats.cost);
    
    // Return refined solution
    solutions->clear();
    solutions->push_back(pose);
    return 1;
}
#endif

} // namespace EntoPose

#endif // ENTO_POSE_GOLD_STANDARD_ABS_H 