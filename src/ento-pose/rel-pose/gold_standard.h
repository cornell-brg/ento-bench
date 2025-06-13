#ifndef ENTO_POSE_GOLD_STANDARD_REL_H
#define ENTO_POSE_GOLD_STANDARD_REL_H

#include <ento-pose/rel-pose/eight_pt.h>
#include <ento-pose/bundle_adjustment.h>
#include <ento-pose/robust-est/loss.h>
#include <ento-util/containers.h>
#include <vector>

namespace EntoPose {

// Gold Standard Relative Pose (H&Z): 8pt initialization + Sampson error refinement
// This implements the Hartley & Zisserman gold standard method:
// 1. Linear initialization using 8-point algorithm
// 2. Nonlinear refinement using Sampson error minimization

template <typename Scalar, size_t N>
int gold_standard_rel(const EntoArray<Vec3<Scalar>, N>& x1,
                      const EntoArray<Vec3<Scalar>, N>& x2,
                      EntoArray<CameraPose<Scalar>, 1>* solutions)
{
    static_assert(N >= 8, "Gold standard relative pose requires at least 8 points");
    
    // Step 1: Linear initialization using 8-point algorithm
    EntoArray<CameraPose<Scalar>, 4> eight_pt_solutions;
    int num_eight_pt_solutions = relpose_8pt<Scalar, N>(x1, x2, &eight_pt_solutions);
    
    if (num_eight_pt_solutions == 0) {
        ENTO_DEBUG("[Gold Standard Rel] 8-point initialization failed");
        return 0;
    }
    
    CameraPose<Scalar> pose = eight_pt_solutions[0];
    ENTO_DEBUG("[Gold Standard Rel] 8-point initialization successful");
    
    // Step 2: Nonlinear refinement using Sampson error minimization
    // Convert homogeneous 3D points to Euclidean 2D points for refinement
    EntoArray<Vec2<Scalar>, N> points2D_1;
    EntoArray<Vec2<Scalar>, N> points2D_2;
    
    for (size_t i = 0; i < N; ++i) {
        // Convert from homogeneous to Euclidean coordinates
        Vec2<Scalar> pt1(x1[i].x() / x1[i].z(), x1[i].y() / x1[i].z());
        Vec2<Scalar> pt2(x2[i].x() / x2[i].z(), x2[i].y() / x2[i].z());
        points2D_1.push_back(pt1);
        points2D_2.push_back(pt2);
    }
    
    // Setup bundle adjustment options for relative pose refinement
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.max_iterations = 20;
    bundle_opt.loss_scale = Scalar(1e-3);
    bundle_opt.verbose = false;
    
    // Use Sampson error refinement (relative pose bundle adjustment)
    using WeightType = UniformWeightVector<Scalar>;
    using LossFunction = TruncatedLoss<Scalar>;
    
    BundleStats<Scalar> stats = refine_relpose<Scalar, WeightType, LossFunction, N>(
        points2D_1, points2D_2, &pose, bundle_opt);
    
    ENTO_DEBUG("[Gold Standard Rel] Sampson refinement: %d iterations, final cost: %f", 
               stats.iterations, stats.cost);
    
    // Return refined solution
    solutions->clear();
    solutions->push_back(pose);
    return 1;
}

#if defined(NATIVE)
// std::vector overload for NATIVE builds
template <typename Scalar>
int gold_standard_rel(const std::vector<Vec3<Scalar>>& x1,
                      const std::vector<Vec3<Scalar>>& x2,
                      std::vector<CameraPose<Scalar>>* solutions)
{
    size_t N = x1.size();
    if (N < 8 || x2.size() != N) {
        ENTO_DEBUG("[Gold Standard Rel] Insufficient points: need >= 8, got %zu", N);
        return 0;
    }
    
    // Step 1: Linear initialization using 8-point algorithm
    std::vector<CameraPose<Scalar>> eight_pt_solutions;
    int num_eight_pt_solutions = relpose_8pt<Scalar, 0>(x1, x2, &eight_pt_solutions);
    
    if (num_eight_pt_solutions == 0) {
        ENTO_DEBUG("[Gold Standard Rel] 8-point initialization failed");
        return 0;
    }
    
    CameraPose<Scalar> pose = eight_pt_solutions[0];
    ENTO_DEBUG("[Gold Standard Rel] 8-point initialization successful");
    
    // Step 2: Nonlinear refinement using Sampson error minimization
    // Convert homogeneous 3D points to Euclidean 2D points for refinement
    EntoUtil::EntoContainer<Vec2<Scalar>, 0> points2D_1;
    EntoUtil::EntoContainer<Vec2<Scalar>, 0> points2D_2;
    
    points2D_1.reserve(N);
    points2D_2.reserve(N);
    
    for (size_t i = 0; i < N; ++i) {
        // Convert from homogeneous to Euclidean coordinates
        Vec2<Scalar> pt1(x1[i].x() / x1[i].z(), x1[i].y() / x1[i].z());
        Vec2<Scalar> pt2(x2[i].x() / x2[i].z(), x2[i].y() / x2[i].z());
        points2D_1.push_back(pt1);
        points2D_2.push_back(pt2);
    }
    
    // Setup bundle adjustment options for relative pose refinement
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.max_iterations = 20;
    bundle_opt.loss_scale = Scalar(1e-3);
    bundle_opt.verbose = false;
    
    // Use Sampson error refinement (relative pose bundle adjustment)
    using WeightType = UniformWeightVector<Scalar>;
    using LossFunction = TruncatedLoss<Scalar>;
    
    BundleStats<Scalar> stats = refine_relpose<Scalar, WeightType, LossFunction, 0>(
        points2D_1, points2D_2, &pose, bundle_opt);
    
    ENTO_DEBUG("[Gold Standard Rel] Sampson refinement: %d iterations, final cost: %f", 
               stats.iterations, stats.cost);
    
    // Return refined solution
    solutions->clear();
    solutions->push_back(pose);
    return 1;
}
#endif

} // namespace EntoPose

#endif // ENTO_POSE_GOLD_STANDARD_REL_H 