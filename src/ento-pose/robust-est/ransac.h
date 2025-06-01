#ifndef RANSAC_H
#define RANSAC_H

#include <limits>
#include <ento-math/core.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/robust-est/relative.h>
#include <ento-pose/robust-est/homography.h>
#include <ento-pose/robust-est/absolute.h>
#include <ento-util/containers.h>

namespace EntoPose
{

// RANSAC state for tracking best models and dynamic iteration count
template <typename Scalar>
struct RansacState {
    size_t best_minimal_inlier_count = 0;
    Scalar best_minimal_msac_score = std::numeric_limits<Scalar>::max();
    size_t dynamic_max_iter = 100000;
    Scalar log_prob_missing_model = std::log(Scalar(1.0) - Scalar(0.9999));
};

// Score multiple models and perform local optimization on the best one
template <typename Scalar, typename Solver, typename Model = CameraPose<Scalar>>
void score_models(const Solver &estimator, 
                  const EntoUtil::EntoContainer<Model, Solver::MaxSolns> &models, 
                  const RansacOptions<Scalar> &opt,
                  RansacState<Scalar> &state, 
                  RansacStats<Scalar> &stats, 
                  Model *best_model) 
{
    // Find best model among candidates
    int best_model_ind = -1;
    size_t inlier_count = 0;
    
    for (size_t i = 0; i < models.size(); ++i) {
        Scalar score_msac = estimator.score_model(models[i], &inlier_count);
        bool more_inliers = inlier_count > state.best_minimal_inlier_count;
        bool better_score = score_msac < state.best_minimal_msac_score;

        if (more_inliers || better_score) {
            if (more_inliers) {
                state.best_minimal_inlier_count = inlier_count;
            }
            if (better_score) {
                state.best_minimal_msac_score = score_msac;
            }
            best_model_ind = i;

            // Check if we should update best model already
            if (score_msac < stats.model_score) {
                stats.model_score = score_msac;
                *best_model = models[i];
                stats.num_inliers = inlier_count;
            }
        }
    }

    if (best_model_ind == -1)
        return;

    // LO-RANSAC: Local optimization (refinement) on best model
    Model refined_model = models[best_model_ind];
    estimator.refine_model(&refined_model);
    stats.refinements++;
    
    Scalar refined_msac_score = estimator.score_model(refined_model, &inlier_count);
    if (refined_msac_score < stats.model_score) {
        stats.model_score = refined_msac_score;
        stats.num_inliers = inlier_count;
        *best_model = refined_model;
    }

    // Update dynamic iteration count based on inlier ratio
    stats.inlier_ratio = static_cast<Scalar>(stats.num_inliers) / static_cast<Scalar>(estimator.num_data_);
    if (stats.inlier_ratio >= Scalar(0.9999)) {
        // Avoid log(prob_outlier) = -inf below
        state.dynamic_max_iter = opt.min_iters;
    } else if (stats.inlier_ratio <= Scalar(0.0001)) {
        // Avoid log(prob_outlier) = 0 below
        state.dynamic_max_iter = opt.max_iters;
    } else {
        const Scalar prob_outlier = Scalar(1.0) - std::pow(stats.inlier_ratio, static_cast<Scalar>(estimator.sample_size_));
        state.dynamic_max_iter = static_cast<size_t>(
            std::ceil(state.log_prob_missing_model / std::log(prob_outlier) * opt.dynamic_trials_mult)
        );
    }
}

// Generic LO-RANSAC template that works with any estimator class
// This is the core RANSAC implementation used by all problem-specific functions
template <typename Scalar, typename Solver, typename Model = CameraPose<Scalar>>
RansacStats<Scalar> ransac(Solver &estimator, const RansacOptions<Scalar> &opt, Model *best_model) 
{
    RansacStats<Scalar> stats;

    if (estimator.num_data_ < estimator.sample_size_) {
        return stats;
    }

    // Initialize stats
    stats.num_inliers = 0;
    stats.model_score = std::numeric_limits<Scalar>::max();
    
    // Initialize RANSAC state
    RansacState<Scalar> state;
    state.dynamic_max_iter = opt.max_iters;
    state.log_prob_missing_model = std::log(Scalar(1.0) - opt.success_prob);

    // Score initial model if it was supplied
    if (opt.score_initial_model) {
        EntoUtil::EntoContainer<Model, Solver::MaxSolns> initial_models;
        initial_models.push_back(*best_model);
        score_models(estimator, initial_models, opt, state, stats, best_model);
    }

    // Main RANSAC loop
    EntoUtil::EntoContainer<Model, Solver::MaxSolns> models;
    for (stats.iters = 0; stats.iters < opt.max_iters; stats.iters++) {
        
        if (stats.iters > opt.min_iters && stats.iters > state.dynamic_max_iter) {
            break;
        }
        
        models.clear();
        estimator.generate_models(&models);
        score_models(estimator, models, opt, state, stats, best_model);
    }

    // Final refinement (always performed)
    Model refined_model = *best_model;
    estimator.refine_model(&refined_model);
    stats.refinements++;
    
    size_t inlier_count = 0;
    Scalar refined_msac_score = estimator.score_model(refined_model, &inlier_count);
    if (refined_msac_score < stats.model_score) {
        *best_model = refined_model;
        stats.num_inliers = inlier_count;
        stats.model_score = refined_msac_score;
    }

    return stats;
}

} // namespace EntoPose

#endif // RANSAC_H
