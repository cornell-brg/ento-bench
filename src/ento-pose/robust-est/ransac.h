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
#include <ento-pose/robust-est/linear_refinement.h>

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
            if (score_msac < stats.model_score) {
                stats.model_score = score_msac;
                *best_model = models[i];
                stats.num_inliers = inlier_count;
            }
        }
    }
    if (best_model_ind == -1)
        return;

    // LO-RANSAC: Local optimization (refinement) on best model using inliers
    Model refined_model = models[best_model_ind];
    if (opt.lo_type != LocalRefinementType::None) {
        // Get inliers for the best model
        EntoUtil::EntoContainer<uint8_t, Solver::N_> inliers;
        int num_inl;
        if constexpr (std::is_same_v<Solver, RelativePoseRobustEstimator<typename Solver::SolverType, Solver::N_>>) {
            num_inl = get_inliers<Scalar, Solver::N_>(refined_model, estimator.x1, estimator.x2, opt.max_epipolar_error * opt.max_epipolar_error, &inliers);
        } else if constexpr (std::is_same_v<Solver, AbsolutePoseRobustEstimator<typename Solver::SolverType, Solver::N_>>) {
            num_inl = get_inliers<Scalar, Solver::N_>(refined_model, estimator.points2D_, estimator.points3D_, opt.max_reproj_error * opt.max_reproj_error, &inliers);
        }

        ENTO_DEBUG("[RANSAC] Number of inliers: %d", num_inl);
        if (num_inl > Solver::sample_size_) {
            ENTO_DEBUG("[RANSAC] Number of inliers is greater than sample size. Performing local optimization.");
            // Gather inlier points
            if constexpr (std::is_same_v<Solver, RelativePoseRobustEstimator<typename Solver::SolverType, Solver::N_>>) {
                ENTO_DEBUG("[RANSAC] Performing linear refinement for relative pose");
                EntoUtil::EntoContainer<Vec2<Scalar>, Solver::N_> x1_inl, x2_inl;
                if constexpr (Solver::N_ == 0) {
                    x1_inl.reserve(num_inl);
                    x2_inl.reserve(num_inl);
                }
                for (size_t pt_k = 0; pt_k < estimator.x1.size(); ++pt_k) {
                    if (inliers[pt_k]) {
                        x1_inl.push_back(estimator.x1[pt_k]);
                        x2_inl.push_back(estimator.x2[pt_k]);
                    }
                }

                if (opt.lo_type == LocalRefinementType::Linear || opt.lo_type == LocalRefinementType::Both) {
                    if (opt.linear_method == LinearRefinementMethod::EightPoint) {
                        ENTO_DEBUG("[RANSAC] Refining model with 8pt linear refinement");
                        if (opt.use_irls) {
                            ENTO_DEBUG("[RANSAC] Using IRLS Huber refinement for 8pt (max_iters=%d, threshold=%f)", 
                                       opt.irls_max_iters, opt.irls_huber_threshold);
                            linear_refine_irls_huber_eight_point<Scalar, Solver::N_>(x1_inl, x2_inl, &refined_model, 
                                                                                     opt.irls_max_iters, opt.irls_huber_threshold);
                        } else {
                            linear_refine_eight_point<Scalar, Solver::N_>(x1_inl, x2_inl, &refined_model);
                        }
                    } else if (opt.linear_method == LinearRefinementMethod::UprightPlanar3pt) {
                        ENTO_DEBUG("[RANSAC] Refining model with upright planar 3pt linear refinement");
                        if (opt.use_irls) {
                            ENTO_DEBUG("[RANSAC] Using IRLS Huber refinement for upright planar 3pt (max_iters=%d, threshold=%f)", 
                                       opt.irls_max_iters, opt.irls_huber_threshold);
                            linear_refine_irls_huber_upright_planar_3pt<Scalar, Solver::N_>(x1_inl, x2_inl, &refined_model, 
                                                                                            opt.irls_max_iters, opt.irls_huber_threshold);
                        } else {
                            linear_refine_upright_planar_3pt<Scalar, Solver::N_>(x1_inl, x2_inl, &refined_model);
                        }
                    }
                }
                // Nonlinear refinement
                if (opt.lo_type == LocalRefinementType::BundleAdjust || opt.lo_type == LocalRefinementType::Both) {
                    // Use estimator's refine_model (bundle adjustment)
                    ENTO_DEBUG("[RANSAC] Refining model with bundle adjustment");
                    estimator.refine_model(&refined_model);
                }

                if (opt.lo_type != LocalRefinementType::None) {
                    stats.refinements++;
                }
            }
            else if constexpr (std::is_same_v<Solver, AbsolutePoseRobustEstimator<typename Solver::SolverType, Solver::N_>>)
            {
                ENTO_DEBUG("[RANSAC] Performing linear refinement for absolute pose");
                EntoUtil::EntoContainer<Vec2<Scalar>, Solver::N_> x1_inl;
                EntoUtil::EntoContainer<Vec3<Scalar>, Solver::N_> x2_inl;
                if constexpr (Solver::N_ == 0) {
                    x1_inl.reserve(num_inl);
                    x2_inl.reserve(num_inl);
                }
                for (size_t pt_k = 0; pt_k < estimator.points2D_.size(); ++pt_k) {
                    if (inliers[pt_k]) {
                        x1_inl.push_back(estimator.points2D_[pt_k]);
                        x2_inl.push_back(estimator.points3D_[pt_k]);
                    }
                }

                if (opt.lo_type == LocalRefinementType::Linear || opt.lo_type == LocalRefinementType::Both) {
                    if (opt.linear_method == LinearRefinementMethod::DLT) {
                        ENTO_DEBUG("[RANSAC] Refining model with DLT linear refinement");
                        if (opt.use_irls) {
                            ENTO_DEBUG("[RANSAC] Using IRLS Huber refinement (max_iters=%d, threshold=%f)", 
                                       opt.irls_max_iters, opt.irls_huber_threshold);
                            linear_refine_irls_huber_dlt<Scalar, Solver::N_>(x1_inl, x2_inl, &refined_model, 
                                                                             opt.irls_max_iters, opt.irls_huber_threshold);
                        } else {
                            linear_refine_dlt<Scalar, Solver::N_>(x1_inl, x2_inl, &refined_model);
                        }
                    }
                }
                // Nonlinear refinement
                if (opt.lo_type == LocalRefinementType::BundleAdjust || opt.lo_type == LocalRefinementType::Both) {
                    // Use estimator's refine_model (bundle adjustment)
                    ENTO_DEBUG("[RANSAC] Refining model with bundle adjustment");
                    estimator.refine_model(&refined_model);
                }

                if (opt.lo_type != LocalRefinementType::None) {
                    stats.refinements++;
                }
            }
        }
        else {
            ENTO_DEBUG("[RANSAC] Number of inliers is less than sample size. No local optimization performed.");
        }
    }
    Scalar refined_msac_score = estimator.score_model(refined_model, &inlier_count);
    if (refined_msac_score < stats.model_score) {
        stats.model_score = refined_msac_score;
        stats.num_inliers = inlier_count;
        *best_model = refined_model;
    }
    stats.inlier_ratio = static_cast<Scalar>(stats.num_inliers) / static_cast<Scalar>(estimator.num_data_);
    if (stats.inlier_ratio >= Scalar(0.9999)) {
        state.dynamic_max_iter = opt.min_iters;
    } else if (stats.inlier_ratio <= Scalar(0.0001)) {
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
    stats.num_inliers = 0;
    stats.model_score = std::numeric_limits<Scalar>::max();
    RansacState<Scalar> state;
    state.dynamic_max_iter = opt.max_iters;
    state.log_prob_missing_model = std::log(Scalar(1.0) - opt.success_prob);
    if (opt.score_initial_model) {
        EntoUtil::EntoContainer<Model, Solver::MaxSolns> initial_models;
        initial_models.push_back(*best_model);
        score_models(estimator, initial_models, opt, state, stats, best_model);
    }
    EntoUtil::EntoContainer<Model, Solver::MaxSolns> models;
    for (stats.iters = 0; stats.iters < opt.max_iters; stats.iters++) {
        if (stats.iters > opt.min_iters && stats.iters > state.dynamic_max_iter) {
            break;
        }
        models.clear();
        estimator.generate_models(&models);
        score_models(estimator, models, opt, state, stats, best_model);
    }
    return stats;
}

} // namespace EntoPose

#endif // RANSAC_H
