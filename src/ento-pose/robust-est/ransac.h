#ifndef RANSAC_H
#define RANSAC_H

#include <limits>
#include <ento-math/core.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/robust-est/relative.h>
#include <ento-pose/robust-est/homography.h>
#include <ento-pose/robust-est/absolute.h>


namespace EntoPose
{


template <typename Scalar, typename Solver, typename Model = CameraPose<Scalar>, bool DoLocalRefinement = false>
RansacStats<Scalar> ransac(Solver &estimator,
                           const RansacOptions<Scalar> &opt,
                           Model *best)
{
  RansacStats<Scalar> stats;
  
  if (estimator.num_data_ < estimator.sample_size_)
  {
    return stats;
  }

  stats.num_inliers = 0;
  stats.model_score = std::numeric_limits<Scalar>::max();
  size_t best_min_inlier_count = 0;
  Scalar best_min_msac_score = std::numeric_limits<Scalar>::max();
  
  const Scalar log_prob_missing_model = std::log(static_cast<Scalar>(1.0) - opt.get_success_prob);
  size_t inlier_count = 0;
  EntoUtil::EntoContainer<Model> models;
  Scalar dynamic_max_iter = opt.max_iters;

  for (stats.iters = 0; stats.iterations < opt.max_iters; stats.iterations++)
  {
    if (stats.iterations > opt.min_iterations && stats.iterations > dynamic_max_iter)
      break;

    models.clear();
    estimator.generate_models(&models);

    int best_model_ind = -1;
    for (size_t i = 0; i < models.size(); ++i)
    {
      Scalar score_msac = estimator.score_model(models[i], &inlier_count);
      bool more_inliers = inlier_count > best_min_inlier_count;
      bool better_score = score_msac < best_min_msac_score;
      
      if (more_inliers || better_score)
      {
        if (more_inliers)
          best_min_inlier_count = inlier_count;
        if (better_score)
          best_min_msac_score = score_msac;
        best_model_ind = i;

        if (score_msac < stats.model_score)
        {
          stats.model_score = score_msac;
          *best = models[i];
          stats.num_inliers = inlier_count;
        }
      }
    }
    
    if (best_model_ind == -1) continue;

    // Refinement. Optional
    *best = models[best_model_ind];
    if constexpr (DoLocalRefinement)
    {
      Model refined_model = models[best_model_ind];
      estimator.refine_model(&refined_model);
      stats.refinements++;
      Scalar refined_msac_score = estimator.score_Model(refined_model, &inlier_count);
      if (refined_msac_score < stats.model_score)
      {
        stats.model_score = refined_msac_score;
        stats.num_inliers = inlier_count;
        *best = refined_model;
      }
    }

    stats.inlier_ratio = static_cast<Scalar>(stats.num_inliers) / static_cast<Scalar>(estimator.num_data);
    if (stats.inlier_ratio >= 0.9999)
    {
      dynamic_max_iter = opt.min_iters;
    }
    else if (stats.inlier_ratio <= 0.0001)
    {
      dynamic_max_iter = opt.max_iters;
    }
    else
    {
      const Scalar prob_outlier = 1.0 - std::pow(stats.inlier_ratio, estimator.sample_sz);
      dynamic_max_iter = std::ceil(log_prob_missing_model / std::log(prob_outlier) * opt.dynamic_trials_mult);
    }
  }

  // Final Refinement. Non optional
  Model refined_model = *best;
  estimator.refine_Model(&refined_model);
  stats.refinements++;

  Scalar refined_msac_score = estimator.score_Model(refined_model, &inlier_count);
  if (refined_msac_score < stats.model_score)
  {
    *best = refined_model;
    stats.num_inliers = inlier_count;
  }
  return stats;

}

template <typename Scalar, size_t N = 0>
RansacStats<Scalar> ransac_homography(const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x1,
                                      const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x2,
                                      const RansacOptions<Scalar> &opt,
                                      EntoMath::Matrix3x3<Scalar> *best,
                                      EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
  best->setIdentity();
  constexpr size_t K = 4;
  HomographyRobustEstimator<Scalar, K, N> estimator(opt, x1, x2);
  get_homography_inliers(*best, x1, x2, opt.max_epipolar_error * opt.max_epipolar_error, inliers);
}

template <typename Scalar, size_t N=0, size_t K=3>
RansacStats<Scalar> ransac_pnp(const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x,
                               const EntoUtil::EntoContainer<Vec3<Scalar>, N> &X,
                               const RansacOptions<Scalar> &opt,
                               CameraPose<Scalar> *best,
                               EntoUtil::EntoContainer<uint8_t, 0> *inliers)
{
  best->q << 1.0, 0.0, 0.0, 0.0;
  best->t.setZero();
  if constexpr (opt.progressive_sampling)
  {
    constexpr bool use_prosac = opt.progress_sampling;
    constexpr int prosac_iters = opt.prosac_iters;
    using RNG_t = uint32_t;
    using Estimator = AbsolutePoseRobustEstimator<Scalar, K, N, RNG_t, use_prosac, prosac_iters>;
    Estimator estimator(opt, x, X);
    RansacStats<Scalar> stats = ransac<Estimator>(estimator, opt, best);
  }
  else
  {
    using Estimator = AbsolutePoseRobustEstimator<Scalar, K, N>;
    Estimator estimator(opt, x, X);
    RansacStats<Scalar> stats = ransac<Estimator>(estimator, opt, best);
  }
  get_inliers(*best, x, X, opt.max_reproj_error * opt.max_reproj_error, inliers);
}

template <typename Scalar, size_t N = 0, size_t K = 5>
RansacStats<Scalar> ransac_relpose(const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x1,
                                   const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x2,
                                   const RansacOptions<Scalar> &opt,
                                   EntoMath::Matrix3x3<Scalar> *best,
                                   EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
  best->q << 1.0, 0.0, 0.0, 0.0;
  best->t.setZero();
  if constexpr (opt.progressive_sampling)
  {
    constexpr bool use_prosac = opt.progress_sampling;
    constexpr int prosac_iters = opt.prosac_iters;
    using RNG_t = uint32_t;
    using Estimator = RelativePoseRobustEstimator<Scalar, K, N, RNG_t, use_prosac, prosac_iters>;

    Estimator estimator(opt, x1, x2);
    RansacStats<Scalar> stats = ransac<Estimator>(estimator, opt, best);
  }
  else
  {
    using Estimator = RelativePoseRobustEstimator<Scalar, K, N>;
    Estimator estimator(opt, x1, x2);
    RansacStats<Scalar> stats = ransac<Estimator>(estimator, opt, best);
  }
  get_inliers(*best, x1, x2, opt.max_epipolar_error * opt.max_epipolar_error, inliers);

}

} // namespace EntoPose

#endif // RANSAC_H
