#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <limits>
#include <cmath>

#include <ento-util/containers.h>
#include <ento-util/debug.h>
#include <ento-math/core.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/synthetic_relpose.h>
#include <ento-pose/rel-pose/upright_planar_three_pt.h>
#include <ento-pose/camera_models.h>

using namespace EntoPose;
using namespace EntoUtil;
using namespace EntoMath;

// ============================================================================
// COPIED ROBUST ESTIMATION CODE (NO MIDDLEMEN)
// ============================================================================

// RANSAC Stats and Options (copied from ransac_util.h)
template <typename Scalar>
struct RansacStats
{
  size_t refinements = 0;
  size_t iters = 0;
  size_t num_inliers = 0;
  Scalar inlier_ratio = 0;
  Scalar model_score = std::numeric_limits<Scalar>::max();
};

enum class LocalRefinementType {
    None,         
    Linear,       
    BundleAdjust, 
    Both          
};

enum class LinearRefinementMethod {
    EightPoint,        
    UprightPlanar3pt,  
    DLT                
};

template <typename Scalar>
struct RansacOptions
{
  size_t max_iters = 10000;
  size_t min_iters = 0;
  Scalar dynamic_trials_mult = 3.0;
  Scalar success_prob = 0.9999;
  Scalar max_reproj_error = 12.0;
  Scalar max_epipolar_error = 1.0;
  unsigned long seed = 0;
  bool score_initial_model = false;
  LocalRefinementType lo_type = LocalRefinementType::None;
  LinearRefinementMethod linear_method = LinearRefinementMethod::EightPoint;
  bool use_irls = false;                    
  int irls_max_iters = 5;                   
  Scalar irls_huber_threshold = Scalar(1.0); 
  bool final_refinement = true;
};

// Bundle Adjustment Options (simplified)
template <typename Scalar>
struct BundleOptionsStandalone
{
  enum class LossType { TRUNCATED };
  LossType loss_type = LossType::TRUNCATED;
  Scalar loss_scale = Scalar(1.0);
  int max_iterations = 20;
};

// Sampson error computation (copied from ransac_util.h)
template <typename Scalar, size_t N = 0>
Scalar compute_sampson_msac_score(const CameraPose<Scalar> &pose,
                                  const EntoContainer<Vec2<Scalar>, N> &x1,
                                  const EntoContainer<Vec2<Scalar>, N> &x2,
                                  Scalar sq_threshold,
                                  size_t *inlier_count)
{
  *inlier_count = 0;
  Matrix3x3<Scalar> E;
  essential_from_motion(pose, &E);

  const Scalar E0_0 = E(0, 0), E0_1 = E(0, 1), E0_2 = E(0, 2);
  const Scalar E1_0 = E(1, 0), E1_1 = E(1, 1), E1_2 = E(1, 2);
  const Scalar E2_0 = E(2, 0), E2_1 = E(2, 1), E2_2 = E(2, 2);

  Scalar score = 0.0;
  for (size_t k = 0; k < x1.size(); ++k) {
    const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
    const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

    const Scalar Ex1_0 = E0_0 * x1_0 + E0_1 * x1_1 + E0_2;
    const Scalar Ex1_1 = E1_0 * x1_0 + E1_1 * x1_1 + E1_2;
    const Scalar Ex1_2 = E2_0 * x1_0 + E2_1 * x1_1 + E2_2;

    const Scalar Ex2_0 = E0_0 * x2_0 + E1_0 * x2_1 + E2_0;
    const Scalar Ex2_1 = E0_1 * x2_0 + E1_1 * x2_1 + E2_1;

    const Scalar C = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;
    const Scalar Cx = Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1;
    const Scalar Cy = Ex2_0 * Ex2_0 + Ex2_1 * Ex2_1;
    const Scalar r2 = C * C / (Cx + Cy);

    std::cout << "    [SCORE] Point " << k << ": Sampson_error=" << r2 << " vs threshold=" << sq_threshold;

    if (r2 < sq_threshold) {
        bool cheirality = check_cheirality(pose, x1[k].homogeneous().normalized(), x2[k].homogeneous().normalized(), Scalar(0.01));
        if (cheirality) {
            (*inlier_count)++;
            score += r2;
            std::cout << " -> INLIER (cheirality OK)" << std::endl;
        } else {
            score += sq_threshold;
            std::cout << " -> OUTLIER (cheirality FAIL)" << std::endl;
        }
    } else {
        score += sq_threshold;
        std::cout << " -> OUTLIER (high error)" << std::endl;
    }
  }
  return score;
}

// Get inliers function (simplified version)
template <typename Scalar, size_t N = 0>
int get_inliers(const CameraPose<Scalar> &pose,
                const EntoContainer<Vec2<Scalar>, N> &x1,
                const EntoContainer<Vec2<Scalar>, N> &x2,
                Scalar sq_threshold,
                EntoContainer<uint8_t, N> *inliers)
{
  inliers->clear();
  if constexpr (N == 0) {
    inliers->resize(x1.size());
  }
  
  int count = 0;
  Matrix3x3<Scalar> E;
  essential_from_motion(pose, &E);

  const Scalar E0_0 = E(0, 0), E0_1 = E(0, 1), E0_2 = E(0, 2);
  const Scalar E1_0 = E(1, 0), E1_1 = E(1, 1), E1_2 = E(1, 2);
  const Scalar E2_0 = E(2, 0), E2_1 = E(2, 1), E2_2 = E(2, 2);

  for (size_t k = 0; k < x1.size(); ++k) {
    const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
    const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

    const Scalar Ex1_0 = E0_0 * x1_0 + E0_1 * x1_1 + E0_2;
    const Scalar Ex1_1 = E1_0 * x1_0 + E1_1 * x1_1 + E1_2;
    const Scalar Ex1_2 = E2_0 * x1_0 + E2_1 * x1_1 + E2_2;

    const Scalar Ex2_0 = E0_0 * x2_0 + E1_0 * x2_1 + E2_0;
    const Scalar Ex2_1 = E0_1 * x2_0 + E1_1 * x2_1 + E2_1;

    const Scalar C = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;
    const Scalar Cx = Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1;
    const Scalar Cy = Ex2_0 * Ex2_0 + Ex2_1 * Ex2_1;
    const Scalar r2 = C * C / (Cx + Cy);

    bool is_inlier = false;
    if (r2 < sq_threshold) {
        bool cheirality = check_cheirality(pose, x1[k].homogeneous().normalized(), x2[k].homogeneous().normalized(), Scalar(0.01));
        if (cheirality) {
            is_inlier = true;
            count++;
        }
    }
    
    if constexpr (N == 0) {
      (*inliers)[k] = is_inlier ? 1 : 0;
    } else {
      inliers->push_back(is_inlier ? 1 : 0);
    }
  }
  return count;
}

// Random Sampler (simplified)
template <typename Scalar, size_t sample_size>
class RandomSampler
{
public:
    RandomSampler(size_t num_data, unsigned long seed) 
        : num_data_(num_data), rng_(seed) {}
    
    void generate_sample(EntoContainer<size_t, sample_size> *sample) {
        sample->clear();
        std::uniform_int_distribution<size_t> dist(0, num_data_ - 1);
        
        // Simple sampling without replacement using EntoContainer
        EntoContainer<bool, 0> used(num_data_, false);  // Dynamic EntoContainer for used flags
        for (size_t i = 0; i < sample_size; ++i) {
            size_t idx;
            do {
                idx = dist(rng_);
            } while (used[idx]);
            used[idx] = true;
            sample->push_back(idx);
        }
    }
    
private:
    size_t num_data_;
    std::default_random_engine rng_;
};

// Upright Planar 3pt Solver Wrapper (to match the estimator interface)
template <typename Scalar>
struct SolverUprightPlanar3pt {
    using scalar_type = Scalar;
    static constexpr size_t MaxSolns = 2;
    static constexpr size_t MinSampleSize = 3;
    
    template <size_t N>
    static void solve(const EntoContainer<Vec3<Scalar>, N> &x1_bear,
                      const EntoContainer<Vec3<Scalar>, N> &x2_bear,
                      EntoContainer<CameraPose<Scalar>, MaxSolns> *solutions) {
        solutions->clear();
        
        // Call the actual solver
        int num_solutions = relpose_upright_planar_3pt<Scalar, N>(x1_bear, x2_bear, solutions);
        
        std::cout << "    [SOLVER] Called upright_planar_3pt with " << x1_bear.size() << " points" << std::endl;
        std::cout << "    [SOLVER] Solver returned " << num_solutions << " solutions" << std::endl;
        
        // Debug: Print the bearing vectors passed to solver
        for (size_t i = 0; i < x1_bear.size(); ++i) {
            std::cout << "    [SOLVER] Input bearing " << i << ": x1=(" 
                      << x1_bear[i](0) << "," << x1_bear[i](1) << "," << x1_bear[i](2) 
                      << ") x2=(" << x2_bear[i](0) << "," << x2_bear[i](1) << "," << x2_bear[i](2) << ")" << std::endl;
        }
        
        // Debug: Print solutions
        for (size_t i = 0; i < solutions->size(); ++i) {
            auto& sol = (*solutions)[i];
            std::cout << "    [SOLVER] Solution " << i << ": t=(" << sol.t(0) << "," << sol.t(1) << "," << sol.t(2) 
                      << ") q=(" << sol.q(0) << "," << sol.q(1) << "," << sol.q(2) << "," << sol.q(3) << ")" << std::endl;
        }
    }
};

// Relative Pose Robust Estimator (copied from relative.h)
template <typename Solver, size_t N = 0>
class RelativePoseRobustEstimator
{
public:
  using Scalar = typename Solver::scalar_type;
  static constexpr size_t MaxSolns = Solver::MaxSolns;
  static constexpr size_t sample_size_ = Solver::MinSampleSize;
  static constexpr size_t N_ = N;
  using SolverType = Solver;

  RelativePoseRobustEstimator(const RansacOptions<Scalar> &opt,
                              const EntoContainer<Vec2<Scalar>, N_> &points2d_1,
                              const EntoContainer<Vec2<Scalar>, N_> &points2d_2)
    : num_data_(points2d_1.size()),
      opt_(opt), x1(points2d_1), x2(points2d_2),
      sampler(num_data_, opt.seed)
  {
    std::cout << "  [ESTIMATOR] Created with " << num_data_ << " correspondences" << std::endl;
    std::cout << "  [ESTIMATOR] Sample size: " << sample_size_ << std::endl;
    std::cout << "  [ESTIMATOR] Max solutions: " << MaxSolns << std::endl;
  }

  void generate_models(EntoContainer<CameraPose<Scalar>, MaxSolns> *models)
  {
    sampler.generate_sample(&sample);
    
    std::cout << "  [ESTIMATOR] Generated sample: ";
    for (size_t i = 0; i < sample.size(); ++i) {
        std::cout << sample[i] << " ";
    }
    std::cout << std::endl;
    
    // Clear and properly size the sample containers
    x1_sample_.clear();
    x2_sample_.clear();
    
    // Convert sampled normalized coordinates to bearing vectors
    for (size_t k = 0; k < sample_size_; ++k)
    {
      Vec3<Scalar> bearing1 = x1[sample[k]].homogeneous().normalized();
      Vec3<Scalar> bearing2 = x2[sample[k]].homogeneous().normalized();
      
      x1_sample_.push_back(bearing1);
      x2_sample_.push_back(bearing2);
      
      std::cout << "  [ESTIMATOR] Sample " << k << " (idx=" << sample[k] << "): norm_coord=(" 
                << x1[sample[k]](0) << "," << x1[sample[k]](1) << ") -> bearing=(" 
                << bearing1(0) << "," << bearing1(1) << "," << bearing1(2) << ")" << std::endl;
    }
    
    std::cout << "  [ESTIMATOR] About to call solver with " << x1_sample_.size() << " points" << std::endl;
    
    // Call the solver
    Solver::template solve<sample_size_>(x1_sample_, x2_sample_, models);
    
    std::cout << "  [ESTIMATOR] Solver returned " << models->size() << " models" << std::endl;
  }

  Scalar score_model(const CameraPose<Scalar> &pose, size_t *inlier_count) const
  {
    std::cout << "  [ESTIMATOR] Scoring model: t=(" << pose.t(0) << "," << pose.t(1) << "," << pose.t(2) 
              << ") q=(" << pose.q(0) << "," << pose.q(1) << "," << pose.q(2) << "," << pose.q(3) << ")" << std::endl;
    
    Scalar score = compute_sampson_msac_score<Scalar, N_>(pose, x1, x2, opt_.max_epipolar_error * opt_.max_epipolar_error, inlier_count);
    
    std::cout << "  [ESTIMATOR] Score: " << score << ", inliers: " << *inlier_count << "/" << x1.size() << std::endl;
    return score;
  }

  void refine_model(CameraPose<Scalar> * /*pose*/) const
  {
    std::cout << "  [ESTIMATOR] Refine model called (simplified - no actual refinement)" << std::endl;
    // Simplified: no actual refinement for debugging
  }

  size_t num_data_;
  RansacOptions<Scalar> opt_;
  const EntoContainer<Vec2<Scalar>, N_> &x1;
  const EntoContainer<Vec2<Scalar>, N_> &x2;

  RandomSampler<Scalar, sample_size_> sampler;
  EntoContainer<Vec3<Scalar>, sample_size_> x1_sample_, x2_sample_;
  EntoContainer<size_t, sample_size_> sample;
};

// RANSAC State
template <typename Scalar>
struct RansacState {
    size_t best_minimal_inlier_count = 0;
    Scalar best_minimal_msac_score = std::numeric_limits<Scalar>::max();
    size_t dynamic_max_iter = 100000;
    Scalar log_prob_missing_model = std::log(Scalar(1.0) - Scalar(0.9999));
};

// Core RANSAC function (simplified version from ransac.h)
template <typename Scalar, typename Solver>
RansacStats<Scalar> ransac_standalone(Solver &estimator, const RansacOptions<Scalar> &opt, CameraPose<Scalar> *best_model) 
{
    std::cout << "[RANSAC] Starting RANSAC with max_iters=" << opt.max_iters << std::endl;
    
    RansacStats<Scalar> stats;
    if (estimator.num_data_ < estimator.sample_size_) {
        std::cout << "[RANSAC] Not enough data points: " << estimator.num_data_ << " < " << estimator.sample_size_ << std::endl;
        return stats;
    }
    
    stats.num_inliers = 0;
    stats.model_score = std::numeric_limits<Scalar>::max();
    RansacState<Scalar> state;
    state.dynamic_max_iter = opt.max_iters;
    state.log_prob_missing_model = std::log(Scalar(1.0) - opt.success_prob);
    
    // Initialize best model to identity
    best_model->q << 1.0, 0.0, 0.0, 0.0;
    best_model->t.setZero();
    
    EntoContainer<CameraPose<Scalar>, Solver::MaxSolns> models;
    
    for (stats.iters = 0; stats.iters < opt.max_iters; stats.iters++) {
        if (stats.iters > opt.min_iters && stats.iters > state.dynamic_max_iter) {
            std::cout << "[RANSAC] Early termination at iteration " << stats.iters << std::endl;
            break;
        }
        
        if (stats.iters % 100 == 0) {
            std::cout << "[RANSAC] Iteration " << stats.iters << ": best_inliers=" << stats.num_inliers 
                      << ", best_score=" << stats.model_score << std::endl;
        }
        
        models.clear();
        estimator.generate_models(&models);
        
        // Score each model
        for (size_t i = 0; i < models.size(); ++i) {
            size_t inlier_count = 0;
            Scalar score_msac = estimator.score_model(models[i], &inlier_count);
            
            bool more_inliers = inlier_count > state.best_minimal_inlier_count;
            bool better_score = score_msac < state.best_minimal_msac_score;
            
            if (more_inliers || better_score) {
                if (more_inliers) {
                    state.best_minimal_inlier_count = inlier_count;
                    std::cout << "[RANSAC] New best inlier count: " << inlier_count << std::endl;
                }
                if (better_score) {
                    state.best_minimal_msac_score = score_msac;
                }
                
                if (score_msac < stats.model_score) {
                    stats.model_score = score_msac;
                    *best_model = models[i];
                    stats.num_inliers = inlier_count;
                    std::cout << "[RANSAC] New best model at iteration " << stats.iters 
                              << ": score=" << score_msac << ", inliers=" << inlier_count << std::endl;
                }
            }
        }
        
        // Update dynamic iteration count
        stats.inlier_ratio = static_cast<Scalar>(stats.num_inliers) / static_cast<Scalar>(estimator.num_data_);
        if (stats.inlier_ratio >= Scalar(0.9999)) {
            state.dynamic_max_iter = opt.min_iters;
        } else if (stats.inlier_ratio <= Scalar(0.0001)) {
            state.dynamic_max_iter = opt.max_iters;
        } else {
            const Scalar prob_outlier = Scalar(1.0) - std::pow(stats.inlier_ratio, static_cast<Scalar>(estimator.sample_size_));
            if (prob_outlier > 0) {
                state.dynamic_max_iter = static_cast<size_t>(
                    std::ceil(state.log_prob_missing_model / std::log(prob_outlier) * opt.dynamic_trials_mult)
                );
            }
        }
    }
    
    std::cout << "[RANSAC] Finished after " << stats.iters << " iterations" << std::endl;
    std::cout << "[RANSAC] Best model: score=" << stats.model_score << ", inliers=" << stats.num_inliers << std::endl;
    
    return stats;
}

// Our standalone robust estimation function (replaces estimate_relative_pose)
template <typename Scalar, size_t N = 0>
RansacStats<Scalar> estimate_relative_pose_standalone(
    const EntoContainer<Vec2<Scalar>, N> &points2D_1,
    const EntoContainer<Vec2<Scalar>, N> &points2D_2,
    const Camera<Scalar, PinholeCameraModel<Scalar>> &camera1,
    const Camera<Scalar, PinholeCameraModel<Scalar>> &camera2,
    const RansacOptions<Scalar> &ransac_opt,
    const BundleOptionsStandalone<Scalar> & /*bundle_opt*/,
    CameraPose<Scalar> *relative_pose,
    EntoContainer<uint8_t, N> *inliers)
{
  using Solver = SolverUprightPlanar3pt<Scalar>;
  
  std::cout << "\n=== STANDALONE ROBUST ESTIMATION ===" << std::endl;
  std::cout << "Input: " << points2D_1.size() << " correspondences" << std::endl;
  
  const size_t num_pts = points2D_1.size();

  // Step 1: Normalize image points for both cameras
  EntoContainer<Vec2<Scalar>, N> x1_calib, x2_calib;
  if constexpr (N == 0) {
    x1_calib.resize(num_pts);
    x2_calib.resize(num_pts);
  }
  
  std::cout << "\nStep 1: Camera unprojection (image -> normalized coordinates)" << std::endl;
  for (size_t k = 0; k < num_pts; ++k) {
    Vec2<Scalar> norm1, norm2;
    camera1.unproject(points2D_1[k], &norm1);
    camera2.unproject(points2D_2[k], &norm2);
    
    std::cout << "  Point " << k << ": img=(" << points2D_1[k](0) << "," << points2D_1[k](1) 
              << ") -> norm=(" << norm1(0) << "," << norm1(1) << ")" << std::endl;
    
    if constexpr (N == 0) {
      x1_calib[k] = norm1;
      x2_calib[k] = norm2;
    } else {
      x1_calib.push_back(norm1);
      x2_calib.push_back(norm2);
    }
  }

  // Step 2: Scale threshold for normalized coordinates
  RansacOptions<Scalar> ransac_opt_scaled = ransac_opt;
  ransac_opt_scaled.max_epipolar_error = ransac_opt.max_epipolar_error / camera1.focal();

  std::cout << "\nStep 2: Threshold scaling" << std::endl;
  std::cout << "  Original threshold: " << ransac_opt.max_epipolar_error << " pixels" << std::endl;
  std::cout << "  Scaled threshold: " << ransac_opt_scaled.max_epipolar_error << " (normalized coords)" << std::endl;
  std::cout << "  Squared threshold: " << (ransac_opt_scaled.max_epipolar_error * ransac_opt_scaled.max_epipolar_error) << " (for Sampson error)" << std::endl;

  // Step 3: Run RANSAC
  std::cout << "\nStep 3: RANSAC" << std::endl;
  using Estimator = RelativePoseRobustEstimator<Solver, N>;
  Estimator estimator(ransac_opt_scaled, x1_calib, x2_calib);
  RansacStats<Scalar> stats = ransac_standalone<Scalar, Estimator>(estimator, ransac_opt_scaled, relative_pose);

  // Step 4: Get final inliers
  std::cout << "\nStep 4: Final inlier computation" << std::endl;
  get_inliers<Scalar, N>(*relative_pose, x1_calib, x2_calib, 
                         ransac_opt_scaled.max_epipolar_error * ransac_opt_scaled.max_epipolar_error, inliers);
  
  // Count final inliers
  size_t final_inlier_count = 0;
  for (size_t i = 0; i < inliers->size(); ++i) {
    if ((*inliers)[i]) final_inlier_count++;
  }
  
  std::cout << "Final inlier count: " << final_inlier_count << "/" << inliers->size() << std::endl;
  stats.num_inliers = final_inlier_count;

  return stats;
}

// ============================================================================
// DATA GENERATION METHODS
// ============================================================================

template<typename Scalar>
void generate_data_enhanced_method(
    EntoContainer<Vec3<Scalar>, 3>& x1_bear,
    EntoContainer<Vec3<Scalar>, 3>& x2_bear,
    CameraPose<Scalar>& true_pose,
    int problem_id = 0)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> uniform_gen(-1.0, 1.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, 0.1);
    Scalar focal_length = Scalar(500.0);
    
    // Generate upright planar motion
    std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
    Scalar rot_selector = rot_scenario(rng);
    
    Scalar yaw_deg;
    if (rot_selector < 0.3) {
        std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
        yaw_deg = small_rot(rng);
    } else if (rot_selector < 0.7) {
        std::uniform_real_distribution<Scalar> medium_rot(-10.0, 10.0);
        yaw_deg = medium_rot(rng);
    } else {
        std::uniform_real_distribution<Scalar> large_rot(-45.0, 45.0);
        yaw_deg = large_rot(rng);
    }
    
    Scalar yaw_rad = yaw_deg * M_PI / 180.0;
    Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
    true_pose.q(0) = q.w();
    true_pose.q(1) = q.x();
    true_pose.q(2) = q.y();
    true_pose.q(3) = q.z();
    
    // Realistic baseline variation for planar motion
    std::uniform_real_distribution<Scalar> baseline_gen(0.05, 5.0);
    Scalar baseline_length = baseline_gen(rng);
    
    // Random direction in XZ plane
    Scalar tx = uniform_gen(rng);
    Scalar tz = uniform_gen(rng);
    Vec3 t_dir = Vec3(tx, 0, tz).normalized();
    true_pose.t = baseline_length * t_dir;
    
    x1_bear.clear();
    x2_bear.clear();
    
    // Generate 3 points
    for (int i = 0; i < 3; ++i) {
        // Generate 3D point
        std::uniform_real_distribution<Scalar> mid_lateral(-2.0, 2.0);
        std::uniform_real_distribution<Scalar> mid_depth(1.0, 10.0);
        Vec3 X = Vec3(mid_lateral(rng), mid_lateral(rng), mid_depth(rng));
        
        // Project to first camera (identity pose)
        Vec3 x1_cam = X;
        if (x1_cam(2) <= Scalar(0.1)) continue;
        
        // Project to second camera
        Vec3 x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        
        // Project to image coordinates
        Vec2 x1_img = Vec2(
            focal_length * x1_cam(0) / x1_cam(2),
            focal_length * x1_cam(1) / x1_cam(2)
        );
        Vec2 x2_img = Vec2(
            focal_length * x2_cam(0) / x2_cam(2),
            focal_length * x2_cam(1) / x2_cam(2)
        );
        
        // Add pixel noise
        x1_img(0) += pixel_noise_gen(rng);
        x1_img(1) += pixel_noise_gen(rng);
        x2_img(0) += pixel_noise_gen(rng);
        x2_img(1) += pixel_noise_gen(rng);
        
        // Convert to bearing vectors
        Vec3 f1 = Vec3(x1_img(0) / focal_length, x1_img(1) / focal_length, Scalar(1.0)).normalized();
        Vec3 f2 = Vec3(x2_img(0) / focal_length, x2_img(1) / focal_length, Scalar(1.0)).normalized();
        
        x1_bear.push_back(f1);
        x2_bear.push_back(f2);
    }
}

// Flexible data generation for any number of points
template<typename Scalar, size_t N>
void generate_data_flexible(
    EntoContainer<Vec2<Scalar>, N>& x1_img,
    EntoContainer<Vec2<Scalar>, N>& x2_img,
    CameraPose<Scalar>& true_pose,
    int num_inliers,
    int num_outliers,
    int problem_id = 0)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> uniform_gen(-1.0, 1.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, 0.5);  // Realistic pixel noise
    Scalar focal_length = Scalar(500.0);
    
    // Generate upright planar motion (same as before)
    std::uniform_real_distribution<Scalar> rot_scenario(0.0, 1.0);
    Scalar rot_selector = rot_scenario(rng);
    
    Scalar yaw_deg;
    if (rot_selector < 0.3) {
        std::uniform_real_distribution<Scalar> small_rot(-2.0, 2.0);
        yaw_deg = small_rot(rng);
    } else if (rot_selector < 0.7) {
        std::uniform_real_distribution<Scalar> medium_rot(-10.0, 10.0);
        yaw_deg = medium_rot(rng);
    } else {
        std::uniform_real_distribution<Scalar> large_rot(-45.0, 45.0);
        yaw_deg = large_rot(rng);
    }
    
    Scalar yaw_rad = yaw_deg * M_PI / 180.0;
    Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
    true_pose.q(0) = q.w();
    true_pose.q(1) = q.x();
    true_pose.q(2) = q.y();
    true_pose.q(3) = q.z();
    
    // Realistic baseline variation for planar motion
    std::uniform_real_distribution<Scalar> baseline_gen(0.1, 3.0);
    Scalar baseline_length = baseline_gen(rng);
    
    // Random direction in XZ plane
    Scalar tx = uniform_gen(rng);
    Scalar tz = uniform_gen(rng);
    Vec3 t_dir = Vec3(tx, 0, tz).normalized();
    true_pose.t = baseline_length * t_dir;
    
    x1_img.clear();
    x2_img.clear();
    
    std::cout << "Generating " << num_inliers << " inliers + " << num_outliers << " outliers = " << (num_inliers + num_outliers) << " total points" << std::endl;
    std::cout << "True pose: yaw=" << yaw_deg << "°, baseline=" << baseline_length << "m" << std::endl;
    
    // Generate inlier points
    int generated_inliers = 0;
    int attempts = 0;
    while (generated_inliers < num_inliers && attempts < num_inliers * 3) {
        attempts++;
        
        // Generate 3D point with good distribution
        std::uniform_real_distribution<Scalar> lateral_gen(-3.0, 3.0);
        std::uniform_real_distribution<Scalar> depth_gen(2.0, 15.0);
        Vec3 X = Vec3(lateral_gen(rng), lateral_gen(rng), depth_gen(rng));
        
        // Project to first camera (identity pose)
        Vec3 x1_cam = X;
        if (x1_cam(2) <= Scalar(0.1)) continue;
        
        // Project to second camera
        Vec3 x2_cam = true_pose.R() * X + true_pose.t;
        if (x2_cam(2) <= Scalar(0.1)) continue;
        
        // Project to image coordinates
        Vec2 img1 = Vec2(
            focal_length * x1_cam(0) / x1_cam(2),
            focal_length * x1_cam(1) / x1_cam(2)
        );
        Vec2 img2 = Vec2(
            focal_length * x2_cam(0) / x2_cam(2),
            focal_length * x2_cam(1) / x2_cam(2)
        );
        
        // Check if points are within reasonable image bounds
        if (std::abs(img1(0)) > 400 || std::abs(img1(1)) > 400 ||
            std::abs(img2(0)) > 400 || std::abs(img2(1)) > 400) {
            continue;
        }
        
        // Add pixel noise
        img1(0) += pixel_noise_gen(rng);
        img1(1) += pixel_noise_gen(rng);
        img2(0) += pixel_noise_gen(rng);
        img2(1) += pixel_noise_gen(rng);
        
        x1_img.push_back(img1);
        x2_img.push_back(img2);
        generated_inliers++;
        
        if (generated_inliers <= 5) {  // Only print first few for brevity
            std::cout << "  Inlier " << generated_inliers << ": (" << img1(0) << "," << img1(1) 
                      << ") -> (" << img2(0) << "," << img2(1) << ")" << std::endl;
        }
    }
    
    if (generated_inliers < num_inliers) {
        std::cout << "WARNING: Only generated " << generated_inliers << "/" << num_inliers << " inliers" << std::endl;
    }
    
    // Generate outlier points
    std::uniform_real_distribution<Scalar> outlier_gen(-200.0, 200.0);
    for (int k = 0; k < num_outliers; ++k) {
        Vec2 outlier1(outlier_gen(rng), outlier_gen(rng));
        Vec2 outlier2(outlier_gen(rng), outlier_gen(rng));
        x1_img.push_back(outlier1);
        x2_img.push_back(outlier2);
        
        if (k < 3) {  // Only print first few for brevity
            std::cout << "  Outlier " << (k+1) << ": (" << outlier1(0) << "," << outlier1(1) 
                      << ") -> (" << outlier2(0) << "," << outlier2(1) << ")" << std::endl;
        }
    }
    
    std::cout << "Final dataset: " << x1_img.size() << " correspondences" << std::endl;
}

// Compute pose errors
template<typename Scalar>
struct PoseErrors {
    Scalar rotation_error_deg;
    Scalar translation_error_deg;
    Scalar max_error() const { return std::max(rotation_error_deg, translation_error_deg); }
};

template<typename Scalar>
PoseErrors<Scalar> compute_pose_errors(const CameraPose<Scalar>& true_pose, const CameraPose<Scalar>& estimated_pose)
{
    PoseErrors<Scalar> errors;
    
    // Rotation error (angular difference)
    auto R_true = true_pose.R();
    auto R_est = estimated_pose.R();
    auto R_error = R_est.transpose() * R_true;
    
    Scalar trace_val = R_error.trace();
    Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
    errors.rotation_error_deg = angle_rad * 180.0 / M_PI;
    
    // Translation error (angle between direction vectors)
    auto t_true = true_pose.t.normalized();
    auto t_est = estimated_pose.t.normalized();
    Scalar dot = std::clamp(t_true.dot(t_est), Scalar(-1), Scalar(1));
    errors.translation_error_deg = std::acos(std::abs(dot)) * 180.0 / M_PI;
    
    return errors;
}

// ============================================================================
// TEST FUNCTIONS
// ============================================================================

template<typename Scalar>
void test_direct_solver(int num_tests = 3)
{
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TESTING DIRECT SOLVER CALLS" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    int successful_solves = 0;
    
    for (int i = 0; i < num_tests; ++i) {
        std::cout << "\n--- Problem " << i << " ---" << std::endl;
        
        EntoContainer<Vec3<Scalar>, 3> x1_bear, x2_bear;
        CameraPose<Scalar> true_pose;
        
        generate_data_enhanced_method<Scalar>(x1_bear, x2_bear, true_pose, i);
        
        if (x1_bear.size() < 3) {
            std::cout << "Failed to generate 3 points" << std::endl;
            continue;
        }
        
        // Test the solver directly
        EntoContainer<CameraPose<Scalar>, 2> solutions;
        int num_solutions = relpose_upright_planar_3pt<Scalar, 3>(x1_bear, x2_bear, &solutions);
        
        std::cout << "Direct solver returned " << num_solutions << " solutions" << std::endl;
        
        if (num_solutions > 0) {
            // Find best solution
            Scalar best_error = std::numeric_limits<Scalar>::max();
            PoseErrors<Scalar> best_errors;
            
            for (int s = 0; s < num_solutions; ++s) {
                auto errors = compute_pose_errors(true_pose, solutions[s]);
                if (errors.max_error() < best_error) {
                    best_error = errors.max_error();
                    best_errors = errors;
                }
            }
            
            if (best_error < 15.0) {
                successful_solves++;
                std::cout << "✓ SUCCESS: R_err=" << best_errors.rotation_error_deg 
                          << "°, T_err=" << best_errors.translation_error_deg << "°" << std::endl;
            } else {
                std::cout << "✗ HIGH ERROR: R_err=" << best_errors.rotation_error_deg 
                          << "°, T_err=" << best_errors.translation_error_deg << "°" << std::endl;
            }
        } else {
            std::cout << "✗ NO SOLUTIONS" << std::endl;
        }
    }
    
    std::cout << "\nDirect Solver Results: " << successful_solves << "/" << num_tests 
              << " (" << (100.0 * successful_solves / num_tests) << "%)" << std::endl;
}

template<typename Scalar>
void test_standalone_robust_estimation(int num_tests = 3)
{
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TESTING STANDALONE ROBUST ESTIMATION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    int successful_solves = 0;
    
    // Create camera
    using CameraModel = PinholeCameraModel<Scalar>;
    std::array<Scalar, 3> params = {Scalar(500.0), Scalar(500.0), Scalar(0.0)};
    Camera<Scalar, CameraModel> camera(Scalar(500.0), Scalar(500.0), params);
    
    // Test different dataset configurations
    struct TestConfig {
        int num_inliers;
        int num_outliers;
        std::string description;
    };
    
    std::vector<TestConfig> configs = {
        {48, 14, "Medium dataset (20 inliers, 30 outliers)"}
    };
    
    for (size_t config_idx = 0; config_idx < configs.size() && config_idx < static_cast<size_t>(num_tests); ++config_idx) {
        auto& config = configs[config_idx];
        
        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "TEST " << config_idx << ": " << config.description << std::endl;
        std::cout << std::string(80, '=') << std::endl;
        
        CameraPose<Scalar> true_pose;
        EntoContainer<Vec2<Scalar>, 0> x1_img, x2_img;  // Dynamic size
        
        // Generate realistic dataset
        generate_data_flexible<Scalar, 0>(x1_img, x2_img, true_pose, 
                                         config.num_inliers, config.num_outliers, config_idx);
        
        if (x1_img.size() < 3) {
            std::cout << "Failed to generate minimum 3 points" << std::endl;
            continue;
        }
        
        // Set up options
        RansacOptions<Scalar> ransac_opt;
        ransac_opt.max_iters = 2000;  // More iterations for larger datasets
        ransac_opt.max_epipolar_error = Scalar(4.0);  // More tolerant threshold for realistic noise
        ransac_opt.success_prob = Scalar(0.99);
        ransac_opt.seed = 42 + config_idx;
        
        BundleOptionsStandalone<Scalar> bundle_opt;
        
        CameraPose<Scalar> estimated_pose;
        EntoContainer<uint8_t, 0> inlier_mask;  // Dynamic size
        
        // Call our standalone robust estimation
        RansacStats<Scalar> stats = estimate_relative_pose_standalone<Scalar, 0>(
            x1_img, x2_img, camera, camera,
            ransac_opt, bundle_opt,
            &estimated_pose, &inlier_mask
        );
        
        std::cout << "\n" << std::string(40, '-') << " FINAL RESULT " << std::string(40, '-') << std::endl;
        
        // More lenient success criteria for realistic datasets
        bool success = (stats.num_inliers >= static_cast<size_t>(std::max(3, config.num_inliers / 2)));
        
        if (success) {
            auto errors = compute_pose_errors(true_pose, estimated_pose);
            
            if (errors.max_error() < 20.0) {  // Slightly more tolerant for realistic noise
                successful_solves++;
                std::cout << "✓ SUCCESS";
            } else {
                std::cout << "✗ HIGH ERROR";
            }
            
            std::cout << " | R_err: " << errors.rotation_error_deg << "°";
            std::cout << " | T_err: " << errors.translation_error_deg << "°";
            std::cout << " | Inliers: " << stats.num_inliers << "/" << inlier_mask.size();
            std::cout << " | Expected: ~" << config.num_inliers;
            std::cout << " | RANSAC iters: " << stats.iters;
            
        } else {
            std::cout << "✗ FAILED - Insufficient inliers: " << stats.num_inliers << "/" << inlier_mask.size();
        }
        std::cout << std::endl;
    }
    
    std::cout << "\nStandalone Robust Estimation Results: " << successful_solves << "/" << std::min(num_tests, (int)configs.size())
              << " (" << (100.0 * successful_solves / std::min(num_tests, (int)configs.size())) << "%)" << std::endl;
}

int main()
{
    using Scalar = float;
    
    std::cout << "COMPREHENSIVE UPRIGHT PLANAR 3PT DEBUG TEST" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    // Test 1: Direct solver calls (should work perfectly)
    test_direct_solver<Scalar>(3);
    
    // Test 2: Standalone robust estimation (debug the pipeline)
    test_standalone_robust_estimation<Scalar>(3);
    
    return 0;
} 