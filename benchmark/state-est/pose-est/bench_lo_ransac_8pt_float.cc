#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-pose/data_gen.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/rel-pose/eight_pt.h>
#include <ento-pose/robust-est/robust_pose_solver.h>
#include <ento-pose/problem-types/robust_pose_problem.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoPose;

int main()
{
    using Scalar = float;
    using MinimalSolver = EntoPose::SolverRel8pt<Scalar>;
    constexpr size_t N = 64;
    using RobustSolver = RobustRelativePoseSolver<MinimalSolver, N>;
    using Problem = RobustRelativePoseProblem<Scalar, RobustSolver, N>;
    
#if defined(SEMIHOSTING)
    initialise_monitor_handles();
#endif

    // Configure clock
    sys_clk_cfg();
    SysTick_Setup();
    __enable_irq();

    // Generic cache setup via config macro
    ENTO_BENCH_SETUP();

    // Print benchmark configuration
    ENTO_BENCH_PRINT_CONFIG();

    const char* base_path = DATASET_PATH;
    const char* rel_path = "rel-pose/gold_standard_rel_noise_1.0000_outliers_0.100.csv";
    
    char dataset_path[512];
    char output_path[256];
    
    if (!EntoUtil::build_file_path(base_path, rel_path,
                                   dataset_path, sizeof(dataset_path)))
    {
        ENTO_DEBUG("ERROR! Could not build file path for bench_lo_ransac_8pt_float!");
    }
    
    // Configure RANSAC options for LO-RANSAC
    RansacOptions<Scalar> ransac_opt;
    ransac_opt.max_iters = 10000;  // Increased for challenging outlier scenarios
    ransac_opt.max_reproj_error = 2.5;  // 1.0 noise * 2.5 = 2.5 pixels (adaptive threshold)
    ransac_opt.success_prob = 0.99;
    ransac_opt.final_refinement = false;
    
    // Enable Nonlinear Local Optimization (LO-RANSAC with Bundle Adjustment)
    ransac_opt.lo_type = LocalRefinementType::BundleAdjust;
    ransac_opt.use_irls = false;  // Use robust loss function in bundle adjustment

    // Camera configuration (identity for normalized coordinates)
    using CameraModel = IdentityCameraModel<Scalar>;
    using Params = std::array<Scalar, 0>;
    Params params;
    Camera<Scalar, CameraModel> camera(1.0, 1.0, params);

    // Bundle adjustment options (used for nonlinear refinement)
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.loss_scale = 0.5 * ransac_opt.max_reproj_error;
    bundle_opt.max_iterations = 100;
    bundle_opt.verbose = false;

    RobustSolver robust_solver(ransac_opt, bundle_opt, camera, camera); // Two cameras for relative pose
    Problem problem(robust_solver);

    printf("File path: %s\n", dataset_path);

    ENTO_BENCH_HARNESS_TYPE(Problem);
    BenchHarness harness(problem, "8pt LO-RANSAC Nonlinear [float] - 1.0px noise, 10% outliers", dataset_path, output_path);
    
    harness.run();
    
    // Print RANSAC statistics for analysis
    printf("\n=== RANSAC Statistics ===\n");
    printf("Iterations: %zu\n", problem.ransac_stats_.iters);
    printf("Refinements: %zu\n", problem.ransac_stats_.refinements);
    printf("Final inliers: %zu\n", problem.ransac_stats_.num_inliers);
    printf("Inlier ratio: %.3f\n", problem.ransac_stats_.inlier_ratio);
    printf("Model score: %.6f\n", problem.ransac_stats_.model_score);
    printf("========================\n");
    
    exit(1);
    return 0;
} 
