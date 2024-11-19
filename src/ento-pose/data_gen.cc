#include <ento-pose/data_gen.h>
#include <cstdint>

#include <chrono>
#include <iostream>
#include <iomanip>

namespace EntoPose
{

template <typename Scalar, typename Solver>
BenchmarkResult<Scalar> benchmark(int n_problems,
                                  const ProblemOptions<Scalar> &options,
                                  Scalar tol = 1e-6) {

    std::vector<AbsolutePoseProblemInstance<Scalar>> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);

    BenchmarkResult<Scalar> result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const AbsolutePoseProblemInstance<Scalar> &instance : problem_instances) {
        std::vector<CameraPose<Scalar>> solutions;

        int sols = Solver::solve(instance, &solutions);

        Scalar pose_error = std::numeric_limits<Scalar>::max();

        result.solutions_ += sols;
        // std::cout << "\nGt: " << instance.pose_gt.R() << "\n"<< instance.pose_gt.t << "\n";
        // std::cout << "gt valid = " << Solver::validator::is_valid(instance, instance.pose_gt, 1.0, tol) << "\n";
        for (const CameraPose<Scalar> &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, 1.0, tol))
                result.valid_solutions_++;
            // std::cout << "Pose: " << pose.R() << "\n" << pose.t << "\n";
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose, 1.0));
        }
        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    std::vector<CameraPose<Scalar>> solutions;
    for (int iter = 0; iter < 10; ++iter) {
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const AbsolutePoseProblemInstance<Scalar> &instance : problem_instances) {
            solutions.clear();
            Solver::solve(instance, &solutions);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());
    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

template <typename Scalar, typename Solver>
BenchmarkResult<Scalar> benchmark_w_extra(int n_problems,
                                          const ProblemOptions<Scalar> &options,
                                          Scalar tol = 1e-6) {

    std::vector<AbsolutePoseProblemInstance<Scalar>> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);

    BenchmarkResult<Scalar> result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const AbsolutePoseProblemInstance<Scalar> &instance : problem_instances) {
        std::vector<CameraPose<Scalar>> solutions;
        std::vector<Scalar> extra;

        int sols = Solver::solve(instance, &solutions, &extra);

        Scalar pose_error = std::numeric_limits<Scalar>::max();

        result.solutions_ += sols;
        for (size_t k = 0; k < solutions.size(); ++k) {
            if (Solver::validator::is_valid(instance, solutions[k], extra[k], tol))
                result.valid_solutions_++;
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, solutions[k], extra[k]));
        }

        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    std::vector<CameraPose<Scalar>> solutions;
    std::vector<Scalar> extra;
    for (int iter = 0; iter < 10; ++iter) {
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const AbsolutePoseProblemInstance<Scalar> &instance : problem_instances) {
            solutions.clear();
            extra.clear();

            Solver::solve(instance, &solutions, &extra);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());
    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

template <typename Scalar, typename Solver>
BenchmarkResult<Scalar> benchmark_relative(int n_problems,
                                           const ProblemOptions<Scalar> &options,
                                           Scalar tol = 1e-6) {

  std::vector<RelativePoseProblemInstance<Scalar>> problem_instances;
  generate_relpose_problems(n_problems, &problem_instances, options);

  BenchmarkResult<Scalar> result;
  result.instances_ = n_problems;
  result.name_ = Solver::name();
  if (options.additional_name_ != "")
  {
    result.name_ += options.additional_name_;
  }
  result.options_ = options;
  std::cout << "Running benchmark: " << result.name_ << std::flush;

  // Run benchmark where we check solution quality
  for (const RelativePoseProblemInstance<Scalar> &instance : problem_instances)
  {
      // CameraPoseVector solutions;
      std::vector<typename Solver::Solution> solutions;

      int sols = Solver::solve(instance, &solutions);

      Scalar pose_error = std::numeric_limits<Scalar>::max();

      result.solutions_ += sols;
      // std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
      for (const typename Solver::Solution &pose : solutions) {
          if (Solver::validator::is_valid(instance, pose, tol))
              result.valid_solutions_++;
          // std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
          pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose));
      }

      if (pose_error < tol)
          result.found_gt_pose_++;
  }

  std::vector<long> runtimes;
  std::vector<typename Solver::Solution> solutions;
  for (int iter = 0; iter < 10; ++iter) {
    auto start_time = std::chrono::high_resolution_clock::now();
    for (const RelativePoseProblemInstance<Scalar> &instance : problem_instances)
    {
      solutions.clear();

      Solver::solve(instance, &solutions);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
  }

  std::sort(runtimes.begin(), runtimes.end());

  result.runtime_ns_ = runtimes[runtimes.size() / 2];
  std::cout << "\r                                                                                \r";
  return result;
}

template <typename Scalar, typename Solver>
BenchmarkResult<Scalar> benchmark_homography(int n_problems,
                                             const ProblemOptions<Scalar> &options,
                                             Scalar tol = 1e-6) {

  std::vector<RelativePoseProblemInstance<Scalar>> problem_instances;
  generate_homography_problems(n_problems, &problem_instances, options);

  BenchmarkResult<Scalar> result;
  result.instances_ = n_problems;
  result.name_ = Solver::name();
  if (options.additional_name_ != "")
  {
    result.name_ += options.additional_name_;
  }
  result.options_ = options;
  std::cout << "Running benchmark: " << result.name_ << std::flush;

  // Run benchmark where we check solution quality
  for (const RelativePoseProblemInstance<Scalar> &instance : problem_instances)
  {
    std::vector<Matrix3x3<Scalar>> solutions;

    int sols = Solver::solve(instance, &solutions);

    Scalar hom_error = std::numeric_limits<Scalar>::max();

    result.solutions_ += sols;
    // std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
    for (const Matrix3x3<Scalar> &H : solutions)
    {
      if (Solver::validator::is_valid(instance, H, tol))
          result.valid_solutions_++;
      // std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
      hom_error = std::min(hom_error, Solver::validator::compute_pose_error(instance, H));
    }

    if (hom_error < tol)
        result.found_gt_pose_++;
  }

  std::vector<long> runtimes;
  std::vector<Matrix3x3<Scalar>> solutions;
  for (int iter = 0; iter < 10; ++iter)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    for (const RelativePoseProblemInstance<Scalar> &instance : problem_instances) 
    {
      solutions.clear();

      Solver::solve(instance, &solutions);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
  }

  std::sort(runtimes.begin(), runtimes.end());

  result.runtime_ns_ = runtimes[runtimes.size() / 2];
  std::cout << "\r                                                                                \r";
  return result;
}

} // namespace EntoPose

template <typename Scalar>
void print_runtime(Scalar runtime_ns) {
    if (runtime_ns < 1e3) {
        std::cout << runtime_ns << " ns";
    } else if (runtime_ns < 1e6) {
        std::cout << runtime_ns / 1e3 << " us";
    } else if (runtime_ns < 1e9) {
        std::cout << runtime_ns / 1e6 << " ms";
    } else {
        std::cout << runtime_ns / 1e9 << " s";
    }
}

template <typename Scalar>
void display_result(const std::vector<EntoPose::BenchmarkResult<Scalar>> &results) {
    // Print PoseLib version and buidling type
    //std::cout << "\n" << poselib_info() << "\n\n";

    int w = 13;
    // display header
    std::cout << std::setw(2 * w) << "Solver";
    std::cout << std::setw(w) << "Solutions";
    std::cout << std::setw(w) << "Valid";
    std::cout << std::setw(w) << "GT found";
    std::cout << std::setw(w) << "Runtime\n";
    for (int i = 0; i < w * 6; ++i)
        std::cout << "-";
    std::cout << "\n";

    int prec = 6;

    for (const EntoPose::BenchmarkResult<Scalar> &result : results) {
        Scalar num_tests = static_cast<Scalar>(result.instances_);
        Scalar solutions = result.solutions_ / num_tests;
        Scalar valid_sols = result.valid_solutions_ / static_cast<Scalar>(result.solutions_) * 100.0;
        Scalar gt_found = result.found_gt_pose_ / num_tests * 100.0;
        Scalar runtime_ns = result.runtime_ns_ / num_tests;

        std::cout << std::setprecision(prec) << std::setw(2 * w) << result.name_;
        std::cout << std::setprecision(prec) << std::setw(w) << solutions;
        std::cout << std::setprecision(prec) << std::setw(w) << valid_sols;
        std::cout << std::setprecision(prec) << std::setw(w) << gt_found;
        std::cout << std::setprecision(prec) << std::setw(w - 3);
        print_runtime(runtime_ns);
        std::cout << "\n";
    }
}

int main() {
    using Scalar = float;
    std::vector<EntoPose::BenchmarkResult<Scalar>> results;

    EntoPose::ProblemOptions<Scalar> options;
    // options.camera_fov_ = 45; // Narrow
    
    //options.camera_fov_ = 75; // Medium
    options.camera_fov_ = 120; // Wide

    Scalar tol = 1e-6;
    constexpr int iters = 1e3;

    // ====================================================
    // Absolute Pose

    // P3P
    EntoPose::ProblemOptions p3p_opt = options;
    p3p_opt.n_point_point_ = 3;
    p3p_opt.n_point_line_ = 0;
    results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverP3P<Scalar>>(iters, p3p_opt, tol));
    printf("results solutions %d \n", results[0].solutions_);
    
    // uP2P
    EntoPose::ProblemOptions up2p_opt = options;
    up2p_opt.n_point_point_ = 2;
    up2p_opt.n_point_line_ = 0;
    up2p_opt.upright_ = true;
    results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverUP2P<Scalar>>(iters, up2p_opt, tol));



    // P6P (DLT)
    // @TODO: Rewrite dlt to conform to pose lib standard.
    //  EntoPose::ProblemOptions p6p_dlt_opt = options;
    //  p6p_dlt_opt.n_point_point_ = 6;
    //  p6p_dlt_opt.n_point_line_ = 0;
    //  results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverP6PDLT<Scalar>>(iters, p6p_dlt_opt, tol));
    
    // Homography 
    // Homograpy 4pt (PoseLib version using QR)
    EntoPose::ProblemOptions homo4pt_opt = options;
    homo4pt_opt.n_point_point_ = 4;
    homo4pt_opt.n_point_line_ = 0;
    results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomography4pt<Scalar, false, 0>>(iters, homo4pt_opt, tol));
    results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomography4pt<Scalar, true, 0>>(iters, homo4pt_opt, tol));

    // Homography N-pt N = 4 (SVD version)
    // @TODO: Rewrite dlt to conform to pose lib standard.
    //  EntoPose::ProblemOptions p4p_dlt_opt = options;
    //  p6p_dlt_opt.n_point_point_ = 6;
    //  p6p_dlt_opt.n_point_line_ = 0;
    //  results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverP6PDLT<Scalar>>(1e6, p6p_dlt_opt, tol));
    
    // Homography N-pt N > 4 (SVD Version)
    //  EntoPose::ProblemOptions pnp_planar_dlt_opt = options;
    //  pnp_planar_dlt_opt.n_point_point_ = 16;
    //  pnp_planar_dlt_opt.n_point_line_ = 0;
    //  results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverP6PDLT<Scalar>>(1e6, p6p_dlt_opt, tol));

    
    // ====================================================
    /*
    // ====================================================
    // Relative Pose
    
    // Relative Pose Upright
    EntoPose::ProblemOptions relupright3pt_opt = options;
    relupright3pt_opt.n_point_point_ = 3;
    relupright3pt_opt.upright_ = true;
    results.push_back(EntoPose::benchmark_relative<Scalar, EntoPose::SolverRelUpright3pt<Scalar>>(1e4, relupright3pt_opt, tol));

    // Relative Pose 8pt
    EntoPose::ProblemOptions rel8pt_opt = options;
    rel8pt_opt.n_point_point_ = 8;
    results.push_back(EntoPose::benchmark_relative<Scalar, EntoPose::SolverRel8pt<Scalar>>(1e4, rel8pt_opt, tol));

    rel8pt_opt.additional_name_ = "(100 pts)";
    rel8pt_opt.n_point_point_ = 100;
    results.push_back(EntoPose::benchmark_relative<Scalar, EntoPose::SolverRel8pt<Scalar>>(1e4, rel8pt_opt, tol));

    // Relative Pose 5pt
    EntoPose::ProblemOptions rel5pt_opt = options;
    rel5pt_opt.n_point_point_ = 5;
    results.push_back(EntoPose::benchmark_relative<Scalar, EntoPose::SolverRel5pt<Scalar>>(1e4, rel5pt_opt, tol));

    // Relative Pose Upright Planar 2pt
    EntoPose::ProblemOptions reluprightplanar2pt_opt = options;
    reluprightplanar2pt_opt.n_point_point_ = 2;
    reluprightplanar2pt_opt.upright_ = true;
    reluprightplanar2pt_opt.planar_ = true;
    results.push_back(
        EntoPose::benchmark_relative<Scalar, EntoPose::SolverRelUprightPlanar2pt<Scalar>>(1e4, reluprightplanar2pt_opt, tol));

    // Relative Pose Upright Planar 3pt
    EntoPose::ProblemOptions reluprightplanar3pt_opt = options;
    reluprightplanar3pt_opt.n_point_point_ = 3;
    reluprightplanar3pt_opt.upright_ = true;
    reluprightplanar3pt_opt.planar_ = true;
    results.push_back(
        EntoPose::benchmark_relative<Scalar, EntoPose::SolverRelUprightPlanar3pt<Scalar>>(1e4, reluprightplanar3pt_opt, tol));
    // ====================================================


    // ====================================================
    // Homography Estimation

    // Homograpy (4pt)
    EntoPose::ProblemOptions homo4pt_opt = options;
    homo4pt_opt.n_point_point_ = 4;
    results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomography4pt<Scalar, false>>(1e5, homo4pt_opt, tol));
    results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomography4pt<Scalar, true>>(1e5, homo4pt_opt, tol));
    */

    display_result<Scalar>(results);

    return 0;
}
