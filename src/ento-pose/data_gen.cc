#include <ento-pose/data_gen.h>
#include <cstdint>

#include <chrono>
#include <iostream>
#include <iomanip>
#include <optional>
#include <ento-util/dataset_writer.h>
#include <ento-util/file_path_util.h>

using namespace EntoUtil;

//constexpr char* dataset_path = DATASET_PATH;

namespace EntoPose
{

template <typename Scalar, typename Solver>
BenchmarkResult<Scalar> benchmark(int n_problems,
                                  const ProblemOptions<Scalar> &options,
                                  Scalar tol = 1e-6,
                                  const std::optional<std::string> &dataset_filename = std::nullopt) 
{
    // Generate problem instances
    std::vector<AbsolutePoseProblemInstance<Scalar>> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);

    // Write problem instances to a file
    if (dataset_filename)
    {
        DatasetWriter<AbsolutePoseProblemInstance<Scalar>> writer(*dataset_filename);
        for (auto &instance : problem_instances) {
            instance.set_num_pts(options.n_point_point_);
            writer.write_instance(instance);
        }
        std::cout << "Serialized " << n_problems << " problems to " << *dataset_filename << "\n";
    }

    // Initialize benchmark result
    BenchmarkResult<Scalar> result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (!options.additional_name_.empty()) {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Benchmark solution quality
    for (const AbsolutePoseProblemInstance<Scalar> &instance : problem_instances) {
        std::vector<CameraPose<Scalar>> solutions;

        int sols = Solver::solve(instance, &solutions);
        Scalar pose_error = std::numeric_limits<Scalar>::max();

        result.solutions_ += sols;
        for (const CameraPose<Scalar> &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, 1.0, tol)) {
                result.valid_solutions_++;
            }
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose, 1.0));
        }

        if (pose_error < tol) {
            result.found_gt_pose_++;
        }
    }

    // Measure runtime
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
  if (options.additional_name_ != "")
  {
    result.name_ += options.additional_name_;
  }
  result.options_ = options;
  std::cout << "Running benchmark: " << result.name_ << std::flush;

  // Run benchmark where we check solution quality
  for (const AbsolutePoseProblemInstance<Scalar> &instance : problem_instances)
  {
    std::vector<CameraPose<Scalar>> solutions;
    std::vector<Scalar> extra;

    int sols = Solver::solve(instance, &solutions, &extra);

    Scalar pose_error = std::numeric_limits<Scalar>::max();

    result.solutions_ += sols;
    for (size_t k = 0; k < solutions.size(); ++k)
    {
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
  for (int iter = 0; iter < 10; ++iter)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    for (const AbsolutePoseProblemInstance<Scalar> &instance : problem_instances)
    {
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
                                           Scalar tol = 1e-6,
                                           const std::optional<std::string> &dataset_filename = std::nullopt) 
{
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

  if (dataset_filename)
  {
    DatasetWriter<RelativePoseProblemInstance<Scalar>> writer(*dataset_filename);
    for (auto &instance : problem_instances)
    {
      // We don't need to store everything from rel problem in homography problem
      // use subset that is stored in HomographyProblemInstance
      instance.set_num_pts(options.n_point_point_);
      writer.write_instance(instance);
    }
    std::cout << "Serialized " << n_problems << " problems to " << *dataset_filename << "\n";
  }

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
                                             Scalar tol = 1e-6,
                                             const std::optional<std::string> &dataset_filename = std::nullopt)
{

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

  if (dataset_filename)
  {
    DatasetWriter<HomographyProblemInstance<Scalar>> writer(*dataset_filename);
    for (auto &instance : problem_instances)
    {
      // We don't need to store everything from rel problem in homography problem
      // use subset that is stored in HomographyProblemInstance
      HomographyProblemInstance<Scalar> tmp(instance);
      tmp.set_num_pts(options.n_point_point_);
      writer.write_instance(tmp);
    }
    std::cout << "Serialized " << n_problems << " problems to " << *dataset_filename << "\n";
  }

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

int main(int argc, char* argv[])
{
  using Scalar = float;
  std::vector<EntoPose::BenchmarkResult<Scalar>> results;

  // Argument parsing
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <iterations> <test_flag (0 or 1)>" << std::endl;
    return 1;
  }

  int iters = std::stoi(argv[1]);
  bool is_test = std::stoi(argv[2]) != 0; // Convert numeric flag to boolean

  EntoPose::ProblemOptions<Scalar> options;
  options.camera_fov_ = 120; // Wide

  Scalar tol = 1e-5;

  std::string base_path = DATASET_PATH;
  std::cout << "Dataset base path: " << base_path << std::endl;

  // Adjust relative paths based on test flag
  std::string prefix = is_test ? "test/" : "";

  // Relative paths with placeholders for scalar type
  std::vector<std::string> relative_paths = {
      "abs-pose/"   + prefix + "p3p_{scalar}_{n}.csv",
      "abs-pose/"   + prefix + "up2p_{scalar}_{n}.csv",
      "abs-pose/"   + prefix + "p6p_{scalar}_{n}.csv",
      "homography/" + prefix + "homography4pt_{scalar}_{n}.csv",
      "homography/" + prefix + "homography{N}pt_{scalar}_{n}.csv",
      "rel-pose/"   + prefix + "rel_linear8pt_{scalar}_{n}.csv",
      "rel-pose/"   + prefix + "rel_linear{N}pt_{scalar}_{n}.csv",
      "rel-pose/"   + prefix + "rel_5pt_{scalar}_{n}.csv",
      "rel-pose/"   + prefix + "rel_pu3pt_{scalar}_{n}.csv",
      "rel-pose/"   + prefix + "rel_pu2pt_{scalar}_{n}.csv"
  };

  // Determine scalar type as a string
  std::string scalar_str = std::is_same<Scalar, float>::value ? "float" : "double";

  // Generate full paths with scalar type
  std::vector<std::string> full_paths;
  for (const auto &relative_path_template : relative_paths)
  {
    std::string relative_path = relative_path_template;

    // Replace "{scalar}" with the actual scalar type
    size_t pos = relative_path.find("{scalar}");
    if (pos != std::string::npos)
      relative_path.replace(pos, 8, scalar_str); // 8 = length of "{scalar}"

    pos = relative_path.find("{n}");
    if (pos != std::string::npos)
      relative_path.replace(pos, 3, std::to_string(iters)); // 8 = length of "{scalar}"
    
    // Build the full path
    std::string full_path = EntoUtil::build_file_path(base_path, relative_path);
    if (full_path.empty())
    {
      std::cerr << "Failed to build file path for: " << relative_path << std::endl;
    }
    else
    {
      full_paths.push_back(full_path);
      std::cout << "Full path: " << full_path << std::endl;
    }
  } 

  std::string p3p_fp             = full_paths[0];
  std::string up2p_fp            = full_paths[1];
  std::string p6p_fp             = full_paths[2];
  std::string homography4pt_fp   = full_paths[3];
  std::string homographyNpt_fp   = full_paths[4];
  std::string rel_linear8pt_fp   = full_paths[5];
  std::string rel_linearNpt_fp   = full_paths[6];
  std::string rel_5pt_fp         = full_paths[7];
  std::string rel_pu2pt_fp       = full_paths[8];
  std::string rel_pu3pt_fp       = full_paths[9];

  // Absolute Pose

  // P3P
  EntoPose::ProblemOptions p3p_opt = options;
  p3p_opt.n_point_point_ = 3;
  results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverP3P<Scalar>>(iters, p3p_opt, tol, p3p_fp));
  printf("results solutions %d \n", results[0].solutions_);
  
  // uP2P
  EntoPose::ProblemOptions up2p_opt = options;
  up2p_opt.n_point_point_ = 2;
  up2p_opt.upright_ = true;
  results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverUP2P<Scalar>>(iters, up2p_opt, tol, up2p_fp));



  // P6P (DLT)
  // @TODO: Rewrite dlt to conform to pose lib standard.
  //  EntoPose::ProblemOptions p6p_dlt_opt = options;
  //  p6p_dlt_opt.n_point_point_ = 6;
  //  results.push_back(EntoPose::benchmark<Scalar, EntoPose::SolverP6PDLT<Scalar>>(iters, p6p_dlt_opt, tol));
  
  // Homography 
  // Homograpy 4pt (PoseLib version using QR)
  EntoPose::ProblemOptions homo4pt_opt = options;
  homo4pt_opt.n_point_point_ = 4;
  results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomography4pt<Scalar, false, 0>>(iters, homo4pt_opt, tol, homography4pt_fp));
  results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomography4pt<Scalar, false, 1>>(iters, homo4pt_opt, tol));

  // Homography N-pt N = 4 (SVD version)
  // @TODO: Rewrite dlt to conform to pose lib standard.
  EntoPose::ProblemOptions p4p_dlt_opt = options;
  p4p_dlt_opt.n_point_point_ = 4;
  results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomographyNptDLT<Scalar, 0, false>>(iters, p4p_dlt_opt, tol));
  
  // Homography N-pt N > 4 (SVD Version)
  for (std::size_t i = 8; i <= 512; i *= 2)
  {
    std::string tmp_fp = homographyNpt_fp;
    size_t pos = tmp_fp.find("{N}");
    if (pos != std::string::npos)
      tmp_fp.replace(pos, 3, std::to_string(i)); // 8 = length of "{scalar}"

    EntoPose::ProblemOptions pnp_homography_opt = options;
    pnp_homography_opt.additional_name_ = "(" + std::to_string(i) + " pts)";
    pnp_homography_opt.n_point_point_ = i;
    results.push_back(EntoPose::benchmark_homography<Scalar, EntoPose::SolverHomography4ptDLT<Scalar, true>>(iters, pnp_homography_opt, tol, tmp_fp));
  }

  
  // ====================================================
  
  // ====================================================
  // Relative Pose
  
  // Relative Pose 8pt
  EntoPose::ProblemOptions rel8pt_opt = options;
  rel8pt_opt.n_point_point_ = 8;
  results.push_back(EntoPose::benchmark_relative<Scalar, EntoPose::SolverRel8pt<Scalar>>(iters, rel8pt_opt, tol, rel_linear8pt_fp));

  for (std::size_t i = 8; i <= 512; i *= 2)
  {
    std::string tmp_fp = rel_linearNpt_fp;
    size_t pos = tmp_fp.find("{N}");
    if (pos != std::string::npos)
      tmp_fp.replace(pos, 3, std::to_string(i)); // 8 = length of "{scalar}"

    EntoPose::ProblemOptions rel8ptN_opt = options;
    rel8ptN_opt.additional_name_ = "(" + std::to_string(i) + " pts)";
    rel8ptN_opt.n_point_point_ = i;
    results.push_back(EntoPose::benchmark_relative<Scalar, EntoPose::SolverRel8pt<Scalar>>(iters, rel8ptN_opt, tol, tmp_fp));
  }

  
  // Relative Pose 5pt
  EntoPose::ProblemOptions rel5pt_opt = options;
  rel5pt_opt.n_point_point_ = 5;
  results.push_back(EntoPose::benchmark_relative<Scalar, EntoPose::SolverRel5pt<Scalar>>(iters, rel5pt_opt, tol, rel_5pt_fp));

  
  // Relative Pose Upright Planar 2pt
  EntoPose::ProblemOptions reluprightplanar2pt_opt = options;
  reluprightplanar2pt_opt.n_point_point_ = 2;
  reluprightplanar2pt_opt.upright_ = true;
  reluprightplanar2pt_opt.planar_ = true;
  results.push_back(
      EntoPose::benchmark_relative<Scalar, EntoPose::SolverRelUprightPlanar2pt<Scalar>>(iters, reluprightplanar2pt_opt, tol, rel_pu2pt_fp));

  // Relative Pose Upright Planar 3pt
  EntoPose::ProblemOptions reluprightplanar3pt_opt = options;
  reluprightplanar3pt_opt.n_point_point_ = 3;
  reluprightplanar3pt_opt.upright_ = true;
  reluprightplanar3pt_opt.planar_ = true;
  results.push_back(
  EntoPose::benchmark_relative<Scalar, EntoPose::SolverRelUprightPlanar3pt<Scalar>>(iters, reluprightplanar3pt_opt, tol, rel_pu3pt_fp));
  // ====================================================
  

  display_result<Scalar>(results);

  return 0;
}
