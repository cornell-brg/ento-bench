#include <ento-pose/prob_gen.h>
#include <ento-pose/data_gen.h>

#include <ento-util/dataset_writer.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>

#include <iostream>

using namespace EntoUtil;
using namespace EntoPose;

template <typename Scalar, typename Generator, typename ProblemInstance>
void generate_robust_pose_experiment(size_t n_problems,
                                     Scalar inlier_ratio,
                                     Generator generator,
                                     const ProblemOptions<Scalar> &options,
                                     const std::optional<std::string> &dataset_filename = std::nullopt)
{
    std::vector<ProblemInstance> problem_instances;
    std::cout << "Generating!" << std::endl;
    generator(n_problems, inlier_ratio, &problem_instances, options);
    std::cout << "Done Generating!" << std::endl;


    // Write problem instances to a file
    if (dataset_filename)
    {
        DatasetWriter<ProblemInstance> writer(*dataset_filename);
        for (auto &instance : problem_instances) {
            instance.set_num_pts(options.n_point_point_);
            writer.write_instance(instance);
        }
        std::cout << "Serialized " << n_problems << " problems to " << *dataset_filename << "\n";
    }
}

// Usage:
//  ./robust_data_gen <num_experiments> <num_pts> <inlier_ratio> <scalar> |test_flag|
int main(int argc, char* argv[])
{
  using Scalar = float;

  if ( argc < 4 || argc > 5 )
  {
    std::cerr << "Usage: " << argv[0] << " <num_experiments> <num_pts> <inlier_ratio>" << std::endl;
    return 1;
  }
  bool is_test;
  if ( argc == 5 ) is_test = std::stoi(argv[5]) != 0;
  else is_test = false; // default False

  int num_experiments = std::stoi(argv[1]);
  int num_points = std::stoi(argv[2]);
  Scalar inlier_ratio = std::stof(argv[3]);


  EntoPose::ProblemOptions<Scalar> options;
  options.camera_fov_ = 120; // Wide, NanEye FoV
                             
  std::string base_path = DATASET_PATH;
  std::cout << "Dataset base path: " << base_path << std::endl;

  // Adjust relative paths based on test flag
  std::string prefix = is_test ? "test/" : "";

  // Relative paths with placeholders for scalar type
  std::vector<std::string> relative_paths = {
      "robust-pose/"   + prefix + "abspose_{scalar}_{n}_{inlier_ratio}.csv",
      "robust-pose/"   + prefix + "relpose_{scalar}_{n}_{inlier_ratio}.csv",
      "robust-pose/"   + prefix + "homography_{scalar}_{n}_{inlier_ratio}.csv"
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
      relative_path.replace(pos, 3, std::to_string(num_experiments)); // 8 = length of "{scalar}"

    pos = relative_path.find("{inlier_ratio}");
    if (pos != std::string::npos)
      relative_path.replace(pos, 14, std::to_string(inlier_ratio));
    
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


  std::cout << "Starting data generation!" << std::endl;

  using RobustAbsPoseGenerator    = decltype(&generate_robust_abspose_problems<Scalar>);
  using RobustRelPoseGenerator    = decltype(&generate_robust_relpose_problems<Scalar>);
  using RobustHomographyGenerator = decltype(&generate_robust_homography_problems<Scalar>);

  using RobustAbsPoseInstance    = RobustAbsolutePoseProblemInstance<Scalar>;
  using RobustRelPoseInstance    = RobustRelativePoseProblemInstance<Scalar>;
  using RobustHomographyInstance = RobustRelativePoseProblemInstance<Scalar>;


  options.n_point_point_ = num_points;
  // Generate experiments for RANSAC absolute pose problems
  std::cout << "Generating Robust Absolute Pose Experiments" << std::endl;
  generate_robust_pose_experiment<Scalar,
    RobustAbsPoseGenerator,
    RobustAbsPoseInstance>(num_experiments,
                           inlier_ratio,
                           generate_robust_abspose_problems<Scalar>,
                           options,
                           full_paths[0]);

  // Generate experiments for RANSAC relative pose problems
  std::cout << "Generating Robust Relative Pose Experiments" << std::endl;
  generate_robust_pose_experiment<Scalar, RobustRelPoseGenerator, RobustRelPoseInstance>(
      num_experiments,
      inlier_ratio,
      generate_robust_relpose_problems<Scalar>,
      options,
      full_paths[1]
  );

  // Generate experiments for RANSAC homography problems
  std::cout << "Generating Robust Homography Experiments" << std::endl;
  generate_robust_pose_experiment<Scalar, RobustHomographyGenerator, RobustHomographyInstance>(
      num_experiments,
      inlier_ratio,
      generate_robust_homography_problems<Scalar>,
      options,
      full_paths[2]
  );
}







