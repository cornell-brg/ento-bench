// #include <ento-util/debug.h>
// #include <ento-util/unittest.h>
// #include <ento-util/file_path_util.h>
// #include <ento-state-est/attitude-est/attitude_estimation_problem.h>
// #include <ento-state-est/attitude-est/attitude_measurement.h>



// const char* file_path = __FILE__;
// constexpr size_t FILEPATH_SIZE = 128;

// char dir_path[FILEPATH_SIZE];
// char test1_input_path[FILEPATH_SIZE];
// char test1_output_path[FILEPATH_SIZE];
// // Add more char arrays for other input/output paths.

// char* full_paths[] = { test1_input_path, test1_output_path };
// constexpr size_t num_paths = 2;

// using namespace EntoAttitude;

// template <typename Scalar>
// class TestAttitudeFilter
// {
//   inline void operator()(const Eigen::Quaternion<Scalar>& q_prev,
//                          [[maybe_unused]] const MARGMeasurement<Scalar>& meas,
//                          [[maybe_unused]] Scalar dt,
//                          Eigen::Quaternion<Scalar>* q)
//   {
//     *q = q_prev; // always returns q_prev;
//   }

//   inline void operator()(const Eigen::Quaternion<Scalar>& q_prev,
//                          [[maybe_unused]] const IMUMeasurement<Scalar>& meas,
//                          [[maybe_unused]] Scalar dt,
//                          Eigen::Quaternion<Scalar>* q)
//   {
//     *q = q_prev; // always returns q_prev;
//   }

//   static constexpr const char* name()
//   {
//     return "Test Attitude Filter";
//   }

// };



// void test_attitude_problem_basic()
// {
//   using Scalar = float;
//   using Kernel = TestAttitudeFilter<Scalar>;
//   static constexpr bool UseMag = true;
//   using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
  
//   Kernel k;
//   Problem problem(k);

//   // @TODO: Test basic functionality.
//   //   Create your own std::string for deserialize and call
//   //   problem.deserialize(). Etc.
  
//   const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 0.4 0.5 0.6 1.0 0.0 0.0 0.0 0.01";
  
//   if (problem.deserialize_impl(test_input)) {
//     std::cout << "Deserialization successful.\n";
    
//     std::cout << "Parsed Measurement Values:\n";
//     std::cout << "Accel: " << problem.measurement_.ax << " " << problem.measurement_.ay << " " << problem.measurement_.az << "\n";
//     std::cout << "Gyro: " << problem.measurement_.gx << " " << problem.measurement_.gy << " " << problem.measurement_.gz << "\n";
    
//     if constexpr (UseMag) {
//         std::cout << "Mag: " << problem.measurement_.mx << " " << problem.measurement_.my << " " << problem.measurement_.mz << "\n";
//     }

//     std::cout << "Ground Truth Quaternion: " << problem.q_gt_.w() << " " 
//               << problem.q_gt_.x() << " " << problem.q_gt_.y() << " " 
//               << problem.q_gt_.z() << "\n";
//     std::cout << "Delta Time: " << problem.dt_ << "\n";
//   } else {
//     std::cout << "Deserialization failed.\n";
//   }
// }



// int main ( int argc, char ** argv )
// {

//   using namespace EntoUtil;
//   int __n;
//   if (argc > 1)
//   {
//     __n = atoi(argv[1]);
//   }
//   else
//   {
//     // For the case we are running on the MCU and we can't pass in args
//     // the same way args are passed for a native build.
//     __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
//     __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
//   }

//   // Setup Directory Path and Test Data Paths
//   get_file_directory(file_path, sizeof(dir_path), dir_path);
//   const char* file_names[] = { "test_attitude_problem_input_1.txt" , "test_attitude_problem_output_1.txt" };
//   build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
//   printf("Generated Paths:\n");
//   for (size_t i = 0; i < num_paths; ++i) {
//     printf("  %s\n", full_paths[i]);
//   }

//   // Run Tests
//   if (__ento_test_num(__n, 1)) test_attitude_problem_basic();
// }


#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-util/file_path_util.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>

const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 128;

char dir_path[FILEPATH_SIZE];
char test1_input_path[FILEPATH_SIZE];
char test1_output_path[FILEPATH_SIZE];
// Add more char arrays for other input/output paths.

char* full_paths[] = { test1_input_path, test1_output_path };
constexpr size_t num_paths = 2;

using namespace EntoAttitude;

template <typename Scalar>
class TestAttitudeFilter {
  inline void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                         [[maybe_unused]] const MARGMeasurement<Scalar>& meas,
                         [[maybe_unused]] Scalar dt,
                         Eigen::Quaternion<Scalar>* q)
  {
    *q = q_prev; // always returns q_prev;
  }

  inline void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                         [[maybe_unused]] const IMUMeasurement<Scalar>& meas,
                         [[maybe_unused]] Scalar dt,
                         Eigen::Quaternion<Scalar>* q)
  {
    *q = q_prev; // always returns q_prev;
  }

  static constexpr const char* name()
  {
    return "Test Attitude Filter";
  }
};

void test_attitude_problem_basic() {
  using Scalar = float;
  using Kernel = TestAttitudeFilter<Scalar>;
  static constexpr bool UseMag = true;
  using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
  
  Kernel k;
  Problem problem(k);

  // Create test input for deserialization
  const char* test_input = "0.1 0.2 0.3 0.01 0.02 0.03 0.4 0.5 0.6 1.0 0.0 0.0 0.0 0.01";
  
  // Test deserialization
  ENTO_TEST_CHECK(problem.deserialize_impl(test_input));
  
  // Test measurement values 
  // Note: We're accessing the correct member names based on the structure
  // Instead of ax, ay, az - the struct likely uses acc (Vector3f)
  ENTO_DEBUG("Parsed Measurement Values:");
  if constexpr (UseMag) {
    // Use proper field names for MARGMeasurement
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[0], 0.1f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[1], 0.2f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[2], 0.3f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[0], 0.01f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[1], 0.02f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[2], 0.03f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[0], 0.4f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[1], 0.5f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.mag[2], 0.6f);
  } else {
    // Use proper field names for IMUMeasurement
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[0], 0.1f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[1], 0.2f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.acc[2], 0.3f);
    
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[0], 0.01f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[1], 0.02f);
    ENTO_TEST_CHECK_FLOAT_EQ(problem.measurement_.gyr[2], 0.03f);
  }
  
  // Test quaternion values
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.w(), 1.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.x(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.y(), 0.0f);
  ENTO_TEST_CHECK_FLOAT_EQ(problem.q_gt_.z(), 0.0f);
  
  // Test delta time
  ENTO_TEST_CHECK_FLOAT_EQ(problem.dt_, 0.01f);
}

int main(int argc, char** argv) {
  using namespace EntoUtil;
  
  ENTO_TEST_BEGIN();
  
  // Get test number from command line or file
  int __n;
  if (argc > 1) {
    __n = atoi(argv[1]);
  } else {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  // Setup Directory Path and Test Data Paths
  get_file_directory(file_path, sizeof(dir_path), dir_path);
  const char* file_names[] = { "test_attitude_problem_input_1.txt", "test_attitude_problem_output_1.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);

  ENTO_DEBUG("Generated Paths:");
  for (size_t i = 0; i < num_paths; ++i) {
    ENTO_DEBUG("  %s", full_paths[i]);
  }

  // Run Tests
  if (__ento_test_num(__n, 1)) test_attitude_problem_basic();
  
  ENTO_TEST_END();
  
  return __failed;
}