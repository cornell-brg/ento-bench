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
class TestAttitudeFilter
{
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


void test_attitude_problem_basic()
{
  using Scalar = float;
  using Kernel = TestAttitudeFilter<Scalar>;
  static constexpr bool UseMag = true;
  using Problem = AttitudeProblem<Scalar, Kernel, UseMag>;
  
  Kernel k;
  Problem problem(k);

  // @TODO: Test basic functionality.
  //   Create your own std::string for deserialize and call
  //   problem.deserialize(). Etc.

}


int main ( int argc, char ** argv )
{

  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  // Setup Directory Path and Test Data Paths
  get_file_directory(file_path, sizeof(dir_path), dir_path);
  const char* file_names[] = { "test_attitude_problem_input_1.txt" , "test_attitude_problem_output_1.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
  printf("Generated Paths:\n");
  for (size_t i = 0; i < num_paths; ++i) {
    printf("  %s\n", full_paths[i]);
  }

  // Run Tests
  if (__ento_test_num(__n, 1)) test_attitude_problem_basic();
}
