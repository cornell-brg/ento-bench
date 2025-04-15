#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-state-est/attitude-est/mahoney.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-math/core.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoAttitude;

// Define a Mahoney solver for the attitude problem
template <typename Scalar, bool UseMag>
class SolverMahoney {
public:
  SolverMahoney(Scalar k_p = Scalar(0.5), Scalar k_i = Scalar(0.1))
    : k_p_(k_p), k_i_(k_i), bias_(EntoMath::Vec3<Scalar>::Zero()) {}
  
  // Solver operator that updates the quaternion state based on measurements
  void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                  const AttitudeMeasurement<Scalar, UseMag>& measurement,
                  Scalar dt,
                  Eigen::Quaternion<Scalar>* q_out) 
  {
    if constexpr (UseMag) {
      *q_out = mahoney_update_marg(q_prev,
                                    measurement.gyr,
                                    measurement.acc,
                                    measurement.mag,
                                    dt,
                                    k_p_,
                                    k_i_,
                                    bias_);
    } else {
      *q_out = mahoney_update_imu(q_prev,
                                   measurement.gyr,
                                   measurement.acc,
                                   dt,
                                   k_p_,
                                   k_i_,
                                   bias_);
    }
  }
  
  // Get solver name (for reporting)
  static constexpr const char* name() {
    if constexpr (UseMag) {
      return "Mahoney MARG Filter";
    } else {
      return "Mahoney IMU Filter";
    }
  }

private:
  Scalar k_p_;
  Scalar k_i_;
  EntoMath::Vec3<Scalar> bias_;
};

// Need to declare this so the Harness class can use it without error
namespace EntoAttitude {
  template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
  class AttitudeProblem;

  // Add this specialization outside the class definition
  template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
  struct SaveResultsTraits {
    static constexpr bool value = false;
  };
}

// Modify the EntoBench namespace to check for SaveResultsTraits instead
// This is a partial specialization for the harness class
namespace EntoBench {
  template <typename T>
  constexpr bool has_save_results_v = false;

  // Specialization for AttitudeProblem
  template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
  constexpr bool has_save_results_v<EntoAttitude::AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>> = 
    EntoAttitude::SaveResultsTraits<Scalar, Kernel, UseMag, IsFilter>::value;
}

int main()
{
  using Scalar = float;
  constexpr bool UseMag = true; // Set to true to use MARG, false for IMU only
  
  // Define our solver and problem types
  using Solver = SolverMahoney<Scalar, UseMag>;
  using Problem = AttitudeProblem<Scalar, Solver, UseMag>;
  
  initialise_monitor_handles();
  
  // Configure max clock rate and set flash latency
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();
  
  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();
  
  const char* base_path = DATASET_PATH;
  const char* rel_path = UseMag ? "datasets/state-est/attitude/mahoney_marg_robobee.txt" : "datasets/state-est/attitude/mahoney_imu_robobee.txt";
  char dataset_path[512];
  char output_path[256];
  
  if (!EntoUtil::build_file_path(base_path, rel_path,
                                dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_mahoney!");
  }
  
  // Create solver and problem instances
  Solver solver(Scalar(0.5), Scalar(0.1));
  Problem problem(solver);
  
  printf("File path: %s\n", dataset_path);
  
  // Create a benchmark harness matching the example's format
  // Note the explicit template parameters: false for no warmup, 1 for reps
  EntoBench::Harness<Problem, false, 1> harness(
    problem, 
    UseMag ? "Bench Mahoney MARG Filter [float]" : "Bench Mahoney IMU Filter [float]",
    dataset_path,
    output_path);
  
  harness.run();
  
  printf("Benchmark completed.\n");
  return 0;
}