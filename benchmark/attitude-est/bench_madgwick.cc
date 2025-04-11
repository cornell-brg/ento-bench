#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-state-est/attitude-est/madgwick.h>
#include <ento-state-est/attitude-est/attitude_problem.h>
#include <ento-math/core.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoAttitude;

// Define a Madgwick solver for the attitude problem
template <typename Scalar, bool UseMag>
class SolverMadgwick {
public:
  SolverMadgwick(Scalar gain = Scalar(0.1))
    : gain_(gain) {}
  
  // Solver operator that updates the quaternion state based on measurements
  void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                  const AttitudeMeasurement<Scalar, UseMag>& measurement,
                  Scalar dt,
                  Eigen::Quaternion<Scalar>* q_out) 
  {
    if constexpr (UseMag) {
      *q_out = madgwick_update_marg(q_prev,
                                   measurement.gyr,
                                   measurement.acc,
                                   measurement.mag,
                                   dt,
                                   gain_);
    } else {
      *q_out = madgwick_update_imu(q_prev,
                                  measurement.gyr,
                                  measurement.acc,
                                  dt,
                                  gain_);
    }
  }
  
  // Get solver name (for reporting)
  static constexpr const char* name() {
    if constexpr (UseMag) {
      return "Madgwick MARG Filter";
    } else {
      return "Madgwick IMU Filter";
    }
  }

private:
  Scalar gain_;
};

int main() {
  using Scalar = float;
  constexpr bool UseMag = true; // Set to true to use MARG, false for IMU only
  
  // Define our solver and problem types
  using Solver = SolverMadgwick<Scalar, UseMag>;
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
  const char* rel_path = UseMag ? "attitude/marg_float_100.csv" : "attitude/imu_float_100.csv";
  char dataset_path[512];
  char output_path[256];
  
  if (!EntoUtil::build_file_path(base_path, rel_path,
                                dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_madgwick!");
  }
  
  // Create solver and problem instances
  Scalar gain = Scalar(0.1); // Madgwick's beta parameter (filter gain)
  Solver solver(gain);
  Problem problem(solver);
  
  printf("File path: %s\n", dataset_path);
  
  // Create a benchmark harness
  // The dimensions will be determined by the AttitudeProblem template
  EntoBench::Harness<Problem> harness(
     problem, 
     UseMag ? "Bench Madgwick MARG Filter [float]" : "Bench Madgwick IMU Filter [float]",
     dataset_path,
     output_path);
  
  harness.run();
  
  printf("Benchmark completed.\n");
  return 0;
}