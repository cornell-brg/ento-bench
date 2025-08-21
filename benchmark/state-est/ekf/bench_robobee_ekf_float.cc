#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

#include <ento-state-est/EKFProblem.h>
#include <ento-state-est/ekf_kernels.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoStateEst;

int main()
{
  initialise_monitor_handles();
  
  using Scalar = float;
  using RoboBeeKernel = RoboBeeEKFKernel<Scalar>;
  using DynamicsModel = RobobeeDynamicsModel<Scalar>;
  using MeasurementModel = RobobeeMeasurementModel<Scalar>;
  using Problem = EKFProblem<RoboBeeKernel, DynamicsModel, MeasurementModel, 1>;
  
  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  // Build input dataset filepath
  const char* base_path = DATASET_PATH;
  const char* rel_path = "state-est/robobee_ekf_data_fixed.csv";  // Use same dataset as test
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_robobee_ekf_float!");
  }

  ENTO_DEBUG("File path: %s", dataset_path);

  // Initialize EKF with matrices for RoboBee
  Eigen::Matrix<Scalar, 10, 10> Q = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.001f;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01f;
  
  RoboBeeKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  Problem problem(std::move(kernel), std::move(dynamics), std::move(measurement),
                  Problem::UpdateMethod::SYNCHRONOUS);
  
  // Set initial conditions for RoboBee EKF (10-state system)
  Eigen::Matrix<Scalar, 10, 1> x0;
  x0.setZero();  // Start with zero initial state
  
  Eigen::Matrix<Scalar, 10, 10> P0 = Eigen::Matrix<Scalar, 10, 10>::Identity() * 0.1f;
  
  problem.setInitialConditions(x0, P0);

  //printf("File path: %s\n", dataset_path);

  // Construct harness and run
  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench RoboBee EKF [float]",
                       dataset_path, output_path);
  harness.run();

  exit(1);
  return 0;
} 