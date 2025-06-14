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
  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  using Problem = EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 1>;
  
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
  const char* rel_path = "state-est/robofly_sync_benchmark.csv";  // Use synchronous dataset
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_ekf_sync_update_float!");
  }

  ENTO_DEBUG("File path: %s", dataset_path);

  // Initialize EKF with realistic noise matrices for RoboFly
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity();
  Q(0,0) = 0.001f; // theta process noise
  Q(1,1) = 0.001f; // vx process noise  
  Q(2,2) = 0.001f; // z process noise
  Q(3,3) = 0.001f; // vz process noise
  
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0,0) = 0.001f*0.001f; // range measurement noise (1mm std)
  R(1,1) = 0.05f*0.05f;   // optical flow noise (0.05 rad/s std)
  R(2,2) = 0.1f*0.1f;     // accel x noise (0.1 m/s^2 std)
  R(3,3) = 0.1f*0.1f;     // accel z noise (0.1 m/s^2 std)
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  // Create problem with SYNCHRONOUS update method
  Problem problem(std::move(kernel), std::move(dynamics), std::move(measurement), 
                  Problem::UpdateMethod::SYNCHRONOUS);
  
  // Set initial conditions for RoboFly EKF
  Eigen::Matrix<Scalar, 4, 1> x0;
  x0 << 0.0f,   // theta = 0 (level attitude)
        0.0f,   // vx = 0 (no initial velocity)
        0.12f,  // z = 0.12m (reasonable hover height)
        0.0f;   // vz = 0 (no initial vertical velocity)
  
  Eigen::Matrix<Scalar, 4, 4> P0 = Eigen::Matrix<Scalar, 4, 4>::Identity();
  P0(0,0) = 0.1f*0.1f;   // theta uncertainty (0.1 rad std)
  P0(1,1) = 0.1f*0.1f;   // vx uncertainty (0.1 m/s std)  
  P0(2,2) = 0.01f*0.01f; // z uncertainty (1cm std)
  P0(3,3) = 0.1f*0.1f;   // vz uncertainty (0.1 m/s std)
  
  problem.setInitialConditions(x0, P0);

  printf("File path: %s\n", dataset_path);

  // Construct harness and run
  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench EKF Synchronous Update [float]",
                       dataset_path, output_path);
  harness.run();

  exit(1);
  return 0;
} 