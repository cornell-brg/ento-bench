#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-state-est/attitude-est/fourati_nonlinear.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-math/core.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoAttitude;

// Define a Fourati solver for the attitude problem
template <typename Scalar, bool UseMag>
class SolverFourati {
public:
  SolverFourati(Scalar gain = Scalar(0.1))
    : gain_(gain) {
    // Define reference quaternions for gravity and magnetic field
    g_q_ = Eigen::Quaternion<Scalar>(Scalar(0), Scalar(0), Scalar(0), Scalar(1)); 
    
    // This represents a pure quaternion [0, 1, 0, 0.5] 
    // where the vector part points mainly along the x-axis (North) with a 
    // component along the z-axis (Down)
    // The Earth's magnetic field doesn't point exactly North but has a dip 
    // angle that varies by location
    m_q_ = Eigen::Quaternion<Scalar>(Scalar(0), Scalar(1), Scalar(0), Scalar(0.5)); // [0,1,0,0.5]
  }
  
  // Solver operator that updates the quaternion state based on measurements
  void operator()(const Eigen::Quaternion<Scalar>& q_prev,
                  const AttitudeMeasurement<Scalar, UseMag>& measurement,
                  Scalar dt,
                  Eigen::Quaternion<Scalar>* q_out) 
  {
    if constexpr (UseMag) {
      // Full MARG update with magnetometer
      *q_out = fourati_update(
          q_prev,
          measurement.gyr,
          measurement.acc,
          measurement.mag,
          dt,
          gain_,
          g_q_,
          m_q_);
    } else {
      
      EntoMath::Vec3<Scalar> zero_mag = EntoMath::Vec3<Scalar>::Zero();
      *q_out = fourati_update(
          q_prev,
          measurement.gyr,
          measurement.acc,
          zero_mag,
          dt,
          gain_,
          g_q_,
          m_q_);
    }
  }
  
  // Get solver name (for reporting)
  static constexpr const char* name() {
    if constexpr (UseMag) {
      return "Fourati MARG Filter";
    } else {
      return "Fourati IMU Filter";
    }
  }

private:
  Scalar gain_; // Filter gain
  Eigen::Quaternion<Scalar> g_q_; // Reference gravity quaternion
  Eigen::Quaternion<Scalar> m_q_; // Reference magnetic field quaternion
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
  using Solver = SolverFourati<Scalar, UseMag>;
  using Problem = AttitudeProblem<Scalar, Solver, UseMag, true>;
  
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
  // Reuse existing dataset for now
  const char* rel_path = "state-est/attitude/fourati_marg_robobee.txt";
  char dataset_path[512];
  char output_path[256];
  
  if (!EntoUtil::build_file_path(base_path, rel_path,
                                dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_fourati!");
    printf("Failed to build path for dataset. Check that the file exists.\n");
    return 1;
  }
  
  printf("File path: %s\n", dataset_path);
  
  // Create solver and problem instances
  Solver solver(Scalar(0.1));  // Default Fourati gain parameter
  Problem problem(solver);
  
  // Create a benchmark harness with updated template parameters including InnerReps
  constexpr bool DoWarmup = false;
  constexpr size_t Reps = 5;
  constexpr size_t InnerReps = 10;
  EntoBench::Harness<Problem, DoWarmup, Reps, InnerReps> harness(
    problem, 
    UseMag ? "Bench Fourati MARG Filter [float]" : "Bench Fourati IMU Filter [float]",
    dataset_path,
    output_path);
  
  harness.run();
  
  printf("Benchmark completed.\n");
  return 0;
}