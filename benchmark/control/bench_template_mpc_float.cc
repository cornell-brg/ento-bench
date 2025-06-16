#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>

#include <ento-control/opt_control_problem.h>
#include <ento-control/robobee_mpc.h>

#include <Eigen/Dense>
#include <vector>
#include <array>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoControl;

/**
 * @brief Template MPC Solver wrapper that implements the OptControlProblem interface
 * 
 * This wrapper adapts the RoboBeeMPC (template MPC with horizon=3) to work with
 * the OptControlProblem interface which expects horizon-length reference trajectories.
 */
class TemplateMPCSolver {
public:
  using State = Eigen::Matrix<float, 12, 1>;
  using Control = Eigen::Matrix<float, 3, 1>;
  static constexpr int StateSize = 12;
  static constexpr int CtrlSize = 3;
  static constexpr int HorizonSize = 3;  // UMPC_N = 3

  TemplateMPCSolver() : mpc_() {}

  bool set_x0(const State& x0) {
    mpc_.set_x0(x0);
    return true; // Always successful now
  }

  /**
   * @brief Set reference trajectory over horizon
   * @param x_ref Reference states [StateSize × HorizonSize] matrix
   * 
   * The template MPC uses constant references over the horizon, so we take
   * the first column as the reference for all horizon steps.
   */
  void set_x_ref(const Eigen::Matrix<float, StateSize, HorizonSize>& x_ref) {
    // Template MPC uses constant reference over horizon
    // Take the first reference state
    State ref_state = x_ref.col(0);
    mpc_.set_x_ref(ref_state);
    
    // Store for dynamics simulation
    x_ref_single_ = ref_state;
  }

  void reset_duals() {
    // Template MPC doesn't expose dual reset, but OSQP handles warm start internally
  }

  bool solve() {
    return mpc_.solve();
  }

  Control get_u0() const {
    return mpc_.get_u0();
  }

  int get_status() const {
    return mpc_.get_status();
  }

  int get_iterations() const {
    return mpc_.get_iterations();
  }

  // Forward simulation method that delegates to RoboBeeMPC
  State simulate_forward(const State& x0, const Control& u, float dt = 5.0f) {
    return mpc_.simulate_forward(x0, u, dt);
  }

private:
  RoboBeeMPC mpc_;
  State x_ref_single_;
};

constexpr int path_len = 100;
constexpr int num_states = 12;   // Template MPC has 12 states: [x,y,z,ox,oy,oz,vx,vy,vz,wx,wy,wz]
constexpr int num_inputs = 3;    // Template MPC has 3 control inputs: [thrust,pitch_moment,roll_moment]
constexpr int len_horizon = 3;   // Template MPC horizon length

int main()
{
  using Scalar = float;
  using Solver = TemplateMPCSolver;
  using Problem = OptControlProblem<Scalar, Solver, num_states, num_inputs, len_horizon, path_len>;
  
#if defined(SEMIHOSTING)
  initialise_monitor_handles();
#endif

  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "control/template_mpc_helix.csv";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for bench_template_mpc_float!");
  }

  // Create solver (RoboBeeMPC uses default parameters)
  Solver solver;
  Problem problem(solver);

  printf("File path: %s\n", dataset_path);

  ENTO_BENCH_HARNESS_TYPE(Problem);
  BenchHarness harness(problem, "Bench Template MPC [float]",
                       dataset_path, output_path);

  harness.run();

  exit(1);
  return 0;
} 