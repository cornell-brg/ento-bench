/**
 * TinyMPC benchmark for Webots Crazyflie figure-8 trajectory.
 *
 * Uses dynamics linearized from the Webots Crazyflie PROTO (mass=0.05kg,
 * arm=0.031m, dt=0.01s) and the same Q/R/N as the working Webots controller.
 */

#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>

#include <cstdio>
#include <ento-control/tinympc_solver.h>
#include <ento-control/opt_control_problem.h>

constexpr int path_len = 5120;
constexpr int num_states = 12;
constexpr int num_inputs = 4;
constexpr int len_horizon = 20;

int main()
{
  using Scalar_t = float;
  using Solver   = TinyMPCSolver< Scalar_t, num_states, num_inputs, len_horizon >;
  using Problem  = OptControlProblem< Scalar_t, Solver, num_states, num_inputs, len_horizon, path_len >;

  const char* base_path = DATASET_PATH;
  const char* rel_path  = "opt-control/entobench_fig8_webots.csv";
  char dataset_path[512];
  char output_path[256];

  if ( !EntoUtil::build_file_path( base_path, rel_path, dataset_path, sizeof( dataset_path ) ) ) {
      ENTO_DEBUG( "ERROR! Could not build file path for TinyMPC Webots" );
  }
  printf("File path: %s\n", dataset_path);

  // Webots Crazyflie dynamics (dt=0.01s, mass=0.05kg)
  Eigen::Matrix< Scalar_t, num_states, num_states > Adyn = ( Eigen::Matrix< Scalar_t, num_states, num_states >() <<
    1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0004905, 0.0000000, 0.0100000, 0.0000000, 0.0000000,  0.0000000, 0.0000016, 0.0000000,
    0.0000000, 1.0000000, 0.0000000, -0.0004905, 0.0000000, 0.0000000, 0.0000000, 0.0100000, 0.0000000, -0.0000016, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0100000,  0.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0100000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0100000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0100000,
    0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0981000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0004905, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, -0.0981000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0004905, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000
  ).finished();

  Eigen::Matrix< Scalar_t, num_states, num_inputs > Bdyn = ( Eigen::Matrix< Scalar_t, num_states, num_inputs >() <<
    -0.0000036,  0.0000036,  0.0000036, -0.0000036,
     0.0000036,  0.0000036, -0.0000036, -0.0000036,
     0.0010000,  0.0010000,  0.0010000,  0.0010000,
    -0.0442857, -0.0442857,  0.0442857,  0.0442857,
    -0.0442857,  0.0442857,  0.0442857, -0.0442857,
     0.0008669, -0.0008669,  0.0008669, -0.0008669,
    -0.0014481,  0.0014481,  0.0014481, -0.0014481,
     0.0014481,  0.0014481, -0.0014481, -0.0014481,
     0.2000000,  0.2000000,  0.2000000,  0.2000000,
    -8.8571429, -8.8571429,  8.8571429,  8.8571429,
    -8.8571429,  8.8571429,  8.8571429, -8.8571429,
     0.1733851, -0.1733851,  0.1733851, -0.1733851
  ).finished();

  Eigen::Matrix< Scalar_t, num_states, 1 > Q{
    5.0, 5.0, 10.0, 30.0, 30.0, 20.0, 2.0, 2.0, 2.0, 10.0, 10.0, 5.0
  };
  Eigen::Matrix< Scalar_t, num_inputs, 1 > R{ 50000.0, 50000.0, 50000.0, 50000.0 };

  Eigen::Matrix< Scalar_t, num_states, len_horizon >     x_min = Eigen::Matrix< Scalar_t, num_states, len_horizon >::Constant( -5 );
  Eigen::Matrix< Scalar_t, num_states, len_horizon >     x_max = Eigen::Matrix< Scalar_t, num_states, len_horizon >::Constant( 5 );
  Eigen::Matrix< Scalar_t, num_inputs, len_horizon - 1 > u_min = Eigen::Matrix< Scalar_t, num_inputs, len_horizon - 1 >::Constant( -0.122625 );
  Eigen::Matrix< Scalar_t, num_inputs, len_horizon - 1 > u_max = Eigen::Matrix< Scalar_t, num_inputs, len_horizon - 1 >::Constant( 14.28 );

  float rho_value = 1.0f;

  static Solver solver( Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true );

  auto tiny_settings = solver.get_settings();
  solver.update_settings( tiny_settings.abs_pri_tol,
                          tiny_settings.abs_dua_tol,
                          100,
                          tiny_settings.check_termination,
                          tiny_settings.en_state_bound,
                          tiny_settings.en_input_bound );
  Problem problem( solver, "entobench_trajectory.csv" );

  EntoBench::Harness harness( problem, "Bench TinyMPC Webots Crazyflie [float]", dataset_path, output_path );
  harness.run();

  return 0;
}
