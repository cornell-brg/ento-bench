#include <Eigen/Dense>
#include <ento-control/opt_control_problem.h>
#include <ento-control/TinyMPCSolver.h>
#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/experiment_io.h>
#include <ento-util/file_path_util.h>

const int path_len = 301;
const int num_states = 12;
const int num_inputs = 4;
const int len_horizon = 10;
const double rho_value = 5.0;

void test_linear_trajectory_double()
{
    using Scalar  = double;
    using Solver  = TinyMPCSolver< Scalar, num_states, num_inputs, len_horizon >;
    using Problem = OptControlProblem< Scalar, Solver, num_states, num_inputs, len_horizon >;
    constexpr Scalar tol = 1e-4;

    Eigen::Matrix< Scalar, num_states, num_states > Adyn = ( Eigen::Matrix< Scalar, num_states, num_states >() <<
        1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000,  0.0000000, 0.0002044, 0.0000000,
        0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0250000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0250000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0250000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0122625, 0.0000000,
        0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000 ).finished();
    Eigen::Matrix< Scalar, num_states, num_inputs > Bdyn = ( Eigen::Matrix< Scalar, num_states, num_inputs >() <<
         -0.0007069,   0.0007773,  0.0007091,  -0.0007795,
          0.0007034,   0.0007747, -0.0007042,  -0.0007739,
          0.0052554,   0.0052554,  0.0052554,   0.0052554,
         -0.1720966,  -0.1895213,  0.1722891,   0.1893288,
         -0.1729419,   0.1901740,  0.1734809,  -0.1907131,
          0.0123423,  -0.0045148, -0.0174024,   0.0095748,
         -0.0565520,   0.0621869,  0.0567283,  -0.0623632,
          0.0562756,   0.0619735, -0.0563386,  -0.0619105,
          0.2102143,   0.2102143,  0.2102143,   0.2102143,
        -13.7677303, -15.1617018, 13.7831318,  15.1463003,
        -13.8353509,  15.2139209, 13.8784751, -15.2570451,
          0.9873856,  -0.3611820, -1.3921880,   0.7659845 ).finished();
    Eigen::Matrix< Scalar, num_states, 1 > Q{ 100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, 4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000 };
    Eigen::Matrix< Scalar, num_inputs, 1 > R{ 4.0, 4.0, 4.0, 4.0 };

    Eigen::Matrix< Scalar, num_states, len_horizon >     x_min = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( -5 );
    Eigen::Matrix< Scalar, num_states, len_horizon >     x_max = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( 5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_min = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( -0.5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_max = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( 0.5 );

    Solver solver( Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true );

    auto tiny_settings = solver.get_settings();
    solver.update_settings( tiny_settings.abs_pri_tol,
                            tiny_settings.abs_dua_tol,
                            100,
                            tiny_settings.check_termination,
                            tiny_settings.en_state_bound,
                            tiny_settings.en_input_bound );

    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/linear.csv";
    char full_path[256];
    ENTO_DEBUG( "================\n" );
    ENTO_DEBUG( "Running test_linear_trajectory..." );
    if ( !EntoUtil::build_file_path( base_path, rel_path, full_path, sizeof( full_path )))
    {
        ENTO_DEBUG( "ERROR! Could not build file path for test_linear_trajectory!" );
    }
    Problem problem( solver );

    ENTO_DEBUG( "File path: %s", full_path );
    EntoUtil::ExperimentIO reader( full_path );

    int num_experiments = 0;
    while ( reader.read_next( problem ))
    {
        problem.solve();
        num_experiments++;
    }
    ENTO_DEBUG( "experiments run: %d", num_experiments );
    // ENTO_TEST_CHECK_INT_EQ( num_experiments, 1 );

    if ( num_experiments > 0 ) {
        problem.validate();
    }
}

void test_linear_trajectory_float()
{
    using Scalar  = float;
    using Solver  = TinyMPCSolver< Scalar, num_states, num_inputs, len_horizon >;
    using Problem = OptControlProblem< Scalar, Solver, num_states, num_inputs, len_horizon >;
    constexpr Scalar tol = 1e-4;

    Eigen::Matrix< Scalar, num_states, num_states > Adyn = ( Eigen::Matrix< Scalar, num_states, num_states >() <<
        1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000,  0.0000000, 0.0002044, 0.0000000,
        0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0250000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0250000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0250000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0122625, 0.0000000,
        0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000 ).finished();
    Eigen::Matrix< Scalar, num_states, num_inputs > Bdyn = ( Eigen::Matrix< Scalar, num_states, num_inputs >() <<
         -0.0007069,   0.0007773,  0.0007091,  -0.0007795,
          0.0007034,   0.0007747, -0.0007042,  -0.0007739,
          0.0052554,   0.0052554,  0.0052554,   0.0052554,
         -0.1720966,  -0.1895213,  0.1722891,   0.1893288,
         -0.1729419,   0.1901740,  0.1734809,  -0.1907131,
          0.0123423,  -0.0045148, -0.0174024,   0.0095748,
         -0.0565520,   0.0621869,  0.0567283,  -0.0623632,
          0.0562756,   0.0619735, -0.0563386,  -0.0619105,
          0.2102143,   0.2102143,  0.2102143,   0.2102143,
        -13.7677303, -15.1617018, 13.7831318,  15.1463003,
        -13.8353509,  15.2139209, 13.8784751, -15.2570451,
          0.9873856,  -0.3611820, -1.3921880,   0.7659845 ).finished();
    Eigen::Matrix< Scalar, num_states, 1 > Q{ 100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, 4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000 };
    Eigen::Matrix< Scalar, num_inputs, 1 > R{ 4.0, 4.0, 4.0, 4.0 };

    Eigen::Matrix< Scalar, num_states, len_horizon >     x_min = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( -5 );
    Eigen::Matrix< Scalar, num_states, len_horizon >     x_max = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( 5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_min = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( -0.5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_max = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( 0.5 );

    Solver solver( Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true );

    auto tiny_settings = solver.get_settings();
    solver.update_settings( tiny_settings.abs_pri_tol,
                            tiny_settings.abs_dua_tol,
                            100,
                            tiny_settings.check_termination,
                            tiny_settings.en_state_bound,
                            tiny_settings.en_input_bound );

    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/linear.csv";
    char full_path[256];
    ENTO_DEBUG( "================\n" );
    ENTO_DEBUG( "Running test_linear_trajectory..." );
    if ( !EntoUtil::build_file_path( base_path, rel_path, full_path, sizeof( full_path )))
    {
        ENTO_DEBUG( "ERROR! Could not build file path for test_linear_trajectory!" );
    }
    Problem problem( solver );

    ENTO_DEBUG( "File path: %s", full_path );
    EntoUtil::ExperimentIO reader( full_path );

    int num_experiments = 0;
    while ( reader.read_next( problem ))
    {
        problem.solve();
        num_experiments++;
    }
    ENTO_DEBUG( "experiments run: %d", num_experiments );
    // ENTO_TEST_CHECK_INT_EQ( num_experiments, 1 );

    if ( num_experiments > 0 ) {
        problem.validate();
    }
}

void test_fig8_trajectory_double()
{
    using Scalar  = double;
    using Solver  = TinyMPCSolver< Scalar, num_states, num_inputs, len_horizon >;
    using Problem = OptControlProblem< Scalar, Solver, num_states, num_inputs, len_horizon >;
    constexpr Scalar tol = 1e-4;

    Eigen::Matrix< Scalar, num_states, num_states > Adyn = ( Eigen::Matrix< Scalar, num_states, num_states >() <<
        1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000,  0.0000000, 0.0002044, 0.0000000,
        0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0250000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0250000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0250000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0122625, 0.0000000,
        0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000 ).finished();
    Eigen::Matrix< Scalar, num_states, num_inputs > Bdyn = ( Eigen::Matrix< Scalar, num_states, num_inputs >() <<
         -0.0007069,   0.0007773,  0.0007091,  -0.0007795,
          0.0007034,   0.0007747, -0.0007042,  -0.0007739,
          0.0052554,   0.0052554,  0.0052554,   0.0052554,
         -0.1720966,  -0.1895213,  0.1722891,   0.1893288,
         -0.1729419,   0.1901740,  0.1734809,  -0.1907131,
          0.0123423,  -0.0045148, -0.0174024,   0.0095748,
         -0.0565520,   0.0621869,  0.0567283,  -0.0623632,
          0.0562756,   0.0619735, -0.0563386,  -0.0619105,
          0.2102143,   0.2102143,  0.2102143,   0.2102143,
        -13.7677303, -15.1617018, 13.7831318,  15.1463003,
        -13.8353509,  15.2139209, 13.8784751, -15.2570451,
          0.9873856,  -0.3611820, -1.3921880,   0.7659845 ).finished();
    Eigen::Matrix< Scalar, num_states, 1 > Q{ 100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, 4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000 };
    Eigen::Matrix< Scalar, num_inputs, 1 > R{ 4.0, 4.0, 4.0, 4.0 };

    Eigen::Matrix< Scalar, num_states, len_horizon >     x_min = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( -5 );
    Eigen::Matrix< Scalar, num_states, len_horizon >     x_max = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( 5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_min = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( -0.5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_max = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( 0.5 );

    Solver solver( Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true );

    auto tiny_settings = solver.get_settings();
    solver.update_settings( tiny_settings.abs_pri_tol,
                            tiny_settings.abs_dua_tol,
                            100,
                            tiny_settings.check_termination,
                            tiny_settings.en_state_bound,
                            tiny_settings.en_input_bound );

    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/fig8.csv";
    char full_path[256];
    ENTO_DEBUG( "================\n" );
    ENTO_DEBUG( "Running test_linear_trajectory..." );
    if ( !EntoUtil::build_file_path( base_path, rel_path, full_path, sizeof( full_path )))
    {
        ENTO_DEBUG( "ERROR! Could not build file path for test_linear_trajectory!" );
    }
    Problem problem( solver );

    ENTO_DEBUG( "File path: %s", full_path );
    EntoUtil::ExperimentIO fileIO( full_path, "fig8_realpath.csv" );

    int num_experiments = 0;
    while ( fileIO.read_next( problem ))
    {
        problem.solve();
        num_experiments++;
    }
    ENTO_DEBUG( "experiments run: %d", num_experiments );
    // ENTO_TEST_CHECK_INT_EQ( num_experiments, 1 );

    if ( num_experiments > 0 ) {
        problem.validate();
    }

    fileIO.write_results( problem );
}

void test_fig8large_trajectory_double()
{
    using Scalar  = double;
    using Solver  = TinyMPCSolver< Scalar, num_states, num_inputs, len_horizon >;
    using Problem = OptControlProblem< Scalar, Solver, num_states, num_inputs, len_horizon >;
    constexpr Scalar tol = 1e-4;

    Eigen::Matrix< Scalar, num_states, num_states > Adyn = ( Eigen::Matrix< Scalar, num_states, num_states >() <<
        1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000,  0.0000000, 0.0002044, 0.0000000,
        0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0250000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0250000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0250000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0122625, 0.0000000,
        0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000 ).finished();
    Eigen::Matrix< Scalar, num_states, num_inputs > Bdyn = ( Eigen::Matrix< Scalar, num_states, num_inputs >() <<
         -0.0007069,   0.0007773,  0.0007091,  -0.0007795,
          0.0007034,   0.0007747, -0.0007042,  -0.0007739,
          0.0052554,   0.0052554,  0.0052554,   0.0052554,
         -0.1720966,  -0.1895213,  0.1722891,   0.1893288,
         -0.1729419,   0.1901740,  0.1734809,  -0.1907131,
          0.0123423,  -0.0045148, -0.0174024,   0.0095748,
         -0.0565520,   0.0621869,  0.0567283,  -0.0623632,
          0.0562756,   0.0619735, -0.0563386,  -0.0619105,
          0.2102143,   0.2102143,  0.2102143,   0.2102143,
        -13.7677303, -15.1617018, 13.7831318,  15.1463003,
        -13.8353509,  15.2139209, 13.8784751, -15.2570451,
          0.9873856,  -0.3611820, -1.3921880,   0.7659845 ).finished();
    Eigen::Matrix< Scalar, num_states, 1 > Q{ 100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, 4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000 };
    Eigen::Matrix< Scalar, num_inputs, 1 > R{ 4.0, 4.0, 4.0, 4.0 };

    Eigen::Matrix< Scalar, num_states, len_horizon >     x_min = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( -5 );
    Eigen::Matrix< Scalar, num_states, len_horizon >     x_max = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( 5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_min = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( -0.5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_max = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( 0.5 );

    Solver solver( Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true );

    auto tiny_settings = solver.get_settings();
    solver.update_settings( tiny_settings.abs_pri_tol,
                            tiny_settings.abs_dua_tol,
                            100,
                            tiny_settings.check_termination,
                            tiny_settings.en_state_bound,
                            tiny_settings.en_input_bound );

    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/fig8large.csv";
    char full_path[256];
    ENTO_DEBUG( "================\n" );
    ENTO_DEBUG( "Running test_linear_trajectory..." );
    if ( !EntoUtil::build_file_path( base_path, rel_path, full_path, sizeof( full_path )))
    {
        ENTO_DEBUG( "ERROR! Could not build file path for test_linear_trajectory!" );
    }
    Problem problem( solver );

    ENTO_DEBUG( "File path: %s", full_path );
    EntoUtil::ExperimentIO fileIO( full_path, "fig8large_realpath.csv" );

    int num_experiments = 0;
    while ( fileIO.read_next( problem ))
    {
        problem.solve();
        num_experiments++;
    }
    ENTO_DEBUG( "experiments run: %d", num_experiments );
    // ENTO_TEST_CHECK_INT_EQ( num_experiments, 1 );

    if ( num_experiments > 0 ) {
        problem.validate();
    }

    fileIO.write_results( problem );
}

void test_fig8xl_trajectory_double()
{
    using Scalar  = double;
    using Solver  = TinyMPCSolver< Scalar, num_states, num_inputs, len_horizon >;
    using Problem = OptControlProblem< Scalar, Solver, num_states, num_inputs, len_horizon >;
    constexpr Scalar tol = 1e-4;

    Eigen::Matrix< Scalar, num_states, num_states > Adyn = ( Eigen::Matrix< Scalar, num_states, num_states >() <<
        1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000,  0.0000000, 0.0002044, 0.0000000,
        0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0250000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0250000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0250000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0122625, 0.0000000,
        0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000,  0.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 1.0000000, 0.0000000,
        0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,  0.0000000, 0.0000000, 1.0000000 ).finished();
    Eigen::Matrix< Scalar, num_states, num_inputs > Bdyn = ( Eigen::Matrix< Scalar, num_states, num_inputs >() <<
         -0.0007069,   0.0007773,  0.0007091,  -0.0007795,
          0.0007034,   0.0007747, -0.0007042,  -0.0007739,
          0.0052554,   0.0052554,  0.0052554,   0.0052554,
         -0.1720966,  -0.1895213,  0.1722891,   0.1893288,
         -0.1729419,   0.1901740,  0.1734809,  -0.1907131,
          0.0123423,  -0.0045148, -0.0174024,   0.0095748,
         -0.0565520,   0.0621869,  0.0567283,  -0.0623632,
          0.0562756,   0.0619735, -0.0563386,  -0.0619105,
          0.2102143,   0.2102143,  0.2102143,   0.2102143,
        -13.7677303, -15.1617018, 13.7831318,  15.1463003,
        -13.8353509,  15.2139209, 13.8784751, -15.2570451,
          0.9873856,  -0.3611820, -1.3921880,   0.7659845 ).finished();
    Eigen::Matrix< Scalar, num_states, 1 > Q{ 100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, 4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000 };
    Eigen::Matrix< Scalar, num_inputs, 1 > R{ 4.0, 4.0, 4.0, 4.0 };

    Eigen::Matrix< Scalar, num_states, len_horizon >     x_min = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( -5 );
    Eigen::Matrix< Scalar, num_states, len_horizon >     x_max = Eigen::Matrix< Scalar, num_states, len_horizon >::Constant( 5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_min = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( -0.5 );
    Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 > u_max = Eigen::Matrix< Scalar, num_inputs, len_horizon - 1 >::Constant( 0.5 );

    Solver solver( Adyn, Bdyn, Q, R, rho_value, x_min, x_max, u_min, u_max, true );

    auto tiny_settings = solver.get_settings();
    solver.update_settings( tiny_settings.abs_pri_tol,
                            tiny_settings.abs_dua_tol,
                            100,
                            tiny_settings.check_termination,
                            tiny_settings.en_state_bound,
                            tiny_settings.en_input_bound );

    const char* base_path = DATASET_PATH;
    const char* rel_path = "opt-control/fig8xl.csv";
    char full_path[256];
    ENTO_DEBUG( "================\n" );
    ENTO_DEBUG( "Running test_linear_trajectory..." );
    if ( !EntoUtil::build_file_path( base_path, rel_path, full_path, sizeof( full_path )))
    {
        ENTO_DEBUG( "ERROR! Could not build file path for test_linear_trajectory!" );
    }
    Problem problem( solver );

    ENTO_DEBUG( "File path: %s", full_path );
    EntoUtil::ExperimentIO fileIO( full_path, "fig8xl_realpath.csv" );

    int num_experiments = 0;
    while ( fileIO.read_next( problem ))
    {
        problem.solve();
        num_experiments++;
    }
    ENTO_DEBUG( "experiments run: %d", num_experiments );
    // ENTO_TEST_CHECK_INT_EQ( num_experiments, 1 );

    if ( num_experiments > 0 ) {
        problem.validate();
    }

    fileIO.write_results( problem );
}

int main ( int argc, char ** argv )
{
    using namespace EntoUtil;
    int __n;
    if ( argc > 1 )
    {
        __n = atoi( argv[1] );
    }
    else
    {
        // For the case we are running on the MCU and we can't pass in args
        // the same way args are passed for a native build.
        __ento_replace_file_suffix( __FILE__, "test_p3p_cmdline.txt" );
        __n = __ento_get_test_num_from_file( __ento_cmdline_args_path_buffer );
    }

    if ( __ento_test_num( __n, 1 )) test_linear_trajectory_double();
    if ( __ento_test_num( __n, 2 )) test_linear_trajectory_float();
    if ( __ento_test_num( __n, 3 )) test_fig8_trajectory_double();
    if ( __ento_test_num( __n, 4 )) test_fig8large_trajectory_double();
    if ( __ento_test_num( __n, 5 )) test_fig8xl_trajectory_double();
}
