#include "CircularBuffer.h"
#include "FgChainUnicycleStateEst.h"
#include "LinearBuffer.h"
#include "ento-util/unittest.h"

#include "sample-path.h"

#include <array>
#include <Eigen/Dense>

void test_fg_chain_est_1_iter_float()
{
    using StatesType = LinearBuffer< Eigen::Matrix< float, 4, 1 >, 10, path_len >;
    StatesType states_1;
    FgChainUnicycleStateEst< float, 10, StatesType,   1 > estimator_1;

    for ( int i = 0; i < path_len; i++ ) {
        estimator_1.update( states_1, observations[i], 1 );
    }

    auto state_array = states_1.get_array();
    printf( "final state iter=1:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );
}

void test_fg_chain_est_5_iter_float()
{
    using StatesType = LinearBuffer< Eigen::Matrix< float, 4, 1 >, 10, path_len >;
    StatesType states_5;
    FgChainUnicycleStateEst< float, 10, StatesType,   5 > estimator_5;
    for ( int i = 0; i < path_len; i++ ) {
        estimator_5.update( states_5, observations[i], 1 );
    }

    state_array = states_5.get_array();
    printf( "final state iter=5:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );
}

void test_fg_chain_est_10_iter_float()
{
    using StatesType = LinearBuffer< Eigen::Matrix< float, 4, 1 >, 10, path_len >;
    StatesType states_10;
    FgChainUnicycleStateEst< float, 10, StatesType,  10 > estimator_10;

    for ( int i = 0; i < path_len; i++ ) {
        estimator_10.update( states_10, observations[i], 1 );
    }

    state_array = states_10.get_array();
    printf( "final state iter=10:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );
}

void test_fg_chain_est_50_iter_float()
{
    using StatesType = LinearBuffer< Eigen::Matrix< float, 4, 1 >, 10, path_len >;
    StatesType states_50;
    FgChainUnicycleStateEst< float, 10, StatesType,  50 > estimator_50;

    for ( int i = 0; i < path_len; i++ ) {
        estimator_50.update( states_50, observations[i], 1 );
    }

    state_array = states_50.get_array();
    printf( "final state iter=50:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );
}

void test_fg_chain_est_100_iter_float()
{
    using StatesType = LinearBuffer< Eigen::Matrix< float, 4, 1 >, 10, path_len >;
    StatesType states_100;
    FgChainUnicycleStateEst< float, 10, StatesType, 100 > estimator_100;

    for ( int i = 0; i < path_len; i++ ) {
        estimator_100.update( states_100, observations[i], 1 );
    }

    state_array = states_100.get_array();
    printf( "final state iter=100:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );
}

int main()
{
    int __n = ( argc == 1 ) ? 0 : atoi( argv[1] );
    if ( __ento_test_num( __n, 1 ) ) test_fg_chain_est_1_iter_float();
    if ( __ento_test_num( __n, 2 ) ) test_fg_chain_est_5_iter_float();
    if ( __ento_test_num( __n, 3 ) ) test_fg_chain_est_10_iter_float();
    if ( __ento_test_num( __n, 4 ) ) test_fg_chain_est_50_iter_float();
    if ( __ento_test_num( __n, 5 ) ) test_fg_chain_est_100_iter_float();
}
