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
    StatesType states;
    Eigen::Matrix< float, 4, 1 > init_state;
    init_state << 0, 0, .01, M_PI / 4;
    FgChainUnicycleStateEst< float, 10, StatesType, 2 > estimator( states, init_state );

    for ( int i = 1; i < path_len; i++ ) {
        estimator.update( observations[i], 1 );
    }

    auto state_array = states.get_array();
    printf( "final state iter=1:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );
}

int main( int argc, char** argv )
{
    int __n = ( argc == 1 ) ? 0 : atoi( argv[1] );
    if ( EntoUtil::__ento_test_num( __n, 1 ) ) test_fg_chain_est_1_iter_float();
}
