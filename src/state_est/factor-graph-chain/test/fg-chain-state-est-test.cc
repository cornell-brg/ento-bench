#include "CircularBuffer.h"
#include "FgChainUnicycleStateEst.h"
#include "LinearBuffer.h"

#include "sample-path.h"

#include <array>
#include <Eigen/Dense>

int main()
{
    using StatesType = LinearBuffer< Eigen::Matrix< float, 4, 1 >, 10, path_len >;
    StatesType states_1;
    StatesType states_5;
    StatesType states_10;
    FgChainUnicycleStateEst< float, 10, StatesType, 5 > estimator_1;
    FgChainUnicycleStateEst< float, 10, StatesType, 5 > estimator_5;
    FgChainUnicycleStateEst< float, 10, StatesType, 5 > estimator_10;

    for ( int i = 0; i < path_len; i++ ) {
        estimator_1.update( states_1, observations[i], 1 );
    }

    for ( int i = 0; i < path_len; i++ ) {
        estimator_5.update( states_5, observations[i], 1 );
    }

    for ( int i = 0; i < path_len; i++ ) {
        estimator_10.update( states_10, observations[i], 1 );
    }

    auto state_array = states_1.get_array();
    printf( "final state iter=1:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );

    state_array = states_5.get_array();
    printf( "final state iter=5:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
    printf( "\n" );

    state_array = states_10.get_array();
    printf( "final state iter=10:\n" );
    printf( "           x          y          v      theta\n" );
    for ( int i = 0; i < state_array.size(); i++ ) {
        printf( "  %10f %10f %10f %10f\n",
                state_array[i](0), state_array[i](1), state_array[i](2), state_array[i](3) );
    }
}
