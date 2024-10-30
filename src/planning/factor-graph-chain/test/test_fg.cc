#include "fg_chain_interpol_unicycle.h"

#include <Eigen/Dense>

#include <cstdio>

int main()
{
/*
    printf( "Test 10 states, float\n" );
    Eigen::Matrix< float, 4, 1 > init_state;
    init_state << 0, 0, 0, 0;
    Eigen::Matrix< float, 4, 1 > final_state;
    final_state << 20, 10, 0, 0;
    FgChainInterpolUnicycle< float, 10 > interpol_f_10( init_state, final_state );
    printf( "    no heading, 10 iter\n" );
    interpol_f_10.interpolate_pos( 10, false );
    interpol_f_10.print_trajectory();
    printf( "    w/ heading, 10 iter\n" );
    interpol_f_10.interpolate_pos( 10, true );
    interpol_f_10.print_trajectory();
*/

    printf( "\n\n" );

    printf( "Test 10 states, double\n" );
    Eigen::Matrix< double, 4, 1 > init_double;
    init_double << 0, 0, 0, 0;
    Eigen::Matrix< double, 4, 1 > final_double;
    final_double << 20, 10, 0, 0;
    FgChainInterpolUnicycle< double, 10 > interpol_d_10( init_double, final_double );
    printf( "    no heading, 10 iter\n" );
    interpol_d_10.interpolate_pos( 10, false );
    interpol_d_10.print_trajectory();
    printf( "    w/ heading, 10 iter\n" );
    interpol_d_10.interpolate_pos( 10, true );
    interpol_d_10.print_trajectory();
}

