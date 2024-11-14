#ifndef __FG_CHAIN_UNICYCLE_STATE_EST_H__
#define __FG_CHAIN_UNICYCLE_STATE_EST_H__

#include "CircularBuffer.h"

#include <array>
#include <Eigen/Dense>

template< typename Scalar, int nstates, typename BufferType, int converge_count >
class FgChainUnicycleStateEst
{
  private:
    CircularBuffer< Eigen::Matrix< Scalar, 2, 1 >, nstates > m_observations;
    Scalar mod2pi( Scalar angle )
    {
        return ( angle + M_PI ) - ( ( 2 * M_PI ) * floor( ( angle + M_PI ) / ( 2 * M_PI ) ) ) - M_PI;
    }

    void unary_residual_and_jacobian( Eigen::Matrix< Scalar, 2, 1 >& observation,
                                      Eigen::Matrix< Scalar, 4, 1 >& state,
                                      Eigen::Matrix< Scalar, 2, 1 >& residual,
                                      Eigen::Matrix< Scalar, 2, 4 >& jacobian )
    {
        residual = observation - state.head(2);
        jacobian << -1,  0,  0,  0,
                     0, -1,  0,  0;
    }

    void binary_residual_and_jacobians( Eigen::Matrix< Scalar, 4, 1 >& state_i,
                                        Eigen::Matrix< Scalar, 4, 1 >& state_ip1,
                                        Scalar dt,
                                        Eigen::Matrix< Scalar, 4, 1 >& residual,
                                        Eigen::Matrix< Scalar, 4, 4 >& jacobian_i,
                                        Eigen::Matrix< Scalar, 4, 4 >& jacobian_ip1 )
    {
        residual = state_ip1 - state_i;
        residual[0] -= state_i[2] * dt * cos( state_i[3] );
        residual[1] -= state_i[2] * dt * sin( state_i[3] );
        residual[3] = mod2pi( residual[3] );

        jacobian_i << -1,  0, -dt * cos( state_i[3] ),  state_i[2] * dt * sin( state_i[3] ),
                       0, -1, -dt * sin( state_i[3] ), -state_i[2] * dt * cos( state_i[3] ),
                       0,  0,                 -1 / dt,                                    0,
                       0,  0,                       0,                              -1 / dt;

        jacobian_ip1 << 1, 0,      0,      0,
                        0, 1,      0,      0,
                        0, 0, 1 / dt,      0,
                        0, 0,      0, 1 / dt;
    }

    void jacobians_and_errors( BufferType& states,
                               Scalar dt,
                               int count,
                               std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 >& A_submats,
                               std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates >& err_vecs )
    {
        for ( int i = 0; i < ( ( 2 * nstates ) - 1 ); i++ ) {
            A_submats[i].setZero();
        }
        for ( int i = 0; i < nstates; i++ ) {
            err_vecs[i].setZero();
        }

        for ( int i = 0; i < count; i++ ) {
            Eigen::Matrix< Scalar, 2, 1 > unary_residual;
            Eigen::Matrix< Scalar, 2, 4 > unary_jacobian;
            unary_residual_and_jacobian( m_observations[i], states[i], unary_residual, unary_jacobian );
            A_submats[2 * i] += unary_jacobian.transpose() * unary_jacobian;
            err_vecs[i]      -= unary_jacobian.transpose() * unary_residual;
        }

        for ( int i = 0; i < ( count - 1 ); i++ ) {
            Eigen::Matrix< Scalar, 4, 1 > binary_residual;
            Eigen::Matrix< Scalar, 4, 4 > binary_jacobian_xi;
            Eigen::Matrix< Scalar, 4, 4 > binary_jacobian_xip1;
            binary_residual_and_jacobians( states[i], states[i + 1], dt, binary_residual, binary_jacobian_xi, binary_jacobian_xip1 );

            A_submats[2 * i]         += binary_jacobian_xi.transpose()   * binary_jacobian_xi;
            A_submats[( 2 * i ) + 1] += binary_jacobian_xi.transpose()   * binary_jacobian_xip1;
            A_submats[2 * ( i + 1 )] += binary_jacobian_xip1.transpose() * binary_jacobian_xip1;
            err_vecs[i]              -= binary_jacobian_xi.transpose()   * binary_residual;
            err_vecs[i + 1]          -= binary_jacobian_xip1.transpose() * binary_residual;
        }
    }

    // per AXLE reference, returns 0 matrix when input is singular
    Eigen::Matrix< Scalar, 4, 4 > mat_inv( Eigen::Matrix< Scalar, 4, 4 > a )
    {
        int piv[4];
        int pivsign = 1;
        bool singular = false;

        Eigen::Matrix< Scalar, 4, 4 > LU = a;

        for ( int i = 0; i < 4; i++ ) {
            piv[i] = i;
        }

        for ( int j = 0; j < 4; j++ ) {
            for ( int i = 0; i < 4; i++ ) {
                int kmax = i < j ? i : j;
                Scalar acc = 0;
                for ( int k = 0; k < kmax; k++ ) {
                    acc += LU( i, k ) * LU( k, j );
                }
                LU( i, j ) -= acc;
            }

            int p = j;
            for ( int i = j + 1; i < 4; i++ ) {
                if ( fabs( LU( i, j ) ) > fabs( LU( p, j ) ) ) {
                    p = i;
                }
            }

            if ( p != j ) {
                LU.row( p ).swap( LU.row( j ) );
                int k = piv[p];
                piv[p] = piv[j];
                piv[j] = k;
                pivsign = -pivsign;
            }

            if ( fabs( LU( j, j ) ) < 1.0e-8 ) {
                singular = true;
            }

            if ( ( j < 4 ) && ( LU( j, j ) != 0 ) ) {
                Scalar lu_jj_inv = 1.0 / LU( j, j );
                for ( int i = j + 1; i < 4; i++ ) {
                    LU( i, j ) *= lu_jj_inv;
                }
            }
        }

        Eigen::Matrix< Scalar, 4, 4 > x;
        x.setZero();
        if ( !singular ) {
            for ( int i = 0; i < 4; i++ ) {
                x( i, piv[i] ) = 1;
            }

            for ( int k = 0; k < 4; k++ ) {
                for ( int i = k + 1; i < 4; i++ ) {
                    for ( int t = 0; t < 4; t++ ) {
                        x( i, t ) += x( k, t ) * -LU( i, k );
                    }
                }
            }

            for ( int k = 3; k >= 0; k-- ) {
                for ( int t = 0; t <4; t++ ) {
                    x( k, t ) *= 1.0 / LU( k, k );
                }
                for ( int i = 0; i < k; i++ ) {
                    for ( int t = 0; t < 4; t++ ) {
                        x( i, t ) += x( k, t ) * -LU( i, k );
                    }
                }
            }
        }

        return x;
    }

    void cholesky_submats( std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 >& A_submats,
                           int count,
                           std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 >& L_submats,
                           std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 >& L_inv_submats )
    {
        L_submats[0] = A_submats[0].llt().matrixL();
        L_inv_submats[0] = mat_inv( L_submats[0] );
        for ( int i = 1; i < ( 2 * count ) - 1; i++ ) {
            if ( ( i & 1 ) != 0 ) {
                L_submats[i] = A_submats[i].transpose() * L_inv_submats[i - 1].transpose();
            }
            else {
                Eigen::Matrix< Scalar, 4, 4 > LLt = L_submats[i - 1] * L_submats[i - 1].transpose();
                L_submats[i] = ( A_submats[i] - LLt ).llt().matrixL();
                L_inv_submats[i] = mat_inv( L_submats[i] );
            }
        }
    }

    void solve_system( std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 >& L_submats,
                       std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 >& L_inv_submats,
                       std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates >& err_vecs,
                       int count,
                       std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates >& state_deltas )
    {
        std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates > u_vecs;
        u_vecs[0] = L_inv_submats[0] * err_vecs[0];
        for ( int i = 1; i < count; i++ ) {
            Eigen::Matrix< Scalar, 4, 1 > LU = L_submats[( 2 * i ) - 1] * u_vecs[i - 1];
            u_vecs[i] = L_inv_submats[2 * i] * ( err_vecs[i] - LU );
        }

        state_deltas[( count - 1 )] = L_inv_submats[2 * ( count - 1 )].transpose() * u_vecs[( count - 1 )];
        for ( int i = ( count - 2 ); i >= 0; i-- ) {
            Eigen::Matrix< Scalar, 4, 1 > LX = L_submats[( 2 * i ) + 1].transpose() * state_deltas[i + 1];
            state_deltas[i] = L_inv_submats[2 * i].transpose() * ( u_vecs[i] - LX );
        }
    }

  public:
    FgChainUnicycleStateEst()
    {}

    void update( BufferType& states,
                 Eigen::Matrix< Scalar, 2, 1 >& observation,
                 Scalar dt )
    {
        m_observations.push( observation );
        Eigen::Matrix< Scalar, 4, 1 > curr_state;
        curr_state(0) = observation(0);
        curr_state(1) = observation(1);
        curr_state(2) = 0;
        curr_state(3) = 0;
        states.push( curr_state );
        int count = ( states.size() >= nstates ) ? nstates : states.size();

        for ( int iter = 0; iter < converge_count; iter++ ) {
            std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 > A_submats;
            std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates > err_vecs;
            jacobians_and_errors( states, dt, count, A_submats, err_vecs );

            std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 > L_submats;
            std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 > L_inv_submats;
            cholesky_submats( A_submats, count, L_submats, L_inv_submats );

            std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates > state_deltas;
            solve_system( L_submats, L_inv_submats, err_vecs, count, state_deltas );

            for ( int i = 0; i < count; i++ ) {
                states[i] += state_deltas[i];
            }
        }
    }

};

#endif // __FG_CHAIN_UNICYCLE_STATE_EST_H__
