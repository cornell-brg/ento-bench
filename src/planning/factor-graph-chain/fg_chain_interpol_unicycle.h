#ifndef __FG_CHAIN_INTERPOL_UNICYCLE_H__
#define __FG_CHAIN_INTERPOL_UNICYCLE_H__

#include <Eigen/Dense>

#define M_PI 3.14159265358979323846

template< typename Scalar, int nstates >
class FgChainInterpolUnicycle
{
  private:
    // states are stored [ x_pos, y_pos, v, theta ]
    std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates > m_states;
    Eigen::Matrix< Scalar, 4, 1 > m_init_state;
    Eigen::Matrix< Scalar, 4, 1 > m_final_state;

    std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 > m_A_submats;
    std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates > m_err_vecs;
    std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 > m_L_submats;
    std::array< Eigen::Matrix< Scalar, 4, 4 >, ( 2 * nstates ) - 1 > m_L_inv_submats;
    std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates > m_U_vecs;
    std::array< Eigen::Matrix< Scalar, 4, 1 >, nstates > m_state_deltas;

    Scalar mod2pi( Scalar angle )
    {
        return ( angle + M_PI ) - ( ( 2 * M_PI ) * floor( ( angle + M_PI ) / ( 2 * M_PI ) ) ) - M_PI;
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

  public:
    FgChainInterpolUnicycle( Eigen::Matrix< Scalar, 4, 1 > init_state, Eigen::Matrix< Scalar, 4, 1 > final_state ) :
        m_init_state( init_state ),
        m_final_state( final_state )
    {
    }

    void interpolate_pos( int niter, bool use_heading )
    {
        // initialize with linear interpolation
        for ( int i = 0; i < nstates; i++ ) {
            Scalar alpha =  ( (Scalar) i ) / nstates;
            for ( int j = 0; j < 4; j++ ) {
                m_states[i][j] = ( ( 1 - alpha ) * m_init_state[j] ) + ( alpha * m_final_state[j] );
            }
        }

        for ( int iter = 0; iter < niter; iter++ ) {
            for ( int i = 0; i < ( ( 2 * nstates ) - 1 ); i++ ) {
                m_A_submats[i].setZero();
            }
            for ( int i = 0; i < nstates; i++ ) {
                m_err_vecs[i].setZero();
            }

            if ( !use_heading ) {
                Eigen::Matrix< Scalar, 2, 1 > r = m_init_state.head(2) - m_states[0].head(2);
                Eigen::Matrix< Scalar, 2, 4 > jac;
                jac << -1,  0, 0, 0,
                        0, -1, 0, 0;
                Eigen::Matrix< Scalar, 2, 2 > weights;
                weights << 1, 0,
                           0, 1;

                Eigen::Matrix< Scalar, 4, 2 > jtw = jac.transpose() * weights;
                m_A_submats[0] += jtw * jac;
                m_err_vecs[0]  -= jtw * r;

                r = m_final_state.head(2) - m_states[( nstates - 1 )].head(2);
                jtw = jac.transpose() * weights;
                m_A_submats[2 * ( nstates - 1 )] += jtw * jac;
                m_err_vecs[( nstates - 1 )]      -= jtw * r;
            }
            else {
                Eigen::Matrix< Scalar, 3, 1 > r = m_init_state( { 0, 1, 3 } ) - m_states[0]( { 0, 1, 3 } );
                r[2] = mod2pi( r[2] );
                Eigen::Matrix< Scalar, 3, 4 > jac;
                jac << -1,  0, 0,  0,
                        0, -1, 0,  0,
                        0,  0, 0, -1;
                Eigen::Matrix< Scalar, 3, 3 > weights;
                weights << 100,   0,   0,
                             0, 100,   0,
                             0,   0, 100;
        
                Eigen::Matrix< Scalar, 4, 3 > jtw = jac.transpose() * weights;
                m_A_submats[0] += jtw * jac;
                m_err_vecs[0]  -= jtw * r;

                r = m_final_state( { 0, 1, 3 } ) - m_states[( nstates - 1 )]( { 0, 1, 3 } );
                r[2] = mod2pi( r[2] );
                jtw = jac.transpose() * weights;
                m_A_submats[2 * ( nstates - 1 )] += jtw * jac;
                m_err_vecs[( nstates - 1 )]      -= jtw * r;
            }

            for ( int i = 0; i < ( nstates - 1 ); i++ ) {
                Scalar dt = 1;

                Eigen::Matrix< Scalar, 4, 1 > r;
                r = m_states[i + 1] - m_states[i];
                r[0] -= m_states[i][2] * dt * cos( m_states[i][3] );
                r[1] -= m_states[i][2] * dt * sin( m_states[i][3] );
                r[3] = mod2pi( r[3] );

                Eigen::Matrix< Scalar, 4, 4 > jacA;
                jacA << -1,  0, -dt * cos( m_states[i][3] ),  m_states[i][2] * dt * sin( m_states[i][3] ),
                         0, -1, -dt * sin( m_states[i][3] ), -m_states[i][2] * dt * cos( m_states[i][3] ),
                         0,  0,                     -1 / dt,                                            0,
                         0,  0,                           0,                                      -1 / dt;

                Eigen::Matrix< Scalar, 4, 4 > jacB;
                jacB << 1, 0,      0,      0,
                        0, 1,      0,      0,
                        0, 0, 1 / dt,      0,
                        0, 0,      0, 1 / dt;

                Eigen::Matrix< Scalar, 4, 4 > weights;
                weights << 1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1;

                Eigen::Matrix< Scalar, 4, 4 > jatw = jacA.transpose() * weights;
                Eigen::Matrix< Scalar, 4, 4 > jbtw = jacB.transpose() * weights;
                m_A_submats[2 * i]         += jatw * jacA;
                m_A_submats[( 2 * i ) + 1] += jatw * jacB;
                m_A_submats[2 * ( i + 1 )] += jbtw * jacB;
                m_err_vecs[i]                  -= jatw * r;
                m_err_vecs[i+1]                -= jbtw * r;
            }

            m_L_submats[0] = m_A_submats[0].llt().matrixL();
            m_L_inv_submats[0] = mat_inv( m_L_submats[0] );
            for ( int i = 1; i < ( 2 * nstates ) - 1; i++ ) {
                if ( ( i & 1 ) != 0 ) {
                    m_L_submats[i] = m_A_submats[i].transpose() * m_L_inv_submats[i - 1].transpose();
                }
                else {
                    Eigen::Matrix< Scalar, 4, 4 > LLt = m_L_submats[i - 1] * m_L_submats[i - 1].transpose();
                    m_L_submats[i] = ( m_A_submats[i] - LLt ).llt().matrixL();
                    m_L_inv_submats[i] = mat_inv( m_L_submats[i] );
                }
            }

            m_U_vecs[0] = m_L_inv_submats[0] * m_err_vecs[0];
            for ( int i = 1; i < nstates; i++ ) {
                Eigen::Matrix< Scalar, 4, 1 > LU = m_L_submats[( 2 * i ) - 1] * m_U_vecs[i - 1];
                m_U_vecs[i] = m_L_inv_submats[2 * i] * ( m_err_vecs[i] - LU );
            }

        
            m_state_deltas[( nstates - 1 )] = m_L_inv_submats[2 * ( nstates - 1 )].transpose() * m_U_vecs[( nstates - 1 )];
            for ( int i = ( nstates - 2 ); i >= 0; i-- ) {
                Eigen::Matrix< Scalar, 4, 1 > LX = m_L_submats[( 2 * i ) + 1].transpose() * m_state_deltas[i + 1];
                m_state_deltas[i] = m_L_inv_submats[2 * i].transpose() * ( m_U_vecs[i] - LX );
            }

            for ( int i = 0; i < nstates; i++ ) {
                m_states[i] += m_state_deltas[i];
            }
        }
    }

    void print_trajectory()
    {
        for ( int i = 0; i < nstates; i++ ) {
            printf( "%3d %15f %15f %15f %15f\n", i, m_states[i](0), m_states[i](1), m_states[i](2), m_states[i](3) );
        }
    }
};

#endif
