#pragma once

#include <Eigen/Dense>

Eigen::IOFormat TinyApiFmt(4, 0, ", ", "\n", "[", "]");

//template <typename Scalar, int StateDim, int InputDim, int Horizon>
//struct Solution
//{
//}


template< typename Scalar_t, int NSTATES, int NINPUTS, int NHORIZON >
class TinyMPCSolver
{
  private:
    struct Settings
    {
        Scalar_t abs_pri_tol;
        Scalar_t abs_dua_tol;
        int max_iter;
        bool check_termination;
        bool en_state_bound;
        bool en_input_bound;
        Settings() :
          abs_pri_tol( 1e-03 ),
          abs_dua_tol( 1e-03),
          max_iter( 1000 ),
          check_termination( true ),
          en_state_bound( true ),
          en_input_bound( true )
        {}
    };
    Settings m_settings;

    struct Solution
    {
        int iter;
        bool solved;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > x;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 > u;
        Solution() :
          iter( 0 ),
          solved( false ),
          x( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          u( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() )
        {}
    };
    Solution m_soln;

    struct Work
    {
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > x;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 > u;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > q;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1> r;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > p;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1> d;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > v;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > vnew;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 > z;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 > znew;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > g;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1> y;
        Eigen::Matrix< Scalar_t, NSTATES, 1 > Q;
        Eigen::Matrix< Scalar_t, NINPUTS, 1 > R;
        Eigen::Matrix< Scalar_t, NSTATES, NSTATES > Adyn;
        Eigen::Matrix< Scalar_t, NSTATES, NINPUTS > Bdyn;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > x_min;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > x_max;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 > u_min;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 > u_max;
        Eigen::Matrix< Scalar_t, NSTATES, NHORIZON > Xref;
        Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 > Uref;
        Eigen::Matrix< Scalar_t, NINPUTS, 1 > Qu;
        Scalar_t primal_residual_state;
        Scalar_t primal_residual_input;
        Scalar_t dual_residual_state;
        Scalar_t dual_residual_input;
        int status;
        int iter;
        Work( const Eigen::Matrix< Scalar_t, NSTATES, NSTATES >& Adyn,
              const Eigen::Matrix< Scalar_t, NSTATES, NINPUTS >& Bdyn,
              const Eigen::Matrix< Scalar_t, NSTATES, 1 >& Q,
              const Eigen::Matrix< Scalar_t, NINPUTS, 1 >& R,
              Scalar_t rho,
              const Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >& x_min,
              const Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >& x_max,
              const Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >& u_min,
              const Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >& u_max ) :
          x( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          u( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() ),
          q( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          r( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() ),
          p( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          d( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() ),
          v( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          vnew( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          z( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() ),
          znew( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() ),
          g( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          y( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() ),
          Q( Q + Eigen::Matrix< Scalar_t, NSTATES, 1 >::Constant( rho ) ),
          R( R + Eigen::Matrix< Scalar_t, NINPUTS, 1 >::Constant( rho ) ),
          Adyn( Adyn ),
          Bdyn( Bdyn ),
          x_min( x_min ),
          x_max( x_max ),
          u_min( u_min ),
          u_max( u_max ),
          Xref( Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero() ),
          Uref( Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero() ),
          Qu( Eigen::Matrix< Scalar_t, NINPUTS, 1 >::Zero() ),
          primal_residual_state( 0 ),
          primal_residual_input( 0 ),
          dual_residual_state( 0 ),
          dual_residual_input( 0 ),
          status( 0 ),
          iter( 0 )
        {}
    };
    Work m_work;

    struct Cache
    {
        Scalar_t rho;
        Eigen::Matrix< Scalar_t, NINPUTS, NSTATES > Kinf;
        Eigen::Matrix< Scalar_t, NSTATES, NSTATES > Pinf;
        Eigen::Matrix< Scalar_t, NINPUTS, NINPUTS > Quu_inv;
        Eigen::Matrix< Scalar_t, NSTATES, NSTATES > AmBKt;
        Cache( const Eigen::Matrix< Scalar_t, NSTATES, NSTATES >& Adyn,
               const Eigen::Matrix< Scalar_t, NSTATES, NINPUTS >& Bdyn,
               const Eigen::Matrix< Scalar_t, NSTATES, 1 >& Q,
               const Eigen::Matrix< Scalar_t, NINPUTS, 1 >& R,
               Scalar_t r, bool verbose ) :
          rho( r )
        {
            Eigen::Matrix< Scalar_t, NSTATES, NSTATES > Q1 = ( Q + Eigen::Matrix< Scalar_t, NSTATES, 1 >::Constant( 2 * rho ) ).asDiagonal();
            Eigen::Matrix< Scalar_t, NINPUTS, NINPUTS > R1 = ( R + Eigen::Matrix< Scalar_t, NINPUTS, 1 >::Constant( 2 * rho ) ).asDiagonal();

#ifdef NATIVE
            if( verbose ) {
                std::cout << "A = " << Adyn.format(TinyApiFmt) << std::endl;
                std::cout << "B = " << Bdyn.format(TinyApiFmt) << std::endl;
                std::cout << "Q = " << Q1.format(TinyApiFmt) << std::endl;
                std::cout << "R = " << R1.format(TinyApiFmt) << std::endl;
                std::cout << "rho = " << rho << std::endl;
            }
#endif

            // Riccati recursion to get Kinf, Pinf
            Eigen::Matrix< Scalar_t, NINPUTS, NSTATES > Ktp1 = Eigen::Matrix< Scalar_t, NINPUTS, NSTATES >::Zero();
            Eigen::Matrix< Scalar_t, NSTATES, NSTATES > Ptp1 = Eigen::Matrix< Scalar_t, NSTATES, 1 >::Constant( rho ).asDiagonal();
            Kinf = Eigen::Matrix< Scalar_t, NINPUTS, NSTATES >::Zero();
            Pinf = Eigen::Matrix< Scalar_t, NSTATES, NSTATES >::Zero();

            for ( int i = 0; i < 1000; i++ ) {
                Kinf = ( R1 + Bdyn.transpose() * Ptp1 * Bdyn ).inverse() * Bdyn.transpose() * Ptp1 * Adyn;
                Pinf = Q1 + Adyn.transpose() * Ptp1 * (Adyn - Bdyn * Kinf);
                // if Kinf converges, break
                if ( ( Kinf - Ktp1 ).cwiseAbs().maxCoeff() < 1e-5 )
                {
#ifdef NATIVE
                    if ( verbose ) {
                        std::cout << "Kinf converged after " << i + 1 << " iterations" << std::endl;
                    }
#endif
                    break;
                }
                Ktp1 = Kinf;
                Ptp1 = Pinf;
            }

            // Compute cached matrices
            Quu_inv = ( R1 + Bdyn.transpose() * Pinf * Bdyn ).inverse();
            AmBKt = ( Adyn - Bdyn * Kinf ).transpose();

#ifdef NATIVE
            if ( verbose ) {
                std::cout << "Kinf = " << Kinf.format(TinyApiFmt) << std::endl;
                std::cout << "Pinf = " << Pinf.format(TinyApiFmt) << std::endl;
                std::cout << "Quu_inv = " << Quu_inv.format(TinyApiFmt) << std::endl;
                std::cout << "AmBKt = " << AmBKt.format(TinyApiFmt) << std::endl;

                std::cout << "\nPrecomputation finished!\n" << std::endl;
            }
#endif
        }
    };
    Cache m_cache;


    void backward_pass_grad()
    {
        for ( int i = NHORIZON - 2; i >= 0; i-- ) {
            m_work.d.col( i ).noalias() = m_cache.Quu_inv * ( m_work.Bdyn.transpose() * m_work.p.col( i + 1 ) + m_work.r.col( i ) );
            m_work.p.col( i ).noalias() = m_work.q.col( i )
                                          + m_cache.AmBKt.lazyProduct( m_work.p.col( i + 1 ) )
                                          - m_cache.Kinf.transpose().lazyProduct( m_work.r.col( i ) ); 
        }
    }

    void forward_pass()
    {
        for ( int i = 0; i < NHORIZON - 1; i++ ) {
            m_work.u.col( i ).noalias() = -m_cache.Kinf.lazyProduct( m_work.x.col( i ) )
                                          - m_work.d.col( i );
            m_work.x.col( i + 1 ).noalias() = m_work.Adyn.lazyProduct( m_work.x.col( i ) )
                                              + m_work.Bdyn.lazyProduct( m_work.u.col( i ) );
        }
    }

    void update_slack()
    {
        m_work.znew = m_work.u + m_work.y;
        m_work.vnew = m_work.x + m_work.g;

        // Box constraints on input
        if ( m_settings.en_input_bound ) {
            m_work.znew = m_work.u_max.cwiseMin( m_work.u_min.cwiseMax( m_work.znew ) );
        }

        // Box constraints on state
        if ( m_settings.en_state_bound ) {
            m_work.vnew = m_work.x_max.cwiseMin( m_work.x_min.cwiseMax( m_work.vnew ) );
        }
    }

    void update_dual()
    {
        m_work.y = m_work.y + m_work.u - m_work.znew;
        m_work.g = m_work.g + m_work.x - m_work.vnew;
    }

    void update_linear_cost()
    {
        m_work.r = -( m_work.Uref.array().colwise() * m_work.R.array() );
        m_work.r.noalias() -= m_cache.rho * ( m_work.znew - m_work.y );
        m_work.q = -( m_work.Xref.array().colwise() * m_work.Q.array() );
        m_work.q.noalias() -= m_cache.rho * ( m_work.vnew - m_work.g );
        m_work.p.col( NHORIZON - 1 ) = -( m_work.Xref.col( NHORIZON - 1 ).transpose().lazyProduct( m_cache.Pinf ) );
        m_work.p.col( NHORIZON - 1 ).noalias() -= m_cache.rho * ( m_work.vnew.col( NHORIZON - 1 ) - m_work.g.col( NHORIZON - 1 ) );
    }

    bool termination_condition()
    {
        if ( m_work.iter % m_settings.check_termination == 0 ) {
            m_work.primal_residual_state = ( m_work.x - m_work.vnew ).cwiseAbs().maxCoeff();
            m_work.dual_residual_state = ( ( m_work.v - m_work.vnew ).cwiseAbs().maxCoeff() ) * m_cache.rho;
            m_work.primal_residual_input = ( m_work.u - m_work.znew ).cwiseAbs().maxCoeff();
            m_work.dual_residual_input = ( ( m_work.z - m_work.znew ).cwiseAbs().maxCoeff() ) * m_cache.rho;

            if ( m_work.primal_residual_state < m_settings.abs_pri_tol &&
                 m_work.primal_residual_input < m_settings.abs_pri_tol &&
                 m_work.dual_residual_state < m_settings.abs_dua_tol &&
                 m_work.dual_residual_input < m_settings.abs_dua_tol ) {
                return true;                 
            }
        }
        return false;
    }

  public:
    // replace setup
    TinyMPCSolver( const Eigen::Matrix< Scalar_t, NSTATES, NSTATES >& Adyn,
                   const Eigen::Matrix< Scalar_t, NSTATES, NINPUTS >& Bdyn,
                   const Eigen::Matrix< Scalar_t, NSTATES, 1 >& Q,
                   const Eigen::Matrix< Scalar_t, NINPUTS, 1 >& R,
                   Scalar_t rho,
                   const Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >& x_min,
                   const Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >& x_max,
                   const Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >& u_min,
                   const Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >& u_max,
                   bool verbose ) :
      m_settings( Settings() ),
      m_soln( Solution() ),
      m_work( Work( Adyn, Bdyn, Q, R, rho, x_min, x_max, u_min, u_max ) ),
      m_cache( Cache( Adyn, Bdyn, Q, R, rho, verbose ) )
    {}

    void solve()
    {
        // Initialize variables
        m_soln.solved = false;
        m_soln.iter = 0;
        m_work.status = 11; // TINY_UNSOLVED
        m_work.iter = 0;

        for ( int i = 0; i < m_settings.max_iter; i++ ) {
            // Solve linear system with Riccati and roll out to get new trajectory
            forward_pass();

            // Project slack variables into feasible domain
            update_slack();

            // Compute next iteration of dual variables
            update_dual();

            // Update linear control cost terms using reference trajectory, duals, and slack variables
            update_linear_cost();

            m_work.iter += 1;

            // Check for whether cost is minimized by calculating residuals
            if ( termination_condition() ) {
                m_work.status = 1; // TINY_SOLVED

                // Save solution
                m_soln.iter = m_work.iter;
                m_soln.solved = true;
                m_soln.x = m_work.vnew;
                m_soln.u = m_work.znew;
                return;
            }

            // Save previous slack variables
            m_work.v = m_work.vnew;
            m_work.z = m_work.znew;

            backward_pass_grad();
        }
        m_soln.iter = m_work.iter;
        m_soln.solved = false;
        m_soln.x = m_work.vnew;
        m_soln.u = m_work.znew;
    }

    void update_settings( Scalar_t abs_pri_tol, Scalar_t abs_dua_tol, int max_iter,
                          bool check_termination, bool en_state_bound, bool en_input_bound )
    {
        m_settings.abs_pri_tol = abs_pri_tol;
        m_settings.abs_dua_tol = abs_dua_tol;
        m_settings.max_iter = max_iter;
        m_settings.check_termination = check_termination;
        m_settings.en_state_bound = en_state_bound;
        m_settings.en_input_bound = en_input_bound;
    }

    const Settings& get_settings() const
    {
        return m_settings;
    }

    void set_x0( const Eigen::Matrix< Scalar_t, NSTATES, 1 >& x0 )
    {
        m_work.x.col( 0 ) = x0;
    }

    Eigen::Matrix< Scalar_t, NSTATES, 1 > get_x0()
    {
        return m_work.x.col( 0 );
    }

    Eigen::Matrix< Scalar_t, NINPUTS, 1 > get_u0() const
    {
        return m_work.u.col( 0 );
    }

    void set_x_ref( const Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >& x_ref )
    {
        m_work.Xref = x_ref;
    }

    void set_u_ref( const Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON >& u_ref )
    {
        m_work.Uref = u_ref;
    }

    void reset_duals()
    {
        m_work.y = Eigen::Matrix< Scalar_t, NINPUTS, NHORIZON - 1 >::Zero();
        m_work.g = Eigen::Matrix< Scalar_t, NSTATES, NHORIZON >::Zero();
    }

    Eigen::Matrix< Scalar_t, NSTATES, NSTATES > get_Adyn() const
    {
        return m_work.Adyn;
    }

    Eigen::Matrix< Scalar_t, NSTATES, NINPUTS > get_Bdyn() const
    {
        return m_work.Bdyn;
    }
};
