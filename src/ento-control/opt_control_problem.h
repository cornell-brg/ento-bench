#ifndef OPT_CONTROL_PROBLEM_H
#define OPT_CONTROL_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/debug.h>

template< typename Scalar, typename Solver, int StateSize, int CtrlSize, int HorizonSize >
class OptControlProblem :
  public EntoBench::EntoProblem< OptControlProblem< Scalar, Solver, StateSize, CtrlSize, HorizonSize >>
{
  public:
    // Expose Template Typenames to Others
    using Scalar_t = Scalar;
    using Solver_t = Solver;

    // Required by Problem Interface for Experiment I/O
    static constexpr bool RequiresDataset_ = true;
    static constexpr bool SavesResults_    = true;
    static constexpr bool RequiresSetup_   = true;
    static constexpr int  SetupLines_      = HorizonSize;

    Solver_t m_solver;
    const double m_tol = 0.1;
    int m_trajectory_len;
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, 0 > m_trajectory;
    int m_iter;
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, 0 > m_real_path;
    Eigen::Matrix< Scalar_t, StateSize, StateSize > m_Adyn;
    Eigen::Matrix< Scalar_t, StateSize, CtrlSize > m_Bdyn;
  
  #ifdef NATIVE
    std::string serialize_impl() const;
    bool deserialize_impl(const std::string &line)
    {
      std::stringstream ss( line );
      EntoUtil::EntoContainer< std::string, StateSize > state_strings;
      for ( int i = 0; i < StateSize; i++ ) {
        std::string line_item;
        std::getline( ss, line_item, ',' );
        state_strings.push_back( line_item );
      }

      Eigen::Matrix< Scalar_t, StateSize, 1 > state_vec;
      for ( int i = 0; i < StateSize; i++ ) {
        Scalar_t val = (Scalar_t) std::stold( state_strings[i] );
        state_vec[i] = val;
      }
      
      m_trajectory.push_back( state_vec );
      if ( m_trajectory_len == 0 ) {
        m_real_path.push_back( state_vec );
      }
      m_trajectory_len++;
      return true;
    }
  #else
    // @TODO: Add serialize implementation for embedded builds.
    const char* serialize_impl() const;
    // @TODO: Add deserialize implementation for embedded builds
    bool deserialize_impl(const char* line);
  #endif

    // @TODO: Complete validate implementation. 
    bool validate_impl() const
    {
      for ( int i = 0; i < m_iter; i++ ) {
        Scalar_t diff = (m_trajectory[i] - m_real_path[i]).norm();
        std::cout << "tracking error: " << diff << std::endl;
      }
      return true;
    }

    // @TODO: Complete solve implementation.
    void solve_impl()
    {
      Eigen::Matrix< Scalar_t, StateSize, 1 >& x0 = m_real_path[m_iter];
      m_solver.set_x0( x0 );
      Eigen::Matrix< Scalar_t, StateSize, HorizonSize > x_ref;
      for ( int i = 0; i < HorizonSize; i++ ) {
        x_ref.col(i) = m_trajectory[m_iter + i];
      }
      m_solver.set_x_ref( x_ref );
      m_solver.reset_duals();
      m_solver.solve();
      m_real_path.push_back( m_Adyn * x0 + m_Bdyn * m_solver.get_u0() );
      m_iter++;
    }

    // @TODO: Complete clear implementation.
    void clear_impl()
    {}

    static constexpr const char* header_impl()
    {
      return "Optimal control state trajectory ( x, y, z, r, p, w, x_dot, y_dot, z_dot, r_dot, p_dot, w_dot )";
    }

    OptControlProblem(Solver solver) : 
      m_solver(std::move(solver)),
      m_trajectory_len(0),
      m_iter(0)
    {
      m_Adyn = solver.get_Adyn();
      m_Bdyn = solver.get_Bdyn();
    }
};

#endif // OPT_CONTROL_PROBLEM_H