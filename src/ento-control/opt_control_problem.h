#ifndef OPT_CONTROL_PROBLEM_H
#define OPT_CONTROL_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/debug.h>

template< typename Scalar, typename Solver, int StateSize, int CtrlSize, int HorizonSize, int PathLen >
class OptControlProblem :
  public EntoBench::EntoProblem< OptControlProblem< Scalar, Solver, StateSize, CtrlSize, HorizonSize, PathLen >>
{
  public:
    // Expose Template Typenames to Others
    using Scalar_t = Scalar;
    using Solver_t = Solver;

    // Required by Problem Interface for Experiment I/O
    static constexpr bool RequiresDataset_ = true;
    static constexpr bool SaveResults_     = false;
    static constexpr bool RequiresSetup_   = true;
    static constexpr int  SetupLines_      = HorizonSize;

  private:
    Solver_t m_solver;
    int m_trajectory_len;
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, PathLen > m_trajectory;
    int m_iter;
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, PathLen > m_real_path;
    Eigen::Matrix< Scalar_t, StateSize, StateSize > m_Adyn;
    Eigen::Matrix< Scalar_t, StateSize, CtrlSize > m_Bdyn;
    int m_written_states;
  
  public:
  #ifdef NATIVE
    std::string serialize_impl() const
    {
      std::stringstream ss;
      for ( int j = 0; j < StateSize; j++ ) {
        ss << m_real_path[m_written_states][j] << ",";
      }
      return ss.str();
    }

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
    const char* serialize_impl() const
    {
      char line[256];
      int pos = 0;
      for ( int i = 0; i < m_iter; i++ ) {
        for ( int j = 0; j < StateSize; j++ ) {
          pos += sprintf( &line[pos], "%f,", m_real_path[i][j] );
        }
        pos += sprintf( &line[pos], "\n" );
      }
      return line;
    }

    bool deserialize_impl(const char* line)
    {
      char* token;
      char* to_parse = const_cast<char*>(line);
      token = strtok( to_parse, "," );
      Eigen::Matrix< Scalar_t, StateSize, 1 > state_vec;
      int i = 0;
      while ( token != NULL ) {
        state_vec[i] = static_cast<Scalar_t>(atof( token ));
        token = strtok( nullptr, "," );
        i++;
      }
      if (i != StateSize)
      {
        return false;
      }

      m_trajectory.push_back( state_vec );
      if ( m_trajectory_len == 0 ) {
        m_real_path.push_back( state_vec );
      }
      m_trajectory_len++;
      return true;
    }
  #endif

    bool validate_impl() const
    {
      for ( int i = 0; i < m_iter; i++ ) {
        Scalar_t diff = (m_trajectory[i] - m_real_path[i]).norm();
#ifdef NATIVE
        std::cout << "tracking error: " << diff << std::endl;
#endif
      }
      return true;
    }

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

    void clear_impl()
    {}

    static constexpr const char* header_impl()
    {
      return "Optimal control state trajectory ( x, y, z, r, p, w, x_dot, y_dot, z_dot, r_dot, p_dot, w_dot )";
    }

    OptControlProblem(Solver solver) : 
      m_solver(std::move(solver)),
      m_trajectory_len(0),
      m_iter(0),
      m_written_states(0)
    {
      m_Adyn = solver.get_Adyn();
      m_Bdyn = solver.get_Bdyn();
    }
};

#endif // OPT_CONTROL_PROBLEM_H
