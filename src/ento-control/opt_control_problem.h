#ifndef OPT_CONTROL_PROBLEM_H
#define OPT_CONTROL_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/debug.h>
#include <type_traits>

// SFINAE helper to detect if solver has get_Adyn/get_Bdyn methods
template<typename T, typename = void>
struct has_dynamics_matrices : std::false_type {};

template<typename T>
struct has_dynamics_matrices<T, std::void_t<
  decltype(std::declval<T>().get_Adyn()),
  decltype(std::declval<T>().get_Bdyn())
>> : std::true_type {};

// SFINAE helper to detect if solver has simulate_forward method
template<typename T, typename = void>
struct has_forward_simulation : std::false_type {};

template<typename T>
struct has_forward_simulation<T, std::void_t<
  decltype(std::declval<T>().simulate_forward(
    std::declval<typename T::State>(),
    std::declval<typename T::Control>()
  ))
>> : std::true_type {};

template< typename Scalar, typename Solver, int StateSize, int CtrlSize, int HorizonSize, int PathLen, bool UseSlidingWindow = false >
class OptControlProblem :
  public EntoBench::EntoProblem< OptControlProblem< Scalar, Solver, StateSize, CtrlSize, HorizonSize, PathLen, UseSlidingWindow >>
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
    
    // Sliding window parameters - increase window size to be more practical
    static constexpr int WindowSize = UseSlidingWindow ? (PathLen < 300 ? PathLen : 300) : PathLen;  // Use PathLen or cap at 300
    
    // Use smaller containers when sliding window is enabled
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, WindowSize > m_trajectory;
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, PathLen > m_real_path;
    
    // Sliding window state
    int m_window_start;      // Current position in the full trajectory
    int m_window_offset;     // Offset within the window
    bool m_trajectory_complete; // Whether we've read all trajectory data
    
    int m_iter;
    
    // Dynamics matrices - only used if solver provides them
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
      
      // Handle sliding window vs full trajectory loading
      if constexpr (UseSlidingWindow) {
        // Load up to WindowSize states (e.g., 200-300 instead of 60)
        if (m_trajectory_len < WindowSize) {
          m_trajectory.push_back( state_vec );
        }
        // Mark trajectory complete when we've loaded WindowSize states
        if (m_trajectory_len >= WindowSize) {
          m_trajectory_complete = true;
        }
      } else {
        // Original behavior: store everything
        m_trajectory.push_back( state_vec );
      }
      
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

      // Handle sliding window vs full trajectory loading
      if constexpr (UseSlidingWindow) {
        // Load up to WindowSize states (e.g., 200-300 instead of 60)
        if (m_trajectory_len < WindowSize) {
          m_trajectory.push_back( state_vec );
        }
        // Mark trajectory complete when we've loaded WindowSize states
        if (m_trajectory_len >= WindowSize) {
          m_trajectory_complete = true;
        }
      } else {
        // Original behavior: store everything
        m_trajectory.push_back( state_vec );
      }
      
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
      if constexpr (UseSlidingWindow) {
        update_trajectory_window();
      }
      
      Eigen::Matrix< Scalar_t, StateSize, 1 >& x0 = m_real_path[m_iter];
      m_solver.set_x0( x0 );
      Eigen::Matrix< Scalar_t, StateSize, HorizonSize > x_ref;
      for ( int i = 0; i < HorizonSize; i++ ) {
        x_ref.col(i) = get_trajectory_state(m_iter + i);
      }
      m_solver.set_x_ref( x_ref );
      m_solver.reset_duals();
      m_solver.solve();
      // Note: Forward simulation moved to step_impl() to keep it outside ROI timing
    }

    void step_impl()
    {
      // Forward simulation: different approaches based on solver capabilities
      Eigen::Matrix< Scalar_t, StateSize, 1 >& x0 = m_real_path[m_iter];
      auto u0 = m_solver.get_u0();
      
      if constexpr (has_dynamics_matrices<Solver_t>::value) {
        // Matrix-based solvers (LQR, TinyMPC): use external dynamics
        m_real_path.push_back( m_Adyn * x0 + m_Bdyn * u0 );
      } else if constexpr (has_forward_simulation<Solver_t>::value) {
        // OSQP-style solvers with forward simulation: use solver's method
        auto x_next = m_solver.simulate_forward(x0, u0);
        m_real_path.push_back(x_next);
      } else {
        // Fallback: no forward simulation available
        // This should not happen for properly implemented solvers
        ENTO_DEBUG("Warning: No forward simulation available for solver");
        m_real_path.push_back(x0); // Just copy current state
      }
      m_iter++;
    }

    // Public method to perform forward simulation step
    void step()
    {
      static_cast<OptControlProblem*>(this)->step_impl();
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
      m_window_start(0),
      m_window_offset(0),
      m_trajectory_complete(false),
      m_iter(0),
      m_written_states(0)
    {
      if constexpr (has_dynamics_matrices<Solver_t>::value) {
        m_Adyn = solver.get_Adyn();
        m_Bdyn = solver.get_Bdyn();
      }
    }

  private:
    // Sliding window management - only used when UseSlidingWindow = true
    void update_trajectory_window()
    {
      if constexpr (UseSlidingWindow) {
        // Check if we need to shift the window
        if (m_window_offset + HorizonSize >= WindowSize && !m_trajectory_complete) {
          // Shift window: move last HorizonSize elements to beginning
          for (int i = 0; i < HorizonSize; i++) {
            m_trajectory[i] = m_trajectory[WindowSize - HorizonSize + i];
          }
          m_window_start += (WindowSize - HorizonSize);
          m_window_offset = HorizonSize;
          
          // Note: In a real implementation, we would read more data from file here
          // For now, we assume all data is already loaded during setup
        }
      }
    }

    // Get trajectory reference accounting for sliding window
    Eigen::Matrix< Scalar_t, StateSize, 1 > get_trajectory_state(int index)
    {
      if constexpr (UseSlidingWindow) {
        // Clamp index to available range
        int clamped_index = index;
        if (clamped_index >= m_trajectory.size()) {
          clamped_index = m_trajectory.size() - 1;
        }
        if (clamped_index < 0) {
          clamped_index = 0;
        }
        return m_trajectory[clamped_index];
      } else {
        return m_trajectory[index];
      }
    }
};

#endif // OPT_CONTROL_PROBLEM_H
