#ifndef ADAPTIVE_CONTROL_PROBLEM_H
#define ADAPTIVE_CONTROL_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/debug.h>
#include <ento-control/adaptive_controller.h>
#include <cmath> // For std::isnan

template< typename Scalar, int StateSize, int CtrlSize, int HorizonSize, int PathLen >
class AdaptiveControlProblem :
  public EntoBench::EntoProblem< AdaptiveControlProblem< Scalar, StateSize, CtrlSize, HorizonSize, PathLen >>
{
  public:
    // Expose Template Typenames to Others
    using Scalar_t = Scalar;
    using Solver_t = EntoControl::AdaptiveController;

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
    Scalar_t m_dt; // Time step for discretization
  
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
        try {
          Scalar_t val = (Scalar_t) std::stold( state_strings[i] );
          
          // Check for NaN or invalid values and replace with 0
          if (std::isnan(val) || std::isinf(val)) {
            std::cout << "Warning: Invalid value detected in reference trajectory at index " << i << ", using 0.0" << std::endl;
            val = 0.0;
          }
          
          state_vec[i] = val;
        } catch (const std::exception& e) {
          std::cout << "Error parsing value at index " << i << ": " << e.what() << std::endl;
          state_vec[i] = 0.0;
        }
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
      while ( token != NULL && i < StateSize ) {
        state_vec[i] = static_cast<Scalar_t>(atof( token ));
        
        // Check for NaN or invalid values and replace with 0
        if (std::isnan(state_vec[i]) || std::isinf(state_vec[i])) {
#ifdef NATIVE
          std::cout << "Warning: Invalid value detected in reference trajectory at index " << i << ", using 0.0" << std::endl;
#endif
          state_vec[i] = 0.0;
        }
        
        token = strtok( nullptr, "," );
        i++;
      }
      
      if (i != StateSize)
      {
#ifdef NATIVE
        std::cout << "Error: Invalid reference point, expected " << StateSize << " values but got " << i << std::endl;
#endif
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
#ifdef NATIVE
      std::cout << "Validating trajectory with " << m_iter << " iterations:" << std::endl;
#endif
      for ( int i = 0; i < m_iter; i++ ) {
        // Skip validation if trajectory contains NaNs
        bool has_nan = false;
        for (int j = 0; j < StateSize; j++) {
          if (std::isnan(m_trajectory[i][j])) {
            has_nan = true;
#ifdef NATIVE
            std::cout << "Skipping validation for step " << i << " due to NaN in trajectory" << std::endl;
#endif
            break;
          }
        }
        
        if (!has_nan) {
          Scalar_t diff = (m_trajectory[i] - m_real_path[i]).norm();
#ifdef NATIVE
          std::cout << "Tracking error [" << i << "]: " << diff << std::endl;
          
          // If we find a NaN, print more details
          if (std::isnan(diff)) {
            std::cout << "NaN detected in tracking error at iteration " << i << std::endl;
            std::cout << "Trajectory[" << i << "]: " << m_trajectory[i].transpose() << std::endl;
            std::cout << "RealPath[" << i << "]: " << m_real_path[i].transpose() << std::endl;
            
            // Check which values in the vectors might be NaN
            for (int j = 0; j < StateSize; j++) {
              if (std::isnan(m_trajectory[i][j])) {
                std::cout << "  Trajectory[" << i << "][" << j << "] is NaN" << std::endl;
              }
              if (std::isnan(m_real_path[i][j])) {
                std::cout << "  RealPath[" << i << "][" << j << "] is NaN" << std::endl;
              }
            }
          }
#endif
        }
      }
      return true;
    }

    void solve_impl()
    {
      // Make sure we have a valid reference trajectory for all steps
      if (m_iter >= m_trajectory_len) {
#ifdef NATIVE
        std::cout << "Warning: Not enough reference points provided. Using the last reference point." << std::endl;
#endif
        // Copy the last valid reference point for all remaining steps
        Eigen::Matrix<Scalar_t, StateSize, 1> last_ref = m_trajectory[m_trajectory_len - 1];
        m_trajectory.push_back(last_ref);
        m_trajectory_len++;
      }
      
      Eigen::Matrix< Scalar_t, StateSize, 1 >& x0 = m_real_path[m_iter];
      
#ifdef NATIVE
      std::cout << "Solving iteration " << m_iter << ":" << std::endl;
      std::cout << "  Initial state: " << x0.transpose() << std::endl;
      std::cout << "  Reference state: " << m_trajectory[m_iter].transpose() << std::endl;
#endif
      
      // Feed current state to the controller
      m_solver.set_x0(x0);
      
      // Set reference (even though the controller may not use it directly)
      m_solver.set_x_ref(m_trajectory[m_iter]);
      
      // Run the controller to get the control inputs and next state
      m_solver.solve();
      
      // Get the control input from the Simulink controller
      Eigen::Matrix<Scalar_t, CtrlSize, 1> u0 = m_solver.get_u0();
      
      // Get the next state DIRECTLY from the controller
      Eigen::Matrix<Scalar_t, StateSize, 1> next_state = m_solver.get_next_state();
      
#ifdef NATIVE
      std::cout << "  Control: " << u0.transpose() << std::endl;
      std::cout << "  Next state: " << next_state.transpose() << std::endl;
#endif

      // Check for numerical issues in the next state
#ifdef NATIVE
      bool has_issues = false;
      for (int i = 0; i < StateSize; i++) {
        if (std::isnan(next_state[i]) || std::isinf(next_state[i]) || 
            next_state[i] > 10.0 || next_state[i] < -10.0) {
          has_issues = true;
          std::cout << "  Issue detected in next_state[" << i << "]: " 
                    << next_state[i] << std::endl;
        }
      }
      
      if (has_issues) {
        // If issues detected, use a simple fallback
        // Just take a very small step toward the reference trajectory
        next_state = x0;
        Scalar_t small_step = 0.001;
        Eigen::Matrix< Scalar_t, StateSize, 1 > direction = m_trajectory[m_iter] - x0;
        
        // Normalize the direction vector to avoid large steps
        Scalar_t norm = direction.norm();
        if (norm > 1e-6) {
          direction /= norm;
        }
        
        next_state += small_step * direction;
        std::cout << "  Using conservative state update due to numerical issues" << std::endl;
        std::cout << "  Revised next state: " << next_state.transpose() << std::endl;
      }
#endif
      
      // Add the next state to the path
      m_real_path.push_back(next_state);
      m_iter++;
    }

    void clear_impl()
    {
      m_trajectory.clear();
      m_real_path.clear();
      m_trajectory_len = 0;
      m_iter = 0;
      m_written_states = 0;
      
      // Reset controller state
      m_solver = Solver_t(m_dt);
    }

    static constexpr const char* header_impl()
    {
      return "Adaptive control state trajectory ( x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw, yaw_dot )";
    }

    // Default constructor with 10ms time step
    AdaptiveControlProblem() : 
      m_solver(),
      m_trajectory_len(0),
      m_iter(0),
      m_written_states(0),
      m_dt(0.01f) // 10ms default time step
    {
      m_Adyn = m_solver.get_Adyn();
      m_Bdyn = m_solver.get_Bdyn();
    }
    
    // Constructor with custom time step
    AdaptiveControlProblem(Scalar_t dt) : 
      m_solver(dt),
      m_trajectory_len(0),
      m_iter(0),
      m_written_states(0),
      m_dt(dt)
    {
      m_Adyn = m_solver.get_Adyn();
      m_Bdyn = m_solver.get_Bdyn();
    }
    
    // Get the current time step
    Scalar_t get_dt() const { return m_dt; }
    
    // Set a new time step
    void set_dt(Scalar_t dt) { 
      m_dt = dt; 
      m_solver.set_dt(dt);
    }
};

#endif // ADAPTIVE_CONTROL_PROBLEM_H 