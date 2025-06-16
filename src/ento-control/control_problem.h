#ifndef CONTROL_PROBLEM_H
#define CONTROL_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/debug.h>
#include <ento-control/adaptive_controller.h>
#include <cmath> // For std::isnan

#if __cplusplus >= 202002L
#include <concepts>

// C++20 concepts for solver capabilities
template<typename T>
concept HasDynamicsMatrices = requires(T t) {
  t.get_Adyn();
  t.get_Bdyn();
};

template<typename T>
concept HasSetDt = requires(T t, float dt) {
  t.set_dt(dt);
};

#else
// C++17 SFINAE fallback for solver capabilities
template<typename T, typename = void>
struct HasDynamicsMatrices : std::false_type {};

template<typename T>
struct HasDynamicsMatrices<T, std::void_t<
  decltype(std::declval<T>().get_Adyn()),
  decltype(std::declval<T>().get_Bdyn())
>> : std::true_type {};

template<typename T, typename = void>
struct HasSetDt : std::false_type {};

template<typename T>
struct HasSetDt<T, std::void_t<
  decltype(std::declval<T>().set_dt(std::declval<float>()))
>> : std::true_type {};

#endif

template< typename Scalar, typename Solver, int StateSize, int CtrlSize, int HorizonSize, int PathLen >
class ControlProblem :
  public EntoBench::EntoProblem< ControlProblem< Scalar, Solver, StateSize, CtrlSize, HorizonSize, PathLen >>
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
    
    // Dynamics matrices - only used if solver provides them
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
      
      // Reset controller state - handle different constructor types
      if constexpr (std::is_constructible_v<Solver_t, Scalar_t>) {
        // Solver has constructor that takes dt
        m_solver = Solver_t(m_dt);
      } else {
        // Solver has default constructor
        m_solver = Solver_t();
      }
    }

    static constexpr const char* header_impl()
    {
      return "Adaptive control state trajectory ( x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw, yaw_dot )";
    }

    // Default constructor with 10ms time step
    ControlProblem() : 
      m_solver(Scalar_t(0.01)),  // Always pass dt to solver constructor
      m_trajectory_len(0),
      m_iter(0),
      m_written_states(0),
      m_dt(0.01f) // 10ms default time step
    {
      if constexpr (HasDynamicsMatrices<Solver_t>) {
        m_Adyn = m_solver.get_Adyn();
        m_Bdyn = m_solver.get_Bdyn();
      }
    }
    
    // Constructor with custom time step
    ControlProblem(Scalar_t dt) : 
      m_solver(dt),
      m_trajectory_len(0),
      m_iter(0),
      m_written_states(0),
      m_dt(dt)
    {
      if constexpr (HasDynamicsMatrices<Solver_t>) {
        m_Adyn = m_solver.get_Adyn();
        m_Bdyn = m_solver.get_Bdyn();
      }
    }
    
    // Constructor with solver instance
    ControlProblem(Solver_t solver) : 
      m_solver(std::move(solver)),
      m_trajectory_len(0),
      m_iter(0),
      m_written_states(0),
      m_dt(0.01f) // 10ms default time step
    {
      if constexpr (HasDynamicsMatrices<Solver_t>) {
        m_Adyn = m_solver.get_Adyn();
        m_Bdyn = m_solver.get_Bdyn();
      }
    }
    
    // Constructor with solver instance and custom time step
    ControlProblem(Solver_t solver, Scalar_t dt) : 
      m_solver(std::move(solver)),
      m_trajectory_len(0),
      m_iter(0),
      m_written_states(0),
      m_dt(dt)
    {
      if constexpr (HasDynamicsMatrices<Solver_t>) {
        m_Adyn = m_solver.get_Adyn();
        m_Bdyn = m_solver.get_Bdyn();
      }
    }
    
    // Get the current time step
    Scalar_t get_dt() const { return m_dt; }
    
    // Set a new time step
    void set_dt(Scalar_t dt) { 
      m_dt = dt; 
      // Only call set_dt on solver if it has this method
      if constexpr (HasSetDt<Solver_t>) {
        m_solver.set_dt(dt);
      }
    }
    
    // Get access to the solver
    Solver_t& get_solver() { return m_solver; }
    const Solver_t& get_solver() const { return m_solver; }
};

// ============================================================================
// TYPE ALIASES FOR COMMON CONTROLLER TYPES
// ============================================================================

// Forward declarations
namespace EntoControl {
  // AdaptiveController is already defined in adaptive_controller.h as a type alias
  template<typename Scalar, typename VehicleTraits, bool UseDecoupledYaw, bool UseIntegralControl>
  class GeometricControllerSolver;
  template<typename Scalar> struct QuadrotorTraits;
  template<typename Scalar> struct RoboBeeTraits;
}

// Adaptive Controller (original)
template<typename Scalar = float, int StateSize = 10, int CtrlSize = 4, int HorizonSize = 1, int PathLen = 2000>
using AdaptiveControlProblem = ControlProblem<Scalar, EntoControl::AdaptiveController, StateSize, CtrlSize, HorizonSize, PathLen>;

// Geometric Controller variants
template<typename Scalar = float, int HorizonSize = 1, int PathLen = 2000>
using GeometricControlProblem = ControlProblem<
  Scalar, 
  EntoControl::GeometricControllerSolver<Scalar, EntoControl::QuadrotorTraits<Scalar>, true, true>,
  13,  // StateSize: [x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz]
  4,   // ControlSize: [thrust, Mx, My, Mz]
  HorizonSize,
  PathLen
>;

template<typename Scalar = float, int HorizonSize = 1, int PathLen = 2000>
using GeometricControlProblemRoboBee = ControlProblem<
  Scalar, 
  EntoControl::GeometricControllerSolver<Scalar, EntoControl::RoboBeeTraits<Scalar>, true, true>,
  13, 4, HorizonSize, PathLen
>;

// Common type aliases
using GeometricControlProblemFloat = GeometricControlProblem<float>;
using GeometricControlProblemDouble = GeometricControlProblem<double>;

// ============================================================================
// REFERENCE-DRIVEN CONTROL PROBLEM (NO FORWARD DYNAMICS)
// ============================================================================

/**
 * @brief Control problem that uses reference trajectories instead of forward dynamics
 * 
 * This is useful for controllers where we don't have a reliable forward dynamics model
 * but we have reference trajectories with expected states. The controller computes
 * control inputs but doesn't simulate forward - instead it uses the reference trajectory
 * as the "observed" state for benchmarking computational performance.
 */
template< typename Scalar, typename Solver, int StateSize, int CtrlSize, int HorizonSize, int PathLen >
class ReferenceControlProblem :
  public EntoBench::EntoProblem< ReferenceControlProblem< Scalar, Solver, StateSize, CtrlSize, HorizonSize, PathLen >>
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
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, PathLen > m_reference_trajectory;
    int m_iter;
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, CtrlSize, 1 >, PathLen > m_control_outputs;
    
    int m_written_controls;
    Scalar_t m_dt; // Time step for discretization
  
  public:
  #ifdef NATIVE
    std::string serialize_impl() const
    {
      std::stringstream ss;
      for ( int j = 0; j < CtrlSize; j++ ) {
        ss << m_control_outputs[m_written_controls][j] << ",";
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
      
      m_reference_trajectory.push_back( state_vec );
      m_trajectory_len++;
      return true;
    }
  #else
    const char* serialize_impl() const
    {
      char line[256];
      int pos = 0;
      for ( int i = 0; i < m_iter; i++ ) {
        for ( int j = 0; j < CtrlSize; j++ ) {
          pos += sprintf( &line[pos], "%f,", m_control_outputs[i][j] );
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

      m_reference_trajectory.push_back( state_vec );
      m_trajectory_len++;
      return true;
    }
  #endif

    bool validate_impl() const
    {
#ifdef NATIVE
      std::cout << "Validating control outputs for " << m_iter << " iterations:" << std::endl;
      for ( int i = 0; i < m_iter; i++ ) {
        std::cout << "Control[" << i << "]: " << m_control_outputs[i].transpose() << std::endl;
      }
#endif
      return true;
    }

    void solve_impl()
    {
      // Make sure we have a valid reference trajectory for this step
      if (m_iter >= m_trajectory_len) {
#ifdef NATIVE
        std::cout << "Warning: Not enough reference points provided. Using the last reference point." << std::endl;
#endif
        // Copy the last valid reference point
        Eigen::Matrix<Scalar_t, StateSize, 1> last_ref = m_reference_trajectory[m_trajectory_len - 1];
        m_reference_trajectory.push_back(last_ref);
        m_trajectory_len++;
      }
      
      // Use reference trajectory as the "current state"
      Eigen::Matrix< Scalar_t, StateSize, 1 >& current_state = m_reference_trajectory[m_iter];
      
#ifdef NATIVE
      std::cout << "Solving iteration " << m_iter << ":" << std::endl;
      std::cout << "  Reference state: " << current_state.transpose() << std::endl;
#endif
      
      // Feed reference state to the controller as "current state"
      m_solver.set_x0(current_state);
      
      // Set reference (for controllers that need a target)
      if (m_iter + 1 < m_trajectory_len) {
        m_solver.set_x_ref(m_reference_trajectory[m_iter + 1]);
      } else {
        m_solver.set_x_ref(current_state); // Stay at current position
      }
      
      // Run the controller to get the control inputs
      m_solver.solve();
      
      // Get the control input from the controller
      Eigen::Matrix<Scalar_t, CtrlSize, 1> u0 = m_solver.get_u0();
      
#ifdef NATIVE
      std::cout << "  Control: " << u0.transpose() << std::endl;
#endif

      // Store the control output for benchmarking
      m_control_outputs.push_back(u0);
      m_iter++;
    }

    void clear_impl()
    {
      m_reference_trajectory.clear();
      m_control_outputs.clear();
      m_trajectory_len = 0;
      m_iter = 0;
      m_written_controls = 0;
      
      // Reset controller state
      if constexpr (std::is_constructible_v<Solver_t, Scalar_t>) {
        m_solver = Solver_t(m_dt);
      } else {
        m_solver = Solver_t();
      }
    }

    static constexpr const char* header_impl()
    {
      return "Control outputs (thrust, moment_x, moment_y, moment_z)";
    }

    // Default constructor with 10ms time step
    ReferenceControlProblem() : 
      m_solver(Scalar_t(0.01)),
      m_trajectory_len(0),
      m_iter(0),
      m_written_controls(0),
      m_dt(0.01f)
    {
    }
    
    // Constructor with custom time step
    ReferenceControlProblem(Scalar_t dt) : 
      m_solver(dt),
      m_trajectory_len(0),
      m_iter(0),
      m_written_controls(0),
      m_dt(dt)
    {
    }
    
    // Constructor with solver instance
    ReferenceControlProblem(Solver_t solver) : 
      m_solver(std::move(solver)),
      m_trajectory_len(0),
      m_iter(0),
      m_written_controls(0),
      m_dt(0.01f)
    {
    }
    
    // Constructor with solver instance and custom time step
    ReferenceControlProblem(Solver_t solver, Scalar_t dt) : 
      m_solver(std::move(solver)),
      m_trajectory_len(0),
      m_iter(0),
      m_written_controls(0),
      m_dt(dt)
    {
    }
    
    // Get the current time step
    Scalar_t get_dt() const { return m_dt; }
    
    // Set a new time step
    void set_dt(Scalar_t dt) { 
      m_dt = dt; 
      if constexpr (HasSetDt<Solver_t>) {
        m_solver.set_dt(dt);
      }
    }
    
    // Get access to the solver
    Solver_t& get_solver() { return m_solver; }
    const Solver_t& get_solver() const { return m_solver; }
};

#endif // CONTROL_PROBLEM_H 