#ifndef OPT_CONTROL_REPLAY_PROBLEM_H
#define OPT_CONTROL_REPLAY_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/debug.h>

/**
 * OptControlReplayProblem
 *
 * Variant of OptControlProblem for validating MPC solvers against
 * captured closed-loop data. Instead of forward-simulating with A*x+B*u,
 * it reads actual states from captured data (e.g., from a Webots simulation).
 *
 * CSV format: 24 columns per row
 *   [12 reference state] , [12 actual state]
 *
 * Header: "Replay trajectory ( ref_x, ref_y, ref_z, ref_r, ref_p, ref_w, ref_x_dot, ref_y_dot, ref_z_dot, ref_r_dot, ref_p_dot, ref_w_dot, act_x, act_y, act_z, act_r, act_p, act_w, act_x_dot, act_y_dot, act_z_dot, act_r_dot, act_p_dot, act_w_dot )"
 *
 * On each iteration:
 *   - x0 = actual state from CSV (not forward-simmed)
 *   - x_ref = reference states from CSV
 *   - solve() computes control output
 *   - Control output can be compared against captured ground truth
 */

template< typename Scalar, typename Solver, int StateSize, int CtrlSize,
          int HorizonSize, int PathLen >
class OptControlReplayProblem :
  public EntoBench::EntoProblem< OptControlReplayProblem< Scalar, Solver, StateSize, CtrlSize, HorizonSize, PathLen >>
{
  public:
    using Scalar_t = Scalar;
    using Solver_t = Solver;

    static constexpr bool RequiresDataset_ = true;
    static constexpr bool SaveResults_     = false;
    static constexpr bool RequiresSetup_   = true;
    static constexpr int  SetupLines_      = HorizonSize;

  private:
    Solver_t m_solver;
    int m_trajectory_len;

    // Reference trajectory (from CSV first 12 columns)
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, PathLen > m_trajectory;
    // Actual states (from CSV last 12 columns)
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, StateSize, 1 >, PathLen > m_actual_states;

    int m_iter;
    FILE* m_control_log;

  public:

#ifdef NATIVE
    std::string serialize_impl() const { return ""; }

    bool deserialize_impl(const std::string &line)
    {
      std::stringstream ss( line );
      // Parse 24 values: 12 reference + 12 actual
      Eigen::Matrix< Scalar_t, StateSize, 1 > ref_vec, act_vec;

      for ( int i = 0; i < StateSize; i++ ) {
        std::string item;
        std::getline( ss, item, ',' );
        ref_vec[i] = (Scalar_t) std::stold( item );
      }
      for ( int i = 0; i < StateSize; i++ ) {
        std::string item;
        std::getline( ss, item, ',' );
        act_vec[i] = (Scalar_t) std::stold( item );
      }

      m_trajectory.push_back( ref_vec );
      m_actual_states.push_back( act_vec );
      m_trajectory_len++;
      return true;
    }
#else
    const char* serialize_impl() const { return ""; }

    bool deserialize_impl(const char* line)
    {
      char* token;
      char* to_parse = const_cast<char*>(line);
      Eigen::Matrix< Scalar_t, StateSize, 1 > ref_vec, act_vec;

      token = strtok( to_parse, "," );
      for ( int i = 0; i < StateSize && token; i++ ) {
        ref_vec[i] = static_cast<Scalar_t>(atof( token ));
        token = strtok( nullptr, "," );
      }
      for ( int i = 0; i < StateSize && token; i++ ) {
        act_vec[i] = static_cast<Scalar_t>(atof( token ));
        token = strtok( nullptr, "," );
      }

      m_trajectory.push_back( ref_vec );
      m_actual_states.push_back( act_vec );
      m_trajectory_len++;
      return true;
    }
#endif

    bool validate_impl() const { return true; }

    void solve_impl()
    {
      // Use actual captured state as x0
      Eigen::Matrix< Scalar_t, StateSize, 1 > x0 = m_actual_states[m_iter];
      m_solver.set_x0( x0 );

      // Build reference horizon from trajectory
      Eigen::Matrix< Scalar_t, StateSize, HorizonSize > x_ref;
      for ( int i = 0; i < HorizonSize; i++ ) {
        int idx = m_iter + i;
        if (idx >= m_trajectory_len) idx = m_trajectory_len - 1;
        x_ref.col(i) = m_trajectory[idx];
      }
      m_solver.set_x_ref( x_ref );
      m_solver.reset_duals();
      m_solver.solve();
    }

    void step()
    {
      // Log control output
#ifdef NATIVE
      if (m_control_log) {
        auto u0 = m_solver.get_u0();
        for (int j = 0; j < CtrlSize; j++) {
          fprintf(m_control_log, "%.10f%s", (double)u0[j], j < CtrlSize-1 ? "," : "\n");
        }
      }
#endif
      m_iter++;
    }

    void clear_impl() {}

    static constexpr const char* header_impl()
    {
      return "Replay trajectory ( ref_x, ref_y, ref_z, ref_r, ref_p, ref_w, ref_x_dot, ref_y_dot, ref_z_dot, ref_r_dot, ref_p_dot, ref_w_dot, act_x, act_y, act_z, act_r, act_p, act_w, act_x_dot, act_y_dot, act_z_dot, act_r_dot, act_p_dot, act_w_dot )";
    }

    OptControlReplayProblem(Solver solver, FILE* log_fp = nullptr) :
      m_solver(std::move(solver)),
      m_trajectory_len(0),
      m_iter(0),
      m_control_log(log_fp)
    {}

    ~OptControlReplayProblem() = default;
};

#endif // OPT_CONTROL_REPLAY_PROBLEM_H
