#ifndef ENTO_CONTROL_OSQP_GENERIC_WRAPPER_H
#define ENTO_CONTROL_OSQP_GENERIC_WRAPPER_H

#include <Eigen/Core>
#include <cstring>                // for std::memcpy
#include <type_traits>            // for std::enable_if_t, std::is_same_v

namespace EntoControl
{

// OSQP solver status codes
enum class OSQPStatus {
  SOLVED = 1,
  SOLVED_INACCURATE = 2,
  MAX_ITER_REACHED = -2,
  PRIMAL_INFEASIBLE = -3,
  PRIMAL_INFEASIBLE_INACCURATE = -4,
  DUAL_INFEASIBLE = -5,
  DUAL_INFEASIBLE_INACCURATE = -6,
  UNSOLVED = -10
};

// Default empty traits for when no traits are needed
struct EmptyTraits {};

template< typename API, typename Traits = EmptyTraits, bool UseWarmStart = false >
class OSQPSolver
{
public:
  using Scalar = float;
  static constexpr int N = API::n_x;
  static constexpr int M = API::n_u;
  static constexpr int H = API::horizon;

  using State = Eigen::Matrix<Scalar, N, 1>;
  using Input = Eigen::Matrix<Scalar, M, 1>;
  using Control = Input; // Alias for SFINAE detection

  // OSQPWorkspace is managed by the C API, we just use it
  // The workspace is pre-allocated and initialized in robofly_init()
  OSQPSolver()
  {
    // Initialize API by getting the workspace
    typename API::Workspace* work = API::get_workspace();
    API::init(work);
    
    if constexpr (UseWarmStart) {
      has_previous_solution_ = false;
      prev_solution_ = Eigen::Matrix<Scalar, M, 1>::Zero();
    }
  }
  
  // No need for a destructor since we don't own the workspace
  
  // Prevent copying
  OSQPSolver(const OSQPSolver&) = delete;
  OSQPSolver& operator=(const OSQPSolver&) = delete;

  bool set_x0(const State& x)
  {
    // Basic validation: check for NaN or Inf values
    if (!x.allFinite()) {
      return false;
    }
    x0_ = x;
    return true;
  }

  void set_x_ref(const State& xr) { xref_ = xr; }

  void reset_duals() {} // no-op for code-gen solver

  bool solve()
  {
    // Get the static workspace pointer
    typename API::Workspace* work = API::get_workspace();
    
    // Expand single reference state to full horizon trajectory
    // The API expects xref to be [N*H] elements (state repeated over horizon)
    Eigen::Matrix<Scalar, N * H, 1> xref_horizon;
    for (int k = 0; k < H; ++k) {
      xref_horizon.template segment<N>(k * N) = xref_;
    }
    
    // Compute b = xref - Φ x0 and update the bounds
    API::update_rhs(work, x0_.data(), xref_horizon.data());

    // Apply warm starting if enabled and available
    if constexpr (UseWarmStart) {
      if (has_previous_solution_) {
        // Use previous solution as warm start
        API::warm_start(work, prev_solution_.data());
      }
    }

    // Solve the QP
    API::solve(work);

    // Get the solution using the API
    const float* solution = API::get_solution(work);
    
    // Copy the solution (first M elements are the control inputs)
    for (int i = 0; i < M; ++i) {
      u0_[i] = solution[i];
    }

    // Store solution for next warm start if enabled
    if constexpr (UseWarmStart) {
      prev_solution_ = u0_;
      has_previous_solution_ = true;
    }

    // Return true if solver converged successfully
    return API::is_optimal(work);
  }

  const Input& get_u0() const   { return u0_; }

  // Get the solver status
  OSQPStatus get_status() const {
    return static_cast<OSQPStatus>(API::get_status(API::get_workspace()));
  }

  // Get iteration count
  int get_iterations() const {
    return API::get_iter_count(API::get_workspace());
  }

  // Getter methods only available if traits are provided (not EmptyTraits)
  template<typename T = Traits>
  static constexpr auto get_Adyn() -> typename std::enable_if_t<!std::is_same_v<T, EmptyTraits>, decltype(T::Adyn)> {
    return T::Adyn;
  }

  template<typename T = Traits>
  static constexpr auto get_Bdyn() -> typename std::enable_if_t<!std::is_same_v<T, EmptyTraits>, decltype(T::Bdyn)> {
    return T::Bdyn;
  }

private:
  State  x0_{}, xref_{};
  Input  u0_{};

  // Additional members for warm start support
  bool has_previous_solution_{false};
  Input prev_solution_{};
};

} // namespace EntoControl

#endif // ENTO_CONTROL_OSQP_GENERIC_WRAPPER_H
