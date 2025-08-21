#ifndef ROBOFLY_API_H
#define ROBOFLY_API_H

#include "osqp.h"
#include "roboflyworkspace.h"
#include "inc/public/osqp_api_functions.h"
#include "inc/public/osqp_api_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

// Use the real OSQP solver defined in roboflyworkspace.c
extern OSQPSolver roboflysolver;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// C++ API struct that matches the expected interface
struct RoboFlyAPI {
  using Workspace = OSQPSolver;
  
  static constexpr int n_x = 10;  // State dimension
  static constexpr int n_u = 3;   // Input dimension  
  static constexpr int horizon = 20; // MPC horizon
  
  // Get workspace
  static inline Workspace* get_workspace() {
    return &roboflysolver;
  }
  
  // Solve the QP
  static inline OSQPInt solve(Workspace* workspace) {
    return osqp_solve(workspace);
  }
  
  // Get solution
  static inline const OSQPFloat* get_solution(Workspace* workspace) {
    return workspace->solution->x;
  }
  
  // Get status
  static inline OSQPInt get_status(Workspace* workspace) {
    return workspace->info->status_val;
  }
  
  // Get iterations
  static inline OSQPInt get_iterations(Workspace* workspace) {
    return workspace->info->iter;
  }
  
  // Update problem data with current state and reference
  static inline OSQPInt update_rhs(Workspace* workspace, 
                                   const OSQPFloat* x0, 
                                   const OSQPFloat* x_ref_horizon) {
    // For now, we'll use the embedded OSQP solver directly without updating
    // The problem data is embedded in the generated code
    // In a full implementation, this would compute b = x_ref_horizon - Phi*x0
    // and call osqp_update_data_vec to update the constraint bounds
    
    // TODO: Implement proper MPC constraint update
    // This requires access to the Phi matrix and proper constraint formulation
    
    return 0; // Success
  }
  
  // Update data vector
  static inline OSQPInt update_data_vec(Workspace* workspace, 
                                        const OSQPFloat* q_new, 
                                        const OSQPFloat* l_new, 
                                        const OSQPFloat* u_new) {
    return osqp_update_data_vec(workspace, q_new, l_new, u_new);
  }
  
  // Warm start
  static inline OSQPInt warm_start(Workspace* workspace, 
                                   const OSQPFloat* x, 
                                   const OSQPFloat* y) {
    return osqp_warm_start(workspace, x, y);
  }
  
  // Cold start
  static inline void cold_start(Workspace* workspace) {
    osqp_cold_start(workspace);
  }
};
#endif

#endif // ROBOFLY_API_H
