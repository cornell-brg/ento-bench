#ifndef OSQP_BRIDGE_TEMPLATE_H
#define OSQP_BRIDGE_TEMPLATE_H

/*
 * Generic OSQP Bridge Template
 * 
 * This template provides a bridge between the generated OSQP API and the actual
 * OSQP solver workspace. It connects to the pre-initialized generated solver.
 * 
 * To use this template:
 * 1. Define SOLVER_PREFIX (e.g., "robofly" or "robobee")
 * 2. Include the appropriate workspace header
 * 3. Include this template
 * 
 * Example usage for RoboFly:
 * #define SOLVER_PREFIX robofly
 * #include "roboflyworkspace.h"
 * #include "osqp_bridge_template.h"
 */

#include "inc/public/osqp_api_types.h"
#include "inc/public/osqp_api_functions.h"
#include <stddef.h>
#include <stdbool.h>

// Macro helpers for concatenating names
#define CONCAT_IMPL(a, b) a ## b
#define CONCAT(a, b) CONCAT_IMPL(a, b)

// Generate function names based on SOLVER_PREFIX
#define SOLVER_VAR CONCAT(SOLVER_PREFIX, solver)
#define INIT_FUNC CONCAT(SOLVER_PREFIX, _init)
#define SOLVE_FUNC CONCAT(SOLVER_PREFIX, _solve)
#define UPDATE_BOUNDS_FUNC CONCAT(SOLVER_PREFIX, _update_bounds)

// External solver reference (defined in workspace)
extern OSQPSolver SOLVER_VAR;

// Bridge functions to connect the generated API to the real OSQP solver

// Function to provide access to the workspace
OSQPWorkspace* osqp_workspace_create() {
    // Return the workspace from the pre-initialized solver
    return SOLVER_VAR.work;
}

// Initialize the solver
void INIT_FUNC(OSQPWorkspace* w) {
    // The solver is already initialized by the generated code
    // Nothing additional needed for embedded OSQP
}

// Solve the QP problem
void SOLVE_FUNC(OSQPWorkspace* w) {
    // Use the real OSQP solve function with the pre-initialized solver
    osqp_solve(&SOLVER_VAR);
}

// Update the bounds (constraints) of the QP problem
void UPDATE_BOUNDS_FUNC(OSQPWorkspace* w, const float* lower, const float* upper) {
    // Update the constraint bounds using the real OSQP API
    osqp_update_data_vec(&SOLVER_VAR, OSQP_NULL, lower, upper);
}

// Get the solution
const float* osqp_get_primal_solution(const OSQPWorkspace* w) {
    return SOLVER_VAR.solution->x;
}

// Get the solver status
int osqp_get_status_val(const OSQPWorkspace* w) {
    return SOLVER_VAR.info->status_val;
}

// Get the iteration count
int osqp_get_iter(const OSQPWorkspace* w) {
    return SOLVER_VAR.info->iter;
}

// Warm start the solver
void osqp_warm_start_x(OSQPWorkspace* w, const float* x) {
    // For embedded OSQP, we call osqp_warm_start with x and NULL for y
    osqp_warm_start(&SOLVER_VAR, x, OSQP_NULL);
}

#endif // OSQP_BRIDGE_TEMPLATE_H 