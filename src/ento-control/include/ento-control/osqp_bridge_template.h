bool solve() {
  // Get workspace
  auto* workspace = APIType::get_workspace();
  if (!workspace) {
    return false;
  }

  // Update problem data with current state and reference
  // Expand single reference to full horizon (20 time steps × 10 states = 200 elements)
  std::vector<OSQPFloat> x_ref_horizon(APIType::n_x * APIType::horizon);
  for (int k = 0; k < APIType::horizon; ++k) {
    for (int i = 0; i < APIType::n_x; ++i) {
      x_ref_horizon[k * APIType::n_x + i] = static_cast<OSQPFloat>(x_ref_[i]);
    }
  }
  
  // Convert current state to OSQPFloat
  std::vector<OSQPFloat> x0_osqp(APIType::n_x);
  for (int i = 0; i < APIType::n_x; ++i) {
    x0_osqp[i] = static_cast<OSQPFloat>(x0_[i]);
  }
  
  // Update the problem constraints with current state and reference
  // This calls the generated update_rhs function that computes b = xref - Φx0
  APIType::update_rhs(workspace, x0_osqp.data(), x_ref_horizon.data());
  
  // Solve the QP
  OSQPInt status = APIType::solve(workspace);

  // Check if solve was successful
  auto solver_status = APIType::get_status(workspace);
  last_status_ = static_cast<OSQPInt>(solver_status);
  last_iterations_ = APIType::get_iterations(workspace);

  // Get solution
  const OSQPFloat* solution = APIType::get_solution(workspace);
  if (!solution) {
    return false;
  }

  // Extract first control input (u0)
  for (int i = 0; i < APIType::n_u; ++i) {
    u0_[i] = static_cast<float>(solution[i]);
  }
  
  return (last_status_ == OSQP_SOLVED || last_status_ == OSQP_SOLVED_INACCURATE);
} 