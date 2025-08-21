#!/usr/bin/env python3
"""
Generate a code-generated OSQP solver (condensed MPC) for RoboBee.

Outputs
-------
src/ento-control/generated/robobee_mpc/      ← C sources, headers
src/ento-control/generated/robobee_mpc/robobee_api.h

Requires
--------
pip install "osqp>=1.0"  numpy  scipy
"""

import re, shutil, sys, os
from pathlib import Path

import numpy as np
import scipy.sparse as sp
import osqp

# ──────────────────────────  helper: parse Eigen header  ───────────────────── #
def parse_eigen_matrix(header_path: Path, name: str) -> np.ndarray:
    """Return the float matrix called `name` from the header."""
    txt = header_path.read_text()
    size_re = rf"Eigen::Matrix<\s*float\s*,\s*(\d+)\s*,\s*(\d+)\s*>\s*{name}"
    m_sz    = re.search(size_re, txt)
    if m_sz is None:
        raise ValueError(f"{name} not found in {header_path}")
    rows, cols = map(int, m_sz.groups())

    body_re = rf"{name}[^;]*?<<\s*(.*?)\)\s*\.finished"
    m_body  = re.search(body_re, txt, re.S)
    if m_body is None:
        raise ValueError(f"could not parse values for {name}")
    nums = [float(tok.strip().rstrip('f'))
            for tok in m_body.group(1).replace('\n', ' ').split(',') if tok.strip()]
    arr  = np.asarray(nums, dtype=np.float32)
    if arr.size != rows * cols:
        raise ValueError(f"{name}: expected {rows*cols} values, got {arr.size}")
    return arr.reshape(rows, cols)

# ─────────────────────── 1. Define RoboBee dynamics  ───────────────────── #
# RoboBee physical parameters
m     = 1.06e-4          # robot mass (kg)
mM    = 1.0e-5           # MoCap marker mass (kg)

Jxx   = 2.32e-9          # kg·m²
Jyy   = 1.97e-9
Jzz   = 0.45e-9

k_T   = 3.27e-5          # N / V
k_r   = 0.50e-6          # N·m / V
k_p   = 0.15e-6          # N·m / V

g     = 9.81             # m s⁻²

# Set up continuous-time dynamics matrices
Ad_c = np.zeros((10, 10), dtype=np.float32)  # 10×10 continuous
Bd_c = np.zeros((10, 3), dtype=np.float32)   # 10×3  continuous

# A matrix setup
Ad_c[0, 3] = Ad_c[1, 4] = Ad_c[2, 5] = 1.0
Ad_c[3, 7] =  g
Ad_c[4, 6] = -g
Ad_c[6, 8] = 1.0
Ad_c[7, 9] = 1.0

# B matrix setup
Bd_c[5, 0] = k_T / (m + mM)
Bd_c[8, 1] = k_r / Jxx
Bd_c[9, 2] = k_p / Jyy

N, M  = Ad_c.shape[0], Bd_c.shape[1]          # 10, 3
dt    = 0.005                                 # 200 Hz loop
H     = 20                                    # horizon

# discretise (Euler); for better fidelity you might use matrix exp
Ad = np.eye(N, dtype=np.float32) + Ad_c * dt
Bd = Bd_c * dt

# ─────────────────────── 2. condensed MPC matrices  ────────────────────────── #
Qdiag = np.array([0.02, 0.02, 0.01, 0.1, 0.1, 0.1, 1, 1, 4, 4], dtype=np.float32)
Rdiag = np.array([2, 1, 1],                           dtype=np.float32)

# 2.1 cost ‖R u‖²  →  P  (MH × MH)
P = sp.block_diag([sp.diags(Rdiag)] * H, format="csc")
q = np.zeros(M * H, dtype=np.float32)                 # filled online if desired

# 2.2 build Φ (NH×N) and Γ (NH×MH)
Phi   = np.zeros((N * H, N),     dtype=np.float32)
Gamma = np.zeros((N * H, M * H), dtype=np.float32)

A_pow = np.eye(N, dtype=np.float32)                   # A^0
for k in range(H):
    Phi[k*N:(k+1)*N] = A_pow @ Ad                     # A^(k+1)

    B_term = Bd.copy()
    for j in range(k, -1, -1):                        # j = k … 0
        Gamma[k*N:(k+1)*N, j*M:(j+1)*M] = B_term
        B_term = Ad @ B_term
    A_pow = Ad @ A_pow                                # A^(k+1)

Aeq = sp.csc_matrix(Gamma)                                           # 200 × 60 constraint
l = np.zeros(N * H, dtype=np.float32)                 # placeholders (will be b)
u = l.copy()

# ─────────────────────── 3.  set up & code-gen OSQP 1.x  ───────────────────── #
prob = osqp.OSQP()
prob.setup(P, q, Aeq, l, u,
           eps_abs=1e-4, eps_rel=1e-4,
           verbose=False)

out_dir = Path("/Users/derinozturk/research/repos/ento-bench/src/ento-control/generated/robobee_mpc")

# Save existing CMakeLists.txt if it exists
cmake_path = out_dir / "CMakeLists.txt"
custom_cmake = None
if cmake_path.exists():
    print(f"Saving existing CMakeLists.txt")
    custom_cmake = cmake_path.read_text()

# Clean directory but don't remove it completely to preserve git info
if out_dir.exists():
    for item in out_dir.iterdir():
        if item.is_file():
            item.unlink()
        elif item.is_dir():
            shutil.rmtree(item)
else:
    out_dir.mkdir(parents=True, exist_ok=True)

prob.codegen(
    str(out_dir),                 # output folder
    prefix   = "robobee", # symbol prefix
    force_rewrite    = True,
    use_float = True
)

# Restore our custom CMakeLists.txt
if custom_cmake is not None:
    print(f"Restoring custom CMakeLists.txt")
    cmake_path.write_text(custom_cmake)
else:
    # Create a default CMakeLists.txt that works with our project
    with cmake_path.open("w") as f:
        f.write("""# OSQP static library for RoboBee MPC
# This is part of the ento-control module, not a separate project

# Embedded OSQP mode
set(OSQP_EMBEDDED_MODE "1")

# Collect source files
file(GLOB OSQP_SOURCES src/*.c)

# Add our custom API implementation
list(APPEND OSQP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/robobee_api.c)

# Create the static library
add_library(osqpstatic STATIC ${OSQP_SOURCES})

# Position-independent code for better compatibility
set_property(TARGET osqpstatic PROPERTY POSITION_INDEPENDENT_CODE ON)

# Set include directories
target_include_directories(osqpstatic PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/private 
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Export the include directories
set(ROBOBEE_MPC_INCLUDE_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/private 
  ${CMAKE_CURRENT_SOURCE_DIR}
  PARENT_SCOPE
)

# Make this library available to the parent scope
set(ROBOBEE_MPC_LIBRARIES osqpstatic PARENT_SCOPE)
""")

# ─────────────────────── 4.  emit C++ API shim  ────────────────────────────── #
api = out_dir / "robobee_api.h"
with api.open("w") as f:
    f.write("#ifndef ENTO_CONTROL_ROBOBEE_API\n")
    f.write("#define ENTO_CONTROL_ROBOBEE_API\n\n")
    
    # Include the necessary headers
    f.write('// Include the necessary OSQP API types and functions\n')
    f.write('#include "inc/public/osqp_api_types.h"\n')
    f.write('#include <cstring>\n\n')
    
    # Forward declarations for OSQP C functions
    f.write('// Forward declarations for OSQP C functions\n')
    f.write('extern "C" {\n')
    f.write('  void robobee_init(OSQPWorkspace* w);\n')
    f.write('  void robobee_solve(OSQPWorkspace* w);\n')
    f.write('  void robobee_update_bounds(OSQPWorkspace* w, const float* lower, const float* upper);\n')
    f.write('  // Additional OSQP API functions we need to access\n')
    f.write('  const float* osqp_get_primal_solution(const OSQPWorkspace* w);\n')
    f.write('  int osqp_get_status_val(const OSQPWorkspace* w);\n')
    f.write('  int osqp_get_iter(const OSQPWorkspace* w);\n')
    f.write('  void osqp_warm_start_x(OSQPWorkspace* w, const float* x);\n')
    f.write('  // Function to create workspace\n')
    f.write('  OSQPWorkspace* osqp_workspace_create(void);\n')
    f.write('  void osqp_workspace_free(OSQPWorkspace* w);\n')
    f.write('}\n\n')
    
    # OSQP status codes
    f.write('// OSQP status codes\n')
    f.write('#define OSQP_SOLVED 1\n')
    f.write('#define OSQP_SOLVED_INACCURATE 2\n')
    f.write('#define OSQP_MAX_ITER_REACHED -2\n')
    f.write('#define OSQP_PRIMAL_INFEASIBLE -3\n')
    f.write('#define OSQP_PRIMAL_INFEASIBLE_INACCURATE -4\n')
    f.write('#define OSQP_DUAL_INFEASIBLE -5\n')
    f.write('#define OSQP_DUAL_INFEASIBLE_INACCURATE -6\n')
    f.write('#define OSQP_UNSOLVED -10\n\n')
    
    # emit Phi
    rows, cols = Phi.shape      # rows=200, cols=10
    f.write(f"static const float robobee_Phi[{rows}][{cols}] = {{\n")
    for r in range(rows):
        vals = ", ".join(f"{v:.8e}f" for v in Phi[r])
        f.write(f"  {{ {vals} }},\n")
    f.write("};\n\n")
    
    # emit API struct
    f.write("struct RoboBeeAPI\n{\n"
            "  using Workspace = OSQPWorkspace;\n\n"
            
            "  // Get the global workspace instance (initialized on first use)\n"
            "  static Workspace* get_workspace() {\n"
            "    // Static flag to track if workspace is initialized\n"
            "    static bool initialized = false;\n"
            "    // This workspace is initialized via embedded C code\n"
            "    static Workspace* workspace = nullptr;\n"
            "    \n"
            "    if (!initialized) {\n"
            "      // Create and initialize the workspace only once\n"
            "      workspace = osqp_workspace_create();\n"
            "      if (workspace) {\n"
            "        init(workspace);\n"
            "        initialized = true;\n"
            "      }\n"
            "    }\n"
            "    return workspace;\n"
            "  }\n\n"
            
            "  static inline void init(Workspace* w) { robobee_init(w); }\n\n"
            "  static inline void update_rhs(Workspace* w,\n"
            "                                const float* x0,\n"
            "                                const float* xref)\n"
            "  {\n"
            "    float b[200];\n"
            "    for (int k = 0; k < 200; ++k) {\n"
            "      float acc = 0.f;\n"
            "      for (int i = 0; i < 10; ++i)\n"
            "        acc += robobee_Phi[k][i] * x0[i];\n"
            "      b[k] = xref[k] - acc;\n"
            "    }\n"
            "    robobee_update_bounds(w, b, b);\n"
            "  }\n\n"
            "  // Warm start the solver with a previous solution\n"
            "  static inline void warm_start(Workspace* w, const float* u_warm)\n"
            "  {\n"
            "    osqp_warm_start_x(w, u_warm);\n"
            "  }\n\n"
            "  // Get the solution (optimal control inputs)\n"
            "  static inline const float* get_solution(Workspace* w) {\n"
            "    return osqp_get_primal_solution(w);\n"
            "  }\n\n"
            "  static inline void solve(Workspace* w) { robobee_solve(w); }\n\n"
            "  // Get the solver status\n"
            "  static inline int get_status(Workspace* w) {\n"
            "    return osqp_get_status_val(w);\n"
            "  }\n\n"
            "  // Check if the solver solution is optimal\n"
            "  static inline bool is_optimal(Workspace* w) {\n"
            "    int status = get_status(w);\n"
            "    return status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;\n"
            "  }\n\n"
            "  // Get iteration count\n"
            "  static inline int get_iter_count(Workspace* w) {\n"
            "    return osqp_get_iter(w);\n"
            "  }\n\n"
            f"  static constexpr int n_x = {N};\n"
            f"  static constexpr int n_u = {M};\n"
            f"  static constexpr int horizon = {H};\n"
            "};\n")

    f.write("#endif //ENTO_CONTROL_ROBOBEE_API\n")

# Create the C implementation file for basic implementation
api_c = out_dir / "robobee_api.c"
with api_c.open("w") as f:
    f.write('#include "inc/public/osqp_api_types.h"\n')
    f.write('#include "inc/public/osqp_api_functions.h"\n\n')
    
    f.write('// This file provides implementations for the declared functions in robobee_api.h\n')
    f.write('// OSQP automatically generates most of the functions we need\n\n')
    
    f.write('// Create workspace function if needed (may be provided by OSQP)\n')
    f.write('OSQPWorkspace* osqp_workspace_create(void) {\n')
    f.write('    // Allocate workspace structure\n')
    f.write('    OSQPWorkspace* work = (OSQPWorkspace*) c_malloc(sizeof(OSQPWorkspace));\n')
    f.write('    if (!work) return NULL;\n')
    f.write('    \n')
    f.write('    // Initialize to NULL all workspace fields\n')
    f.write('    work->data = NULL;\n')
    f.write('    work->settings = NULL;\n')
    f.write('    work->scaling = NULL;\n')
    f.write('    work->solution = NULL;\n')
    f.write('    work->rho_vec = NULL;\n')
    f.write('    work->rho_inv_vec = NULL;\n')
    f.write('    work->constr_type = NULL;\n')
    f.write('    work->pol = NULL;\n')
    f.write('    work->linsys_solver = NULL;\n')
    f.write('    work->info = NULL;\n')
    f.write('    work->first_run = 1;\n')
    f.write('    work->clear_update_time = 0;\n')
    f.write('    \n')
    f.write('    return work;\n')
    f.write('}\n')

print(f"Generated OSQP solver into {out_dir}") 