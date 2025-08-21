#!/usr/bin/env python3
"""
Generate RoboFly LQR constants header.

Outputs:
  src/ento-control/lqr_traits_robofly.h   (over-writes!)

Requires:
  numpy, scipy
"""

import numpy as np
from  scipy.linalg import solve_continuous_are
from  pathlib     import Path
import textwrap

# ──────────────────────────────────── physical constants ──────────────────── #
m     = 1.46e-4          # robot mass (kg)
mM    = 2.0e-5           # MoCap marker mass (kg)

Jxx   = 3.12e-9          # kg·m²
Jyy   = 2.97e-9
Jzz   = 0.55e-9

k_T   = 3.27e-5          # N / V
k_r   = 0.48e-6          # N·m / V
k_p   = 0.11e-6          # N·m / V

g     = 9.81             # m s⁻²

# ───────────────────────────────────── build A, B ─────────────────────────── #
A = np.zeros((10, 10))
B = np.zeros((10,  3))

A[0, 3] = A[1, 4] = A[2, 5] = 1.0
A[3, 7] =  g
A[4, 6] = -g
A[6, 8] = 1.0
A[7, 9] = 1.0

B[5, 0] = k_T / (m + mM)
B[8, 1] = k_r / Jxx
B[9, 2] = k_p / Jyy

# ───────────────────────────────────── weights ────────────────────────────── #
Q = np.diag([0.02, 0.02, 0.01, 0.1, 0.1, 0.1, 1, 1, 4, 4])
R = np.diag([2, 1, 1])

# ───────────────────────────── solve continuous ARE ───────────────────────── #
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P       # (3×10)

# ───────────────────────────── code-gen helpers ───────────────────────────── #
def tidy(mat: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    """Zero-clip very small values for readability."""
    out = mat.copy()
    out[np.abs(out) < eps] = 0.0
    return out.astype(np.float32)

def emit_matrix(name: str, mat: np.ndarray) -> str:
    """Return a nicely formatted Eigen static-constexpr initialiser."""
    mat = tidy(mat)
    rows, cols = mat.shape
    row_lines = []
    for r in range(rows):
        row_lines.append("      " + ", ".join(f"{v:.8e}f" for v in mat[r]))
    body = ",\n".join(row_lines)
    return textwrap.dedent(f"""\
        static constexpr Eigen::Matrix<float,{rows},{cols}> {name} =
          (Eigen::Matrix<float,{rows},{cols}>() <<
{body}).finished();

    """)

# ───────────────────────────── write header file ──────────────────────────── #
hdr_path = Path("../src/ento-control/lqr_traits_robofly.h").resolve()
hdr_path.parent.mkdir(parents=True, exist_ok=True)

with hdr_path.open("w") as f:
    f.write("#ifndef ENTO_CONTROL_LQR_TRAITS_ROBOFLY\n")
    f.write("#define ENTO_CONTROL_LQR_TRAITS_ROBOFLY\n\n")
    f.write("#include <Eigen/Core>\n\n")
    f.write("struct RoboFlyLQRTraits {\n")
    f.write("  using Scalar = float;\n")
    f.write("  static constexpr int N = 10;\n")
    f.write("  static constexpr int M = 3;\n\n")
    f.write(textwrap.indent(emit_matrix("Adyn", A), "  "))
    f.write(textwrap.indent(emit_matrix("Bdyn", B), "  "))
    f.write(textwrap.indent(emit_matrix("K",    K), "  "))
    f.write("};\n\n")
    f.write("#endif  // ENTO_CONTROL_LQR_TRAITS_ROBOFLY\n")

print(f"Wrote {hdr_path}")
