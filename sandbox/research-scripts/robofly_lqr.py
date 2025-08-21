# lqr_process_robobee.py

import numpy as np
import pandas as pd
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt

# ─────────────────────────────────────────────────────────────────────────────
# 1) Read the Excel data
#    (install xlrd first: pip install xlrd)
# ─────────────────────────────────────────────────────────────────────────────
xls_path = '44182_2025_22_MOESM3_ESM.xls'
# adjust sheet_name if needed, or use sheet index
df = pd.read_excel(xls_path, engine='xlrd')

# Suppose the sheet has columns: 'time', 'dx','dy','dz','u','v','w','phi','theta','p','q', 'u_measured_1', 'u_measured_2', 'u_measured_3'
t   = df['time'].values
sigma_cols = ['dx','dy','dz','u','v','w','phi','theta','p','q']
u_meas_cols = ['u_measured_1','u_measured_2','u_measured_3']
sigma    = df[sigma_cols].values        # shape (N,10)
u_meas = df[u_meas_cols].values     # shape (N,3)

# Desired setpoint (for example hover at zero)
sigma_des = np.zeros(10)

# ─────────────────────────────────────────────────────────────────────────────
# 2) Build A_d, B_d, Q, R, solve ARE, compute K
#    (paste Ad, Bd from the Supplementary into these arrays)
# ─────────────────────────────────────────────────────────────────────────────
Ad = np.array([
    # … 10×10 matrix from supplement …
])
Bd = np.array([
    # … 10×3 matrix from supplement …
])

Q = np.diag([0.02,0.02,0.01,0.1,0.1,0.1,1,1,4,4])
R = np.diag([2.0,1.0,1.0])

P = solve_continuous_are(Ad, Bd, Q, R)
K = np.linalg.inv(R) @ Bd.T @ P     # shape (3×10)

# ─────────────────────────────────────────────────────────────────────────────
# 3) Compute predicted u and plot
# ─────────────────────────────────────────────────────────────────────────────
u_pred = - (K @ (sigma - sigma_des).T).T     # shape (N,3)

plt.figure(figsize=(8,6))
for i in range(3):
    plt.plot(t, u_meas[:,i], '--', label=f'$u_{{meas,{i+1}}}$')
    plt.plot(t, u_pred[:,i],  '-', label=f'$u_{{pred,{i+1}}}$')
plt.xlabel('Time (s)')
plt.ylabel('Control inputs')
plt.title('Measured vs. LQR-Predicted Control')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
