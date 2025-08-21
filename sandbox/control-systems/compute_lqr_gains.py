#!/usr/bin/env python3
"""
Compute the actual LQR gains used in robognat
"""

import numpy as np
import control as ct
from numpy import array

# Physical parameters (from robognat)
m = 10e-6  # mass [kg]
l = 0.004  # length [m]
J = m * l**2 / 12  # moment of inertia
b = 1.0 * m  # drag coefficient (simplified)
d_z = 0.0002  # distance from CM to center of drag
c = (l/2 + 4.6e-3 * 2./3)**2 * b  # rotational drag
g = 9.81

print(f'Physical parameters:')
print(f'  m = {m:.2e} kg')
print(f'  J = {J:.2e} kg*m^2')
print(f'  b = {b:.2e} Ns/m')
print(f'  c = {c:.2e} Ns*m')
print()

# System matrices (6-state version without wind)
A = array([[0,                1,    0,              0, 0,        0], # thetay
           [0,         -c/J,    0, -b*d_z/J, 0,        0], # omegay
           [0,                0,    0,              1, 0,        0], # x
           [g,   -b*d_z/m,    0,       -b/m, 0,        0], # vx
           [0,                0,    0,              0, 0,        1], # z
           [0,                0,    0,              0, 0, -b/m]])# vz

B = array([[0,     0], 
           [0,     1], 
           [0,     0],
           [0,     0], 
           [0,     0],
           [1,     0]])

# LQR weights (from robognat paper)
QQ = np.diag([10, 10, 100, 10, 100, 10]) 
RR = np.diag([1, 1e-3])

print('State cost matrix QQ diagonal:', QQ.diagonal())
print('Control cost matrix RR diagonal:', RR.diagonal())
print()

# Compute LQR gains
K, S, E = ct.lqr(A, B, QQ, RR)
print('Computed LQR gains K:')
print(K)
print()
print('Shape:', K.shape)
print()
print('K[0] (z acceleration gains):', K[0])
print('K[1] (angular acceleration gains):', K[1])
print()

print('Copy-paste for trajectory generator:')
print('K = array([')
print(f'    {K[0]},  # u[0]: z accel')
print(f'    {K[1]}   # u[1]: angular accel')
print('])') 