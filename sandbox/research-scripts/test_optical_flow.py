#!/usr/bin/env python3
import sys, os, numpy as np
from numpy import array, sin, cos

# Setup robognat import
original_dir = os.getcwd()
robognat_dir = os.path.join(original_dir, 'external/robognat_simulator')
sys.path.insert(0, robognat_dir)
os.chdir(robognat_dir)
from robognat_clean import *
os.chdir(original_dir)

# Test clean signal for a sample state
q_test = array([0.01, 0.1, 0.1, 0.05, 0.12, 0.02, 0])  # [theta, omega, x, vx, z, vz, vw]
thetay = q_test[0]
omegay = q_test[1] 
v_world = q_test[[3, 5]]  # [vx, vz]
z = q_test[4]
R2 = R(thetay)
v_body = R2.T @ v_world
cos_theta = cos(thetay)

# Clean optical flow calculation
clean_OF = omegay - v_body[0] * cos_theta / z
print(f'Clean optical flow value: {clean_OF:.4f} rad/s')
print(f'Components: omega={omegay:.4f}, v_body[0]={v_body[0]:.4f}, cos(theta)={cos_theta:.4f}, z={z:.4f}')
print(f'v_body[0]*cos(theta)/z term: {v_body[0] * cos_theta / z:.4f}')

print(f'\nNoise comparison:')
print(f'Original robognat noise std: {p.sensor_noise_std[2]}')
print(f'Our RoboFly noise std: 0.05')
print(f'Ratio: {p.sensor_noise_std[2] / 0.05:.1f}x higher noise in original') 