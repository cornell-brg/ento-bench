#!/usr/bin/env python3
import re

# Read the file
with open('external/adaptive-controller-simulink/partial_data2.h', 'r') as f:
    content = f.read()

# Extract all the data lines using regex
pattern = r'-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+'
matches = re.findall(pattern, content)

# Keep ALL points including repetitions
all_points = matches

# Write to CSV
with open('full_trajectory_data.csv', 'w') as f:
    f.write('x,y,z,roll,pitch,yaw\n')
    for point in all_points:
        f.write(point + '\n')

print(f'Extracted {len(all_points)} total trajectory points (including repetitions)')
print('First 5 points:')
for i, point in enumerate(all_points[:5]):
    print(f'{i}: {point}')
print('...')
print('Last 5 points:')
for i, point in enumerate(all_points[-5:], len(all_points)-5):
    print(f'{i}: {point}')

print(f'\nSaved to full_trajectory_data.csv') 