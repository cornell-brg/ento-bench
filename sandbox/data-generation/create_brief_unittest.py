import cv2
import math
import numpy as np
np.set_printoptions(linewidth=200)
# The 7x7 patch
patch = np.array([
  [  0,   0,  10,  12,  12,   0,   0],
  [  0,  12,   0,   0,   0,  20,   0],
  [ 10,   0,   0,   0,   0,   0,  13],
  [ 20,   0,   0, 255,   0,   0,  14],
  [ 30,   0,   0,   0,   0,   0,  14],
  [  0,  20,   0,   0,   0,  25,   0],
  [  0,   0,  10,  12,  12,   0,   0]
], dtype=np.uint8)

# Create a 31x31 blank image
img = np.zeros((31, 31), dtype=np.uint8)

# Center coordinates for the patch
start_x = 15 - patch.shape[1] // 2
start_y = 15 - patch.shape[0] // 2

# Paste the patch into the center
img[start_y:start_y+7, start_x:start_x+7] = patch

# Write the image in ASCII PGM (P2) format with aligned spacing
with open("test_img3_brief_centered.txt", "w") as f:
  f.write("P2\n")
  f.write("31 31\n")
  f.write("255\n")
  for row in img:
    row_str = "".join(f"{val:>3} " for val in row).rstrip()
    f.write(row_str + "\n")

# BRIEF test pattern (use a fixed one for now — 256 tests)
# Each pair is (dy1, dx1, dy2, dx2) relative to keypoint
np.random.seed(42)  # Make reproducible

bit_pattern_flat = [
    8,-3, 9,5,         
    4,2, 7,-12,    
    -11,9, -8,2,   
    7,-12, 12,-13,  
    2,-13, 2,12,    
    1,-7, 1,6,      
    -2,-10, -2,-4,  
    -13,-13, -11,-8,
    -13,-3, -12,-9, 
    10,4, 11,9,     
    -13,-8, -8,-9,  
    -11,7, -9,12,   
    7,7, 12,6,      
    -4,-5, -3,0,    
    -13,2, -12,-3,  
    -9,0, -7,5,     
    12,-6, 12,-1,   
    -3,6, -2,12,    
    -6,-13, -4,-8,  
    11,-13, 12,-8,  
    4,7, 5,1,       
    5,-3, 10,-3,    
    3,-7, 6,12,     
    -8,-7, -6,-2,   
    -2,11, -1,-10,  
    -13,12, -8,10,  
    -7,3, -5,-3,    
    -4,2, -3,7,     
    -10,-12, -6,11, 
    5,-12, 6,-7,    
    5,-6, 7,-1,     
    1,0, 4,-5,      
    9,11, 11,-13,   
    4,7, 4,12,      
    2,-1, 4,4,      
    -4,-12, -2,7,   
    -8,-5, -7,-10,  
    4,11, 9,12,     
    0,-8, 1,-13,    
    -13,-2, -8,2,   
    -3,-2, -2,3,    
    -6,9, -4,-9,    
    8,12, 10,7,     
    0,9, 1,3,       
    7,-5, 11,-10,   
    -13,-6, -11,0,  
    10,7, 12,1,     
    -6,-3, -6,12,   
    10,-9, 12,-4,   
    -13,8, -8,-12,  
    -13,0, -8,-4,   
    3,3, 7,8,       
    5,7, 10,-7,     
    -1,7, 1,-12,    
    3,-10, 5,6,     
    2,-4, 3,-10,    
    -13,0, -13,5,   
    -13,-7, -12,12, 
    -13,3, -11,8,   
    -7,12, -4,7,    
    6,-10, 12,8,    
    -9,-1, -7,-6,   
    -2,-5, 0,12,    
    -12,5, -7,5,    
    3,-10, 8,-13,   
    -7,-7, -4,5,    
    -3,-2, -1,-7,   
    2,9, 5,-11,     
    -11,-13, -5,-13,
    -1,6, 0,-1,     
    5,-3, 5,2,      
    -4,-13, -4,12,  
    -9,-6, -9,6,    
    -12,-10, -8,-4, 
    10,2, 12,-3,    
    7,12, 12,12,    
    -7,-13, -6,5,   
    -4,9, -3,4,     
    7,-1, 12,2,     
    -7,6, -5,1,     
    -13,11, -12,5,  
    -3,7, -2,-6,    
    7,-8, 12,-7,    
    -13,-7, -11,-12,
    1,-3, 12,12,    
    2,-6, 3,0,      
    -4,3, -2,-13,   
    -1,-13, 1,9,    
    7,1, 8,-6,      
    1,-1, 3,12,     
    9,1, 12,6,      
    -1,-9, -1,3,    
    -13,-13, -10,5, 
    7,7, 10,12,     
    12,-5, 12,9,    
    6,3, 7,11,      
    5,-13, 6,10,    
    2,-12, 2,3,     
    3,8, 4,-6,      
    2,6, 12,-13,    
    9,-12, 10,3,    
    -8,4, -7,9,     
    -11,12, -4,-6,  
    1,12, 2,-8,     
    6,-9, 7,-4,     
    2,3, 3,-2,      
    6,3, 11,0,      
    3,-3, 8,-8,     
    7,8, 9,3,       
    -11,-5, -6,-4,  
    -10,11, -5,10,  
    -5,-8, -3,12,   
    -10,5, -9,0,    
    8,-1, 12,-6,    
    4,-6, 6,-11,    
    -10,12, -8,7,   
    4,-2, 6,7,      
    -2,0, -2,12,    
    -5,-8, -5,2,    
    7,-6, 10,12,    
    -9,-13, -8,-8,  
    -5,-13, -5,-2,  
    8,-8, 9,-13,    
    -9,-11, -9,0,   
    1,-8, 1,-2,     
    7,-4, 9,1,      
    -2,1, -1,-4,    
    11,-6, 12,-11,  
    -12,-9, -6,4,   
    3,7, 7,12,      
    5,5, 10,8,      
    0,-4, 2,8,      
    -9,12, -5,-13,  
    0,7, 2,12,      
    -1,2, 1,7,      
    5,11, 7,-9,     
    3,5, 6,-8,      
    -13,-4, -8,9,   
    -5,9, -3,-3,    
    -4,-7, -3,-12,  
    6,5, 8,0,       
    -7,6, -6,12,    
    -13,6, -5,-2,   
    1,-10, 3,10,    
    4,1, 8,-4,      
    -2,-2, 2,-13,   
    2,-12, 12,12,   
    -2,-13, 0,-6,   
    4,1, 9,3,       
    -6,-10, -3,-5,  
    -3,-13, -1,1,   
    7,5, 12,-11,    
    4,-2, 5,-7,     
    -13,9, -9,-5,   
    7,1, 8,6,       
    7,-8, 7,6,      
    -7,-4, -7,1,    
    -8,11, -7,-8,   
    -13,6, -12,-8,  
    2,4, 3,9,       
    10,-5, 12,3,    
    -6,-5, -6,7,    
    8,-3, 9,-8,     
    2,-12, 2,8,     
    -11,-2, -10,3,  
    -12,-13, -7,-9, 
    -11,0, -10,-5,  
    5,-3, 11,8,     
    -2,-13, -1,12,  
    -1,-8, 0,9,     
    -13,-11, -12,-5,
    -10,-2, -10,11, 
    -3,9, -2,-13,   
    2,-3, 3,2,      
    -9,-13, -4,0,   
    -4,6, -3,-10,   
    -4,12, -2,-7,   
    -6,-11, -4,9,   
    6,-3, 6,11,     
    -13,11, -5,5,   
    11,11, 12,6,    
    7,-5, 12,-2,    
    -1,12, 0,7,     
    -4,-8, -3,-2,   
    -7,1, -6,7,     
    -13,-12, -8,-13,
    -7,-2, -6,-8,   
    -8,5, -6,-9,    
    -5,-1, -4,5,    
    -13,7, -8,10,   
    1,5, 5,-13,     
    1,0, 10,-13,    
    9,12, 10,-1,    
    5,-8, 10,-9,    
    -1,11, 1,-13,   
    -9,-3, -6,2,    
    -1,-10, 1,12,   
    -13,1, -8,-10,  
    8,-11, 10,-6,   
    2,-13, 3,-6,    
    7,-13, 12,-9,   
    -10,-10, -5,-7, 
    -10,-8, -8,-13, 
    4,-6, 8,5,      
    3,12, 8,-13,    
    -4,2, -3,-3,    
    5,-13, 10,-12,  
    4,-13, 5,-1,    
    -9,9, -4,3,     
    0,3, 3,-9,      
    -12,1, -6,1,    
    3,2, 4,-8,      
    -10,-10, -10,9, 
    8,-13, 12,12,   
    -8,-12, -6,-5,  
    2,2, 3,7,       
    10,6, 11,-8,    
    6,8, 8,-12,     
    -7,10, -6,5,    
    -3,-9, -3,9,    
    -1,-13, -1,5,   
    -3,-7, -3,4,    
    -8,-2, -8,3,    
    4,2, 12,12,     
    2,-5, 3,11,     
    6,-9, 11,-13,   
    3,-1, 7,12,     
    11,-1, 12,4,    
    -3,0, -3,6,     
    4,-11, 4,12,    
    2,-4, 2,1,      
    -10,-6, -8,1,   
    -13,7, -11,1,   
    -13,12, -11,-13,
    6,0, 11,-13,    
    0,-1, 1,4,      
    -13,3, -9,-2,   
    -9,8, -6,-3,    
    -13,-6, -8,-2,  
    5,-9, 8,10,     
    2,7, 3,-9,      
    -1,-6, -1,-1,   
    9,5, 11,-2,     
    11,-3, 12,-8,   
    3,0, 3,5,       
    -1,4, 0,10,     
    3,-6, 4,5,      
    -13,0, -10,5,   
    5,8, 12,11,     
    8,9, 9,-6,      
    7,-4, 8,-12,    
    -10,4, -10,9,   
    7,3, 12,4,      
    9,-7, 10,-2,    
    7,0, 12,-2,     
    -1,-6, 0,-11,   
]
assert len(bit_pattern_flat) % 4 == 0, "Pattern length must be divisible by 4"
pattern = [(bit_pattern_flat[i], bit_pattern_flat[i+1],
            bit_pattern_flat[i+2], bit_pattern_flat[i+3])
           for i in range(0, len(bit_pattern_flat), 4)]

print("constexpr std::array<PatternPoint, BRIEF_NUM_PAIRS> bit_pattern_31 = {{")
for i, (dy1, dx1, dy2, dx2) in enumerate(pattern):
  line = f"  {{{dy1:>3}, {dx1:>3}, {dy2:>3}, {dx2:>3}}}"
  if i != len(pattern) - 1:
    line += ","
  if i % 4 == 3:
    print(line)
  else:
    print(line, end=" ")
print("}};")


key_y, key_x = 15, 15  # center of image

# Compute BRIEF descriptor
desc_bits = []
for dy1, dx1, dy2, dx2 in pattern:
  y1, x1 = key_y + dy1, key_x + dx1
  y2, x2 = key_y + dy2, key_x + dx2

  if (0 <= y1 < 31 and 0 <= x1 < 31 and
      0 <= y2 < 31 and 0 <= x2 < 31):
    bit = int(img[y1, x1] < img[y2, x2])
  else:
    bit = 0  # Out-of-bounds gets 0
  desc_bits.append(bit)

# Convert bits to bytes
little_endian = True  # Set to False for big-endian (original Python style)

desc_bytes = []
for i in range(0, 256, 8):
  byte_bits = desc_bits[i:i+8]
  if little_endian:
    byte_bits = list(reversed(byte_bits))  # Reverse bits within byte
  byte_str = ''.join(str(b) for b in byte_bits)
  desc_bytes.append(int(byte_str, 2))

# Output
desc_hex = ''.join(f"{b:02x}" for b in desc_bytes)

print("BRIEF descriptor (hex):")
print(desc_hex)

print("BRIEF descriptor (byte values):")
print(','.join(str(b) for b in desc_bytes))

orb = cv2.ORB_create(
  nfeatures=10,       
  scaleFactor=1.2,   
  nlevels=1,         
  edgeThreshold=0,   
  patchSize=31,     
  fastThreshold=30    
)

# Detect keypoints and compute descriptors
kps, descs = orb.detectAndCompute(img, None)

# Print all detected keypoints and their descriptors
for i, (kp, d) in enumerate(zip(kps, descs)):
  print(f"Keypoint #{i}: (x={kp.pt[0]:.1f}, y={kp.pt[1]:.1f})")
  print(f"  Angle (deg): {kp.angle:.2f}")
  print(f"  Angle (rad): {math.radians(kp.angle):.3f}")
  print(f"  Descriptor: {list(d)}")


print(img[14,24], img[6,20])

import math

# Assume: pattern, img, key_x, key_y, and angle_rad are already defined
angle_rad = math.radians(119.87)
cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
img_blur = cv2.GaussianBlur(img, (7, 7), 2)

desc_bits = []

for i, (dx1, dy1, dx2, dy2) in enumerate(pattern):
  # Pre-rotation absolute coordinates
  pre_x1, pre_y1 = key_x + dx1, key_y + dy1
  pre_x2, pre_y2 = key_x + dx2, key_y + dy2

  # Rotated offsets
  rx1 = dx1 * cos_a - dy1 * sin_a
  ry1 = dx1 * sin_a + dy1 * cos_a
  rx2 = dx2 * cos_a - dy2 * sin_a
  ry2 = dx2 * sin_a + dy2 * cos_a

  # Rotated absolute coordinates
  x1 = int(round(key_x + rx1))
  y1 = int(round(key_y + ry1))
  x2 = int(round(key_x + rx2))
  y2 = int(round(key_y + ry2))

  if (0 <= x1 < 31 and 0 <= y1 < 31 and 0 <= x2 < 31 and 0 <= y2 < 31):
    val1, val2 = img_blur[y1, x1], img_blur[y2, x2]
    bit = int(val1 < val2)
  else:
    bit = 0  # out-of-bounds fallback

  desc_bits.append(bit)
  print(f"i={i:3} | Pre-rot: ({pre_y1:2},{pre_x1:2}) vs ({pre_y2:2},{pre_x2:2})"
        f" | Rot: ({y1:2},{x1:2})={val1 if 'val1' in locals() else 'X'}"
        f" vs ({y2:2},{x2:2})={val2 if 'val2' in locals() else 'X'} | bit={bit}")

# Convert bits to bytes
desc_bytes = []
for i in range(0, 256, 8):
  byte_bits = desc_bits[i:i+8]
  byte_bits = list(reversed(byte_bits))  # ORB uses little-endian bit packing
  byte_str = ''.join(str(b) for b in byte_bits)
  desc_bytes.append(int(byte_str, 2))

print("\nDescriptor (byte values):")
print(desc_bytes)

print("\nDescriptor (hex):")
print(''.join(f"{b:02x}" for b in desc_bytes))

print(img_blur)
kernel = cv2.getGaussianKernel(7, 2).flatten()

for ksize in [3, 5, 7]:
    kernel = cv2.getGaussianKernel(ksize, 2).flatten()
    formatted = ", ".join(f"{x:.8f}" for x in kernel)
    print(f"// Kernel size: {ksize} (sigma = 2)")
    print(f"return {{{formatted}}};\n")
