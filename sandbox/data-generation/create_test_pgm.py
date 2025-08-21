#!/usr/bin/env python3
import numpy as np
import math

def create_gaussian_blob_pgm(base_filename, width=32, height=32, center_x=15.5, center_y=15.5, sigma=4.0, amplitude=255):
    """Create both binary (P5) and ASCII (P2) PGM images with a Gaussian blob pattern"""
    
    # Create coordinate grids
    x = np.arange(width)
    y = np.arange(height)
    X, Y = np.meshgrid(x, y)
    
    # Calculate distances from center
    dx = X - center_x
    dy = Y - center_y
    distance_sq = dx**2 + dy**2
    
    # Generate Gaussian blob with FULL amplitude
    image = amplitude * np.exp(-distance_sq / (2.0 * sigma**2))
    
    # Convert to integer and clamp to 0-255
    image = np.round(image).astype(np.uint8)
    image = np.clip(image, 0, 255)
    
    # Create binary version (P5)
    binary_filename = base_filename.replace('.pgm', '_binary.pgm')
    with open(binary_filename, 'wb') as f:
        # PGM header (ASCII)
        f.write(b'P5\n')
        f.write(f'{width} {height}\n'.encode('ascii'))
        f.write(b'255\n')
        
        # Binary image data
        f.write(image.tobytes())
    
    # Create ASCII version (P2)
    ascii_filename = base_filename.replace('.pgm', '_ascii.pgm')
    with open(ascii_filename, 'w') as f:
        # PGM header
        f.write("P2\n")
        f.write(f"{width} {height}\n")
        f.write("255\n")
        
        # ASCII image data
        for y in range(height):
            for x in range(width):
                f.write(f"{image[y, x]} ")
            f.write("\n")
    
    print(f"Created binary version: {binary_filename}")
    print(f"Created ASCII version: {ascii_filename}")
    print(f"Image stats: min={image.min()}, max={image.max()}, center={image[16,16]}")
    
    return binary_filename, ascii_filename

if __name__ == "__main__":
    binary_file, ascii_file = create_gaussian_blob_pgm("external/vedaldi2006-siftpp/blob_32x32_test.pgm") 