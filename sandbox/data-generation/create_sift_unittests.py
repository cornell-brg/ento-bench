from abc import ABC, abstractmethod
import numpy as np
import cv2
import argparse
import math

class ScalarFormat(ABC):
    @property
    @abstractmethod
    def type_str(self) -> str:
        pass

    @abstractmethod
    def format_value(self, v) -> str:
        pass

    @abstractmethod
    def comment(self) -> str:
        pass

class Uint8Format(ScalarFormat):
    @property
    def type_str(self):
        return "uint8_t"

    def format_value(self, v):
        return str(int(v))

    def comment(self):
        return "// uint8_t image data"

class FixedFormat(ScalarFormat):
    def __init__(self, total_bits=16, frac_bits=8):
        self.total_bits = total_bits
        self.frac_bits  = frac_bits
        self.int_bits   = total_bits - frac_bits

    @property
    def type_str(self):
        return f'int{self.total_bits}_t'

    def format_value(self, v):
        return str(int(round(v * (1 << self.frac_bits))))

    def comment(self):
        return f"// fixed-point Q{self.int_bits}.{self.frac_bits} format"

class FloatFormat(ScalarFormat):

    @property
    def type_str(self):
        return "float"
    
    def format_value(self, v):
        return f"{v:.12f}"

    def comment(self):
        return "// float format"

class DoubleFormat(ScalarFormat):

    @property
    def type_str(self):
        return "double"

    def format_value(self, v):
        return f"{v:.12f}"

    def comment(self):
        return "// double format"

def format_named_array(arrs, name: str, fmt: ScalarFormat) -> str:
    lines = []
    lines.append(fmt.comment())
    lines.append(f"{fmt.type_str} {name}[{len(arrs)}][{arrs[0].shape[0]}][{arrs[0].shape[1]}] = {{")
    for arr in arrs:
        lines.append("  {")
        for row in arr:
            row_str = ", ".join(fmt.format_value(v) for v in row)
            lines.append(f"    {{ {row_str} }},")
        lines.append("  },")
        lines.append("")  # space between layers
    lines.append("};")
    return "\n".join(lines)

def generate_image(pattern, H, W, seed=None):
    if pattern == "impulse":
        img = np.zeros((H, W), dtype=np.uint8)
        img[H // 2, W // 2] = 255
    elif pattern == "blob-impulse":
        img = np.zeros((32, 32), dtype=np.uint8)
        cv2.circle(img, center=(16, 16), radius=3, color=255, thickness=-1)
        img = cv2.GaussianBlur(img, ksize=(5, 5), sigmaX=1.0)
        # Add salt and pepper noise to avoid all-zeros regions that cause extrema detection issues
        rng = np.random.default_rng(seed if seed is not None else 42)
        img = img.astype(np.float32)
        
        # Add small amount of salt and pepper noise
        noise_prob = 0.1  # 10% of pixels get noise
        noise_mask = rng.random(img.shape) < noise_prob
        salt_mask = rng.random(img.shape) < 0.5  # 50% salt, 50% pepper
        
        # Salt noise (bright spots): add 5-15 to selected pixels
        salt_noise = noise_mask & salt_mask
        img[salt_noise] += rng.uniform(5, 15, size=np.sum(salt_noise))
        
        # Pepper noise (dark spots): add 1-8 to selected pixels (small positive values)
        pepper_noise = noise_mask & ~salt_mask
        img[pepper_noise] += rng.uniform(1, 8, size=np.sum(pepper_noise))
        
        img = np.clip(img, 0, 255).astype(np.uint8)
    elif pattern == "random":
        rng = np.random.default_rng(seed)
        img = rng.integers(0, 256, size=(H, W), dtype=np.uint8)
    elif pattern == "gradient":
        x_grad = np.linspace(0, 255, W, dtype=np.uint8)
        img = np.tile(x_grad, (H, 1))
    else:
        raise ValueError(f"Unsupported pattern: {pattern}")
    return img

def detect_extrema(dogs):
    extrema = []
    num_layers = len(dogs)
    H, W = dogs[0].shape

    for s in range(1, num_layers - 1):  # exclude boundary layers
        prev = dogs[s - 1]
        curr = dogs[s]
        next_ = dogs[s + 1]

        for y in range(1, H - 1):
            for x in range(1, W - 1):
                val = curr[y, x]

                neighborhood = []
                for dz, img in zip([-1, 0, 1], [prev, curr, next_]):
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dz == 0 and dx == 0 and dy == 0:
                                continue
                            neighborhood.append(img[y + dy, x + dx])

                if all(val > n for n in neighborhood) or all(val < n for n in neighborhood):
                    extrema.append((x, y, s, val))
    return extrema

def simulate_opencv_createInitialImage(img, doubleImageSize=True, sigma=1.6, enable_precise_upscale=False):
    """
    Simulate OpenCV's createInitialImage function to create the exact input 
    that OpenCV processes internally. This allows fair comparison with MCU implementation.
    """
    SIFT_INIT_SIGMA = 0.5  # OpenCV assumes input has this much blur
    SIFT_FIXPT_SCALE = 1   # For float images
    
    # Convert to float (like OpenCV does)
    gray_fpt = img.astype(np.float32) * SIFT_FIXPT_SCALE
    
    if doubleImageSize:
        # Calculate sigma difference (accounting for 2x upsampling effect)
        sig_diff = np.sqrt(max(sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4, 0.01))
        
        if enable_precise_upscale:
            # Use warpAffine like OpenCV (more precise)
            dbl = np.zeros((gray_fpt.shape[0]*2, gray_fpt.shape[1]*2), dtype=np.float32)
            H = np.array([[0.5, 0, 0], [0, 0.5, 0]], dtype=np.float32)
            dbl = cv2.warpAffine(gray_fpt, H, (gray_fpt.shape[1]*2, gray_fpt.shape[0]*2), 
                               flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, 
                               borderMode=cv2.BORDER_REFLECT)
        else:
            # Use resize like OpenCV (faster)
            dbl = cv2.resize(gray_fpt, (gray_fpt.shape[1]*2, gray_fpt.shape[0]*2), 
                           interpolation=cv2.INTER_LINEAR)
        
        # Apply Gaussian blur
        result = cv2.GaussianBlur(dbl, (0, 0), sig_diff, sig_diff)
        return result
    else:
        # No doubling case
        sig_diff = np.sqrt(max(sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA, 0.01))
        result = cv2.GaussianBlur(gray_fpt, (0, 0), sig_diff, sig_diff)
        return result

def main():
    parser = argparse.ArgumentParser(description="Generate Gaussian blur test data as C arrays.")
    parser.add_argument("--height", type=int, default=7, help="Image height")
    parser.add_argument("--width", type=int, default=7, help="Image width")
    parser.add_argument("--kernel-size", type=int, default=5, help="Gaussian kernel size")
    parser.add_argument("--pattern", choices=["impulse", "blob-impulse", "random", "gradient"], default="impulse", help="Pattern type")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for 'random' pattern")
    parser.add_argument("--output", type=str, default=None, help="Output file name (optional)")
    parser.add_argument("--num-dog-layers", type=int, default=4, help="Number of DoG layers.")
    parser.add_argument("--format", choices=["float", "double", "fixed"], default="float", help="Output data format")
    parser.add_argument("--frac-bits", type=int, default=8, help="Fractional bits for fixed-point format (is used)")
    parser.add_argument("--dec-bits", type=int, default=8, help="Decimal bits for fixed-point format (is used)")
    args = parser.parse_args()

    if args.format == "float":
        fmt = FloatFormat()
    elif args.format == "double":
        fmt = DoubleFormat()
    elif args.format == "fixed":
        total_bits = args.frac_bits + args.dec_bits
        fmt = FixedFormat(total_bits=total_bits, frac_bits=args.frac_bits)
    else:
        raise ValueError("Unsupported Format")

    if args.output is None:
        args.output = f"sift_{args.pattern}_{args.height}x{args.width}_{fmt.type_str}.txt"

    H, W = args.height, args.width
    num_dog_layers = args.num_dog_layers
    num_blur_levels = num_dog_layers + 1
    kernel_sz = args.kernel_size
    img = generate_image(args.pattern, H, W, seed=args.seed).astype(float)

    num_blur_levels = num_dog_layers + 1
    base_sigma = 1.6
    k = math.pow(2, 1.0 / num_dog_layers)

    gaussians = []
    # Pre-blur the input image with sigma=0.5 (standard SIFT assumption)
    initial_sigma = 0.0
    #current_img = cv2.GaussianBlur(img, (5,5), sigmaX=initial_sigma, sigmaY=initial_sigma)
    current_img = img.copy()
    
    # Apply incremental blurring to create all Gaussian levels
    sigma_prev = initial_sigma
    for i in range(num_blur_levels):
        # Calculate target sigma and delta for incremental blur
        sigma_curr = base_sigma * (k**i)  # 1.6, 1.6*k, 1.6*k^2, etc.
        delta_sigma = math.sqrt(sigma_curr*sigma_curr - sigma_prev*sigma_prev)
        
        print(f"Python: gaussian[{i}] sigma_curr={sigma_curr:.6f}, sigma_prev={sigma_prev:.6f}, delta={delta_sigma:.6f}")
        
        # Apply incremental blur to current image
        current_img = cv2.GaussianBlur(current_img, (5,5), sigmaX=delta_sigma, sigmaY=delta_sigma)
        gaussians.append(current_img)
        sigma_prev = sigma_curr

    # now print the 5×5 around (16,16):
    cy, cx = H//2, W//2  # 16,16
    y0, x0 = cy - 2, cx - 2

    np.set_printoptions(formatter={'float': '{:6.2f}'.format})
    for i, g in enumerate(gaussians):
        window = g[y0:y0+5, x0:x0+5]
        print(f"python gaussian[{i}] @ ({cy},{cx}):")
        print(window)
        print()

    golden_dogs = []
    for i in range(num_dog_layers):
        golden_dog = gaussians[i+1] - gaussians[i]
        golden_dogs.append(golden_dog)

    extrema = detect_extrema(golden_dogs)
    print(f"Detected {len(extrema)} extrema (pre-interpolation):")
    for x, y, s, val in extrema:
        print(f"  → (x={x}, y={y}, s={s}) = {val:.2f}")

    print(len(golden_dogs))
    output_lines = []
    
    # Add necessary headers
    output_lines.append("#include <cstdint>")
    output_lines.append("")
    
    # NOTE: Array generation moved after doubled image processing

    # Test different SIFT configurations to debug the multiple identical keypoints issue
    print("=== Testing different SIFT configurations ===")
    
    # Test 1: Default SIFT
    sift_default = cv2.SIFT_create()
    kps_default = sift_default.detect(img.astype(np.uint8), None)
    print(f"Default SIFT: {len(kps_default)} keypoints")
    
    # Test 2: SIFT with contrast threshold 0.03
    sift = cv2.SIFT_create()
    kps = sift.detect(img.astype(np.uint8), None)
    print(f"SIFT (contrast=0.03): {len(kps)} keypoints")
    
    # Test 3: SIFT with higher contrast threshold
    sift_high = cv2.SIFT_create(contrastThreshold=0.1)
    kps_high = sift_high.detect(img.astype(np.uint8), None)
    print(f"SIFT (contrast=0.1): {len(kps_high)} keypoints")
    
    # Test 4: Try with impulse pattern instead of blob-impulse
    if args.pattern == "blob-impulse":
        impulse_img = np.zeros((32, 32), dtype=np.uint8)
        impulse_img[16, 16] = 255
        kps_impulse = sift.detect(impulse_img, None)
        print(f"SIFT on pure impulse: {len(kps_impulse)} keypoints")
    
    print()
    
    # NEW: Create preprocessed images like OpenCV does internally
    print("=== Testing OpenCV Preprocessing Simulation ===")
    
    # Create the exact image that OpenCV processes internally (doubled + blurred)
    img_doubled = simulate_opencv_createInitialImage(img, doubleImageSize=True, sigma=1.6)
    print(f"Created doubled image: {img.shape} → {img_doubled.shape}")
    
    # Create the image for MCU (no doubling, just initial blur)
    img_mcu = simulate_opencv_createInitialImage(img, doubleImageSize=False, sigma=1.6)
    print(f"Created MCU image: {img.shape} → {img_mcu.shape}")
    
    # Test OpenCV on original (it will double internally)
    kps_original = sift.detect(img.astype(np.uint8), None)
    print(f"OpenCV on original 32x32 (internal doubling): {len(kps_original)} keypoints")
    
    # Show what the doubled image looks like
    print(f"\nDoubled image stats:")
    print(f"  Shape: {img_doubled.shape}")
    print(f"  Min/Max: {img_doubled.min():.2f}/{img_doubled.max():.2f}")
    print(f"  Center 5x5:")
    cy, cx = img_doubled.shape[0]//2, img_doubled.shape[1]//2
    center_window = img_doubled[cy-2:cy+3, cx-2:cx+3]
    np.set_printoptions(formatter={'float': '{:6.2f}'.format})
    print(center_window)
    
    print(f"\nMCU image stats:")
    print(f"  Shape: {img_mcu.shape}")
    print(f"  Min/Max: {img_mcu.min():.2f}/{img_mcu.max():.2f}")
    print(f"  Center 5x5:")
    cy, cx = img_mcu.shape[0]//2, img_mcu.shape[1]//2
    center_window = img_mcu[cy-2:cy+3, cx-2:cx+3]
    print(center_window)
    
    print("\nOpenCV keypoints on original image:")
    for i, kp in enumerate(kps_original):
        octave = kp.octave & 0xFF
        layer = (kp.octave >> 8) & 0xFF
        print(f"  KP{i}: ({kp.pt[0]:.3f}, {kp.pt[1]:.3f}) size={kp.size:.3f} resp={kp.response:.6f} oct={octave} layer={layer}")
    
    # Save preprocessed images as C arrays for MCU testing
    print("\n=== Saving Preprocessed Images for MCU Testing ===")
    
    # Save doubled image (what OpenCV processes internally)
    img_doubled_uint8 = np.clip(img_doubled, 0, 255).astype(np.uint8)
    
    # Save MCU image (no doubling, just initial blur)
    img_mcu_uint8 = np.clip(img_mcu, 0, 255).astype(np.uint8)
    
    # Write doubled image array
    with open("src/ento-feature2d/test/test_sift_doubled_image.h", "w") as f:
        f.write("#pragma once\n")
        f.write("#include <cstdint>\n\n")
        doubled_array_str = format_named_array([img_doubled_uint8], "test_image_doubled", Uint8Format())
        f.write(doubled_array_str)
        f.write(f"const int test_image_doubled_width = {img_doubled.shape[1]};\n")
        f.write(f"const int test_image_doubled_height = {img_doubled.shape[0]};\n")
    
    # Write MCU image array  
    with open("src/ento-feature2d/test/test_sift_mcu_image.h", "w") as f:
        f.write("#pragma once\n")
        f.write("#include <cstdint>\n\n")
        mcu_array_str = format_named_array([img_mcu_uint8], "test_image_mcu", Uint8Format())
        f.write(mcu_array_str)
        f.write(f"const int test_image_mcu_width = {img_mcu.shape[1]};\n")
        f.write(f"const int test_image_mcu_height = {img_mcu.shape[0]};\n")
    
    print(f"Saved doubled image to test_sift_doubled_image.h ({img_doubled.shape})")
    print(f"Saved MCU image to test_sift_mcu_image.h ({img_mcu.shape})")
    print()
    
    # Use original results for main output (as requested)
    chosen_keypoints = kps_original
    
    # REPLACE: Generate Gaussian and DoG pyramids from the DOUBLED image instead of original
    print("\n=== Generating Gaussian/DoG pyramids from doubled image ===")
    
    # Use the doubled image as the base for pyramid generation
    base_img = img_doubled.astype(np.float32)
    H, W = base_img.shape
    print(f"Base image shape: {base_img.shape}")
    
    # Generate Gaussian pyramid from doubled image
    gaussians_doubled = []
    current_img = base_img.copy()
    
    # First layer is the doubled+blurred image itself
    gaussians_doubled.append(current_img)
    
    # Generate remaining layers with incremental blur
    k = 2**(1/3)  # 2^(1/s) where s=3 intervals per octave
    base_sigma = 1.6
    sigma_prev = base_sigma  # The doubled image already has sigma=1.6 effective
    
    for i in range(1, num_dog_layers + 1):
        sigma_curr = base_sigma * (k**i)
        delta_sigma = math.sqrt(sigma_curr*sigma_curr - sigma_prev*sigma_prev)
        
        print(f"Doubled pyramid: gaussian[{i}] sigma_curr={sigma_curr:.6f}, sigma_prev={sigma_prev:.6f}, delta={delta_sigma:.6f}")
        
        current_img = cv2.GaussianBlur(current_img, (5,5), sigmaX=delta_sigma, sigmaY=delta_sigma)
        gaussians_doubled.append(current_img)
        sigma_prev = sigma_curr
    
    # Show center 5x5 for doubled pyramids
    cy, cx = H//2, W//2  # Center of 64x64 image
    y0, x0 = cy - 2, cx - 2
    
    np.set_printoptions(formatter={'float': '{:6.2f}'.format})
    for i, g in enumerate(gaussians_doubled):
        window = g[y0:y0+5, x0:x0+5]
        print(f"doubled gaussian[{i}] @ ({cy},{cx}):")
        print(window)
        print()
    
    # Generate DoG pyramid from doubled Gaussians
    golden_dogs_doubled = []
    for i in range(num_dog_layers):
        golden_dog = gaussians_doubled[i+1] - gaussians_doubled[i]
        golden_dogs_doubled.append(golden_dog)
    
    # Detect extrema in doubled DoG pyramid
    extrema_doubled = detect_extrema(golden_dogs_doubled)
    print(f"Detected {len(extrema_doubled)} extrema in doubled pyramid (pre-interpolation):")
    for x, y, s, val in extrema_doubled:
        print(f"  → (x={x}, y={y}, s={s}) = {val:.2f}")
    
    # Replace the original data with doubled data for output
    img = img_doubled_uint8  # Use doubled image as base
    gaussians = gaussians_doubled  # Use doubled Gaussians
    golden_dogs = golden_dogs_doubled  # Use doubled DoGs
    
    print(f"Replaced data: img {img.shape}, {len(gaussians)} gaussians, {len(golden_dogs)} DoGs")
    
    # NOW generate the arrays with doubled data
    output_lines.append(format_named_array([img], "img", Uint8Format()))
    output_lines.append(format_named_array(gaussians, "golden_Gaussians", fmt))
    output_lines.append("")  # blank line between sections
    output_lines.append(format_named_array(golden_dogs, "golden_DoGs", fmt))

    output_lines.append("// Ground truth keypoints from OpenCV SIFT")
    
    # Debug output to understand the keypoint detection issue
    print(f"Number of keypoints detected: {len(chosen_keypoints)}")
    print("Keypoint coordinates:", [(kp.pt) for kp in chosen_keypoints])
    print("Keypoint responses:", [kp.response for kp in chosen_keypoints])
    print("Keypoint octaves:", [kp.octave for kp in chosen_keypoints])
    print("Keypoint layers:", [(kp.octave >> 8) & 0xFF for kp in chosen_keypoints])
    print()
    
    for i, kp in enumerate(chosen_keypoints):
        x, y = kp.pt
        size = kp.size
        response = kp.response
        octave = kp.octave & 0xFF
        layer  = (kp.octave >> 8) & 0xFF

        output_lines.append(f"// Keypoint {i}:")
        output_lines.append(f"//   x = {x:.4f}")
        output_lines.append(f"//   y = {y:.4f}")
        output_lines.append(f"//   size = {size:.8f}")
        output_lines.append(f"//   response = {response:.8f}")
        output_lines.append(f"//   octave = {octave}, layer = {layer}")
        output_lines.append("")
    
    # Add MCU comparison data as comments
    output_lines.append("// === MCU Implementation Reference ===")
    output_lines.append("// For MCU implementation without doubling:")
    output_lines.append(f"// - Use test_image_mcu.h ({img_mcu.shape}) as input")
    output_lines.append(f"// - Expected different results due to no image doubling")
    output_lines.append(f"// - Focus on relative performance vs exact coordinate matching")
    output_lines.append("")

    with open(args.output, 'w') as f:
        f.write("\n".join(output_lines) + "\n")

    print(f'Generate DoG output saved to: {args.output}')

if __name__ == "__main__":
    main()
