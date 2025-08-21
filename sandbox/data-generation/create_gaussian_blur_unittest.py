import argparse
import numpy as np
import cv2
import os

def print_as_c_array_uint8(arr, varname="g"):
  lines = [f"{varname} = {{"]
  for row in arr:
    row_str = ", ".join(f"{int(round(v))}" for v in row)
    lines.append(f"  {{ {row_str} }},")
  lines.append("};")
  return "\n".join(lines)

def print_as_c_array_float(arr, varname="g"):
  lines = [f"{varname} = {{"]
  for row in arr:
    row_str = ", ".join(f"{v:.6f}" for v in row)
    lines.append(f"  {{ {row_str} }},")
  lines.append("};")
  return "\n".join(lines)

def generate_image(pattern, H, W, seed=None):
  if pattern == "impulse":
    img = np.zeros((H, W), dtype=np.uint8)
    img[H // 2, W // 2] = 255
  elif pattern == "random":
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 256, size=(H, W), dtype=np.uint8)
  elif pattern == "gradient":
    x_grad = np.linspace(0, 255, W, dtype=np.uint8)
    img = np.tile(x_grad, (H, 1))
  else:
    raise ValueError(f"Unsupported pattern: {pattern}")
  return img

def main():
  parser = argparse.ArgumentParser(description="Generate Gaussian blur test data as C arrays.")
  parser.add_argument("--height", type=int, default=7, help="Image height")
  parser.add_argument("--width", type=int, default=7, help="Image width")
  parser.add_argument("--kernels", type=int, nargs="+", default=[3, 5, 7], help="List of kernel sizes")
  parser.add_argument("--pattern", choices=["impulse", "random", "gradient"], default="impulse", help="Pattern type")
  parser.add_argument("--seed", type=int, default=42, help="Random seed for 'random' pattern")
  parser.add_argument("--output", type=str, default=None, help="Output file name (optional)")
  args = parser.parse_args()

  if args.output is None:
    args.output = f"gaussian_{args.pattern}_{args.height}x{args.width}.txt"

  H, W = args.height, args.width
  results = []

  img = generate_image(args.pattern, H, W, seed=args.seed)

  results.append("// -------- original image --------")
  results.append(print_as_c_array_uint8(img, varname="orig"))

  for k in args.kernels:
    blurred = cv2.GaussianBlur(img.astype(float), (k, k), sigmaX=0)

    results.append(f"// -------- kernel size {k} uint8_t --------")
    results.append(print_as_c_array_uint8(blurred, varname=f"g_k{k}_uint8"))

    results.append(f"// -------- kernel size {k} float --------")
    results.append(print_as_c_array_float(blurred.astype(np.float32), varname=f"g_k{k}_float"))

  with open(args.output, "w") as f:
    f.write("\n\n".join(results))

  print(f"Output written to {os.path.abspath(args.output)}")

if __name__ == "__main__":
  main()
