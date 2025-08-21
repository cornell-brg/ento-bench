#!/usr/bin/env python3
"""
Generate perception datasets from PNG images.

This script takes a directory containing PNG images and generates resized versions
in PGM format for use with ento-bench perception benchmarks.

Usage:
    python generate_perception_dataset.py --input /path/to/png/images --output /path/to/output
    python generate_perception_dataset.py --input my-images --format png  # Keep as PNG
"""

import os
import argparse
from PIL import Image

def process_images(input_root, output_root, out_format, sizes):
    """Process images from input directory and generate resized versions."""
    
    # Create output directories
    output_dirs = {}
    for size_name, (width, height) in sizes.items():
        out_dir = os.path.join(output_root, size_name)
        os.makedirs(out_dir, exist_ok=True)
        output_dirs[out_dir] = (width, height)
        print(f"Created output directory: {out_dir}")

    # Gather image paths
    image_paths = []
    
    # Check if input_root is a directory with subdirectories (like middlebury structure)
    if os.path.isdir(input_root):
        subdirs = [d for d in os.listdir(input_root) if os.path.isdir(os.path.join(input_root, d))]
        
        if subdirs:
            # Process subdirectories
            for subdir in sorted(subdirs):
                full_path = os.path.join(input_root, subdir)
                for fname in sorted(os.listdir(full_path)):
                    if fname.lower().endswith('.png'):
                        image_paths.append(os.path.join(full_path, fname))
        else:
            # Process files directly in the root directory
            for fname in sorted(os.listdir(input_root)):
                if fname.lower().endswith('.png'):
                    image_paths.append(os.path.join(input_root, fname))
    
    if not image_paths:
        print(f"❌ No PNG images found in {input_root}")
        return
    
    print(f"Found {len(image_paths)} PNG images to process")

    # Process each image
    for idx, img_path in enumerate(image_paths):
        print(f"Processing image {idx + 1}/{len(image_paths)}: {os.path.basename(img_path)}")
        
        with Image.open(img_path) as img:
            img_gray = img.convert('L')

            for out_dir, size in output_dirs.items():
                resized = img_gray.resize(size, Image.BICUBIC)
                width, height = resized.size
                pixels = resized.load()

                output_file = os.path.join(out_dir, f"image{idx}.{out_format.lower()}")

                if out_format.lower() == "pgm":
                    with open(output_file, "w") as f:
                        f.write("P2\n")
                        f.write(f"{width} {height}\n")
                        f.write("255\n")
                        for y in range(height):
                            row = [str(pixels[x, y]) for x in range(width)]
                            f.write(" ".join(row) + "\n")
                elif out_format.lower() == "png":
                    resized.save(output_file)
                else:
                    raise ValueError("Unsupported format. Use 'pgm' or 'png'.")

                size_name = os.path.basename(out_dir)
                print(f"  [{size_name}] Saved {output_file}")

    print(f"\n✅ Done! {len(image_paths)} images processed into {list(sizes.keys())} as .{out_format}.")

def main():
    parser = argparse.ArgumentParser(
        description="Generate perception datasets from PNG images",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with default sizes
  python generate_perception_dataset.py --input my-images --output datasets/my-dataset

  # Use custom output format
  python generate_perception_dataset.py --input my-images --output datasets/my-dataset --format png

  # Add tiny size (31x31) for very small benchmarks
  python generate_perception_dataset.py --input my-images --output datasets/my-dataset --include-tiny
        """
    )
    
    parser.add_argument("--input", type=str, required=True,
                        help="Input directory containing PNG images")
    parser.add_argument("--output", type=str, required=True,
                        help="Output directory for generated datasets")
    parser.add_argument("--format", choices=["pgm", "png"], default="pgm",
                        help="Output image format (default: pgm)")
    parser.add_argument("--include-tiny", action="store_true",
                        help="Include tiny (31x31) size for very small benchmarks")
    
    args = parser.parse_args()

    # Define standard sizes used by ento-bench perception benchmarks
    sizes = {
        'small': (80, 80),
        'medium': (160, 160),
        'large': (320, 320),
    }
    
    if args.include_tiny:
        sizes['tiny'] = (31, 31)

    print(f"Input directory: {args.input}")
    print(f"Output directory: {args.output}")
    print(f"Output format: {args.format}")
    print(f"Sizes to generate: {list(sizes.keys())}")
    print()

    if not os.path.exists(args.input):
        print(f"❌ Input directory does not exist: {args.input}")
        return 1

    process_images(args.input, args.output, args.format, sizes)
    return 0

if __name__ == "__main__":
    exit(main()) 