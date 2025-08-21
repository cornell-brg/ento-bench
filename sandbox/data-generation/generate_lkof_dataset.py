import os
import cv2
import argparse
import numpy as np
from pathlib import Path

def ensure_dir(path):
  os.makedirs(path, exist_ok=True)

def write_pgm(image, path):
  height, width = image.shape
  with open(path, "w") as f:
    f.write("P2\n")
    f.write(f"{width} {height}\n")
    f.write("255\n")
    for y in range(height):
      row = " ".join(str(int(val)) for val in image[y])
      f.write(row + "\n")

def write_features(path, points):
  with open(path, "w") as f:
    f.write(f"{len(points)}\n")
    for pt in points:
      f.write(f"{pt[0]:.4f},{pt[1]:.4f}\n")

def draw_debug_flow(img, pts0, pts1, color=(0, 0, 255)):
  vis = cv2.cvtColor(img.copy(), cv2.COLOR_GRAY2BGR)
  for (x0, y0), (x1, y1) in zip(pts0, pts1):
    cv2.circle(vis, (int(round(x0)), int(round(y0))), 2, (0, 255, 0), -1)
    cv2.arrowedLine(vis,
                    (int(round(x0)), int(round(y0))),
                    (int(round(x1)), int(round(y1))),
                    color, 1, tipLength=0.3)
  return vis

def parse_criteria(s):
  try:
    max_iter, epsilon = map(float, s.split(","))
    return (int(max_iter), epsilon)
  except:
    raise argparse.ArgumentTypeError("LK criteria must be in format max_iter,epsilon (e.g., 30,0.01)")

def generate_sparse_of_dataset(img_dir, output_dir, resolution_name, img_size,
                                max_feats=10, max_pairs=None,
                                quality_level=0.01, min_distance=5,
                                lk_win_size=15, lk_max_level=2,
                                lk_criteria=(30, 0.01), debug_viz=False):

  img_dir = Path(img_dir)
  output_dir = Path(output_dir)
  output_dir_name = output_dir.name
  dataset_id = img_dir.name

  pgm_dir = output_dir / dataset_id / f"pgm/{resolution_name}"
  feat_dir = output_dir / dataset_id / f"feat/{resolution_name}"
  debug_dir = output_dir / dataset_id / f"debug/{resolution_name}"
  #dataset_txt_path = output_dir / f"sparse_of_{resolution_name}_{dataset_id}.txt"
  dataset_txt_path = output_dir / f"sparse_of_{resolution_name}.txt"

  ensure_dir(pgm_dir)
  ensure_dir(feat_dir)
  if debug_viz:
    ensure_dir(debug_dir)

  images = sorted([img for img in img_dir.glob("*.png")])
  if max_pairs:
    images = images[:max_pairs + 1]

  dataset_lines = ["Sparse Optical Flow Dataset"]

  for i in range(len(images) - 1):
    img0 = cv2.imread(str(images[i]), cv2.IMREAD_GRAYSCALE)
    img1 = cv2.imread(str(images[i + 1]), cv2.IMREAD_GRAYSCALE)
    img0 = cv2.resize(img0, img_size)
    img1 = cv2.resize(img1, img_size)

    pts0 = cv2.goodFeaturesToTrack(img0,
                                   maxCorners=max_feats,
                                   qualityLevel=quality_level,
                                   minDistance=min_distance)
    if pts0 is None or len(pts0) < max_feats:
      print(f"\033[91m[Warning] Frame {i}: only found {0 if pts0 is None else len(pts0)} features.\033[0m")
      continue

    pts1, status, _ = cv2.calcOpticalFlowPyrLK(img0, img1, pts0, None,
                                               winSize=(lk_win_size, lk_win_size),
                                               maxLevel=lk_max_level,
                                               criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, *lk_criteria))

    pts0 = pts0[status[:, 0] == 1][:max_feats]
    pts1 = pts1[status[:, 0] == 1][:max_feats]

    if len(pts0) < max_feats:
      print(f"\033[91m[Warning] Frame {i}: only tracked {len(pts0)} valid features.\033[0m")
      continue

    pgm0_name = f"image{i}.pgm"
    pgm1_name = f"image{i+1}.pgm"
    write_pgm(img0, pgm_dir / pgm0_name)
    write_pgm(img1, pgm_dir / pgm1_name)

    feat0_name = f"image{i}_feats.txt"
    feat1_name = f"image{i}_feats_next_gt.txt"
    write_features(feat_dir / feat0_name, pts0.squeeze())
    write_features(feat_dir / feat1_name, pts1.squeeze())

    rel_pgm0  = f"{output_dir_name}/{dataset_id}/pgm/{resolution_name}/{pgm0_name}"
    rel_pgm1  = f"{output_dir_name}/{dataset_id}/pgm/{resolution_name}/{pgm1_name}"
    rel_feat0 = f"{output_dir_name}/{dataset_id}/feat/{resolution_name}/{feat0_name}"
    rel_feat1 = f"{output_dir_name}/{dataset_id}/feat/{resolution_name}/{feat1_name}"
    dataset_lines.append(f"{rel_pgm0},{rel_pgm1},{rel_feat0},{rel_feat1}")

    if debug_viz:
      pts0_np = pts0.squeeze()
      pts1_np = pts1.squeeze()
      if pts0_np.ndim == 1:
        pts0_np = np.expand_dims(pts0_np, axis=0)
        pts1_np = np.expand_dims(pts1_np, axis=0)

      vis_img = draw_debug_flow(img0, pts0_np, pts1_np)
      debug_img_path = debug_dir / f"pair{i}_viz.png"
      cv2.imwrite(str(debug_img_path), vis_img)

  with open(dataset_txt_path, "w") as f:
    for line in dataset_lines:
      f.write(line + "\n")

  print(f"✅ Generated {resolution_name} dataset: {dataset_txt_path}")

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate Sparse Optical Flow Dataset for EntoBench")

  parser.add_argument("--input-dir", required=True, help="Input image directory containing .png frames")
  parser.add_argument("--output-dir", required=True, help="Output base directory")
  parser.add_argument("--max-feats", type=int, default=10, help="Max features to track per image")
  parser.add_argument("--max-pairs", type=int, default=None, help="Max number of image pairs to process")

  parser.add_argument("--quality-level", type=float, default=0.01, help="GFTT quality level")
  parser.add_argument("--min-distance", type=int, default=5, help="GFTT min distance between features")

  parser.add_argument("--lk-win-size", type=int, default=15, help="LK window size (square)")
  parser.add_argument("--lk-max-level", type=int, default=2, help="LK max pyramid level")
  parser.add_argument("--lk-criteria", type=parse_criteria, default="30,0.01", help="LK termination criteria (format: max_iter,epsilon)")

  parser.add_argument("--debug-viz", action="store_true", help="Enable debug visualizations")

  args = parser.parse_args()

  resolutions = {
    "small": (80, 80),
    "medium": (160, 160),
    "large": (320, 320),
  }

  for res_name, size in resolutions.items():
    generate_sparse_of_dataset(
      img_dir=args.input_dir,
      output_dir=args.output_dir,
      resolution_name=res_name,
      img_size=size,
      max_feats=args.max_feats,
      max_pairs=args.max_pairs,
      quality_level=args.quality_level,
      min_distance=args.min_distance,
      lk_win_size=args.lk_win_size,
      lk_max_level=args.lk_max_level,
      lk_criteria=args.lk_criteria,
      debug_viz=args.debug_viz
    )
