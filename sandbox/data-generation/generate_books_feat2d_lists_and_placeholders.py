import os

def write_dummy_feature(feat_path):
  with open(feat_path, 'w') as f:
    f.write("1\n")
    f.write("0, 0\n")

def write_dummy_descriptor(desc_path):
  with open(desc_path, 'w') as f:
    f.write("1\n")
    f.write("64,32,16,0,0,8,0,0,96,0,32,8,2,0,64,16,0,4,64,0,0,32,128,0,0,0,0,0,64,0,2,0\n")

def generate_lists_and_touch(resolution, num_images, algorithms, dataset_root):
  rel_path_base = f"feat2d/books/{resolution}"
  abs_path_base = os.path.join(dataset_root, rel_path_base)
  os.makedirs(abs_path_base, exist_ok=True)

  for algo in algorithms:
    lines = []
    output_txt = f"{algo.lower()}_{resolution}_books_data.txt"
    output_txt_path = os.path.join(dataset_root, "feat2d", output_txt)
    os.makedirs(os.path.dirname(output_txt_path), exist_ok=True)

    for i in range(num_images):
      base = f"image{i}"
      pgm_path = f"{rel_path_base}/{base}.pgm"
      feat_path_rel = f"{rel_path_base}/{base}_feats.txt"
      desc_path_rel = f"{rel_path_base}/{base}_descs.txt"

      feat_path_abs = os.path.join(dataset_root, feat_path_rel)
      desc_path_abs = os.path.join(dataset_root, desc_path_rel)

      if algo == "FAST":
        line = f"{pgm_path},{feat_path_rel}"
        write_dummy_feature(feat_path_abs)
      elif algo == "BRIEF":
        line = f"{pgm_path},{feat_path_rel},{desc_path_rel}"
        write_dummy_feature(feat_path_abs)
        write_dummy_descriptor(desc_path_abs)
      else:  # FAST+BRIEF or ORB
        line = f"{pgm_path},{feat_path_rel},{desc_path_rel}"
        write_dummy_feature(feat_path_abs)
        write_dummy_descriptor(desc_path_abs)

      lines.append(line)

    with open(output_txt_path, 'w') as f:
      f.write("Feature Recognition Problem\n")
      f.write("\n".join(lines))

    print(f"✅ Generated {output_txt_path} with {len(lines)} entries.")

if __name__ == "__main__":
  import argparse

  script_dir = os.path.dirname(os.path.abspath(__file__))
  project_root = os.path.abspath(os.path.join(script_dir, ".."))
  dataset_root = os.path.join(project_root, "datasets")

  resolutions = ["tiny", "small", "medium", "large"]
  num_images_per_res = {
    "tiny": 3,
    "small": 18,
    "medium": 18,
    "large": 18
  }

  algorithms = ["FAST", "BRIEF", "FASTBRIEF", "ORB"]

  for res in resolutions:
    generate_lists_and_touch(res, num_images_per_res[res], algorithms, dataset_root)
