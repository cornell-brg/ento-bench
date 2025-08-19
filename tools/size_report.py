"""
size_report.py  –  EntoBench code-size delta reporter

Run from **inside** a configured build directory.  The script:
1. Builds each baseline ELF once.
2. Builds every "real" benchmark the user cares about.
3. Uses `arm-none-eabi-size` to grab FLASH (text+data) and RAM (bss) sizes.
4. Emits a CSV (stdout and ./size_report.csv) with columns:
     category,benchmark,baseline,flash_bytes,delta_flash,ram_bytes,delta_ram

Update the BENCHMARKS dict below to add / remove targets.
"""
import csv
import subprocess
import sys
from pathlib import Path
import re, collections, tempfile, os, argparse

# Capstone disassembler (Thumb mode for Cortex-M)
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
from capstone import CS_GRP_JUMP
from capstone import arm as _arm_consts

# Initialise once
_MD = Cs(CS_ARCH_ARM, CS_MODE_THUMB)
_MD.detail = True

# ---------- CONFIG -----------------------------------------------------------

ARM_SIZE = "arm-none-eabi-size"
OBJCOPY = "arm-none-eabi-objcopy"

# Map each real benchmark -> (baseline target, category label)
BENCHMARKS = {
    # Feature recognition
    "bench-fastbrief-medium":      ("bench-empty-feature-recognition", "feature_recognition"),
    "bench-orb-medium":            ("bench-empty-feature-recognition", "feature_recognition"),
    "bench-sift-medium":            ("bench-empty-feature-recognition", "feature_recognition"),


    # Optical flow
    "bench-lkof-small":            ("bench-empty-optical-flow", "optical_flow"),
    "bench-block-of-small":        ("bench-empty-optical-flow", "optical_flow"),
    "bench-image-interp-of-small": ("bench-empty-optical-flow", "optical_flow"),

    # Attitude estimation
    "bench-madgwick-float-imu":    ("bench-empty-attitude-est", "attitude"),
    "bench-mahony-float-imu":      ("bench-empty-attitude-est", "attitude"),
    "bench-fourati-float-marg":     ("bench-empty-attitude-est", "attitude"),

    # EKF
    "bench-ekf-sync-update-float":       ("bench-empty-ekf", "ekf"),
    "bench-ekf-seq-update-float":        ("bench-empty-ekf", "ekf"),
    "bench-ekf-truncated-update-float":  ("bench-empty-ekf", "ekf"),
    "bench-robobee-ekf-float":           ("bench-empty-ekf", "ekf"),

    # Absolute pose
    "bench-p3p-float":                   ("bench-empty-abs-pose", "abs_pose"),
    "bench-dlt-float":                   ("bench-empty-abs-pose", "abs_pose"),
    "bench-up2p-float":                  ("bench-empty-abs-pose", "abs_pose"),
    "bench-gold-standard-abs-float":     ("bench-empty-abs-pose", "abs_pose"),

    # Relative pose
    "bench-8pt-float":                   ("bench-empty-rel-pose", "rel_pose"),
    "bench-5pt-float":                   ("bench-empty-rel-pose", "rel_pose"),
    "bench-upright-3pt-float":           ("bench-empty-rel-pose", "rel_pose"),
    "bench-upright-planar-3pt-float":    ("bench-empty-rel-pose", "rel_pose"),
    "bench-upright-planar-2pt-float":    ("bench-empty-rel-pose", "rel_pose"),
    "bench-gold-standard-rel-float":     ("bench-empty-rel-pose", "rel_pose"),
    "bench-homography-float":            ("bench-empty-homography", "homography"),

    # Robust
    "bench-lo-ransac-p3p-nonlinear-float": ("bench-empty-robust-abs-pose", "robust_abs_pose"),
    "bench-ransac-5pt-float":              ("bench-empty-robust-rel-pose", "robust_rel_pose"),

    # Control
    "bench-robofly-lqr":             ("bench-empty-opt-control", "opt_control"),
    "bench-robofly-tinympc":         ("bench-empty-opt-control", "opt_control"),
    "bench-template-mpc-float":      ("bench-empty-opt-control", "opt_control"),

    "bench-geometric-controller-float": ("bench-empty-geometric-control", "geometric_control"),
    "bench-adaptive-controller-float":  ("bench-empty-adaptive-control", "adaptive_control"),
}

# ---------------------------------------------------------------------------
# Command-line args (so user can force objdump counting)
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--use-objdump", action="store_true",
                    help="Count instructions with objdump/regex instead of Capstone")
KNOWN_ARGS, _ = parser.parse_known_args()

def run(cmd):
    subprocess.check_call(cmd, shell=True)


def build(target):
    print(f"[BUILD] {target}")
    run(f"cmake --build . --target {target} -j")


def find_elf(target):
    # Most ELFs live in benchmark/*/bin/   but fall back to search
    p = Path(".").glob(f"**/{target}.elf")
    for path in p:
        return path
    raise FileNotFoundError(f"ELF for {target} not found")


def size_of(target):
    elf = find_elf(target)
    res = subprocess.check_output([ARM_SIZE, str(elf)]).decode().strip().splitlines()[-1]
    # columns: text data bss dec hex filename
    parts = res.split()
    text, data, bss = map(int, parts[0:3])
    flash = text + data
    ram = bss
    return flash, ram


# ---------------- Instruction buckets -----------------
# Any mnemonic that does not match these regexes falls into "other".
BUCKET_ORDER = ["float", "int", "mem", "branch", "other"]

# ---------------- Disassembly helpers -----------------

def _text_bytes(elf: Path) -> bytes:
    """Extract raw .text section bytes using objcopy."""
    with tempfile.NamedTemporaryFile(delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.check_call([OBJCOPY, "-O", "binary", "--only-section=.text", str(elf), tmp_path],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with open(tmp_path, "rb") as f:
            return f.read()
    finally:
        os.remove(tmp_path)


def classify_ins(ins):
    """Return bucket name for a Capstone instruction"""
    if any(g in ins.groups for g in _FP_GROUPS):
        return "float"
    if CS_GRP_JUMP in ins.groups:
        return "branch"
    mn = ins.mnemonic
    if mn.startswith(("ldr", "ldm", "str", "stm", "push", "pop", "vldr", "vstr")):
        return "mem"
    return "int"


def inst_counts(elf: Path) -> collections.Counter:
    """Return Counter(bucket -> count) using Capstone groups for classification."""
    code = _text_bytes(elf)
    cnt = collections.Counter()
    for ins in _MD.disasm(code, 0):
        bucket = classify_ins(ins)
        cnt[bucket] += 1
    # ensure all keys present
    for b in BUCKET_ORDER:
        cnt.setdefault(b, 0)
    return cnt


_FP_GROUPS = {
    _arm_consts.ARM_GRP_DPVFP,
    _arm_consts.ARM_GRP_FPARMV8,
    _arm_consts.ARM_GRP_VFP2,
    _arm_consts.ARM_GRP_VFP3,
    _arm_consts.ARM_GRP_VFP4,
}

# ---------------- Objdump fallback ----------------------------------------

_OBJ_RE = re.compile(r"^\s*[0-9a-f]+:\s+[0-9a-f ]+\s+([a-z0-9.]+)")

def inst_counts_objdump(elf: Path) -> collections.Counter:
    """Instruction counting using arm-none-eabi-objdump text output."""
    txt = subprocess.check_output(["arm-none-eabi-objdump", "-d", "-j", ".text", str(elf)]).decode()
    cnt = collections.Counter()
    for line in txt.splitlines():
        m = _OBJ_RE.match(line)
        if not m:
            continue
        op = m.group(1)
        if op.startswith("v"):  # any VFP/NEON mnemonic starts with v
            cnt["float"] += 1
        elif op.startswith(("ldr", "ldm", "str", "stm", "push", "pop")):
            cnt["mem"] += 1
        elif op.startswith("b") or op in ("cbnz", "cbz", "bx", "blx", "it"):  # branch family
            cnt["branch"] += 1
        else:
            cnt["int"] += 1
    for b in BUCKET_ORDER:
        cnt.setdefault(b, 0)
    return cnt

def main():
    # 1. build all baselines first (deduplicated)
    baselines = {base for base, _ in BENCHMARKS.values()}
    for b in baselines:
        build(b)

    baseline_sizes = {b: size_of(b) for b in baselines}
    if KNOWN_ARGS.use_objdump:
        baseline_instrs = {b: inst_counts_objdump(find_elf(b)) for b in baselines}
    else:
        baseline_instrs = {b: inst_counts(find_elf(b)) for b in baselines}

    rows = []
    for bench, (base, cat) in BENCHMARKS.items():
        build(bench)
        flash, ram = size_of(bench)
        base_flash, base_ram = baseline_sizes[base]
        base_cnt = baseline_instrs[base]

        row = {
            "category": cat,
            "benchmark": bench,
            "baseline": base,
            "flash": flash,
            "delta_flash": flash - base_flash,
            "ram": ram,
            "delta_ram": ram - base_ram,
        }

        if KNOWN_ARGS.use_objdump:
            cnt = inst_counts_objdump(find_elf(bench))
        else:
            cnt = inst_counts(find_elf(bench))

        for bucket in BUCKET_ORDER:
            row[f"{bucket}_cnt"] = cnt[bucket]
            row[f"delta_{bucket}"] = cnt[bucket] - base_cnt[bucket]

        rows.append(row)

    # output CSV
    csv_path = Path("size_report.csv")
    base_cols = ["category","benchmark","baseline","flash","delta_flash","ram","delta_ram"]
    bucket_cols = [f"{b}_cnt" for b in BUCKET_ORDER] + [f"delta_{b}" for b in BUCKET_ORDER]

    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=base_cols + bucket_cols)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nWrote {csv_path} with {len(rows)} entries.")

    # also print summarized table
    for r in rows:
        print(f"{r['benchmark']:40s} FLASH {r['delta_flash']:8d} RAM {r['delta_ram']:6d} "
              f"INT {r['delta_int']:6d} FLOAT {r['delta_float']:6d} MEM {r['delta_mem']:6d} BR {r['delta_branch']:6d}")

if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode) 
