#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 -d <experiment-dir>"
  echo "  -d : path to experiment directory"
  echo "       e.g. experiments/ae/m4/example/benchmark-example-cache"
  exit 1
}

DIR=""
while getopts "d:h" opt; do
  case "$opt" in
    d) DIR="$OPTARG" ;;
    h|*) usage ;;
  esac
done

[[ -z "$DIR" ]] && usage
if [[ ! -d "$DIR" ]]; then
  echo "ERROR: Directory not found: $DIR" >&2
  exit 2
fi

# Derive experiment name from the last path component
NAME="$(basename "$DIR")"

echo "[1/3] Renaming files in: $DIR"
python3 tools/rename_files.py "$DIR" --execute

echo "[2/3] Syncing current and logic traces..."
python3 tools/sync_current_logic_traces.py --directory "$DIR"

echo "[3/3] Analyzing experiment: $NAME"
python3 tools/analyze_single_experiment_v3.py "$DIR" "$NAME"

echo "Done. Results written in: $DIR"
