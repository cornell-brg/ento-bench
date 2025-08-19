#!/opt/homebrew/bin/bash

# Test version of benchmark script with smaller parameter space

set -e  # Exit on any error

# Configuration - paths relative to ento-bench root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # Go up one level from tools/
BUILD_DIR="${SCRIPT_DIR}/build/native"
RESULTS_DIR="${SCRIPT_DIR}/benchmark_results/abs_pose_test"
TOOL="${BUILD_DIR}/src/ento-pose/abs-pose/bin/generate_minimal_solver_data_abs_enhanced"

# Create results directory structure
mkdir -p "${RESULTS_DIR}/logs"
mkdir -p "${RESULTS_DIR}/data"

# Small test grid
SOLVERS=("dlt")
NOISE_LEVELS=("0.0" "0.01")
PRECISION_TYPES=("float")
DATA_MODES=("traditional")
PROBLEM_COUNTS=(100)

# DLT test configurations - just 2 sizes
DLT_POINT_CONFIGS=(6 16)

# Test configurations
declare -A SOLVER_POINTS
SOLVER_POINTS[dlt]=128

echo "=== Testing Enhanced Absolute Pose Benchmarking ==="
echo "Results will be saved to: ${RESULTS_DIR}"
echo "Start time: $(date)"
echo ""

# Function to run a single benchmark
run_benchmark() {
    local solver="$1"
    local noise_levels="$2"
    local precision="$3"
    local data_mode="$4"
    local problems="$5"
    local dlt_points="$6"  # Optional, only for DLT
    
    # Determine point count and solver flags
    local points
    local solver_flag
    local extra_args=""
    
    case "$solver" in
        "dlt")
            # For DLT, use the specific point count passed in
            if [[ -n "$dlt_points" ]]; then
                points="$dlt_points"
            else
                points=${SOLVER_POINTS[dlt]}
            fi
            solver_flag="--dlt-only"
            ;;
    esac
    
    # Add precision flag
    if [[ "$precision" == "double" ]]; then
        extra_args="$extra_args --double"
    fi
    
    # Add realistic mode flag
    if [[ "$data_mode" == "realistic" ]]; then
        extra_args="$extra_args --realistic"
    fi
    
    # Create unique identifier for this run
    local run_id="${solver}_${precision}_${data_mode}_p${problems}_noise${noise_levels}"
    if [[ "$solver" == "dlt" && -n "$dlt_points" ]]; then
        run_id="${run_id}_dlt${dlt_points}"
    fi
    
    local log_file="${RESULTS_DIR}/logs/${run_id}.log"
    local data_dir="${RESULTS_DIR}/data/${run_id}"
    
    echo "Running: $solver ($precision, $data_mode, $problems problems)"
    if [[ "$solver" == "dlt" && -n "$dlt_points" ]]; then
        echo "  DLT points: $dlt_points"
    fi
    echo "  Noise level: $noise_levels"
    echo "  Log: $(basename "$log_file")"
    
    # Create data directory for this run
    mkdir -p "$data_dir"
    
    # Change to data directory so CSV files are saved there
    cd "$data_dir"
    
    # Build complete command
    local cmd="$TOOL $solver_flag --points $points --problems $problems --noise-levels \"$noise_levels\" $extra_args"
    
    echo "Command: $cmd"
    
    # Run the tool and capture all output
    echo "Command: $cmd" > "$log_file"
    echo "Started: $(date)" >> "$log_file"
    echo "========================================" >> "$log_file"
    
    if eval "$cmd" >> "$log_file" 2>&1; then
        echo "Completed: $(date)" >> "$log_file"
        echo "  ✓ Success"
        return 0
    else
        echo "Failed: $(date)" >> "$log_file"
        echo "  ✗ Failed (check log)"
        return 1
    fi
}

# Main benchmarking loop
total_runs=0
successful_runs=0
failed_runs=0

for solver in "${SOLVERS[@]}"; do
    for precision in "${PRECISION_TYPES[@]}"; do
        for data_mode in "${DATA_MODES[@]}"; do
            for problems in "${PROBLEM_COUNTS[@]}"; do
                for noise_level in "${NOISE_LEVELS[@]}"; do
                    
                    if [[ "$solver" == "dlt" ]]; then
                        # Run DLT with different point configurations
                        for dlt_points in "${DLT_POINT_CONFIGS[@]}"; do
                            ((total_runs++))
                            if run_benchmark "$solver" "$noise_level" "$precision" "$data_mode" "$problems" "$dlt_points"; then
                                ((successful_runs++))
                            else
                                ((failed_runs++))
                            fi
                            echo ""
                        done
                    fi
                done
            done
        done
    done
done

echo ""
echo "=== Test Complete ==="
echo "Total runs: $total_runs"
echo "Successful: $successful_runs" 
echo "Failed: $failed_runs"
echo "Results saved to: ${RESULTS_DIR}"

# Return to original directory
cd "${SCRIPT_DIR}" 