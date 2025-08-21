#!/bin/bash

# Quick Test - Enhanced Absolute Pose Benchmarking Script
# Reduced parameters for initial testing

set -e  # Exit on any error

# Configuration - paths relative to ento-bench root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # Go up one level from tools/
BUILD_DIR="${SCRIPT_DIR}/build/native"
RESULTS_DIR="${SCRIPT_DIR}/benchmark_results/abs_pose_test"
TOOL="${BUILD_DIR}/src/ento-pose/abs-pose/bin/generate_minimal_solver_data_abs_enhanced"

# Create results directory structure
mkdir -p "${RESULTS_DIR}/logs"
mkdir -p "${RESULTS_DIR}/data"
mkdir -p "${RESULTS_DIR}/summary"

# Reduced grid for testing
SOLVERS=("p3p")  # Just test P3P first
NOISE_LEVELS=("0.0,0.01")  # Just two noise levels for quick test
PRECISION_TYPES=("float")  # Just float for now
DATA_MODES=("traditional")  # Just traditional mode
PROBLEM_COUNTS=(1000)  # Just 10 problems for quick test

echo "=== Quick Test - Enhanced Absolute Pose Benchmarking System ==="
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
    
    # Determine point count and solver flags based on solver type
    local points=3  # P3P uses 3 points
    local solver_flag="--p3p-only"
    local extra_args=""
    
    # Add precision flag
    if [[ "$precision" == "double" ]]; then
        extra_args="$extra_args --double"
    fi
    
    # Add realistic mode flag
    if [[ "$data_mode" == "realistic" ]]; then
        extra_args="$extra_args --realistic"
    fi
    
    # Create unique identifier for this run
    local run_id="${solver}_${precision}_${data_mode}_p${problems}"
    
    local log_file="${RESULTS_DIR}/logs/${run_id}.log"
    local data_dir="${RESULTS_DIR}/data/${run_id}"
    
    echo "Running: $solver ($precision, $data_mode, $problems problems)"
    echo "  Noise levels: $noise_levels"
    echo "  Log: $(basename "$log_file")"
    
    # Create data directory for this run
    mkdir -p "$data_dir"
    
    # Change to data directory so CSV files are saved there
    cd "$data_dir"
    
    # Build complete command
    local cmd="$TOOL $solver_flag --points $points --problems $problems --noise-levels \"$noise_levels\" $extra_args"
    
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

# Simple test loop
total_runs=0
successful_runs=0
failed_runs=0

for solver in "${SOLVERS[@]}"; do
    for precision in "${PRECISION_TYPES[@]}"; do
        for data_mode in "${DATA_MODES[@]}"; do
            for problems in "${PROBLEM_COUNTS[@]}"; do
                noise="${NOISE_LEVELS[0]}"  # Just use first noise set
                
                ((total_runs++))
                if run_benchmark "$solver" "$noise" "$precision" "$data_mode" "$problems"; then
                    ((successful_runs++))
                else
                    ((failed_runs++))
                fi
                echo ""
            done
        done
    done
done

echo ""
echo "=== Quick Test Complete ==="
echo "Total runs: $total_runs"
echo "Successful: $successful_runs" 
echo "Failed: $failed_runs"
echo "Results saved to: ${RESULTS_DIR}"
echo "End time: $(date)"

# Return to original directory
cd "${SCRIPT_DIR}" 