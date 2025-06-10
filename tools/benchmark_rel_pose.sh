#!/opt/homebrew/bin/bash

# Enhanced Relative Pose Benchmarking Script
# Runs systematic parameter sweeps and logs results for analysis/plotting

# Temporarily disable exit on error for debugging
# set -e  # Exit on any error

# Configuration - paths relative to ento-bench root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # Go up one level from tools/
BUILD_DIR="${SCRIPT_DIR}/build/native"
RESULTS_DIR="${SCRIPT_DIR}/benchmark_results/rel_pose"
TOOL="${BUILD_DIR}/src/ento-pose/rel-pose/bin/generate_minimal_solver_data_rel_enhanced"

# Create results directory structure
mkdir -p "${RESULTS_DIR}/logs"
mkdir -p "${RESULTS_DIR}/data"
mkdir -p "${RESULTS_DIR}/summary"

# Grid of parameters to test
SOLVERS=("8pt" "5pt" "upright_3pt" "upright_planar_3pt" "upright_planar_2pt")  # Added 8pt back
PRECISIONS=("float" "double")
DATA_MODES=("realistic" "traditional")  # Changed order: realistic first (default)
#NOISE_LEVELS=(0.0 0.001 0.005 0.01 0.015 0.02 0.025 0.03 0.04 0.05 0.06 0.07 0.08 0.09 0.1)
NOISE_LEVELS=(0.5 1.0 2.5)
NUM_PROBLEMS=1000

# Initialize counters
TOTAL_RUNS=0
SUCCESSFUL_RUNS=0
FAILED_RUNS=0

echo "=== Enhanced Relative Pose Benchmarking ==="
echo "Start time: $(date)"
echo "Tool: ${TOOL}"
echo "Results directory: ${RESULTS_DIR}"
echo "Parameters:"
echo "  Solvers: ${SOLVERS[*]}"
echo "  Precisions: ${PRECISIONS[*]}"
echo "  Data Modes: ${DATA_MODES[*]}" 
echo "  Noise Levels: ${NOISE_LEVELS[*]}"
echo "  Problems per run: ${NUM_PROBLEMS}"
echo ""

# Check if tool exists
if [[ ! -f "${TOOL}" ]]; then
    echo "Error: Tool not found at ${TOOL}"
    echo "Please build the project first: make -C build/native"
    exit 1
fi

# Function to run a single benchmark
run_benchmark() {
    local solver="$1"
    local precision="$2"
    local data_mode="$3"
    local noise_level="$4"
    local num_points="$5"  # NEW: Accept point count as parameter
    
    TOTAL_RUNS=$((TOTAL_RUNS + 1))
    
    # Build command arguments
    local precision_flag=""
    if [[ "${precision}" == "double" ]]; then
        precision_flag="--double"
    fi
    
    local data_mode_flag=""
    if [[ "${data_mode}" == "realistic" ]]; then
        data_mode_flag="--realistic"
    fi
    
    # Set solver flag based on solver type
    local solver_flag=""
    case "${solver}" in
        "8pt") 
            solver_flag="--8pt-only"
            ;;
        "5pt") 
            solver_flag="--5pt-only" 
            ;;
        "upright_3pt") 
            solver_flag="--upright-3pt-only"
            ;;
        "upright_planar_3pt") 
            solver_flag="--upright-planar-3pt-only"
            ;;
        "upright_planar_2pt") 
            solver_flag="--upright-planar-2pt-only"
            ;;
        *) echo "Unknown solver: ${solver}"; return 1 ;;
    esac
    
    # Output files - UPDATED: Include point count in filename for linear solvers
    local output_prefix
    if [[ "${solver}" == "8pt" || "${solver}" == "upright_planar_3pt" ]]; then
        # Linear solvers: include point count in filename
        output_prefix="${solver}_${precision}_${data_mode}_p${NUM_PROBLEMS}_noise${noise_level}_N${num_points}"
    else
        # Minimal solvers: use standard filename (point count is fixed)
        output_prefix="${solver}_${precision}_${data_mode}_p${NUM_PROBLEMS}_noise${noise_level}"
    fi
    
    local log_file="${RESULTS_DIR}/logs/${output_prefix}.log"
    local summary_file="${RESULTS_DIR}/summary/${output_prefix}_summary.txt"
    
    echo "[$TOTAL_RUNS] Running: ${solver} ${precision} ${data_mode} noise=${noise_level} (N=${num_points} points)"
    
    # Build command
    local cmd="${TOOL} ${solver_flag} ${precision_flag} ${data_mode_flag} --problems ${NUM_PROBLEMS} --points ${num_points} --noise-levels \"${noise_level}\""
    
    # Execute benchmark
    if timeout 300 bash -c "${cmd}" > "${log_file}" 2>&1; then
        echo "  ✓ Success"
        SUCCESSFUL_RUNS=$((SUCCESSFUL_RUNS + 1))
        echo "Successful: ${solver} ${precision} ${data_mode} noise=${noise_level} N=${num_points}" >> "${summary_file}"
        echo "Extracted: ${output_prefix}_summary.txt"
    else
        echo "  ✗ Failed (check log)"
        FAILED_RUNS=$((FAILED_RUNS + 1))
        echo "Failed: ${solver} ${precision} ${data_mode} noise=${noise_level} N=${num_points}" >> "${summary_file}"
    fi
}

# Function to get point counts for each solver type
get_point_counts() {
    local solver="$1"
    case "${solver}" in
        "8pt") 
            echo "8 16 32 64 128"  # Multiple N levels for linear solver
            ;;
        "5pt") 
            echo "5"               # Minimal solver: fixed N
            ;;
        "upright_3pt") 
            echo "3"               # Minimal solver: fixed N
            ;;
        "upright_planar_3pt") 
            echo "3 8 16 32"       # Multiple N levels for linear solver
            ;;
        "upright_planar_2pt") 
            echo "2"               # Minimal solver: fixed N
            ;;
        *) echo ""; return 1 ;;
    esac
}

# Run all combinations - UPDATED: Support multiple point counts
for solver in "${SOLVERS[@]}"; do
    for precision in "${PRECISIONS[@]}"; do
        for data_mode in "${DATA_MODES[@]}"; do
            for noise_level in "${NOISE_LEVELS[@]}"; do
                # Get point counts for this solver
                point_counts=$(get_point_counts "${solver}")
                if [[ -z "${point_counts}" ]]; then
                    echo "Error: Unknown solver ${solver}"
                    continue
                fi
                
                # Run benchmark for each point count
                for num_points in ${point_counts}; do
                    run_benchmark "${solver}" "${precision}" "${data_mode}" "${noise_level}" "${num_points}"
                done
            done
        done
    done
done

# Generate summary
SUMMARY_FILE="${RESULTS_DIR}/benchmark_summary.txt"
cat > "${SUMMARY_FILE}" << EOF
=== Benchmark Summary ===
Start time: $(date)
Total runs: ${TOTAL_RUNS}
Successful: ${SUCCESSFUL_RUNS}
Failed: ${FAILED_RUNS}
Success rate: $(echo "scale=1; ${SUCCESSFUL_RUNS} * 100 / ${TOTAL_RUNS}" | bc -l)%

Configuration:
- Solvers: ${SOLVERS[*]}
- Precisions: ${PRECISIONS[*]}  
- Data Modes: ${DATA_MODES[*]}
- Noise Levels: ${NOISE_LEVELS[*]}
- Problems per run: ${NUM_PROBLEMS}

Results saved to: ${RESULTS_DIR}
EOF

echo ""
echo "=== Benchmark Complete ==="
echo "Total runs: ${TOTAL_RUNS}"
echo "Successful: ${SUCCESSFUL_RUNS}"
echo "Failed: ${FAILED_RUNS}"
echo "Results saved to: ${RESULTS_DIR}"
echo "Summary: ${SUMMARY_FILE}"
echo "End time: $(date)"

echo ""
echo "=== Running Analysis and Generating Plots ==="
echo "Running analysis script with conda sandbox environment..."

# Run analysis script in conda environment
if conda activate sandbox 2>/dev/null && python "${SCRIPT_DIR}/analyze_rel_pose_results.py"; then
    echo "✓ Analysis complete! Plots saved to: ${RESULTS_DIR}/plots/"
else
    echo "✗ Analysis failed or conda environment not available"
    echo "You can run the analysis manually:"
    echo "  conda activate sandbox"
    echo "  python analyze_rel_pose_results.py"
fi 