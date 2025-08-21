#!/opt/homebrew/bin/bash

# Enhanced Relative Pose Benchmarking Script
# Runs systematic parameter sweeps and logs results for analysis/plotting

# Temporarily disable exit on error for debugging
# set -e  # Exit on any error

# Configuration - paths relative to ento-bench root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # Go up one level from tools/
BUILD_DIR="${SCRIPT_DIR}/build-cs4"
RESULTS_DIR="${SCRIPT_DIR}/benchmark_results/rel_pose"
TOOL="${BUILD_DIR}/src/ento-pose/rel-pose/bin/generate_minimal_solver_data_rel_enhanced"

# Create results directory structure
mkdir -p "${RESULTS_DIR}/logs"
mkdir -p "${RESULTS_DIR}/data"
mkdir -p "${RESULTS_DIR}/summary"

# Grid of parameters to test
SOLVERS=("8pt" "5pt" "upright_3pt" "upright_planar_3pt" "upright_planar_2pt")  # Added 8pt back
PRECISIONS=("float" "double")
DATA_MODES=("realistic")  # Changed order: realistic first (default)
#NOISE_LEVELS=(0.0 0.001 0.005 0.01 0.015 0.02 0.025 0.03 0.04 0.05 0.06 0.07 0.08 0.09 0.1)
NOISE_LEVELS=(0.1 0.5 1.0)
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
    local num_points="$5"  # Accept point count as parameter
    
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
    
    # Create unique identifier for this run (matching abs_pose pattern)
    local run_id="${solver}_${precision}_${data_mode}_p${NUM_PROBLEMS}_noise${noise_level}"
    if [[ "${solver}" == "8pt" || "${solver}" == "upright_planar_3pt" ]]; then
        # Linear solvers: include point count in filename
        run_id="${run_id}_N${num_points}"
    fi
    
    local log_file="${RESULTS_DIR}/logs/${run_id}.log"
    local data_dir="${RESULTS_DIR}/data/${run_id}"
    
    echo "Running: ${solver} (${precision}, ${data_mode}, ${NUM_PROBLEMS} problems)"
    if [[ "${solver}" == "8pt" || "${solver}" == "upright_planar_3pt" ]]; then
        echo "  Points: ${num_points}"
    fi
    echo "  Noise level: ${noise_level}"
    echo "  Log: $(basename "$log_file")"
    
    # Create data directory for this run
    mkdir -p "$data_dir"
    
    # Change to data directory so CSV files are saved there
    cd "$data_dir"
    
    # Build command
    local cmd="${TOOL} ${solver_flag} ${precision_flag} ${data_mode_flag} --problems ${NUM_PROBLEMS} --points ${num_points} --noise-levels \"${noise_level}\""
    
    # Run the tool and capture all output
    echo "Command: $cmd" > "$log_file"
    echo "Started: $(date)" >> "$log_file"
    echo "========================================" >> "$log_file"
    
    if eval "$cmd" >> "$log_file" 2>&1; then
        echo "Completed: $(date)" >> "$log_file"
        echo "  ✓ Success"
        SUCCESSFUL_RUNS=$((SUCCESSFUL_RUNS + 1))
        return 0
    else
        echo "Failed: $(date)" >> "$log_file"
        echo "  ✗ Failed (check log)"
        FAILED_RUNS=$((FAILED_RUNS + 1))
        return 1
    fi
}

# Function to extract statistics from log files
extract_statistics() {
    local log_file="$1"
    local output_file="$2"
    
    echo "# Extracted statistics from: $(basename "$log_file")" > "$output_file"
    echo "# Generated: $(date)" >> "$output_file"
    echo "" >> "$output_file"
    
    # Extract configuration info
    echo "=== Configuration ===" >> "$output_file"
    grep -E "(Scalar type|Problems|Points|Data generation|Noise levels)" "$log_file" | head -10 >> "$output_file" || true
    echo "" >> "$output_file"
    
    # Extract all statistics sections
    echo "=== Statistics ===" >> "$output_file"
    awk '/=== .* Statistics \(noise=.*\) ===/,/================================/ {print}' "$log_file" >> "$output_file" || true
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

# Run all combinations - Support multiple point counts
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
                    echo ""
                done
            done
        done
    done
done

# Extract statistics from all log files
echo "=== Extracting Statistics ==="
for log_file in "${RESULTS_DIR}/logs"/*.log; do
    if [[ -f "$log_file" ]]; then
        base_name=$(basename "$log_file" .log)
        summary_file="${RESULTS_DIR}/summary/${base_name}_summary.txt"
        extract_statistics "$log_file" "$summary_file"
        echo "Extracted: $(basename "$summary_file")"
    fi
done

# Generate overall summary
SUMMARY_FILE="${RESULTS_DIR}/benchmark_summary.txt"
echo "=== Enhanced Relative Pose Benchmark Summary ===" > "$SUMMARY_FILE"
echo "Generated: $(date)" >> "$SUMMARY_FILE"
echo "Total runs: $TOTAL_RUNS" >> "$SUMMARY_FILE"
echo "Successful: $SUCCESSFUL_RUNS" >> "$SUMMARY_FILE"
echo "Failed: $FAILED_RUNS" >> "$SUMMARY_FILE"
echo "Success rate: $(echo "scale=1; 100*$SUCCESSFUL_RUNS/$TOTAL_RUNS" | bc -l)%" >> "$SUMMARY_FILE"
echo "" >> "$SUMMARY_FILE"

echo "Data files saved in: ${RESULTS_DIR}/data/" >> "$SUMMARY_FILE"
echo "Log files saved in: ${RESULTS_DIR}/logs/" >> "$SUMMARY_FILE"
echo "Summaries saved in: ${RESULTS_DIR}/summary/" >> "$SUMMARY_FILE"
echo "" >> "$SUMMARY_FILE"

# List generated data files
echo "=== Generated Data Files ===" >> "$SUMMARY_FILE"
find "${RESULTS_DIR}/data" -name "*.csv" | wc -l | xargs echo "Total CSV files:" >> "$SUMMARY_FILE"
echo "" >> "$SUMMARY_FILE"

echo "=== Run Details ===" >> "$SUMMARY_FILE"
for log_file in "${RESULTS_DIR}/logs"/*.log; do
    if [[ -f "$log_file" ]]; then
        echo "$(basename "$log_file"): $(grep -c "✓\|✗" "$log_file" 2>/dev/null || echo "0") results" >> "$SUMMARY_FILE"
    fi
done

echo ""
echo "=== Benchmark Complete ==="
echo "Total runs: $TOTAL_RUNS"
echo "Successful: $SUCCESSFUL_RUNS" 
echo "Failed: $FAILED_RUNS"
echo "Results saved to: ${RESULTS_DIR}"
echo "Summary: ${SUMMARY_FILE}"
echo "End time: $(date)"

# Automatically run analysis and generate plots
echo ""
echo "=== Running Analysis and Generating Plots ==="
ANALYSIS_SCRIPT="${SCRIPT_DIR}/tools/analyze_rel_pose_results.py"

if [[ -f "$ANALYSIS_SCRIPT" ]]; then
    # Use conda environment with proper initialization
    if command -v conda &> /dev/null; then
        echo "Running analysis script with conda sandbox environment..."
        # Initialize conda for this shell session and activate
        eval "$(conda shell.bash hook)"
        if conda activate sandbox && python "$ANALYSIS_SCRIPT" "$RESULTS_DIR"; then
            echo "✓ Analysis complete! Plots saved to: ${RESULTS_DIR}/plots/"
        else
            echo "✗ Analysis script failed. Check the logs above."
            echo "You can run it manually with:"
            echo "  conda activate sandbox && python $ANALYSIS_SCRIPT $RESULTS_DIR"
        fi
    elif command -v python3 &> /dev/null; then
        echo "Conda not found, falling back to python3..."
        if python3 "$ANALYSIS_SCRIPT" "$RESULTS_DIR"; then
            echo "✓ Analysis complete! Plots saved to: ${RESULTS_DIR}/plots/"
        else
            echo "✗ Analysis script failed. Check the logs above."
        fi
    else
        echo "⚠ Neither conda nor python3 found. Please run analysis manually:"
        echo "  conda activate sandbox && python $ANALYSIS_SCRIPT $RESULTS_DIR"
    fi
else
    echo "⚠ Analysis script not found at: $ANALYSIS_SCRIPT"
    echo "  Please run analysis manually if needed."
fi

# Return to original directory
cd "${SCRIPT_DIR}" 
