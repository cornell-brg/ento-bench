#!/opt/homebrew/bin/bash

# Enhanced Absolute Pose Benchmarking Script
# Runs systematic parameter sweeps and logs results for analysis/plotting

# Temporarily disable exit on error for debugging
# set -e  # Exit on any error

# Configuration - paths relative to ento-bench root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # Go up one level from tools/
BUILD_DIR="${SCRIPT_DIR}/build/native"
RESULTS_DIR="${SCRIPT_DIR}/benchmark_results/abs_pose"
TOOL="${BUILD_DIR}/src/ento-pose/abs-pose/bin/generate_minimal_solver_data_abs_enhanced"

# Create results directory structure
mkdir -p "${RESULTS_DIR}/logs"
mkdir -p "${RESULTS_DIR}/data"
mkdir -p "${RESULTS_DIR}/summary"

# Grid of parameters to test
SOLVERS=("p3p" "up2p" "dlt")
NOISE_LEVELS=("0.0" "0.001" "0.005" "0.01" "0.02" "0.03" "0.04" "0.05" "0.06" "0.07" "0.08" "0.09" "0.1")
PRECISION_TYPES=("float" "double")
DATA_MODES=("traditional" "realistic")
PROBLEM_COUNTS=(1000)

# DLT specific configurations
DLT_POINT_CONFIGS=(6 8 16 32 64 128)

# Test configurations
declare -A SOLVER_POINTS
SOLVER_POINTS[p3p]=3
SOLVER_POINTS[up2p]=2
SOLVER_POINTS[dlt]=128  # Use more points for DLT overdetermined system

echo "=== Enhanced Absolute Pose Benchmarking System ==="
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
    local dlt_points="$6"  # Optional, only for DLT - this is the actual point count to use
    
    # Determine point count and solver flags
    local points
    local solver_flag
    local extra_args=""
    
    case "$solver" in
        "p3p")
            points=${SOLVER_POINTS[p3p]}
            solver_flag="--p3p-only"
            ;;
        "up2p")
            points=${SOLVER_POINTS[up2p]}
            solver_flag="--up2p-only"
            ;;
        "dlt")
            # For DLT, use the specific point count passed in, not the default
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
    
    # Build complete command - noise_levels is now a single value
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
                    else
                        # Run P3P/UP2P with standard configuration
                        echo "DEBUG: About to run $solver with noise=$noise_level, precision=$precision, data_mode=$data_mode, problems=$problems"
                        
                        ((total_runs++))
                        if run_benchmark "$solver" "$noise_level" "$precision" "$data_mode" "$problems"; then
                            ((successful_runs++))
                            echo "DEBUG: $solver run succeeded"
                        else
                            ((failed_runs++))
                            echo "DEBUG: $solver run failed"
                        fi
                        echo ""
                    fi
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
summary_file="${RESULTS_DIR}/benchmark_summary.txt"
echo "=== Enhanced Absolute Pose Benchmark Summary ===" > "$summary_file"
echo "Generated: $(date)" >> "$summary_file"
echo "Total runs: $total_runs" >> "$summary_file"
echo "Successful: $successful_runs" >> "$summary_file"
echo "Failed: $failed_runs" >> "$summary_file"
echo "Success rate: $(echo "scale=1; 100*$successful_runs/$total_runs" | bc -l)%" >> "$summary_file"
echo "" >> "$summary_file"

echo "Data files saved in: ${RESULTS_DIR}/data/" >> "$summary_file"
echo "Log files saved in: ${RESULTS_DIR}/logs/" >> "$summary_file"
echo "Summaries saved in: ${RESULTS_DIR}/summary/" >> "$summary_file"
echo "" >> "$summary_file"

# List generated data files
echo "=== Generated Data Files ===" >> "$summary_file"
find "${RESULTS_DIR}/data" -name "*.csv" | wc -l | xargs echo "Total CSV files:" >> "$summary_file"
echo "" >> "$summary_file"

echo "=== Run Details ===" >> "$summary_file"
for log_file in "${RESULTS_DIR}/logs"/*.log; do
    if [[ -f "$log_file" ]]; then
        echo "$(basename "$log_file"): $(grep -c "✓\|✗" "$log_file" 2>/dev/null || echo "0") results" >> "$summary_file"
    fi
done

echo ""
echo "=== Benchmark Complete ==="
echo "Total runs: $total_runs"
echo "Successful: $successful_runs" 
echo "Failed: $failed_runs"
echo "Results saved to: ${RESULTS_DIR}"
echo "Summary: ${summary_file}"
echo "End time: $(date)"

# Automatically run analysis and generate plots
echo ""
echo "=== Running Analysis and Generating Plots ==="
ANALYSIS_SCRIPT="${SCRIPT_DIR}/tools/analyze_abs_pose_results.py"

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