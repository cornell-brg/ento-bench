#!/bin/bash

# Vectorization Comparison Benchmark Script
# This script runs the same benchmark with vectorization enabled and disabled
# to compare performance differences

set -e

BENCHMARK_NAME=${1:-"bench_block_of_small"}
OUTPUT_DIR=${2:-"vectorization_results"}
REPS=${3:-5}

echo "=== Block-Based Optical Flow Vectorization Comparison ==="
echo "Benchmark: $BENCHMARK_NAME"
echo "Output Directory: $OUTPUT_DIR"
echo "Repetitions: $REPS"
echo

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Function to update config and run benchmark
run_benchmark() {
    local vectorization_enabled=$1
    local output_suffix=$2
    
    echo "--- Running with vectorization $output_suffix ---"
    
    # Update the config file
    cat > benchmark/configs/perception_benchmarks.json << EOF
{
  "perception": {
    "reps": $REPS,
    "inner_reps": 1,
    "verbosity": 1,
    "enable_caches": true,
    "enable_vectorization": $vectorization_enabled
  }
}
EOF
    
    # Run the benchmark
    local output_file="$OUTPUT_DIR/${BENCHMARK_NAME}_${output_suffix}.log"
    echo "Running benchmark, output to: $output_file"
    
    if [ -f "build/benchmark/perception/$BENCHMARK_NAME" ]; then
        ./build/benchmark/perception/$BENCHMARK_NAME > "$output_file" 2>&1
        echo "Completed: $output_file"
    else
        echo "Error: Benchmark executable not found at build/benchmark/perception/$BENCHMARK_NAME"
        echo "Please build the project first with: make -C build"
        exit 1
    fi
    
    # Extract key metrics
    local avg_cycles=$(grep "Average cycles:" "$output_file" | awk '{print $3}')
    local min_cycles=$(grep "Min cycles:" "$output_file" | awk '{print $3}')
    local max_cycles=$(grep "Max cycles:" "$output_file" | awk '{print $3}')
    
    echo "  Average cycles: $avg_cycles"
    echo "  Min cycles: $min_cycles"
    echo "  Max cycles: $max_cycles"
    echo
}

# Run with vectorization enabled
run_benchmark "true" "vectorized"

# Run with vectorization disabled  
run_benchmark "false" "scalar"

# Generate comparison report
echo "=== Vectorization Comparison Report ==="
echo "Generated at: $(date)"
echo

vectorized_avg=$(grep "Average cycles:" "$OUTPUT_DIR/${BENCHMARK_NAME}_vectorized.log" | awk '{print $3}')
scalar_avg=$(grep "Average cycles:" "$OUTPUT_DIR/${BENCHMARK_NAME}_scalar.log" | awk '{print $3}')

if [ -n "$vectorized_avg" ] && [ -n "$scalar_avg" ]; then
    echo "Vectorized average cycles: $vectorized_avg"
    echo "Scalar average cycles: $scalar_avg"
    
    # Calculate speedup (using bc for floating point arithmetic)
    if command -v bc >/dev/null 2>&1; then
        speedup=$(echo "scale=2; $scalar_avg / $vectorized_avg" | bc)
        improvement=$(echo "scale=1; ($scalar_avg - $vectorized_avg) / $scalar_avg * 100" | bc)
        echo "Speedup: ${speedup}x"
        echo "Performance improvement: ${improvement}%"
    else
        echo "Install 'bc' for speedup calculations"
    fi
else
    echo "Could not extract cycle counts for comparison"
fi

echo
echo "Detailed logs available in: $OUTPUT_DIR/"
echo "- ${BENCHMARK_NAME}_vectorized.log"
echo "- ${BENCHMARK_NAME}_scalar.log" 