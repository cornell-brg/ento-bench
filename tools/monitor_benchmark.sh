#!/opt/homebrew/bin/bash

# Benchmark Progress Monitor
# Shows real-time progress of the benchmark with completion bar

RESULTS_DIR="../benchmark_results/abs_pose"
LOGS_DIR="${RESULTS_DIR}/logs"

# Expected totals based on benchmark configuration
EXPECTED_TOTAL_RUNS=32  # 4 P3P + 4 UP2P + 24 DLT runs
EXPECTED_NOISE_LEVELS=13  # Number of noise levels in each run

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

clear_line() {
    echo -e "\033[2K\r\c"
}

progress_bar() {
    local current=$1
    local total=$2
    local width=50
    local percentage=$((current * 100 / total))
    local completed=$((current * width / total))
    local remaining=$((width - completed))
    
    printf "["
    printf "%*s" $completed | tr ' ' '='
    if [ $completed -lt $width ]; then
        printf ">"
        printf "%*s" $((remaining - 1))
    fi
    printf "] %d%% (%d/%d)" $percentage $current $total
}

echo "=== Enhanced Absolute Pose Benchmark Monitor ==="
echo "Monitoring: ${RESULTS_DIR}"
echo "Press Ctrl+C to stop monitoring"
echo ""

while true; do
    if [ ! -d "$LOGS_DIR" ]; then
        echo "Waiting for benchmark to start..."
        sleep 2
        continue
    fi
    
    # Count completed log files
    completed_runs=0
    total_log_files=0
    active_runs=0
    
    # Count all log files
    if ls "$LOGS_DIR"/*.log >/dev/null 2>&1; then
        total_log_files=$(ls "$LOGS_DIR"/*.log 2>/dev/null | wc -l | tr -d ' ')
        
        # Check each log file for completion
        for log_file in "$LOGS_DIR"/*.log; do
            if [ -f "$log_file" ]; then
                if grep -q "=== Data Generation Complete! ===" "$log_file" 2>/dev/null; then
                    ((completed_runs++))
                elif grep -q "Started:" "$log_file" 2>/dev/null; then
                    ((active_runs++))
                fi
            fi
        done
    fi
    
    # Currently running process check
    running_processes=$(ps aux | grep -c "[g]enerate_minimal_solver_data_abs_enhanced" || echo "0")
    
    clear_line
    echo -e "${BLUE}=== Benchmark Progress ===${NC}"
    echo ""
    
    # Overall run progress
    echo -e "${YELLOW}Run Progress:${NC}"
    progress_bar $completed_runs $EXPECTED_TOTAL_RUNS
    echo ""
    echo ""
    
    # Detailed status
    echo -e "${GREEN}Completed runs:${NC} $completed_runs"
    echo -e "${BLUE}Active runs:${NC} $active_runs"
    echo -e "${YELLOW}Total log files:${NC} $total_log_files"
    echo -e "${YELLOW}Running processes:${NC} $running_processes"
    echo ""
    
    # Show currently active log files (last few lines)
    if [ $active_runs -gt 0 ]; then
        echo -e "${BLUE}Currently running:${NC}"
        for log_file in "$LOGS_DIR"/*.log; do
            if [ -f "$log_file" ] && grep -q "Started:" "$log_file" 2>/dev/null && ! grep -q "=== Data Generation Complete! ===" "$log_file" 2>/dev/null; then
                base_name=$(basename "$log_file" .log)
                # Get the last non-empty line that looks like progress
                last_line=$(tail -10 "$log_file" 2>/dev/null | grep -E "(Problem|noise|✓|Processing)" | tail -1 || echo "Running...")
                echo "  • $base_name: $last_line"
            fi
        done
        echo ""
    fi
    
    # Show recent completions
    if [ $completed_runs -gt 0 ]; then
        echo -e "${GREEN}Recently completed:${NC}"
        for log_file in "$LOGS_DIR"/*.log; do
            if [ -f "$log_file" ] && grep -q "=== Data Generation Complete! ===" "$log_file" 2>/dev/null; then
                base_name=$(basename "$log_file" .log)
                completion_time=$(grep "=== Data Generation Complete! ===" "$log_file" -A 5 | head -1 || echo "")
                echo "  ✓ $base_name"
            fi
        done | tail -5  # Show last 5 completions
        echo ""
    fi
    
    # Estimated completion
    if [ $completed_runs -gt 0 ] && [ $active_runs -gt 0 ]; then
        remaining_runs=$((EXPECTED_TOTAL_RUNS - completed_runs))
        echo -e "${YELLOW}Remaining runs:${NC} $remaining_runs"
        
        # Simple time estimation based on completion rate
        if [ $completed_runs -gt 1 ]; then
            # This is very rough - you might want to improve this
            echo -e "${YELLOW}Status:${NC} Benchmark in progress..."
        fi
        echo ""
    fi
    
    # Check if completely done
    if [ $completed_runs -eq $EXPECTED_TOTAL_RUNS ]; then
        echo -e "${GREEN}🎉 BENCHMARK COMPLETE! 🎉${NC}"
        echo ""
        echo "All $EXPECTED_TOTAL_RUNS runs completed successfully!"
        echo "Results are ready in: $RESULTS_DIR"
        echo ""
        echo "Run analysis with:"
        echo "  cd tools && python analyze_abs_pose_results.py"
        break
    fi
    
    # Wait before next update
    sleep 5
    
    # Clear screen for next update
    echo -e "\033[2J\033[H"
done 