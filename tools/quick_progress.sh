#!/opt/homebrew/bin/bash

# Quick Progress Checker for Benchmark
echo "=== Benchmark Progress Check ==="
echo ""

# Check if benchmark is running
if ps aux | grep -q "[b]enchmark_abs_pose.sh"; then
    echo "✅ Benchmark is RUNNING"
else
    echo "❌ Benchmark is NOT running"
fi

# Check log files
LOGS_DIR="../benchmark_results/abs_pose/logs"
if [ -d "$LOGS_DIR" ]; then
    total_logs=$(ls "$LOGS_DIR"/*.log 2>/dev/null | wc -l | tr -d ' ')
    completed=$(grep -l "=== Data Generation Complete! ===" "$LOGS_DIR"/*.log 2>/dev/null | wc -l | tr -d ' ')
    
    echo "📊 Log files: $total_logs created, $completed completed"
    echo "📈 Progress: $completed/32 runs complete ($(( completed * 100 / 32 ))%)"
    
    # Show what's currently running
    echo ""
    echo "🔄 Currently active:"
    for log in "$LOGS_DIR"/*.log; do
        if [ -f "$log" ] && grep -q "Started:" "$log" 2>/dev/null && ! grep -q "=== Data Generation Complete! ===" "$log" 2>/dev/null; then
            base=$(basename "$log" .log)
            last_line=$(tail -5 "$log" 2>/dev/null | grep -E "(Problem|noise|Processing)" | tail -1 || echo "Running...")
            echo "  • $base: $last_line"
        fi
    done
    
    # Show recent completions
    if [ $completed -gt 0 ]; then
        echo ""
        echo "✅ Recently completed:"
        grep -l "=== Data Generation Complete! ===" "$LOGS_DIR"/*.log 2>/dev/null | xargs -I {} basename {} .log | tail -3 | sed 's/^/  ✓ /'
    fi
else
    echo "📂 No results directory found yet"
fi

echo ""
echo "Commands:"
echo "  tail -f benchmark_full.log     # Watch full output"
echo "  ./monitor_benchmark.sh         # Real-time monitor" 
echo "  ./quick_progress.sh             # Check status (this script)" 