#!/bin/bash

# =============================================================================
# EntoBench Pose Estimation Benchmark Runner
# =============================================================================
# This script runs groups of pose estimation benchmarks with timeout support,
# progress tracking, and result collection.
#
# Usage:
#   ./run_pose_benchmarks.sh [OPTIONS] [BENCHMARK_GROUP]
#
# Examples:
#   ./run_pose_benchmarks.sh --group absolute-pose --timeout 60
#   ./run_pose_benchmarks.sh --group relative-pose --timeout 120 --output-timeout 10
#   ./run_pose_benchmarks.sh --group gold-standard --verbose
#   ./run_pose_benchmarks.sh --list-groups
# =============================================================================

set -euo pipefail

# Default configuration
DEFAULT_TIMEOUT=60
DEFAULT_OUTPUT_TIMEOUT=10
DEFAULT_BUILD_DIR="build"
DEFAULT_RESULTS_DIR="benchmark_results"
VERBOSE=false
DRY_RUN=false

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Benchmark group definitions
declare -A BENCHMARK_GROUPS

# Absolute pose benchmarks
BENCHMARK_GROUPS[absolute-pose]="bench-p3p-float bench-p3p-double bench-up2p-float bench-up2p-double bench-dlt-float bench-dlt-double"

# Relative pose benchmarks  
BENCHMARK_GROUPS[relative-pose]="bench-8pt-float bench-8pt-double bench-5pt-float bench-5pt-double bench-upright-3pt-float bench-upright-3pt-double"

# DLT scaling benchmarks
BENCHMARK_GROUPS[dlt-scaling]="bench-dlt8-float bench-dlt16-float bench-dlt16-double bench-dlt32-float bench-dlt32-double bench-dlt64-float bench-dlt64-double"

# 8pt scaling benchmarks
BENCHMARK_GROUPS[8pt-scaling]="bench-8pt16-float bench-8pt16-double bench-8pt32-float bench-8pt32-double bench-8pt64-float bench-8pt64-double"

# Upright planar benchmarks
BENCHMARK_GROUPS[upright-planar]="bench-upright-planar-3pt-float bench-upright-planar-3pt-double bench-upright-planar-2pt-float bench-upright-planar-2pt-double"

# Upright planar scaling benchmarks
BENCHMARK_GROUPS[upright-planar-scaling]="bench-upright-planar-3pt8-float bench-upright-planar-3pt8-double bench-upright-planar-3pt16-float bench-upright-planar-3pt16-double bench-upright-planar-3pt32-float bench-upright-planar-3pt32-double"

# Gold standard benchmarks (when implemented)
BENCHMARK_GROUPS[gold-standard]="bench-gold-standard-abs-float bench-gold-standard-abs-double bench-gold-standard-rel-float bench-gold-standard-rel-double"

# All pose estimation benchmarks
BENCHMARK_GROUPS[all-pose]="${BENCHMARK_GROUPS[absolute-pose]} ${BENCHMARK_GROUPS[relative-pose]}"

# Quick test group (fast benchmarks for testing)
BENCHMARK_GROUPS[quick-test]="bench-p3p-float bench-8pt-float bench-dlt-float"

# Performance group (comprehensive benchmarks)
BENCHMARK_GROUPS[performance]="${BENCHMARK_GROUPS[all-pose]} ${BENCHMARK_GROUPS[dlt-scaling]} ${BENCHMARK_GROUPS[8pt-scaling]}"

# Function to print usage
print_usage() {
    cat << EOF
EntoBench Pose Estimation Benchmark Runner

USAGE:
    $0 [OPTIONS] [BENCHMARK_GROUP]

OPTIONS:
    -g, --group GROUP           Benchmark group to run (see --list-groups)
    -t, --timeout SECONDS      Maximum time per benchmark (default: ${DEFAULT_TIMEOUT}s)
    -o, --output-timeout SEC    Timeout if no output for SEC seconds (default: ${DEFAULT_OUTPUT_TIMEOUT}s)
    -b, --build-dir DIR         Build directory (default: ${DEFAULT_BUILD_DIR})
    -r, --results-dir DIR       Results output directory (default: ${DEFAULT_RESULTS_DIR})
    -v, --verbose               Enable verbose output
    -n, --dry-run               Show what would be run without executing
    -l, --list-groups           List available benchmark groups
    -h, --help                  Show this help message

BENCHMARK GROUPS:
    absolute-pose               P3P, UP2P, DLT benchmarks
    relative-pose               8pt, 5pt, upright 3pt benchmarks
    dlt-scaling                 DLT with different point counts (8, 16, 32, 64)
    8pt-scaling                 8pt with different point counts (16, 32, 64)
    upright-planar              Upright planar 2pt and 3pt benchmarks
    upright-planar-scaling      Upright planar with different point counts
    gold-standard               Gold standard absolute and relative pose
    all-pose                    All basic pose estimation benchmarks
    quick-test                  Fast subset for testing (P3P, 8pt, DLT float)
    performance                 Comprehensive performance evaluation

EXAMPLES:
    # Run absolute pose benchmarks with 60s timeout
    $0 --group absolute-pose --timeout 60

    # Run quick test with verbose output
    $0 --group quick-test --verbose

    # Run all pose benchmarks with custom timeouts
    $0 --group all-pose --timeout 120 --output-timeout 15

    # Dry run to see what would be executed
    $0 --group performance --dry-run

    # List all available groups
    $0 --list-groups

TIMEOUT BEHAVIOR:
    - Each benchmark has a maximum runtime of TIMEOUT seconds
    - If no output is seen for OUTPUT_TIMEOUT seconds, the benchmark is killed
    - Results are collected even for timed-out benchmarks
    - Summary shows success/timeout/failure counts

OUTPUT:
    - Individual benchmark logs: \${RESULTS_DIR}/\${benchmark_name}.log
    - Summary report: \${RESULTS_DIR}/summary.txt
    - Timing information: \${RESULTS_DIR}/timing.csv
EOF
}

# Function to list available benchmark groups
list_groups() {
    echo -e "${BLUE}Available Benchmark Groups:${NC}"
    echo
    for group in "${!BENCHMARK_GROUPS[@]}"; do
        benchmarks=(${BENCHMARK_GROUPS[$group]})
        count=${#benchmarks[@]}
        echo -e "  ${CYAN}${group}${NC} (${count} benchmarks)"
        if [[ $VERBOSE == true ]]; then
            echo "    Benchmarks: ${BENCHMARK_GROUPS[$group]}"
        fi
    done
    echo
    echo "Use --verbose with --list-groups to see individual benchmarks in each group."
}

# Function to log with timestamp
log() {
    echo -e "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

# Function to log with color
log_info() {
    log "${BLUE}[INFO]${NC} $*"
}

log_success() {
    log "${GREEN}[SUCCESS]${NC} $*"
}

log_warning() {
    log "${YELLOW}[WARNING]${NC} $*"
}

log_error() {
    log "${RED}[ERROR]${NC} $*"
}

# Function to run a single benchmark with timeout
run_benchmark() {
    local benchmark_name="$1"
    local timeout="$2"
    local output_timeout="$3"
    local results_dir="$4"
    local build_dir="$5"
    
    local log_file="${results_dir}/${benchmark_name}.log"
    local timing_file="${results_dir}/${benchmark_name}.timing"
    local start_time=$(date +%s)
    
    log_info "Running ${benchmark_name} (timeout: ${timeout}s, output timeout: ${output_timeout}s)"
    
    if [[ $DRY_RUN == true ]]; then
        echo "  [DRY RUN] Would execute: timeout ${timeout}s stdbuf -oL -eL ${build_dir}/bin/${benchmark_name}"
        return 0
    fi
    
    # Create a temporary script for the benchmark execution
    local temp_script=$(mktemp)
    cat > "$temp_script" << EOF
#!/bin/bash
cd "${build_dir}"
exec ./bin/${benchmark_name}
EOF
    chmod +x "$temp_script"
    
    # Run with timeout and output monitoring
    local exit_code=0
    local timed_out=false
    local output_timed_out=false
    
    # Use timeout with output monitoring
    if timeout --preserve-status "${timeout}s" \
       stdbuf -oL -eL \
       timeout --preserve-status "${output_timeout}s" \
       bash -c "
           exec 3< <(bash '$temp_script' 2>&1)
           while IFS= read -r -u 3 line || [[ -n \$line ]]; do
               echo \"\$line\"
               echo \"\$line\" >> '$log_file'
           done
       " > >(tee -a "$log_file") 2>&1; then
        exit_code=0
    else
        exit_code=$?
        if [[ $exit_code -eq 124 ]]; then
            timed_out=true
            log_warning "${benchmark_name} timed out after ${timeout}s"
            echo "BENCHMARK TIMED OUT AFTER ${timeout}s" >> "$log_file"
        elif [[ $exit_code -eq 142 ]]; then
            output_timed_out=true
            log_warning "${benchmark_name} output timed out after ${output_timeout}s"
            echo "BENCHMARK OUTPUT TIMED OUT AFTER ${output_timeout}s" >> "$log_file"
        else
            log_error "${benchmark_name} failed with exit code ${exit_code}"
            echo "BENCHMARK FAILED WITH EXIT CODE ${exit_code}" >> "$log_file"
        fi
    fi
    
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    
    # Record timing information
    echo "${benchmark_name},${start_time},${end_time},${duration},${exit_code},${timed_out},${output_timed_out}" >> "${results_dir}/timing.csv"
    
    # Clean up
    rm -f "$temp_script"
    
    if [[ $timed_out == true ]]; then
        return 124
    elif [[ $output_timed_out == true ]]; then
        return 142
    else
        return $exit_code
    fi
}

# Function to generate summary report
generate_summary() {
    local results_dir="$1"
    local group_name="$2"
    local benchmarks=("${@:3}")
    
    local summary_file="${results_dir}/summary.txt"
    local timing_file="${results_dir}/timing.csv"
    
    log_info "Generating summary report: ${summary_file}"
    
    cat > "$summary_file" << EOF
EntoBench Pose Estimation Benchmark Results
==========================================

Group: ${group_name}
Date: $(date)
Total Benchmarks: ${#benchmarks[@]}

EOF
    
    # Count results
    local success_count=0
    local timeout_count=0
    local output_timeout_count=0
    local failure_count=0
    local total_duration=0
    
    if [[ -f "$timing_file" ]]; then
        echo "Individual Benchmark Results:" >> "$summary_file"
        echo "=============================" >> "$summary_file"
        echo >> "$summary_file"
        
        while IFS=',' read -r name start_time end_time duration exit_code timed_out output_timed_out; do
            if [[ "$name" == "benchmark_name" ]]; then continue; fi  # Skip header
            
            total_duration=$((total_duration + duration))
            
            local status=""
            if [[ "$timed_out" == "true" ]]; then
                status="TIMEOUT"
                timeout_count=$((timeout_count + 1))
            elif [[ "$output_timed_out" == "true" ]]; then
                status="OUTPUT_TIMEOUT"
                output_timeout_count=$((output_timeout_count + 1))
            elif [[ "$exit_code" == "0" ]]; then
                status="SUCCESS"
                success_count=$((success_count + 1))
            else
                status="FAILED"
                failure_count=$((failure_count + 1))
            fi
            
            printf "%-30s %10s %8ds\n" "$name" "$status" "$duration" >> "$summary_file"
        done < "$timing_file"
    fi
    
    cat >> "$summary_file" << EOF

Summary Statistics:
==================
Successful:        ${success_count}
Timed out:         ${timeout_count}
Output timed out:  ${output_timeout_count}
Failed:            ${failure_count}
Total duration:    ${total_duration}s

Log files location: ${results_dir}/
Timing data:       ${timing_file}

EOF
    
    # Print summary to console
    echo
    log_info "Benchmark Results Summary:"
    echo -e "  ${GREEN}Successful:${NC}        ${success_count}"
    echo -e "  ${YELLOW}Timed out:${NC}         ${timeout_count}"
    echo -e "  ${YELLOW}Output timed out:${NC}  ${output_timeout_count}"
    echo -e "  ${RED}Failed:${NC}            ${failure_count}"
    echo -e "  ${BLUE}Total duration:${NC}    ${total_duration}s"
    echo
    echo -e "Full report: ${CYAN}${summary_file}${NC}"
}

# Main function
main() {
    local group_name=""
    local timeout=$DEFAULT_TIMEOUT
    local output_timeout=$DEFAULT_OUTPUT_TIMEOUT
    local build_dir=$DEFAULT_BUILD_DIR
    local results_dir=$DEFAULT_RESULTS_DIR
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -g|--group)
                group_name="$2"
                shift 2
                ;;
            -t|--timeout)
                timeout="$2"
                shift 2
                ;;
            -o|--output-timeout)
                output_timeout="$2"
                shift 2
                ;;
            -b|--build-dir)
                build_dir="$2"
                shift 2
                ;;
            -r|--results-dir)
                results_dir="$2"
                shift 2
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            -n|--dry-run)
                DRY_RUN=true
                shift
                ;;
            -l|--list-groups)
                list_groups
                exit 0
                ;;
            -h|--help)
                print_usage
                exit 0
                ;;
            *)
                if [[ -z "$group_name" ]]; then
                    group_name="$1"
                else
                    log_error "Unknown option: $1"
                    print_usage
                    exit 1
                fi
                shift
                ;;
        esac
    done
    
    # Validate arguments
    if [[ -z "$group_name" ]]; then
        log_error "Benchmark group must be specified"
        echo
        print_usage
        exit 1
    fi
    
    if [[ ! -v BENCHMARK_GROUPS["$group_name"] ]]; then
        log_error "Unknown benchmark group: $group_name"
        echo
        echo "Available groups:"
        list_groups
        exit 1
    fi
    
    # Check build directory
    if [[ ! -d "$build_dir" ]]; then
        log_error "Build directory not found: $build_dir"
        exit 1
    fi
    
    if [[ ! -d "$build_dir/bin" ]]; then
        log_error "Benchmark binaries directory not found: $build_dir/bin"
        exit 1
    fi
    
    # Create results directory
    mkdir -p "$results_dir"
    
    # Get benchmarks for the group
    local benchmarks=(${BENCHMARK_GROUPS[$group_name]})
    
    log_info "Starting benchmark group: ${group_name}"
    log_info "Benchmarks to run: ${#benchmarks[@]}"
    log_info "Timeout per benchmark: ${timeout}s"
    log_info "Output timeout: ${output_timeout}s"
    log_info "Results directory: ${results_dir}"
    
    if [[ $VERBOSE == true ]]; then
        log_info "Benchmark list: ${benchmarks[*]}"
    fi
    
    # Initialize timing CSV
    echo "benchmark_name,start_time,end_time,duration_s,exit_code,timed_out,output_timed_out" > "${results_dir}/timing.csv"
    
    # Run benchmarks
    local current=1
    local total=${#benchmarks[@]}
    
    for benchmark in "${benchmarks[@]}"; do
        echo
        log_info "Progress: ${current}/${total} - ${benchmark}"
        
        # Check if benchmark binary exists
        if [[ ! -f "$build_dir/bin/$benchmark" ]]; then
            log_warning "Benchmark binary not found: $build_dir/bin/$benchmark"
            echo "${benchmark},0,0,0,127,false,false" >> "${results_dir}/timing.csv"
            echo "BENCHMARK BINARY NOT FOUND" > "${results_dir}/${benchmark}.log"
            current=$((current + 1))
            continue
        fi
        
        run_benchmark "$benchmark" "$timeout" "$output_timeout" "$results_dir" "$build_dir"
        local result=$?
        
        if [[ $result -eq 0 ]]; then
            log_success "${benchmark} completed successfully"
        elif [[ $result -eq 124 ]]; then
            log_warning "${benchmark} timed out"
        elif [[ $result -eq 142 ]]; then
            log_warning "${benchmark} output timed out"
        else
            log_error "${benchmark} failed"
        fi
        
        current=$((current + 1))
    done
    
    # Generate summary
    echo
    generate_summary "$results_dir" "$group_name" "${benchmarks[@]}"
    
    log_success "Benchmark run completed!"
}

# Run main function with all arguments
main "$@" 