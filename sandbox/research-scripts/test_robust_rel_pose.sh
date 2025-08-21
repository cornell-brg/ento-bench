#!/bin/bash

# test_all_solvers.sh - Test all relative pose solvers with identical data
# Usage: ./test_all_solvers.sh [points] [problems] [outlier_ratio] [noise] [data_generator]

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to get color based on success rate
get_color() {
    local rate=$1
    if (( $(echo "$rate >= 90" | bc -l) )); then
        echo "$GREEN"
    elif (( $(echo "$rate >= 70" | bc -l) )); then
        echo "$YELLOW"
    else
        echo "$RED"
    fi
}

# Function to run a single test
run_test() {
    local solver=$1
    local precision=$2
    local points=$3
    local problems=$4
    local outliers=$5
    local noise=$6
    local data_gen=$7
    
    echo -e "${CYAN}Testing $solver ($precision precision)...${NC}"
    
    # Run the test and capture output
    output=$(./src/ento-pose/rel-pose/bin/generate_robust_relative_pose_data \
        --solver-type "$solver" \
        --precision "$precision" \
        --points "$points" \
        --problems "$problems" \
        --outlier-ratio "$outliers" \
        --noise "$noise" \
        --refinement nonlinear \
        --data-generator "$data_gen" 2>/dev/null)
    
    # Extract key metrics using grep and awk
    success_rate=$(echo "$output" | grep "Success rate:" | awk '{print $3}' | sed 's/%//')
    mean_iterations=$(echo "$output" | grep -A5 "Iteration Statistics" | grep "Mean:" | awk '{print $2}')
    mean_rot_error=$(echo "$output" | grep -A5 "Rotation Error Statistics" | grep "Mean:" | awk '{print $2}' | sed 's/°//')
    mean_trans_error=$(echo "$output" | grep -A5 "Translation Error Statistics" | grep "Mean:" | awk '{print $2}' | sed 's/°//')
    
    # Handle cases where no successful solutions exist
    if [[ -z "$mean_rot_error" ]]; then
        mean_rot_error="N/A"
        mean_trans_error="N/A"
    fi
    
    # Format numbers for display
    if [[ "$mean_iterations" =~ ^[0-9]+\.[0-9]+$ ]]; then
        mean_iterations=$(printf "%.1f" "$mean_iterations")
    fi
    if [[ "$mean_rot_error" != "N/A" && "$mean_rot_error" =~ ^[0-9]+\.[0-9]+$ ]]; then
        mean_rot_error=$(printf "%.3f" "$mean_rot_error")
    fi
    if [[ "$mean_trans_error" != "N/A" && "$mean_trans_error" =~ ^[0-9]+\.[0-9]+$ ]]; then
        mean_trans_error=$(printf "%.3f" "$mean_trans_error")
    fi
    
    # Store results in global arrays for summary
    solvers_tested+=("$solver ($precision)")
    success_rates+=("$success_rate")
    iterations+=("$mean_iterations")
    rot_errors+=("$mean_rot_error")
    trans_errors+=("$mean_trans_error")
    
    # Get color for success rate
    color=$(get_color "$success_rate")
    
    # Print results
    printf "  ${color}Success: %6s%%${NC} | Iter: %8s | RotErr: %8s° | TransErr: %8s°\n" \
           "$success_rate" "$mean_iterations" "$mean_rot_error" "$mean_trans_error"
}

# Main function
main() {
    # Default parameters
    POINTS=32
    PROBLEMS=3
    OUTLIERS=0.0
    NOISE=0.0
    DATA_GEN="simple"
    PRECISION="both"  # "float", "double", or "both"
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --points)
                POINTS="$2"
                shift 2
                ;;
            --problems)
                PROBLEMS="$2"
                shift 2
                ;;
            --outliers)
                OUTLIERS="$2"
                shift 2
                ;;
            --noise)
                NOISE="$2"
                shift 2
                ;;
            --data-generator)
                DATA_GEN="$2"
                shift 2
                ;;
            --precision)
                PRECISION="$2"
                shift 2
                ;;
            --help)
                echo "Usage: $0 [OPTIONS]"
                echo "Options:"
                echo "  --points N          Number of points (default: 32)"
                echo "  --problems N        Number of problems (default: 3)"
                echo "  --outliers RATIO    Outlier ratio 0.0-1.0 (default: 0.0)"
                echo "  --noise LEVEL       Noise level (default: 0.0)"
                echo "  --data-generator TYPE  Data generator: simple, realistic, entobench (default: simple)"
                echo "  --precision TYPE    Precision: float, double, both (default: both)"
                echo "  --help              Show this help"
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done
    
    # Initialize arrays for storing results
    solvers_tested=()
    success_rates=()
    iterations=()
    rot_errors=()
    trans_errors=()
    
    # List of solvers to test
    solvers=("5pt" "8pt" "upright_3pt" "upright_planar_2pt" "upright_planar_3pt")
    
    echo -e "${BLUE}=== EntoBench Relative Pose Solver Comparison ===${NC}"
    echo -e "Parameters: ${POINTS} points, ${PROBLEMS} problems, ${OUTLIERS} outliers, ${NOISE} noise, ${DATA_GEN} data"
    echo ""
    
    # Test each solver
    for solver in "${solvers[@]}"; do
        if [[ "$PRECISION" == "both" ]]; then
            run_test "$solver" "float" "$POINTS" "$PROBLEMS" "$OUTLIERS" "$NOISE" "$DATA_GEN"
            run_test "$solver" "double" "$POINTS" "$PROBLEMS" "$OUTLIERS" "$NOISE" "$DATA_GEN"
        else
            run_test "$solver" "$PRECISION" "$POINTS" "$PROBLEMS" "$OUTLIERS" "$NOISE" "$DATA_GEN"
        fi
        echo ""
    done
    
    # Summary table
    echo -e "${BLUE}=== Summary Table ===${NC}"
    printf "%-25s | %8s | %8s | %10s | %12s\n" "Solver" "Success%" "Iter" "RotErr(°)" "TransErr(°)"
    printf "%-25s-+-%8s-+-%8s-+-%10s-+-%12s\n" "-------------------------" "--------" "--------" "----------" "------------"
    
    for i in "${!solvers_tested[@]}"; do
        color=$(get_color "${success_rates[$i]}")
        printf "${color}%-25s${NC} | %8s | %8s | %10s | %12s\n" \
               "${solvers_tested[$i]}" "${success_rates[$i]}%" "${iterations[$i]}" "${rot_errors[$i]}" "${trans_errors[$i]}"
    done
    
    echo ""
    
    # Analysis
    echo -e "${BLUE}=== Analysis ===${NC}"
    
    # Find best performer (highest success rate)
    best_idx=0
    best_rate=${success_rates[0]}
    for i in "${!success_rates[@]}"; do
        if (( $(echo "${success_rates[$i]} > $best_rate" | bc -l) )); then
            best_rate=${success_rates[$i]}
            best_idx=$i
        fi
    done
    
    # Find most efficient (lowest iterations among successful solvers)
    min_iter=999999
    most_efficient_idx=-1
    for i in "${!success_rates[@]}"; do
        if (( $(echo "${success_rates[$i]} >= 90" | bc -l) )); then
            if [[ "${iterations[$i]}" != "N/A" ]] && (( $(echo "${iterations[$i]} < $min_iter" | bc -l) )); then
                min_iter=${iterations[$i]}
                most_efficient_idx=$i
            fi
        fi
    done
    
    # Find most accurate (lowest rotation error among successful solvers)
    min_rot_err=999999
    most_accurate_idx=-1
    for i in "${!success_rates[@]}"; do
        if (( $(echo "${success_rates[$i]} >= 90" | bc -l) )); then
            if [[ "${rot_errors[$i]}" != "N/A" ]] && (( $(echo "${rot_errors[$i]} < $min_rot_err" | bc -l) )); then
                min_rot_err=${rot_errors[$i]}
                most_accurate_idx=$i
            fi
        fi
    done
    
    echo -e "${GREEN}Best Success Rate:${NC} ${solvers_tested[$best_idx]} (${success_rates[$best_idx]}%)"
    
    if [[ $most_efficient_idx -ne -1 ]]; then
        echo -e "${GREEN}Most Efficient:${NC} ${solvers_tested[$most_efficient_idx]} (${iterations[$most_efficient_idx]} iterations)"
    fi
    
    if [[ $most_accurate_idx -ne -1 ]]; then
        echo -e "${GREEN}Most Accurate:${NC} ${solvers_tested[$most_accurate_idx]} (${rot_errors[$most_accurate_idx]}° rotation error)"
    fi
    
    # Precision comparison for 5pt
    echo ""
    echo -e "${BLUE}=== Precision Impact Analysis ===${NC}"
    if [[ "$PRECISION" == "both" ]]; then
        float_5pt_idx=-1
        double_5pt_idx=-1
        for i in "${!solvers_tested[@]}"; do
            if [[ "${solvers_tested[$i]}" == "5pt (float)" ]]; then
                float_5pt_idx=$i
            elif [[ "${solvers_tested[$i]}" == "5pt (double)" ]]; then
                double_5pt_idx=$i
            fi
        done
        
        if [[ $float_5pt_idx -ne -1 && $double_5pt_idx -ne -1 ]]; then
            float_rate=${success_rates[$float_5pt_idx]}
            double_rate=${success_rates[$double_5pt_idx]}
            improvement=$(echo "$double_rate - $float_rate" | bc -l)
            
            echo -e "5pt Algorithm Precision Impact:"
            echo -e "  Float:  ${success_rates[$float_5pt_idx]}% success"
            echo -e "  Double: ${success_rates[$double_5pt_idx]}% success"
            echo -e "  ${GREEN}Improvement: +${improvement}%${NC}"
            
            if (( $(echo "$improvement > 20" | bc -l) )); then
                echo -e "  ${YELLOW}⚠️  Significant precision sensitivity detected!${NC}"
                echo -e "  ${YELLOW}   Recommendation: Use double precision for 5pt algorithm${NC}"
            fi
        fi
    fi
    
    echo ""
    echo -e "${CYAN}Legend:${NC}"
    echo -e "  ${GREEN}Green${NC}: ≥90% success rate (excellent)"
    echo -e "  ${YELLOW}Yellow${NC}: 70-89% success rate (good)"
    echo -e "  ${RED}Red${NC}: <70% success rate (poor)"
}

# Check if bc is available
if ! command -v bc &> /dev/null; then
    echo "Error: 'bc' calculator is required but not installed."
    echo "Please install bc: brew install bc (macOS) or apt-get install bc (Ubuntu)"
    exit 1
fi

# Run main function with all arguments
main "$@" 