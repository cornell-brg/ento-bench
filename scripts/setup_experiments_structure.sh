#!/bin/bash

# =============================================================================
# EntoBench Experiment Structure Setup Script
# =============================================================================
# This script creates the standard experiment folder structure used in EntoBench
# for organizing benchmark experiments by project, MCU, and algorithm stage.
#
# Structure:
# experiments/
# ├── [project-name]/
# │   ├── [mcu-name]/
# │   │   ├── control/
# │   │   │   ├── adaptive-controller/
# │   │   │   ├── geometric-controller/
# │   │   │   ├── lqr/
# │   │   │   ├── mpc/
# │   │   │   └── tinympc/
# │   │   ├── ekf/
# │   │   │   ├── robobee-ekf/
# │   │   │   ├── robofly-ekf/
# │   │   │   ├── seq-update/
# │   │   │   ├── sync-update/
# │   │   │   └── truncated-update/
# │   │   ├── perception/
# │   │   │   ├── bbof/
# │   │   │   ├── fastbrief/
# │   │   │   ├── iiof/
# │   │   │   ├── lkof/
# │   │   │   ├── orb/
# │   │   │   └── sift/
# │   │   ├── pose-est/
# │   │   │   ├── 3pt/
# │   │   │   ├── 5pt/
# │   │   │   ├── 8pt/
# │   │   │   ├── dlt/
# │   │   │   ├── homography/
# │   │   │   ├── p3p/
# │   │   │   └── up2p/
# │   │   └── example/
# │   │       └── benchmark-example/
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    cat << EOF
Usage: $0 [OPTIONS] [MCU_NAMES...]

OPTIONS:
    -h, --help          Show this help message
    -d, --directory     Base directory for experiments (default: ./experiments)
    -p, --project       Project name folder (default: ae)
    -f, --force         Force creation even if directories exist
    -v, --verbose       Verbose output

MCU_NAMES:
    Space-separated list of MCU names to create folders for.
    If none provided, creates default MCU folders.

EXAMPLES:
    # Create default structure: experiments/ae/[default-mcus]/
    $0

    # Create custom MCU folders in default project
    $0 stm32h7 stm32g4 stm32u5

    # Create in custom project name
    $0 -p iiswc stm32h7

    # Create in custom directory and project
    $0 -d /path/to/experiments -p myproject stm32h7

    # Force creation (overwrite existing)
    $0 -f stm32h7

EOF
}

# Default values
BASE_DIR="./experiments"
PROJECT_NAME="ae"
FORCE=false
VERBOSE=false
MCU_NAMES=()

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -d|--directory)
            BASE_DIR="$2"
            shift 2
            ;;
        -p|--project)
            PROJECT_NAME="$2"
            shift 2
            ;;
        -f|--force)
            FORCE=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -*)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
        *)
            MCU_NAMES+=("$1")
            shift
            ;;
    esac
done

# Set default MCU names if none provided
if [ ${#MCU_NAMES[@]} -eq 0 ]; then
    MCU_NAMES=("m0+" "m33" "m4" "m7" "m55")
    print_status "No MCU names provided, using defaults: ${MCU_NAMES[*]}"
fi

# Function to create directory structure
create_experiment_structure() {
    local mcu_name="$1"
    local project_dir="$BASE_DIR/$PROJECT_NAME"
    local mcu_dir="$project_dir/$mcu_name"
    
    if [ "$VERBOSE" = true ]; then
        print_status "Creating structure for MCU: $mcu_name in project: $PROJECT_NAME"
    fi
    
    # Create project directory
    mkdir -p "$project_dir"
    
    # Create MCU directory
    if [ -d "$mcu_dir" ] && [ "$FORCE" = false ]; then
        print_warning "MCU directory '$mcu_dir' already exists. Use -f to force creation."
        return
    fi
    
    mkdir -p "$mcu_dir"
    
    # Create stage directories with algorithm subdirectories
    local stages=(
        "control:adaptive-controller,geometric-controller,lqr,mpc,tinympc"
        "ekf:robobee-ekf,robofly-ekf,seq-update,sync-update,truncated-update"
        "perception:bbof,fastbrief,iiof,lkof,orb,sift"
        "pose-est:3pt,5pt,8pt,dlt,homography,p3p,up2p"
        "example:benchmark-example"
    )
    
    for stage_info in "${stages[@]}"; do
        local stage_name="${stage_info%%:*}"
        local algorithms="${stage_info##*:}"
        local stage_dir="$mcu_dir/$stage_name"
        
        if [ "$VERBOSE" = true ]; then
            print_status "  Creating stage: $stage_name"
        fi
        
        mkdir -p "$stage_dir"
        
        # Create algorithm subdirectories
        IFS=',' read -ra ALG_ARRAY <<< "$algorithms"
        for algorithm in "${ALG_ARRAY[@]}"; do
            local alg_dir="$stage_dir/$algorithm"
            mkdir -p "$alg_dir"
            
            if [ "$VERBOSE" = true ]; then
                print_status "    Created: $algorithm"
            fi
        done
    done
    
    print_success "Created experiment structure for MCU: $mcu_name"
}

# Main execution
main() {
    print_status "Setting up EntoBench experiment structure"
    print_status "Base directory: $BASE_DIR"
    print_status "Project name: $PROJECT_NAME"
    print_status "MCU names: ${MCU_NAMES[*]}"
    print_status "Force mode: $FORCE"
    
    # Create base experiments directory
    if [ ! -d "$BASE_DIR" ]; then
        print_status "Creating base experiments directory: $BASE_DIR"
        mkdir -p "$BASE_DIR"
    fi
    
    # Create structure for each MCU
    for mcu_name in "${MCU_NAMES[@]}"; do
        create_experiment_structure "$mcu_name"
    done
    
    print_success "Experiment structure setup complete!"
    print_status ""
    print_status "Created structure:"
    print_status "  Base: $BASE_DIR"
    print_status "  └── $PROJECT_NAME/"
    for mcu_name in "${MCU_NAMES[@]}"; do
        print_status "      ├── $mcu_name/"
        print_status "      │   ├── control/ (5 algorithms)"
        print_status "      │   ├── ekf/ (5 algorithms)"
        print_status "      │   ├── perception/ (6 algorithms)"
        print_status "      │   ├── pose-est/ (7 algorithms)"
        print_status "      │   └── example/ (1 algorithm)"
    done
    print_status ""
    print_status "Each algorithm folder is ready for experiment data."
    print_status "Use this structure to organize your benchmark results by project, MCU, and algorithm type."
}

# Run main function
main "$@" 