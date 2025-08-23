#!/bin/bash

# =============================================================================
# EntoBench Build Directory Setup Script
# =============================================================================
# This script creates separate build directories for different STM32 MCUs
# and runs CMake with the appropriate toolchain files for each.
#
# Build directories created:
# - build-g474/  (STM32G474RE)
# - build-h7a3/  (STM32H7A3ZI)
# - build-u575/  (STM32U575ZI)
# - build-c092rc/ (STM32C092RC)
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
Usage: $0 [OPTIONS]

OPTIONS:
    -h, --help          Show this help message
    -f, --force         Force recreation of existing build directories
    -v, --verbose       Verbose CMake output (default: quiet)
    -c, --clean         Clean build directories before setup
    -d, --directory     Base directory for builds (default: ./build)

EXAMPLES:
    # Create all build directories with quiet CMake
    $0

    # Force recreation of existing directories
    $0 -f

    # Clean existing directories first
    $0 -c

    # Verbose CMake output
    $0 -v

    # Custom build directory location
    $0 -d /path/to/builds

EOF
}

# Default values
BASE_DIR="./build"
FORCE=false
VERBOSE=false
CLEAN=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -f|--force)
            FORCE=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -c|--clean)
            CLEAN=true
            shift
            ;;
        -d|--directory)
            BASE_DIR="$2"
            shift 2
            ;;
        -*)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
        *)
            print_error "Unexpected argument: $1"
            show_usage
            exit 1
            ;;
    esac
done

# MCU configurations (using parallel arrays for compatibility)
MCU_NAMES=("g474" "h7a3" "u575")
TOOLCHAIN_FILES=("stm32-g474re.cmake" "stm32-h7a3zi.cmake" "stm32-u575ziq.cmake")
OPENOCD_CFGS=("semihosting_stm32g4.cfg" "semihosting_stm32h7.cfg" "semihosting_stm32u5.cfg")

# Function to setup build directory for a specific MCU
setup_mcu_build() {
    local mcu_name="$1"
    local mcu_index="$2"
    local toolchain_file="${TOOLCHAIN_FILES[$mcu_index]}"
    local openocd_cfg="${OPENOCD_CFGS[$mcu_index]}"
    local build_dir="$BASE_DIR/build-$mcu_name"
    
    print_status "Setting up build directory for $mcu_name..."
    
    # Check if directory exists and handle force/clean options
    if [ -d "$build_dir" ]; then
        if [ "$FORCE" = true ] || [ "$CLEAN" = true ]; then
            print_status "Removing existing build directory: $build_dir"
            rm -rf "$build_dir"
        else
            print_warning "Build directory '$build_dir' already exists. Use -f to force recreation."
            return
        fi
    fi
    
    # Create build directory
    mkdir -p "$build_dir"
    cd "$build_dir"
    
    print_status "Running CMake for $mcu_name..."
    
    # Run CMake with appropriate flags
    if [ "$VERBOSE" = true ]; then
        # Verbose mode - show all output
        cmake \
            -DCMAKE_TOOLCHAIN_FILE="../../stm32-cmake/$toolchain_file" \
            -DFETCH_ST_SOURCES=True \
            -DOPENOCD_CFG="$openocd_cfg" \
            -DLATENCY_MEASUREMENT=1 \
            ../..
    else
        # Quiet mode - suppress output, only show errors
        cmake \
            -DCMAKE_TOOLCHAIN_FILE="../../stm32-cmake/$toolchain_file" \
            -DFETCH_ST_SOURCES=True \
            -DOPENOCD_CFG="$openocd_cfg" \
            -DLATENCY_MEASUREMENT=1 \
            ../.. > /dev/null 2>&1
    fi
    
    # Check if CMake succeeded
    if [ $? -eq 0 ]; then
        print_success "CMake completed successfully for $mcu_name"
    else
        print_error "CMake failed for $mcu_name"
        print_warning "Build directory '$build_dir' was created but CMake configuration failed."
        print_warning "You may need to fix toolchain issues or dependencies manually."
        # Don't exit, continue with other MCUs
        return 1
    fi
    
    # Return to original directory
    cd - > /dev/null
}

# Main execution
main() {
    print_status "Setting up EntoBench build directories"
    print_status "Base directory: $BASE_DIR"
    print_status "Force mode: $FORCE"
    print_status "Verbose mode: $VERBOSE"
    print_status "Clean mode: $CLEAN"
    
    # Create base build directory
    if [ ! -d "$BASE_DIR" ]; then
        print_status "Creating base build directory: $BASE_DIR"
        mkdir -p "$BASE_DIR"
    fi
    
    # Track success/failure
    local successful_builds=()
    local failed_builds=()
    
    # Setup each MCU build directory
    for i in "${!MCU_NAMES[@]}"; do
        local mcu_name="${MCU_NAMES[$i]}"
        if setup_mcu_build "$mcu_name" "$i"; then
            successful_builds+=("$mcu_name")
        else
            failed_builds+=("$mcu_name")
        fi
        echo ""  # Add spacing between MCUs
    done
    
    print_status ""
    print_status "Build setup summary:"
    if [ ${#successful_builds[@]} -gt 0 ]; then
        print_success "Successful builds (${#successful_builds[@]}):"
        for mcu in "${successful_builds[@]}"; do
            print_status "  ├── build-$mcu/ ✓"
        done
    fi
    
    if [ ${#failed_builds[@]} -gt 0 ]; then
        print_warning "Failed builds (${#failed_builds[@]}):"
        for mcu in "${failed_builds[@]}"; do
            print_status "  ├── build-$mcu/ ✗"
        done
        print_warning "Some builds failed. Check the output above for errors."
        print_warning "You may need to fix toolchain issues or dependencies manually."
    fi
    
    print_status ""

}

# Run main function
main "$@" 