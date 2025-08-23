#!/bin/bash

# =============================================================================
# EntoBench Clean and Rebuild Script
# =============================================================================
# This script cleans CMake cache/files and rebuilds all working MCU build directories
# Usage: ./clean_and_rebuild.sh [options]
# Options:
#   -h, --help     Show this help message
#   -v, --verbose  Show CMake output (default: quiet)
#   -f, --force    Force rebuild even if build directory doesn't exist
#   -c, --clean    Clean build directories after rebuilding (for testing)
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored status messages
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

# Function to show help
show_help() {
    cat << EOF
EntoBench Clean and Rebuild Script

Usage: $0 [options]

Options:
    -h, --help     Show this help message
    -v, --verbose  Show CMake output (default: quiet)
    -f, --force    Force rebuild even if build directory doesn't exist
    -c, --clean    Clean build directories after rebuilding (for testing)

This script will:
1. Clean CMake cache and files from all working build directories
2. Rerun CMake for each MCU to regenerate build files
3. Suppress output by default (use -v for verbose)

Example:
    $0              # Clean and rebuild all working builds (quiet)
    $0 -v           # Clean and rebuild with CMake output visible
    $0 -f           # Force rebuild even if directories don't exist
EOF
}

# Parse command line arguments
VERBOSE=false
FORCE=false
CLEAN_AFTER=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -f|--force)
            FORCE=true
            shift
            ;;
        -c|--clean)
            CLEAN_AFTER=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# MCU configurations (same as setup_build_directories.sh)
MCU_NAMES=("g474" "h7a3" "u575")
TOOLCHAIN_FILES=("stm32-g474re.cmake" "stm32-h7a3zi.cmake" "stm32-u575ziq.cmake")
OPENOCD_CFGS=("semihosting_stm32g4.cfg" "semihosting_stm32h7.cfg" "semihosting_stm32u5.cfg")

# Base directory
BASE_DIR="./build"

print_status "EntoBench Clean and Rebuild Script"
print_status "Base directory: $BASE_DIR"
print_status "Verbose mode: $VERBOSE"
print_status "Force mode: $FORCE"
print_status "Clean after rebuild: $CLEAN_AFTER"
print_status ""

# Function to clean and rebuild a specific MCU
clean_and_rebuild_mcu() {
    local mcu_name="$1"
    local toolchain_file="$2"
    local openocd_cfg="$3"
    local build_dir="$BASE_DIR/build-$mcu_name"
    
    print_status "Processing $mcu_name..."
    
    # Check if build directory exists
    if [ ! -d "$build_dir" ]; then
        if [ "$FORCE" = true ]; then
            print_warning "Build directory '$build_dir' doesn't exist, but force mode is enabled"
            mkdir -p "$build_dir"
        else
            print_warning "Build directory '$build_dir' doesn't exist, skipping (use -f to force)"
            return 1
        fi
    fi
    
    # Clean CMake cache and files
    print_status "  Cleaning CMake cache and files for $mcu_name..."
    cd "$build_dir"
    
    # Remove CMake cache and files
    rm -rf CMakeCache.txt CMakeFiles/ cmake_install.cmake Makefile
    
    # Return to original directory
    cd - > /dev/null
    
    print_status "  Running CMake for $mcu_name..."
    
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
        return 0
    else
        print_error "CMake failed for $mcu_name"
        return 1
    fi
}

# Function to clean build directory after rebuild
clean_build_directory() {
    local mcu_name="$1"
    local build_dir="$BASE_DIR/build-$mcu_name"
    
    if [ "$CLEAN_AFTER" = true ]; then
        print_status "  Cleaning build directory for $mcu_name (testing mode)..."
        rm -rf "$build_dir"
        print_success "Cleaned build directory for $mcu_name"
    fi
}

# Main execution
main() {
    # Ensure base directory exists
    if [ ! -d "$BASE_DIR" ]; then
        print_error "Base directory '$BASE_DIR' doesn't exist!"
        print_error "Run setup_build_directories.sh first to create build directories."
        exit 1
    fi
    
    # Track success/failure
    local successful_rebuilds=()
    local failed_rebuilds=()
    
    # Process each MCU
    for i in "${!MCU_NAMES[@]}"; do
        local mcu_name="${MCU_NAMES[$i]}"
        local toolchain_file="${TOOLCHAIN_FILES[$i]}"
        local openocd_cfg="${OPENOCD_CFGS[$i]}"
        
        if clean_and_rebuild_mcu "$mcu_name" "$toolchain_file" "$openocd_cfg"; then
            successful_rebuilds+=("$mcu_name")
            clean_build_directory "$mcu_name"
        else
            failed_rebuilds+=("$mcu_name")
        fi
        
        echo ""  # Add spacing between MCUs
    done
    
    # Print summary
    print_status ""
    print_status "Clean and rebuild summary:"
    if [ ${#successful_rebuilds[@]} -gt 0 ]; then
        print_success "Successful rebuilds (${#successful_rebuilds[@]}):"
        for mcu in "${successful_rebuilds[@]}"; do
            print_status "  ├── build-$mcu/ ✓"
        done
    fi
    
    if [ ${#failed_rebuilds[@]} -gt 0 ]; then
        print_warning "Failed rebuilds (${#failed_rebuilds[@]}):"
        for mcu in "${failed_rebuilds[@]}"; do
            print_status "  ├── build-$mcu/ ✗"
        done
        print_warning "Some rebuilds failed. Check the output above for errors."
    fi
    
    if [ "$CLEAN_AFTER" = true ]; then
        print_status ""
        print_status "Note: Build directories were cleaned after rebuild (testing mode)"
    fi
}

# Run main function
main "$@" 