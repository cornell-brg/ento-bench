#!/opt/homebrew/bin/bash

# Copy Benchmark Results to Datasets
# Copies specific benchmark results (0.0 and 0.01 noise, float/double) to datasets directory

# set -e  # Exit on any error - commented out to continue on missing files

# Configuration - paths relative to ento-bench root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BENCHMARK_ABS_DIR="${SCRIPT_DIR}/benchmark_results/abs_pose/data"
BENCHMARK_REL_DIR="${SCRIPT_DIR}/benchmark_results/rel_pose/data"
DATASETS_ABS_DIR="${SCRIPT_DIR}/datasets/abs-pose"
DATASETS_REL_DIR="${SCRIPT_DIR}/datasets/rel-pose"

# Target noise levels
NOISE_LEVELS=("0.0" "0.01")
PRECISIONS=("float" "double")

echo "=== Copying Benchmark Datasets ==="
echo "From: ${BENCHMARK_ABS_DIR}"
echo "To: ${DATASETS_ABS_DIR}"
echo ""

# Create target directories if they don't exist
mkdir -p "${DATASETS_ABS_DIR}"
mkdir -p "${DATASETS_REL_DIR}"

# Function to copy abs-pose datasets
copy_abs_pose_datasets() {
    local copied_count=0
    
    echo "=== Copying Absolute Pose Datasets ==="
    
    for precision in "${PRECISIONS[@]}"; do
        for noise in "${NOISE_LEVELS[@]}"; do
            echo "Processing ${precision} precision, noise ${noise}..."
            
            # Determine the noise suffix for CSV files
            local noise_suffix
            if [[ "$noise" == "0.0" ]]; then
                noise_suffix="0.000"
            else
                noise_suffix="${noise}0"
            fi
            
            # P3P datasets
            local p3p_dir="${BENCHMARK_ABS_DIR}/p3p_${precision}_traditional_p1000_noise${noise}"
            if [[ -d "$p3p_dir" ]]; then
                local p3p_csv="${p3p_dir}/p3p_noise_${noise_suffix}.csv"
                if [[ -f "$p3p_csv" ]]; then
                    local target_name="p3p_${precision}_noise${noise}.csv"
                    cp "$p3p_csv" "${DATASETS_ABS_DIR}/${target_name}"
                    echo "  ✓ Copied: ${target_name}"
                    ((copied_count++))
                else
                    echo "  ✗ Missing: $(basename "$p3p_csv")"
                fi
            else
                echo "  ✗ Missing directory: $(basename "$p3p_dir")"
            fi
            
            # UP2P datasets
            local up2p_dir="${BENCHMARK_ABS_DIR}/up2p_${precision}_traditional_p1000_noise${noise}"
            if [[ -d "$up2p_dir" ]]; then
                local up2p_csv="${up2p_dir}/up2p_noise_${noise_suffix}.csv"
                if [[ -f "$up2p_csv" ]]; then
                    local target_name="up2p_${precision}_noise${noise}.csv"
                    cp "$up2p_csv" "${DATASETS_ABS_DIR}/${target_name}"
                    echo "  ✓ Copied: ${target_name}"
                    ((copied_count++))
                else
                    echo "  ✗ Missing: $(basename "$up2p_csv")"
                fi
            else
                echo "  ✗ Missing directory: $(basename "$up2p_dir")"
            fi
            
            # DLT datasets - multiple point configurations
            local dlt_points=(6 8 16 32 64 128)
            for points in "${dlt_points[@]}"; do
                local dlt_dir="${BENCHMARK_ABS_DIR}/dlt_${precision}_traditional_p1000_noise${noise}_dlt${points}"
                if [[ -d "$dlt_dir" ]]; then
                    local dlt_csv="${dlt_dir}/dlt_noise_${noise_suffix}.csv"
                    if [[ -f "$dlt_csv" ]]; then
                        local target_name="dlt${points}_${precision}_noise${noise}.csv"
                        cp "$dlt_csv" "${DATASETS_ABS_DIR}/${target_name}"
                        echo "  ✓ Copied: ${target_name}"
                        ((copied_count++))
                    else
                        echo "  ✗ Missing: $(basename "$dlt_csv") in $(basename "$dlt_dir")"
                    fi
                else
                    echo "  ✗ Missing directory: $(basename "$dlt_dir")"
                fi
            done
        done
    done
    
    echo "Absolute pose datasets copied: ${copied_count}"
    echo ""
}

# Function to copy rel-pose datasets 
copy_rel_pose_datasets() {
    local copied_count=0
    
    echo "=== Copying Relative Pose Datasets ==="
    
    # Check if rel-pose benchmark results exist
    if [[ ! -d "$BENCHMARK_REL_DIR" ]]; then
        echo "⚠ Relative pose benchmark results directory not found: $BENCHMARK_REL_DIR"
        echo "Run relative pose benchmarks first to generate datasets."
        return 0
    fi
    
    for precision in "${PRECISIONS[@]}"; do
        for noise in "${NOISE_LEVELS[@]}"; do
            echo "Processing ${precision} precision, noise ${noise}..."
            
            # 5pt datasets
            local fivept_dir="${BENCHMARK_REL_DIR}/5pt_${precision}_realistic_p1000_noise${noise}"
            if [[ -d "$fivept_dir" ]]; then
                # Look for CSV files in the directory
                local csv_files=($(find "$fivept_dir" -name "*.csv" -type f))
                if [[ ${#csv_files[@]} -gt 0 ]]; then
                    local target_name="5pt_${precision}_noise${noise}.csv"
                    cp "${csv_files[0]}" "${DATASETS_REL_DIR}/${target_name}"
                    echo "  ✓ Copied: ${target_name}"
                    ((copied_count++))
                else
                    echo "  ✗ No CSV files in: $(basename "$fivept_dir")"
                fi
            else
                echo "  ✗ Missing directory: 5pt_${precision}_realistic_p1000_noise${noise}"
            fi
            
            # 8pt datasets - multiple point configurations
            local eightpt_points=(8 16 32 64 128)
            for points in "${eightpt_points[@]}"; do
                local eightpt_dir="${BENCHMARK_REL_DIR}/8pt_${precision}_realistic_p1000_noise${noise}_N${points}"
                if [[ -d "$eightpt_dir" ]]; then
                    local csv_files=($(find "$eightpt_dir" -name "*.csv" -type f))
                    if [[ ${#csv_files[@]} -gt 0 ]]; then
                        local target_name="8pt${points}_${precision}_noise${noise}.csv"
                        cp "${csv_files[0]}" "${DATASETS_REL_DIR}/${target_name}"
                        echo "  ✓ Copied: ${target_name}"
                        ((copied_count++))
                    else
                        echo "  ✗ No CSV files in: $(basename "$eightpt_dir")"
                    fi
                else
                    echo "  ✗ Missing directory: 8pt_${precision}_realistic_p1000_noise${noise}_N${points}"
                fi
            done
            
            # Upright 3pt datasets
            local upright3pt_dir="${BENCHMARK_REL_DIR}/upright_3pt_${precision}_realistic_p1000_noise${noise}"
            if [[ -d "$upright3pt_dir" ]]; then
                local csv_files=($(find "$upright3pt_dir" -name "*.csv" -type f))
                if [[ ${#csv_files[@]} -gt 0 ]]; then
                    local target_name="upright_3pt_${precision}_noise${noise}.csv"
                    cp "${csv_files[0]}" "${DATASETS_REL_DIR}/${target_name}"
                    echo "  ✓ Copied: ${target_name}"
                    ((copied_count++))
                else
                    echo "  ✗ No CSV files in: $(basename "$upright3pt_dir")"
                fi
            else
                echo "  ✗ Missing directory: upright_3pt_${precision}_realistic_p1000_noise${noise}"
            fi
            
            # Upright planar 3pt datasets - multiple point configurations
            local upright_planar3pt_points=(3 8 16 32)
            for points in "${upright_planar3pt_points[@]}"; do
                local upright_planar3pt_dir="${BENCHMARK_REL_DIR}/upright_planar_3pt_${precision}_realistic_p1000_noise${noise}_N${points}"
                if [[ -d "$upright_planar3pt_dir" ]]; then
                    local csv_files=($(find "$upright_planar3pt_dir" -name "*.csv" -type f))
                    if [[ ${#csv_files[@]} -gt 0 ]]; then
                        local target_name="upright_planar_3pt${points}_${precision}_noise${noise}.csv"
                        cp "${csv_files[0]}" "${DATASETS_REL_DIR}/${target_name}"
                        echo "  ✓ Copied: ${target_name}"
                        ((copied_count++))
                    else
                        echo "  ✗ No CSV files in: $(basename "$upright_planar3pt_dir")"
                    fi
                else
                    echo "  ✗ Missing directory: upright_planar_3pt_${precision}_realistic_p1000_noise${noise}_N${points}"
                fi
            done
            
            # Upright planar 2pt datasets
            local upright_planar2pt_dir="${BENCHMARK_REL_DIR}/upright_planar_2pt_${precision}_realistic_p1000_noise${noise}"
            if [[ -d "$upright_planar2pt_dir" ]]; then
                local csv_files=($(find "$upright_planar2pt_dir" -name "*.csv" -type f))
                if [[ ${#csv_files[@]} -gt 0 ]]; then
                    local target_name="upright_planar_2pt_${precision}_noise${noise}.csv"
                    cp "${csv_files[0]}" "${DATASETS_REL_DIR}/${target_name}"
                    echo "  ✓ Copied: ${target_name}"
                    ((copied_count++))
                else
                    echo "  ✗ No CSV files in: $(basename "$upright_planar2pt_dir")"
                fi
            else
                echo "  ✗ Missing directory: upright_planar_2pt_${precision}_realistic_p1000_noise${noise}"
            fi
        done
    done
    
    echo "Relative pose datasets copied: ${copied_count}"
    echo ""
}

# Execute the copy operations
copy_abs_pose_datasets

NOISE_LEVELS=("0.0" "0.5" "1.0" "2.5")
copy_rel_pose_datasets

echo "=== Copy Complete ==="
echo "Datasets available in:"
echo "  Absolute pose: ${DATASETS_ABS_DIR}"
echo "  Relative pose: ${DATASETS_REL_DIR}"
echo ""
echo "You can now use these datasets for consistent benchmarking and testing." 