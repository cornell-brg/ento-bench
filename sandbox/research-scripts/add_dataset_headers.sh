#!/opt/homebrew/bin/bash

# Add CSV Headers to Dataset Files
# Adds appropriate headers to abs-pose and rel-pose dataset files

set -e  # Exit on any error

# Configuration - paths relative to ento-bench root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASETS_ABS_DIR="${SCRIPT_DIR}/datasets/abs-pose"
DATASETS_REL_DIR="${SCRIPT_DIR}/datasets/rel-pose"

# Headers
ABS_POSE_HEADER="problem_type,num_points,pose_gt,scale_gt,focal_gt,x_point,X_point"
REL_POSE_HEADER="problem_type,pose_gt,scale_gt,focal_gt,x1,x2"

echo "=== Adding CSV Headers to Dataset Files ==="
echo "Abs-pose header: ${ABS_POSE_HEADER}"
echo "Rel-pose header: ${REL_POSE_HEADER}"
echo ""

# Function to add header to a file
add_header() {
    local file="$1"
    local header="$2"
    local temp_file="${file}.tmp"
    
    echo "Processing: $(basename "$file")"
    
    # Create temporary file with header + original content
    echo "$header" > "$temp_file"
    cat "$file" >> "$temp_file"
    
    # Replace original with updated file
    mv "$temp_file" "$file"
}

# Process absolute pose datasets
echo "=== Processing Absolute Pose Datasets ==="
if [[ -d "$DATASETS_ABS_DIR" ]]; then
    abs_count=0
    for file in "$DATASETS_ABS_DIR"/*.csv; do
        if [[ -f "$file" ]]; then
            add_header "$file" "$ABS_POSE_HEADER"
            abs_count=$((abs_count + 1))
        fi
    done
    echo "Updated $abs_count absolute pose dataset files"
else
    echo "⚠ Absolute pose datasets directory not found: $DATASETS_ABS_DIR"
fi

echo ""

# Process relative pose datasets  
echo "=== Processing Relative Pose Datasets ==="
if [[ -d "$DATASETS_REL_DIR" ]]; then
    rel_count=0
    for file in "$DATASETS_REL_DIR"/*.csv; do
        if [[ -f "$file" ]]; then
            add_header "$file" "$REL_POSE_HEADER"
            rel_count=$((rel_count + 1))
        fi
    done
    echo "Updated $rel_count relative pose dataset files"
else
    echo "⚠ Relative pose datasets directory not found: $DATASETS_REL_DIR"
fi

echo ""
echo "=== Header Addition Complete ==="
echo "All dataset files now have appropriate CSV headers." 