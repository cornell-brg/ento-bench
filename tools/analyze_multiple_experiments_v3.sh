#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <parent_dir>"
    echo "Example: $0 /path/to/data"
    exit 1
fi

# Assign command-line arguments to variables
PARENT_DIR="$1"

# Iterate over all subdirectories in the parent directory
for DATASET in "$PARENT_DIR"/*; do
    # Only process directories
    if [ -d "$DATASET" ]; then
        BASENAME=$(basename "$DATASET")

        # Skip special or filtered directories
        if [[ "$BASENAME" == *"meas" ]] || [[ "$BASENAME" == "plots" ]] || [[ "$BASENAME" == *"dcache" ]]; then
            echo "Skipping $BASENAME..."
            continue
        fi

        echo "Analyzing dataset: $BASENAME"
        python3 analyze_single_experiment_v3.py "$PARENT_DIR" "$BASENAME"
        echo "---------------------------------------------"
    fi
done
