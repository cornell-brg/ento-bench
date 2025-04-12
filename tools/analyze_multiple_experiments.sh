#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 5 ]; then
    echo "Usage: $0 <parent_dir> <window_size> <rising_threshold> <falling_threshold> <plot_data>"
    echo "Example: $0 /path/to/data 10 0.7 0.3 true"
    exit 1
fi

# Assign command-line arguments to variables
PARENT_DIR="$1"
WINDOW_SIZE="$2"
RISING_THRESHOLD="$3"
FALLING_THRESHOLD="$4"
PLOT_DATA="$5"

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
        python3 analyze_single_experiment.py "$PARENT_DIR" "$BASENAME" "$WINDOW_SIZE" "$RISING_THRESHOLD" "$FALLING_THRESHOLD" "$PLOT_DATA"
        echo "---------------------------------------------"
    fi
done
