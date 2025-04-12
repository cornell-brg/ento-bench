#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 6 ]; then
    echo "Usage: $0 <parent_dir> <dataset_name> <window_size> <rising_threshold> <falling_threshold> <plot_data>"
    echo "Example: $0 /path/to/data xor_full_6500x1 10 0.7 0.3 true"
    exit 1
fi

# Assign command-line arguments to variables
PARENT_DIR="$1"
DATASET_NAME="$2"
WINDOW_SIZE="$3"
RISING_THRESHOLD="$4"
FALLING_THRESHOLD="$5"
PLOT_DATA="$6"

# Run the Python script with the provided arguments
python3 analyze_single_experiment.py "$PARENT_DIR" "$DATASET_NAME" "$WINDOW_SIZE" "$RISING_THRESHOLD" "$FALLING_THRESHOLD" "$PLOT_DATA"
