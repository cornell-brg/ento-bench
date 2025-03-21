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

# Run the Python script with the provided arguments
python3 analyze_multiple_experiments.py "$PARENT_DIR" "$WINDOW_SIZE" "$RISING_THRESHOLD" "$FALLING_THRESHOLD" "$PLOT_DATA"