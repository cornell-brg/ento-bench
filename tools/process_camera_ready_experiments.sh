#!/bin/bash

# Ensure a path argument is provided
if [ $# -ne 1 ]; then
  echo "Usage: $0 <base_path>"
  exit 1
fi

# Get the base path from the command line argument
base_path="$1"

# List of datasets (previously called directories)
# datasets=("dlt-osj-optim-2Nx3")
datasets=("energy-ubench")

# Base command
base_command="python sync_current_logic_traces.py --traverse_subdirs True --field_order=1"

# Iterate over each dataset and run the command
for dataset in "${datasets[@]}"; do
    # Construct the full command with the current dataset path
    dataset_path="$base_path/$dataset"
    echo "Processing dataset: $dataset"

    full_command="$base_command --directory $dataset_path"

    # Print and run the command
    echo "Running: $full_command"
    $full_command
done
