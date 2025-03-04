#!/bin/bash

# Ensure a base directory is provided
if [ $# -ne 1 ]; then
  echo "Usage: $0 <base_directory>"
  exit 1
fi

# Get the base directory from the command line argument
base_dir="$1"

# Check if the directory exists
if [ ! -d "$base_dir" ]; then
  echo "Error: Directory '$base_dir' does not exist."
  exit 1
fi

echo "Scanning '$base_dir' for cleanup..."

# Find all CSV files, but exclude:
# - CSVs inside 'orig-meas' folders
# - CSVs that match the experiment folder name
csv_files=()
while IFS= read -r file; do
  parent_dir=$(basename "$(dirname "$file")")
  grandparent_dir=$(basename "$(dirname "$(dirname "$file")")")
  filename=$(basename "$file" .csv)

  # Skip if the file is inside an "orig-meas" folder
  if [[ "$parent_dir" == "orig-meas" || "$grandparent_dir" == "orig-meas" ]]; then
    continue
  fi

  # Only delete CSV files that do NOT match their experiment folder name
  if [[ "$filename" != "$parent_dir" ]]; then
    csv_files+=("$file")
  fi
done < <(find "$base_dir" -type f -name "*.csv")

if [ ${#csv_files[@]} -gt 0 ]; then
  echo "Found ${#csv_files[@]} CSV files to delete (excluding 'orig-meas' and experiment-named CSVs):"
  for file in "${csv_files[@]}"; do
    echo "  $file"
  done
  read -p "Delete all these CSV files? (y/n): " confirm_csv
  if [[ $confirm_csv == "y" ]]; then
    for file in "${csv_files[@]}"; do
      rm "$file"
    done
    echo "Deleted all non-experiment-named CSV files."
  else
    echo "Skipping CSV deletion."
  fi
else
  echo "No eligible CSV files found for deletion."
fi

# Find and delete folders that match their parent directory name, except inside "orig-meas"
matching_dirs=()
while IFS= read -r dir; do
  parent_dir=$(basename "$(dirname "$dir")")
  current_dir=$(basename "$dir")  
  grandparent_dir=$(basename "$(dirname "$(dirname "$dir")")")

  # Skip deletion if inside "orig-meas"
  if [[ "$parent_dir" == "orig-meas" || "$grandparent_dir" == "orig-meas" ]]; then
    continue
  fi

  # Check if the folder name matches its parent
  if [[ "$parent_dir" == "$current_dir" ]]; then
    matching_dirs+=("$dir")
  fi
done < <(find "$base_dir" -type d)

if [ ${#matching_dirs[@]} -gt 0 ]; then
  echo "Found ${#matching_dirs[@]} folders where the name matches the parent (excluding 'orig-meas'):"
  for dir in "${matching_dirs[@]}"; do
    echo "  $dir"
  done
  read -p "Delete all these folders? (y/n): " confirm_dirs
  if [[ $confirm_dirs == "y" ]]; then
    for dir in "${matching_dirs[@]}"; do
      rm -rf "$dir"
    done
    echo "Deleted all matching folders."
  else
    echo "Skipping folder deletion."
  fi
else
  echo "No matching folders found."
fi

echo "Cleanup complete."
