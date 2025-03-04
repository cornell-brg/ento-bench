import pandas as pd
from pathlib import Path
import os

# Specify the directory containing CSV files

csv_directory = Path('./svd_ofast/svd_8x3/PM_8x3/').resolve()
print(csv_directory.exists())

# List CSV files in the directory
csv_files = [file for file in os.listdir(csv_directory) if file.endswith('.csv')]

# Initialize an empty list to store DataFrames
data_frames = []

# Loop through each CSV file, read it as a DataFrame, split the data, and append to the list
for file in csv_files:
    file_path = os.path.join(csv_directory, file)
    data = pd.read_csv(file_path, sep=';', header=none, names=['time', 'current'])
    data_frames.append(data)

# concatenate the list of dataframes into a single dataframe
combined_data = pd.concat(data_frames, ignore_index=true)

# specify the output file name
output_file = csv_directory / '..' / 'svd_8x3_pm.csv'

# write the combined dataframe to a new csv file

combined_data.to_csv(output_file, index=false)
print(combined_data)
print("processing complete.")
