import pandas as pd
from pathlib import Path
from argparse import ArgumentParser
import zipfile
import numpy as np

def separate_and_combine(path: Path) -> Path:
    dir_name = path.stem
    # specify the output file name
    output_file = path / f'{dir_name}_pm.csv'

    if output_file.exists(): 
        print('Output file already exists')
        return output_file

    subdirs = [subdir for subdir in path.iterdir() if subdir.is_dir()]
    pm_dir = None
    print(subdirs, path)
    for subdir in subdirs:
        print(subdir)
        if subdir.is_dir() and 'PM' in subdir.stem:
            pm_dir = subdir
            break
        
    if pm_dir is None:
        stpm_files = list(path.glob('*.stpm'))
        if len(stpm_files) == 0:
            raise FileNotFoundError(f'Did not find power monitor data in directory: {path}')
        else:
            if len(stpm_files) > 1:
                print('Found more than one stpm file. ' + \
                      'Unzipping the first one found: ' + \
                      f'{stpm_files[0]}')
            stpm_file = stpm_files[0]
            pm_dir = stpm_file.parent / stpm_file.stem
            with zipfile.ZipFile(stpm_file, 'r') as zip_ref:
                zip_ref.extractall(pm_dir)
            

    csv_files = list(pm_dir.glob('*.csv'))
    data_frames = []
    for file in csv_files:
        if 'summary' in file.stem: continue
        data = pd.read_csv(pm_dir / file, sep=';', header=None, names=['time', 'current'])
        data_frames.append(data)

    # concatenate the list of dataframes into a single dataframe
    combined_data = pd.concat(data_frames, ignore_index=True)
    #t0 = combined_data['time'][0]
    #combined_data['time'] -= t0
    print(combined_data.head())


    # write the combined dataframe to a new csv file

    combined_data.to_csv(output_file, index=False)
    print(combined_data.head())
    print(f"Completed combining power monitor data for file: {path}.")
    return output_file

def combine_files(file1: Path, file2: Path) -> Path:
    print(f'Combining {file1.stem} and {file2.stem}.')
    #print(file1.parent)
    df_pm = pd.read_csv(file1)
    #df_pm['time']
    df_sal = pd.read_csv(file2)

    df_pm.sort_values(by="time", inplace=True)
    print(df_pm.iloc[218*100:int(218.5*100) , :])
    #merged_df = pd.merge_asof(df1, df2, on="time", direction="nearest", tolerance=0.01)
    merged_df = df_pm.merge(df_sal, how='outer', on='time')
    print(np.sum(merged_df['latency'] == 1))
    print(len(merged_df['latency']))
    merged_df.sort_values(by='time', inplace=True)
    num_spikes = np.sum(merged_df['latency'] == 1)
    print(f"Merged df num latency spikes: {num_spikes}")

    #series_latency = pd.Series(df_sal['latency'].values, index=df_sal['time'].values)
    #series_current = pd.Series(df_pm['current'].values, index=df_pm['time'].values)
    #print(series_current)
    #merged_df = pd.DataFrame({'latency': series_latency, 'current': series_current})
    #merged_df['time'] = merged_df.index.values.copy()
    # Sort the merged DataFrame by time
    #merged_df = merged_df.sort_values(by="time")
    #print(df1.head())
    #print(df2.head())

    print(f'Finished merging {file1.stem} and {file2.stem}')
    print(merged_df.head())
    # Save the combined DataFrame to a new CSV file
    output_file = file1.parent / f'{file1.parent.stem}_combined.csv'
    print(f'Saving merged df to {output_file}.')
    merged_df.to_csv(output_file, index=False)
    print(f'Nan counts: {merged_df.isna().sum()}')
    print(np.sum(merged_df['latency'] == 1))

    print("Combining files complete.")
    return output_file

def fill_gaps_with_zeros(df: pd.DataFrame, test_name: str) -> pd.DataFrame:
    new_rows = []

    prev_time = None
    current_group = []

    print(df.columns)
    for _, row in df.iterrows():
        time = row["time"]
        value1 = row["trigger"]
        value2 = row["latency"]

        if prev_time is None:
            prev_time = time
            current_group.append((time, value1, value2))
        else:
            time_diff = time - prev_time
            while time_diff >= 0.00001:  # Change increment to 0.00001
                new_time = round(prev_time + 0.00001, 5)  # Change increment to 0.00001
                new_rows.append((new_time, 0, 0))
                prev_time = new_time
                time_diff -= 0.00001  # Change increment to 0.00001
            current_group.append((time, value1, value2))
            prev_time = time

    new_df = pd.DataFrame(new_rows,
                          columns=["time",
                                   "trigger",
                                   "latency"])
    return new_df

def process_sal(input_file: Path, test_type: str = 'PIL', field_order: int = 0):
    df = pd.read_csv(input_file)
    print(df.head())
    

    test_name = input_file.parent.stem
    column_names = df.columns.values
    print(f'Original columns names: {column_names}')
    print(f'FIELD ORDER: {field_order}\n\n\n\n\n')
    if field_order == 0:
        if len(column_names) == 4:
            new_names = ['time', 'trigger_com', 'trigger', 'latency']
            new_column_names = { old_name: new_name for old_name, new_name in zip(column_names, new_names)}
            print(new_column_names)
            df.rename(columns=new_column_names, inplace=True)
            print(df.columns.values)
            df = df[['time', 'trigger', 'latency']]
        elif len(column_names) == 3:
            new_names = ['time', 'trigger', 'latency']
            new_column_names = { old_name: new_name for old_name, new_name in zip(column_names, new_names)}
            print(new_column_names)
            df.rename(columns=new_column_names, inplace=True)
            print(df.columns.values)
        else:
            raise AttributeError('Got unknown test type.')
    else: 
        if len(column_names) == 3:
            df = df.iloc[:, [0, 2, 1]]
            column_names = df.columns.values
            print(f'Reordered df!!! \n')
            print(df.head())
            new_names = ['time', 'trigger', 'latency']
            new_column_names = { old_name: new_name for old_name, new_name in zip(column_names, new_names)}
            print(new_column_names)
            df.rename(columns=new_column_names, inplace=True)
            print(df.columns.values)
        else:
            raise AttributeError('Got unknown test type.')

    print(f'{input_file} dataframe latency has length: {len(df["latency"])}')
    output_file = input_file.parent / f'{input_file.parent.stem}_processed.csv'

    # Find the index of the first occurrence of 1 in Trigger column
    first_occurrence_index = df[df["trigger"] == 1].index[0]
    print(first_occurrence_index)

    # Discard rows before the first occurrence of 1 in Trigger column
    # We only care about values after trigger (start of measurements)
    print(df.head())
    t0 = df['time'][first_occurrence_index] # 0.1 ms delay?
    df = df.iloc[first_occurrence_index:, :].reset_index()
    print(df)

    # Change all second value column to be 1
    indices = np.where(df['latency'] == 1)[0]
    print(len(indices))
    print(indices)
    tstarts = df['time'][indices]
    print(f'Latency tstarts: {tstarts}\n')
    indices += 1
    tends = df['time'][indices]
    print(f'Latency tends: {tends}\n')
    df.latency.values[indices] = 1

    #print("-----------------df with 1s---------------------")
    #print(df)
    # Reformat the Time column to start from 0 and increment by 0.00001
    #df["time"] = (df["time"] - df.iloc[0, 0]).apply(lambda x: round(x % 1, 5))  # Change increment to 0.00001

    # Fill gaps and consecutive ones
    #filled_df = fill_gaps_with_zeros(df, test_name)
    filled_df = df
    print("---------------filled_df---------------")
    print(filled_df.head())
    # Combine the original and modified data
    #combined_df = pd.concat([df, filled_df])

    # Sort the combined DataFrame by time
    filled_df.sort_values(by="time", inplace=True)
    print("----------------sorted df------------------------")
    print(filled_df.head())

    print(f'Time zero: {t0}')
    # 0.0005 was found adhoc looking at measured data
    filled_df["time"] = (filled_df["time"] - t0)
    print(f'Aligned df: {filled_df.head()}')
    filled_df["time"] = (filled_df["time"]) * 1000
    #filled_df["time"] = round(filled_df["time"],2)
    
    # Save the combined DataFrame to a new CSV file
    filled_df.to_csv(output_file, index=False)

    print("---------------final df-------------------------")
    # filtered_rows = combined_df[combined_df['B11(SVD Algorithm)'] == 1]
    print(filled_df.head())
    print(len(filled_df['time']))

    print("Processing complete.")
    return output_file

def main():
    parser = ArgumentParser()
    parser.add_argument('--directory', metavar='d', type=str)
    parser.add_argument('--traverse_subdirs', type=bool, default=False,
                        required=False)
    parser.add_argument('--field_order', type=int, default=0, required=False)
    args = parser.parse_args()

    args.directory = Path(args.directory).resolve()
    print(args.directory)
    if not args.directory.exists(): raise FileNotFoundError(f'Could not find directory: {args.directory}')
    pd.set_option('display.precision', 10)

    if args.traverse_subdirs:
        subdirs = [f for f in args.directory.iterdir() if f.is_dir()]
        for subdir in subdirs:
            if 'original-meas' in subdir.name: continue
            if 'orig-meas' in subdir.name: continue
            if 'plot' in str(subdir):
                continue
            print(f'Subdir: {subdir}')
            sal_files = list(subdir.glob(f'{subdir.name}.csv'))
            if len(sal_files) > 1:
                print("Found multiple logical analyzer traces. " + \
                      f"Taking the first one: {sal_files[0]}.")
            sal_file = sal_files[0]
            print(f"Processing subdir: {subdir}")
            filled_sal_file = process_sal(sal_file, field_order=args.field_order)
            combined_pm = separate_and_combine(subdir)
            merged_pm_sal = combine_files(combined_pm, filled_sal_file)
    else:
        print('Separating and combining')
        combined_pm = separate_and_combine(args.directory)
        sal_files = list(args.directory.glob(f'{args.directory.name}.csv'))
        if len(sal_files) > 1:
            print("Found multiple logical analyzer traces. " + \
                  f"Taking the first one: {sal_files[0]}.")
        sal_file = sal_files[0]
        filled_sal_file = process_sal(sal_file, field_order=args.field_order)
        merged_pm_sal = combine_files(combined_pm, filled_sal_file)


if __name__ == '__main__':
    main()
