import os
import pandas as pd
import yaml
import matplotlib.pyplot as plt

def load_summary_files(directory):
    """
    Load the summary CSV and YAML files from the given directory.
    """
    summary_csv_path = os.path.join(directory, "summary.csv")
    summary_yaml_path = os.path.join(directory, "summary.yaml")

    if not os.path.exists(summary_csv_path) or not os.path.exists(summary_yaml_path):
        return None, None

    # Load the CSV file using pandas
    df = pd.read_csv(summary_csv_path)

    # Load the YAML file using PyYAML
    with open(summary_yaml_path, "r") as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)

    return df, yaml_data

def group_data_by_keypoints_and_horizon(data_dir, task_name, run_mode):
    """
    Traverse through directories in `data_dir`, load data, and group by keypoint method and optimisation horizon.
    """
    grouped_data = {}

    # Loop through each subdirectory
    for subdir in os.listdir(data_dir):
        full_subdir_path = os.path.join(data_dir, subdir)

        # Ensure it's a directory and contains task_name and run_mode in the name
        if os.path.isdir(full_subdir_path) and task_name in subdir and run_mode in subdir:
            df, yaml_data = load_summary_files(full_subdir_path)

            if df is not None and yaml_data is not None:
                # Get keypoint_method and optimisation_horizon
                keypoint_method = yaml_data.get("keypoint_name", "Unknown")
                optimisation_horizon = yaml_data.get("optimisation horizon", "Unknown")

                # Group data by keypoint method and optimisation horizon
                if keypoint_method not in grouped_data:
                    grouped_data[keypoint_method] = {}
                
                if optimisation_horizon not in grouped_data[keypoint_method]:
                    grouped_data[keypoint_method][optimisation_horizon] = []

                grouped_data[keypoint_method][optimisation_horizon].append(df)

    return grouped_data

def plot_grouped_data(grouped_data):
    """
    Plot the grouped data, with optimisation horizon on x-axis
    """
    fig, axs = plt.subplots(2, 1, figsize=(10, 12), sharex=True)
    
    def sort_horizons(horizons):
        try:
            # Try to convert horizons to numeric for sorting
            return sorted(horizons, key=lambda h: float(h))
        except ValueError:
            # If any horizon can't be converted to a float, fallback to sorting by string
            return sorted(horizons)
        
    for keypoint_method, horizons in grouped_data.items():
        x_values = []
        y_values = []
        
        # Sort the horizons
        print(horizons)
        sorted_horizons = sort_horizons(horizons.keys())

        # Loop over optimisation horizons
        for horizon in sorted_horizons:
            dfs = horizons[horizon]
            # Collect horizon value and average y_column value across the dataframes for this horizon
            x_values.append(horizon)
            y_avg = pd.concat(dfs)["Final cost"].mean()
            y_values.append(y_avg)

        # Plot the line for this keypoint method
        axs[0].plot(x_values, y_values, label=keypoint_method, marker='o')
        
    axs[0].set_ylabel("Final cost")
    axs[0].set_title(f'Final cost vs Optimisation Horizon by Keypoint Method')
    axs[0].legend()
    # axs[0].grid(True)
    
    for keypoint_method, horizons in grouped_data.items():
        x_values = []
        y_values = []
        
        # Sort the horizons
        sorted_horizons = sort_horizons(horizons.keys())

        # Loop over optimisation horizons
        for horizon in sorted_horizons:
            dfs = horizons[horizon]
            # Collect horizon value and average y_column value across the dataframes for this horizon
            x_values.append(horizon)
            optimisation_time = pd.concat(dfs)["Average optimisation time (ms)"].mean()
            y_avg = 1000 / optimisation_time
            y_values.append(y_avg)

        # Plot the line for this keypoint method
        axs[1].plot(x_values, y_values, label=keypoint_method, marker='o')
        
    axs[1].set_ylabel("Control frequency (Hz)")
    axs[1].set_title(f'Control frequency (Hz) vs Optimisation Horizon by Keypoint Method')
    axs[1].legend()
    # axs[1].grid(True)
    
    # Bottom graph needs to plot control frequency
    control_frequency = 1 / optimisation_time
    
    plt.show()


if __name__ == "__main__":
    # Directory containing subdirectories
    data_dir = "../iLQR/walker"  # Change this to the path of your directory
    y_column = "Final cost"  # Change this to the column name from CSV you want on the Y-axis

    grouped_data = group_data_by_keypoints_and_horizon(data_dir, "walker", "synchronus")
    plot_grouped_data(grouped_data)
