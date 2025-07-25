import os
import pandas as pd
import yaml
import matplotlib.pyplot as plt
import numpy as np

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
    fig, axs = plt.subplots(figsize=(10, 12))
    
    def sort_horizons(horizons):
        try:
            # Try to convert horizons to numeric for sorting
            return sorted(horizons, key=lambda h: float(h))
        except ValueError:
            # If any horizon can't be converted to a float, fallback to sorting by string
            return sorted(horizons)
        
    # Sort horizons for SI_1 and VC methods
    
        
    # We want to plot Baseline and Velocity Change case
    
    colors_baseline = ["#0000ff", "#4169e1", "#87ceeb"]
    colors_vc = ["#006400", "#008000", "#00ff00"]
    
    bar_width = 0.8
    bar_offset = bar_width/2
    index = 0
    # x_ticks = [20, 30, 40, 50, 60, 70, 80]
    x_ticks = np.arange(20, 81, 10)
    
    bar_width = 3                    # Width of each bar
    bar_offset = bar_width / 2 + 0.1
    
    # ---------------------------- SI1 ------------------------------------------------
    derivs_times = []
    bp_times = []
    fp_times = []
    bp_and_derivs_times = []
    
    for horizon in sorted(grouped_data['SI_1']):
        
        derivs_times.append(grouped_data['SI_1'][horizon][0]['Average time derivs (ms)'].mean())
        bp_times.append(grouped_data['SI_1'][horizon][0]['Average time BP (ms)'].mean())
        fp_times.append(grouped_data['SI_1'][horizon][0]['Average time FP (ms)'].mean())
        bp_and_derivs_times.append(grouped_data['SI_1'][horizon][0]['Average time BP (ms)'].mean() + 
                               grouped_data['SI_1'][horizon][0]['Average time derivs (ms)'].mean())
        
    
    axs.bar(x_ticks - bar_offset, derivs_times, width=bar_width, color=colors_baseline[0], label='Derivatives - Baseline')
    axs.bar(x_ticks - bar_offset, bp_times, width=bar_width, color=colors_baseline[1], bottom=derivs_times, label='BP - Baseline')
    axs.bar(x_ticks - bar_offset, fp_times, width=bar_width, color=colors_baseline[2], bottom=bp_and_derivs_times, label='FP - Baseline')
    
    
    # ---------------------------------- Velocity change --------------------------------------
    derivs_times = []
    bp_times = []
    fp_times = []
    bp_and_derivs_times = []
    
    for horizon in sorted(grouped_data['*VC_2_20']):
        
        derivs_times.append(grouped_data['*VC_2_20'][horizon][0]['Average time derivs (ms)'].mean())
        bp_times.append(grouped_data['*VC_2_20'][horizon][0]['Average time BP (ms)'].mean())
        fp_times.append(grouped_data['*VC_2_20'][horizon][0]['Average time FP (ms)'].mean())
        bp_and_derivs_times.append(grouped_data['*VC_2_20'][horizon][0]['Average time BP (ms)'].mean() + 
                               grouped_data['*VC_2_20'][horizon][0]['Average time derivs (ms)'].mean())
        
    
    axs.bar(x_ticks + bar_offset, derivs_times, width=bar_width, color=colors_vc[0], label='Derivatives - Velocity Change')
    axs.bar(x_ticks + bar_offset, bp_times, width=bar_width, color=colors_vc[1], bottom=derivs_times, label='BP - Velocity Change')
    axs.bar(x_ticks + bar_offset, fp_times, width=bar_width, color=colors_vc[2], bottom=bp_and_derivs_times, label='FP - Velocity Change')
    
    axs.set_ylabel("Iteration time (ms)")
    axs.set_xlabel("Horizon (time-steps)")
    axs.legend()
        

    
    plt.show()


if __name__ == "__main__":
    # Directory containing subdirectories
    data_dir = "../iLQR/walker"  # Change this to the path of your directory
    y_column = "Final cost"  # Change this to the column name from CSV you want on the Y-axis

    grouped_data = group_data_by_keypoints_and_horizon(data_dir, "walker", "synchronus")
    plot_grouped_data(grouped_data)
