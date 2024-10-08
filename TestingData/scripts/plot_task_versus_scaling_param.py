# Plotting script to plot how C.R scales with different parameters used for a specific key-point method.

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
    grouped_data = []
    vel_change_threshold = []

    # Loop through each subdirectory
    for subdir in os.listdir(data_dir):
        full_subdir_path = os.path.join(data_dir, subdir)

        # Ensure it's a directory and contains task_name and run_mode in the name
        if os.path.isdir(full_subdir_path) and task_name in subdir and run_mode in subdir:
            df, yaml_data = load_summary_files(full_subdir_path)

            if df is not None and yaml_data is not None:
                # Get keypoint_method and optimisation_horizon
                keypoint_method = yaml_data.get("keypoint_name", "Unknown")
                # optimisation_horizon = yaml_data.get("optimisation horizon", "Unknown")
                
                if "VC_" in keypoint_method:
                    grouped_data.append(df)
                    vel_change_threshold.append(yaml_data.get("velocity_change_thresholds", "Unknown")[0])

                # Group data by keypoint method and optimisation horizon
                # if keypoint_method not in grouped_data:
                #     grouped_data[keypoint_method] = df
                
                # if optimisation_horizon not in grouped_data[keypoint_method]:
                #     grouped_data[keypoint_method][optimisation_horizon] = []

                # grouped_data[keypoint_method][optimisation_horizon].append(df)

    return grouped_data, vel_change_threshold

def plot_grouped_data(grouped_data, vel_change_thresoholds):
    """
    Plot the grouped data, with optimisation horizon on x-axis
    """
    
    # Sot lsits in terms of ascending vel_change_thresholds
    sorted_pairs = sorted(zip(grouped_data, vel_change_thresoholds), key=lambda x: x[1])

    # Unpack the sorted pairs back into two lists
    sorted_list_of_dfs, sorted_second_list = zip(*sorted_pairs)

    # Convert them back to lists (since zip returns tuples)
    grouped_data = list(sorted_list_of_dfs)
    vel_change_thresoholds = list(sorted_second_list)
    
    # fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    fig, ax1 = plt.subplots(figsize=(10, 5))
    
    x_values = []
    y_values = []
    for i, df in enumerate(grouped_data):
        
        x_values.append(vel_change_thresoholds[i])
        y_values.append(1 - df['Cost reduction'].mean())
        
        # Sort the horizons

        # Loop over optimisation horizons
        # for horizon in sorted_horizons:
        #     dfs = horizons[horizon]
        #     # Collect horizon value and average y_column value across the dataframes for this horizon
        #     x_values.append(horizon)
        #     y_avg = pd.concat(dfs)["Cost reduction"].mean()
        #     y_values.append(y_avg)

        # # Plot the line for this keypoint method
        # axs[0].plot(x_values, y_values, label=keypoint_method, marker='o')
        
    # Plot the first line (y1) with the left y-axis
    ax1.plot(x_values, y_values, 'g-', label='Final cost', marker='o')  # 'g-' means green solid line
    ax1.set_xlabel('Velocity change thresholds (m/s)')
    # ax1.set_ylabel('Final Cost', color='g')
    ax1.tick_params(axis='y', labelcolor='g')
        
    # axs[0].plot(x_values, y_values, marker='o')
    # axs[0].set_ylabel("C.R")
    
    # Create a second y-axis on the right
    ax2 = ax1.twinx()
    
    
    # Plot percentage of derivatives computed
    x_values = []
    y_values = []
    for i, df in enumerate(grouped_data):
        
        x_values.append(vel_change_thresoholds[i])
        y_values.append(df['Average percent derivs'].mean())
        
    # Plot the second line (y2) with the right y-axis
    ax2.plot(x_values, y_values, 'b-', label='Percentage of derivatives', marker='o')  # 'b-' means blue solid line
    # ax2.set_ylabel('Percentage of derivatives', color='b')
    ax2.tick_params(axis='y', labelcolor='b')
    
    # Add legends
    fig.legend(loc='upper right', bbox_to_anchor=(0.5, 0.85))
    # Set title
    fig.suptitle("Optimisation performance of Acrobot versus velocity change threshold")
        
    # axs[1].plot(x_values, y_values, marker='o')
    # axs[1].set_ylabel("Average percentage of derivatives")
    
    # Plot number of optimisation iterations
    # x_values = []
    # y_values = []
    # for i, df in enumerate(grouped_data):
        
    #     x_values.append(vel_change_thresoholds[i])
    #     y_values.append(df['Number iterations'].mean())
        
    # axs[2].plot(x_values, y_values, marker='o')
    # axs[2].set_ylabel("Number of iterations")
    # axs[2].set_xlabel("Velcocity change threshold (m/s)")
    # axs[0].set_title("Acrobot optimisation performance verus velocity change threshold")
    
    plt.show()

if __name__ == "__main__":
    # Directory containing subdirectories
    data_dir = "../scaling_vel_change"  # Change this to the path of your directory
    y_column = "cost reduction"  # Change this to the column name from CSV you want on the Y-axis

    grouped_data, vel_change_thresholds = group_data_by_keypoints_and_horizon(data_dir, "acrobot", "openloop")
    plot_grouped_data(grouped_data, vel_change_thresholds)
