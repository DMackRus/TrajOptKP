import matplotlib.pyplot as plt
from sys import argv
import os
import pandas as pd
import yaml
import numpy as np

iterations = "4_10"
run_mode = "openloop"

def main(task_name):
    
    names_iLQR, dataframes_iLQR, yamlfiles_iLQR = load_iLQR_data(task_name)
    
    names_SCVX, dataframes_SCVX, yamlfiles_SCVX = load_SCVX_data(task_name)
    
    
    # Sort data frames - just keep the desired methods and order them
    methods = ["SI_1", "SI_5", "contact_change", 
                "contact_change_dyn"]

    # Sort iLQR
    
    # Keep only entries where the name is in `methods`
    filtered = [(i, name) for i, name in enumerate(names_iLQR) if name in methods]

    # Sort them based on the method order
    sorted_indices = sorted(filtered, key=lambda x: methods.index(x[1]))
    sorted_indices = [i for i, _ in sorted_indices]

    # Apply ordering
    names_iLQR = [names_iLQR[i] for i in sorted_indices]
    dataframes_iLQR = [dataframes_iLQR[i] for i in sorted_indices]
    
    # Sort SCVX
    filtered = [(i, name) for i, name in enumerate(names_SCVX) if name in methods]

    # Sort them based on the method order
    sorted_indices = sorted(filtered, key=lambda x: methods.index(x[1]))
    sorted_indices = [i for i, _ in sorted_indices]

    # Apply ordering
    names_SCVX = [names_SCVX[i] for i in sorted_indices]
    dataframes_SCVX = [dataframes_SCVX[i] for i in sorted_indices]
    
    # print(dataframes_iLQR)
    
    plot_timing_breakdown_data(task_name, names_iLQR, dataframes_iLQR, dataframes_SCVX)
    
    # print(dataframes_SCVX)
    # plot_timing_breakdown_data_SCVX(names_SCVX, dataframes_SCVX)
    
def plot_timing_breakdown_data(task_name, method_names, dataframes_iLQR, dataframes_SCVX):
    num_SCVX_methods = len(dataframes_SCVX)
    
    # Define the timing categories (excluding the total "optimisation time" for plotting)
    categories_iLQR = ["Optimisation time (ms)", "Total time keypoints (ms)", 
                  "Total time FD (ms)", "Total time interpolation (ms)", 
                  "Total time cost derivs (ms)", "Total time BP (ms)",
                  "Total time FP (ms)"]
    categories_SCVX = ["Optimisation time (ms)", "Total time keypoints (ms)", 
                  "Total time FD (ms)", "Total time interpolation (ms)", 
                  "Total time cost derivs (ms)", "Total time QP (ms)",
                  "Total time FP (ms)"]
    
    plot_categories_iLQR = categories_iLQR[1:]  # Skip the first one for stacking
    plot_categories_SCVX = categories_SCVX[1:]  # Skip the first one for stacking
    
    color_map_SCVX = {
        "Total time keypoints (ms)": "#1f77b4",     # Blue
        "Total time FD (ms)": "#ff7f0e",            # Orange
        "Total time interpolation (ms)": "#2ca02c", # Green
        "Total time cost derivs (ms)": "#9467bd",   # Purple
        "Total time QP (ms)": "#d62728",            # Red
        "Total time FP (ms)": "#8c564b"             # Brown
    }
    
    color_map_iLQR = {
        "Total time keypoints (ms)": "#1f77b4",     # Blue
        "Total time FD (ms)": "#ff7f0e",            # Orange
        "Total time interpolation (ms)": "#2ca02c", # Green
        "Total time cost derivs (ms)": "#9467bd",   # Purple
        "Total time BP (ms)": "#d62728",            # Red
        "Total time FP (ms)": "#8c564b"             # Brown
    }
    
    fig, axes = plt.subplots(1, 2, figsize=(8, 6), sharey=True)
    
    # ----------------------- iLQR results on first axes ------------------------
    num_methods = len(dataframes_iLQR)
    bar_data = []

    # Compute mean values for each category per method
    for df in dataframes_iLQR:
        means = df[plot_categories_iLQR].mean()
        bar_data.append(means.values)
    
    bar_data = np.array(bar_data)  # Shape: (num_methods, num_plot_categories)
    
    bar_width = 0.5
    indices = np.arange(num_methods)
    
    bottoms = np.zeros(num_methods)
    for i, category in enumerate(plot_categories_iLQR):
        values = bar_data[:, i]
        axes[0].bar(indices, values, bar_width, bottom=bottoms,
               label=category, color=color_map_iLQR[category])
        # ax.bar(indices, values, bar_width, bottom=bottoms, label=category, color=colors[i])
        bottoms += values

    # axes[0].set_ylabel('Average Time per Run (ms)')
    axes[0].set_title('iLQR')
    axes[0].set_xticks(indices)
    # axes[0].xticks(rotation="45")
    # plt.xticks(rotation=90)
    axes[0].set_xticklabels(method_names, rotation=25, ha='right')
    # ax.legend(loc='upper right', bbox_to_anchor=(1.35, 1.0))
    axes[0].legend()
    
    # ------------------------ SCVX results on second axes ----------------------------
    num_methods = len(dataframes_SCVX)
    bar_data = []

    # Compute mean values for each category per method
    for df in dataframes_SCVX:
        means = df[plot_categories_SCVX].mean()
        bar_data.append(means.values)
    
    bar_data = np.array(bar_data)  # Shape: (num_methods, num_plot_categories)
    
    bar_width = 0.5
    indices = np.arange(num_methods)
    
    bottoms = np.zeros(num_methods)
    for i, category in enumerate(plot_categories_SCVX):
        values = bar_data[:, i]
        axes[1].bar(indices, values, bar_width, bottom=bottoms,
               label=category, color=color_map_SCVX[category])
        # ax.bar(indices, values, bar_width, bottom=bottoms, label=category, color=colors[i])
        bottoms += values

    # axes[1].set_ylabel('Average Time per Run (ms)')
    axes[1].set_title('SCVX')
    axes[1].set_xticks(indices)
    axes[1].set_xticklabels(method_names, rotation=25, ha='right')
    # ax.legend(loc='upper right', bbox_to_anchor=(1.35, 1.0))
    axes[1].legend()

    # fig.shared_yaxes = True
    # fig.tight_layout()
    fig.tight_layout(rect=[0.03, 0.03, 1, 0.92])
    fig.suptitle(f'Optimization Timing Breakdown for Task: {task_name}', fontsize=16)
    fig.supylabel('Total Time per Run (ms)')
    fig.supxlabel('Methods')
    # fig.suptitle(f'Timing Breakdown for Task: {task_name}', y=1.02)
    plt.show()
    
def make_names(iLQR_yaml_files):
    names = []
    
    for i in range(len(iLQR_yaml_files)):
        names.append(iLQR_yaml_files[i]['keypoint_name'])
    
    return names

def load_iLQR_data(task):
    # Load all iLQR algorithms
    global run_mode
    global iterations
    
    run_iterations = run_mode + "_" + iterations
    
    dataframes_iLQR = []
    yamlfiles_iLQR = []
        
    # Directory from where to load data (TODO - this is hardcoded)
    current_dir = "iLQR-FD/iLQR_4_10_results"

    entries = os.listdir(current_dir)

    # Sort the entries alphabetically - ensures that the order is the same for all lists
    entries.sort()

    for folder in entries:
        # Only add the data if the task name is correct
        if task not in folder:
            continue
        
        if run_iterations not in folder:
            continue
        
        folder_path = os.path.join(current_dir, folder)
        file_path = folder_path + "/summary.csv"
        
        try:
            df = pd.read_csv(file_path)
        except:
            continue
                
        # Now you can work with the DataFrame 'df'
        # For example, print the first few rows
        # print(f"Data from {file_path}:")
        # print(df.head())
        
        dataframes_iLQR.append(df)
        
        # Load yaml file
        file_name_yaml = folder_path + "/summary.yaml"
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            yamlfiles_iLQR.append(yaml_data)  # Add YAML data to the list
            
    names = make_names(yamlfiles_iLQR)
        
        
    return names, dataframes_iLQR, yamlfiles_iLQR

def load_SCVX_data(task):
    # Load all iLQR algorithms
    global base_dir
    global run_mode
    global iterations
    iterations = "6_10"
    
    run_iterations = run_mode + "_" + iterations
    
    dataframes_SCVX = []
    yamlfiles_SCVX = []
        
    # Directory from where to load data (TODO - this is hardcoded)
    current_dir = "SCVX"

    entries = os.listdir(current_dir)

    # Sort the entries alphabetically - ensures that the order is the same for all lists
    entries.sort()

    for folder in entries:
        # Only add the data if the task name is correct
        if task not in folder:
            continue
        
        if run_iterations not in folder:
            continue
        
        folder_path = os.path.join(current_dir, folder)
        file_path = folder_path + "/summary.csv"
        
        try:
            df = pd.read_csv(file_path)
        except:
            continue
                
        # Now you can work with the DataFrame 'df'
        # For example, print the first few rows
        # print(f"Data from {file_path}:")
        # print(df.head())
        
        dataframes_SCVX.append(df)
        
        # Load yaml file
        file_name_yaml = folder_path + "/summary.yaml"
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            yamlfiles_SCVX.append(yaml_data)  # Add YAML data to the list
            
    names_SCVX = make_names(yamlfiles_SCVX)
        
        
    return names_SCVX, dataframes_SCVX, yamlfiles_SCVX


if __name__ == "__main__":
    
    if(len(argv) < 2):
        print("Usage: python plot_iLQR_SCVX_iteration_timings.py <task_name>")
        exit(1)
    
    task_name = argv[1]
    main(task_name)