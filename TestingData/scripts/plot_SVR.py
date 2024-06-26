import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv
import os
import sys
import yaml

green_shades = ['#006400', '#2E8B57', '#90EE90']
blue_shades = ['#00008B', '#4169E1', '#ADD8E6']

task_name = "push_mcl" # push_mcl, or push_soft, or push_soft_into_rigid
run_mode = "asynchronus" # openloop or asynchronus
task_num = -1

# "../saved_asyncronus_MPC_push_mcl_2_BEST"
# "../saved_asyncronus_MPC_push_soft"

# base_dir = "../saved_asyncronus_MPC_push_mcl_2_BEST" # or ".."
base_dir = "../saved_MPC_push_mcl_scaling_Rho_Best"

show_ind_trajecs = True

# When openloop
columns_openloop = ["Cost reduction", "Optimisation time (ms)", "Number iterations", "Average num dofs", "Average percent derivs", "Average time derivs (ms)", "Average time BP (ms)", "Average time FP (ms)"]

# When asynchronus MPC
#Final cost,Final dist,Average dofs,Average optimisation time (ms),Average percent derivs,
#Average time derivs (ms),Average time BP (ms),Average time FP (ms),Average surprise

def main():
    global task_name
    global task_num
    global run_mode
    
    # if(len(sys.argv) < 2):
    #     pass
    # else:
    #     # argument 0 is the program name
    #     task = sys.argv[1]
    #     if(len(sys.argv) > 2):
    #         task_num = sys.argv[2]
            
    # if(task_num != -1):
    #     plot_task(task, "AA_1_50", task_num)
    # plot_summary(task)
    
    names, dataframes_iLQR, dataframes_iLQR_SVR, yamlfiles_iLQR, yamlfiles_iLQR_SVR = load_raw_data(run_mode, task_name)
    
    if(run_mode == "openloop"):
        names = make_names(yamlfiles_iLQR, yamlfiles_iLQR_SVR)
        plot_openloop_data(names, dataframes_iLQR, dataframes_iLQR_SVR)
    elif(run_mode == "asynchronus"):
        names = make_names(yamlfiles_iLQR, yamlfiles_iLQR_SVR)
        plot_asynchronus_data(names, dataframes_iLQR, dataframes_iLQR_SVR)
        
    else:
        print("invalid run mode argument")
    
    
def plot_openloop_data(names, dataframes_iLQR, dataframes_iLQR_SVR):
    
    graphs_to_plot = ['Cost reduction', 'Optimisation time (ms)', 'Average num dofs']
    columns_per_graph = [['Cost reduction'], 
                         ['Optimisation time (ms)', 'Average time derivs (ms)', 'Average time BP (ms)', 'Average time FP (ms)'],
                         ['Average num dofs']]
    
    generate_plots_confidence(names, dataframes_iLQR, dataframes_iLQR_SVR, graphs_to_plot, columns_per_graph)

def plot_asynchronus_data(names, dataframes_iLQR, dataframes_iLQR_SVR):
    
    graphs_to_plot = ['Final cost', "Average optimisation time (ms)", 'Average dofs']
    columns_per_graph = [['Final cost'], 
                         ['Average optimisation time (ms)', 'Average time derivs (ms)', 'Average time BP (ms)', 'Average time FP (ms)'], 
                         ['Average dofs']]
    
    generate_table_data(names, dataframes_iLQR, dataframes_iLQR_SVR, graphs_to_plot)
    generate_plots_confidence(names, dataframes_iLQR, dataframes_iLQR_SVR, graphs_to_plot, columns_per_graph)
    
    
    
def generate_plots_confidence(names, dataframes_iLQR, dataframes_iLQR_SVR, graphs, columns_per_graph):
    global task_name
    global run_mode
    
    num_iLQR_methods = len(dataframes_iLQR)
    
    final_cost_means = []
    final_dist_means = []
    optimisation_time_means = []
    avg_num_dofs_means = []
    
    final_cost_std = []
    final_dist_std = []
    optimisation_time_std = []
    avg_num_dofs_std = []

    # Number of subplots
    num_subplots = len(graphs)
    num_data_points = len(dataframes_iLQR) + len(dataframes_iLQR_SVR)
    x = range(num_data_points)
    
    # confidence interval 95%
    z = 1.96
    
    # 90 % confidence interval
    # z = 1.645

    # Create a figure and axes for subplots
    fig, axes = plt.subplots(num_subplots, 1, sharex=True, figsize=(8, num_subplots*4))
    
    for i, graph_name in enumerate(graphs):
        
        # if columns_per_graph > 1
        if(len(columns_per_graph[i]) > 1):
            
            # Just compute ci for first attribute (optimisation time)
            overall_time_means = []
            overall_std_dev = []
            overall_margin_of_errors = []
            overall_ci = []
            
            # confidence interval of 95 %
            num_data_rows = dataframes_iLQR[0].shape[0]
            column_name = columns_per_graph[i][0]
            
            # Loop through iLQR
            for data_frame in dataframes_iLQR:
                overall_time_means.append(data_frame[column_name].mean())
                overall_std_dev.append(data_frame[column_name].std())
                
            # Loop through iLQR_SVR
            for data_frame in dataframes_iLQR_SVR:
                overall_time_means.append(data_frame[column_name].mean())
                overall_std_dev.append(data_frame[column_name].std())
                
            margin_of_errors = z * (overall_std_dev / np.sqrt(num_data_rows))
            ci = margin_of_errors
                
            # axes[i].bar(x, overall_time_means, yerr=margin_of_errors, capsize=5, color='gray')   
            # axes[i].bar(x, overall_time_means, capsize=5, color='gray')
            bottom_of_graphs = [0] * num_data_points
            last_means = []
                
            for j in range(1, len(columns_per_graph[i])):
                means = []
                column_name = columns_per_graph[i][j]
                
                # bar_colors = green_shades[j] * num_iLQR_methods + blue_shades[j] * (len(names) - num_iLQR_methods)
                bar_colors = [green_shades[j-1]] * num_iLQR_methods + [blue_shades[j-1]] * (len(names) - num_iLQR_methods)
                
                # Loop through iLQR
                for data_frame in dataframes_iLQR:
                    means.append(data_frame[column_name].mean())
                
                # Loop through iLQR_SVR
                for data_frame in dataframes_iLQR_SVR:
                    means.append(data_frame[column_name].mean())
                    
                print(f'means: {means}')
                print(f'bottoms: {bottom_of_graphs}')
                axes[i].bar(x, means, bottom = bottom_of_graphs, color=bar_colors)
                
                # Update bottom of graphs
                for k in range(num_data_points):
                    bottom_of_graphs[k] += means[k]
                    
                last_means = means
                    
            # Additional unacounted for timings = overall - (means + bottoms)
            # additional = last_means + bottom_of_graphs
            # axes[i].bar(x, additional, yerr = ci, color='gray')
                
            axes[i].set_ylabel(graph_name, fontsize = 13)

        else:
            # Compute mean and standard deviation
            means = []
            std_dev = []
            margin_of_errors = []
            ci = []
            
            # confidence interval of 95 %
            num_data_rows = dataframes_iLQR[0].shape[0]
            
            bar_colors = [green_shades[0]] * num_iLQR_methods + [blue_shades[0]] * (len(names) - num_iLQR_methods)

            
            # Loop through iLQR
            for data_frame in dataframes_iLQR:
                
                # Calculate Q1 (25th percentile) and Q3 (75th percentile)
                Q1 = data_frame[graph_name].quantile(0)
                Q3 = data_frame[graph_name].quantile(0.6)

                # Calculate the IQR
                IQR = Q3 - Q1

                # Define the lower and upper bounds for outliers
                # lower_bound = Q1 - 1.5 * IQR
                lower_bound = 0
                upper_bound = Q3 + 1.5 * IQR

                # Filter the DataFrame to remove outliers
                df_filtered = data_frame[(data_frame[graph_name] >= lower_bound) & (data_frame[graph_name] <= upper_bound)]
                # df_filtered = data_frame
            
                means.append(df_filtered[graph_name].mean())
                std_dev.append(df_filtered[graph_name].std())
                
            # Loop through iLQR_SVR
            for data_frame in dataframes_iLQR_SVR:
                
                # Calculate Q1 (25th percentile) and Q3 (75th percentile)
                Q1 = data_frame[graph_name].quantile(0)
                Q3 = data_frame[graph_name].quantile(0.6)

                # Calculate the IQR
                IQR = Q3 - Q1

                # Define the lower and upper bounds for outliers
                # lower_bound = Q1 - 1.5 * IQR
                lower_bound = 0
                upper_bound = Q3 + 1.5 * IQR

                # Filter the DataFrame to remove outliers
                df_filtered = data_frame[(data_frame[graph_name] >= lower_bound) & (data_frame[graph_name] <= upper_bound)]
                # df_filtered = data_frame
                
                means.append(df_filtered[graph_name].mean())
                std_dev.append(df_filtered[graph_name].std())
                
            margin_of_errors = z * (std_dev / np.sqrt(num_data_rows))
            ci = margin_of_errors
                
            axes[i].bar(x, means, yerr=margin_of_errors, capsize=5, color=bar_colors)
            axes[i].set_ylabel(graph_name, fontsize = 13)
    
    plt.xticks(x, names, rotation=45, ha='right') 
    figure_title = task_name + "_" + run_mode
    fig.suptitle(figure_title, fontsize = 20)
    
    plt.show()
    
    
def generate_table_data(names, dataframes_iLQR, dataframes_iLQR_SVR, columns):
    
    num_methods = len(names)
    num_iLQR_methods = len(dataframes_iLQR)
    means = np.zeros((num_methods, len(columns)))
    std_dev = np.zeros((num_methods, len(columns)))
    confidence_intervals = np.zeros((num_methods, len(columns)))
    num_data_points = np.zeros((num_methods, len(columns)))
    
    z = 1.645
    
    # iLQR
    for i, column in enumerate(columns):
        print(column)
        
        for j, data_frame in enumerate(dataframes_iLQR):
            
            Q1 = data_frame[column].quantile(0)
            Q3 = data_frame[column].quantile(0.6)

            # Calculate the IQR
            IQR = Q3 - Q1

            # Define the lower and upper bounds for outliers
            lower_bound = 0
            upper_bound = Q3 + 1.5 * IQR

            # Filter the DataFrame to remove outliers
            df_filtered = data_frame[(data_frame[column] >= lower_bound) & (data_frame[column] <= upper_bound)]
        
            means[j, i] = (df_filtered[column].mean())
            std_dev[j, i] = (df_filtered[column].std())
            
            confidence_intervals[j, i] = z * std_dev[j, i] / np.sqrt(df_filtered[column].size)
            num_data_points[j, i] = df_filtered[column].size
         
    # iLQR_SVR   
    for i, column in enumerate(columns):
        
        for j, data_frame in enumerate(dataframes_iLQR_SVR):
            
            Q1 = data_frame[column].quantile(0)
            Q3 = data_frame[column].quantile(0.6)

            # Calculate the IQR
            IQR = Q3 - Q1

            # Define the lower and upper bounds for outliers
            # lower_bound = Q1 - 1.5 * IQR
            lower_bound = 0
            upper_bound = Q3 + 1.5 * IQR

            # Filter the DataFrame to remove outliers
            df_filtered = data_frame[(data_frame[column] >= lower_bound) & (data_frame[column] <= upper_bound)]
        
            means[j + num_iLQR_methods, i] = (df_filtered[column].mean())
            std_dev[j + num_iLQR_methods, i] = (df_filtered[column].std())
            
            confidence_intervals[j + num_iLQR_methods, i] = z * std_dev[j + num_iLQR_methods, i] / np.sqrt(df_filtered[column].size)
            num_data_points[j + num_iLQR_methods, i] = df_filtered[column].size
            
    #Normalise cost values via iLQR-Basline
    base_cost = means[1,0]
    for i in range(num_methods):
        means[i, 0] /= base_cost
        std_dev[i, 0] /= base_cost
        
        confidence_intervals[i, 0] = z * (std_dev[i, 0] / np.sqrt(num_data_points[i, 0]))
        
    
    
          
    for i, method_name in enumerate(names):
        print("------------------------------------------------")
        print(method_name)
        # print(f'{columns[0]}: {means[i, 0]:.2f}[{std_dev[i, 0]:.2f}]')
        # print(f'{columns[1]}: {means[i, 1]:.2f}[{std_dev[i, 1]:.2f}]')
        # print(f'{columns[2]}: {means[i, 2]:.2f}[{std_dev[i, 2]:.2f}]')  
        print(f'{columns[0]}: {means[i, 0]:.2f}+-{confidence_intervals[i, 0]:.4f}')
        print(f'{columns[1]}: {means[i, 1]:.2f}+-{confidence_intervals[i, 1]:.4f}')
        print(f'{columns[2]}: {means[i, 2]:.2f}+-{confidence_intervals[i, 2]:.4f}')  
        
        
    
def make_names(iLQR_yaml_files, iLQR_SVR_yaml_files):
    names = []
    
    for i in range(len(iLQR_yaml_files)):
        name = "iLQR"
        names.append(name)
        
    for i in range(len(iLQR_SVR_yaml_files)):
        K_thresh = iLQR_SVR_yaml_files[i]['K_matrix_threshold']
        num_dofs_readd = iLQR_SVR_yaml_files[i]['num_dofs_readd']
        SVD_method = iLQR_SVR_yaml_files[i]['Eigen vector method']
        
        if(SVD_method):
            name = "iLQR-SVR-SVD-" + str(K_thresh) + "_" + str(num_dofs_readd)
        else:
            name = "iLQR-SVR-Sum-" + str(K_thresh) + "_" + str(num_dofs_readd)
            
        
        names.append(name)
    
    return names

    
def load_raw_data(run_mode, task_name):
    # Load all iLQR algorithms
    global base_dir
    
    names = []
    dataframes_iLQR = []
    yamlfiles_iLQR = []
    dataframes_iLQR_SVR = []
    yamlfiles_iLQR_SVR = []
    
    # Load the iLQR data
    # base_dir = "../iLQR"
    print(base_dir)
    current_dir = base_dir + "/iLQR"
    for folder in os.listdir(current_dir):
        print("folder: " + str(folder))
        # Only add the data if the task name is correct
        if task_name not in folder:
            continue
        
        # Only add the data if the run_mode is correct
        if run_mode not in folder:
            continue
        
        folder_path = os.path.join(current_dir, folder)
        file_path = folder_path + "/summary.csv"
        print(file_path)
        
        try:
            df = pd.read_csv(file_path)
        except:
            continue
                
        # Now you can work with the DataFrame 'df'
        # For example, print the first few rows
        # print(f"Data from {file_path}:")
        print(df.head())
        
        dataframes_iLQR.append(df)
        names.append(folder)
        
        # Load yaml file
        file_name_yaml = folder_path + "/summary.yaml"
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            yamlfiles_iLQR.append(yaml_data)  # Add YAML data to the list
        
    # Load all iLQR_SVR data
    # base_dir = "../iLQR_SVR"
    current_dir = base_dir + "/iLQR_SVR"
    for folder in os.listdir(current_dir):
        
        # Only add the data if the task name is correct
        if task_name not in folder:
            continue
        
        # Only add the data if the run_mode is correct
        if run_mode not in folder:
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
        print(df.head())
        
        dataframes_iLQR_SVR.append(df)
        names.append(folder)
        
         # Load yaml file
        file_name_yaml = folder_path + "/summary.yaml"
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            yamlfiles_iLQR_SVR.append(yaml_data)  # Add YAML data to the list
        
        # Load summary.csv and summaray.yaml
        
    return names, dataframes_iLQR, dataframes_iLQR_SVR, yamlfiles_iLQR, yamlfiles_iLQR_SVR
        
        
if __name__ == "__main__":
    main()
        
        
        
        
        
    