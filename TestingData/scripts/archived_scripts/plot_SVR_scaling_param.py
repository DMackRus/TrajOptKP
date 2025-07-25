from collections import defaultdict
import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv
import os
import sys
import yaml
from scipy.signal import savgol_filter
from matplotlib.lines import Line2D

green_shades = ['#006400', '#2E8B57', '#90EE90']
blue_shades = ['#00008B', '#4169E1', '#ADD8E6']

task_name = "push_mcl" #push_soft or push_mcl
run_mode = "asynchronus" # openloop or asynchronus
task_num = -1

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
    
    desired_param = "num_dofs_readd" # or num_dofs_readd, K_matrix_threshold

    # scaling_params = [10, 20, 50]
    # scaling_params = [0.1, 0.2, 0.5]
    scaling_params = [0, 5, 10]
    results = dict()
    for param in scaling_params:
        names, dataframes_iLQR_SVR, yamlfiles_iLQR_SVR = load_raw_data(run_mode, task_name, desired_param, param)
        names, values = make_names(yamlfiles_iLQR_SVR, desired_param)

        # Step 1: Sort the list of integers and keep track of the indices
        sorted_indices = sorted(range(len(values)), key=lambda k: values[k])

        # Step 2: Reorder the list of dataframes based on the sorted indices
        sorted_dataframes = [dataframes_iLQR_SVR[i] for i in sorted_indices]
        
        sorted_names = [names[i] for i in sorted_indices]

        means, std_dev, confidence_intervals = generate_table_data(sorted_names, sorted_dataframes, ['Final cost', "Average optimisation time (ms)", 'Average dofs'])

        results[str(param)] = [means, std_dev, confidence_intervals]

        # plot_scaling_SVR_parameters(sorted_names, sorted_dataframes)
        
    plot_line_graphs(sorted_names, results, desired_param)

    # Optionally, sort the list of integers itself (for clarity, not necessary if only reordering dataframes is needed)
    # sorted_integers = sorted(list_of_integers)
    

def plot_line_graphs(names, data, desired_param):
    # Create three subfigures in figure
    # Each plot needs three lines, that comes from data.keys()

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

    keys = data.keys()
    print(keys)
    print(names)


    # print(data[keys[0]])
    num_methods = len(keys)
    print(f'num methods: {num_methods}')
    x = range(0, len(names))

    colors_keys = ['#4A90E2', '#FF6F61', '#8AB17D']


    for i, key in enumerate(keys):
        print(key)
        # ---------------- Final cost -----------------------

        # trend = savgol_filter(data[key][0]['Final cost'], window_length=5, polyorder=2)
        # print(data[key][0]['Final cost'])
        # lower_bound = [data_point - 100 for data_point in data[key][0]['Final cost']]
        # upper_bound = [data_point + 100 for data_point in data[key][0]['Final cost']]
        # upper_bound = []
        # lower_bound = []
        # for i in range(len(data[key][0]['Final cost'])):
        #     upper_bound.append(data[key][0]['Final cost'][i] + data[key][2]['Final cost'][i])
        #     lower_bound.append(data[key][0]['Final cost'][i] - data[key][2]['Final cost'][i])
            
        upper_bound = [x - y for x, y in zip(data[key][0]['Final cost'], data[key][2]['Final cost'])]

        # Adding the lists element-wise
        lower_bound = [x + y for x, y in zip(data[key][0]['Final cost'], data[key][2]['Final cost'])]
            
        axes[0].plot(x, data[key][0]['Final cost'], label=key, color=colors_keys[i])
        axes[0].fill_between(x, lower_bound, upper_bound, color=colors_keys[i], alpha=0.2)
        # axes[0].plot(x, trend, label=key, color=colors_keys[i], alpha = 1.0, linewidth=2)

        # ---------------- Optimisation time -----------------------
        # trend = savgol_filter(data[key][0]['Average optimisation time (ms)'], window_length=11, polyorder=2)
        # axes[1].plot(x, data[key][0]['Average optimisation time (ms)'], label=key, color=colors_keys[i])
        # axes[1].plot(x, trend, label=key, color=colors_bold[i])

        # ---------------- Number dofs -----------------------
        # trend = savgol_filter(data[key][0]['Average dofs'], window_length=11, polyorder=2)
        axes[1].plot(x, data[key][0]['Average dofs'], label=key, color=colors_keys[i])
        # axes[2].plot(x, trend, label=key, color=colors_bold[i])
    
    # axes[0].plot(x, means, capsize=5)
    axes[0].set_ylabel("Final cost", fontsize = 13)
    axes[1].set_ylabel("Optimisation time (ms)", fontsize = 13)
    axes[1].set_ylabel("Num dofs", fontsize = 13)
    
    # fig.suptitle("Scaling parameter: " + task_name + "_" + run_mode, fontsize = 20)
    # fig.suptitle("Push mcl: Scaling Rho", fontsize = 16)

    legend_lines = [
        Line2D([0], [0], color=colors_keys[0], lw=2),
        Line2D([0], [0], color=colors_keys[1], lw=2),
        Line2D([0], [0], color=colors_keys[2], lw=2)
    ]
    
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(names)

    # Custom legend labels
    legend_labels = []
    if(desired_param == "K_matrix_threshold"):
        axes[1].set_xlabel("Theta", fontsize = 13)
        for key in keys:
            legend_labels.append("Rho = " + str(key))
    else:
        axes[1].set_xlabel("Rho", fontsize = 13)
        for key in keys:
            legend_labels.append("Theta = " + str(key))
        

    # Add custom legend
    axes[1].legend(legend_lines, legend_labels)

    plt.show()


def plot_scaling_SVR_parameters(names, dataframes_iLQR_SVR):
    
    graphs_to_plot = ['Final cost', "Average optimisation time (ms)", 'Average dofs']
    columns_per_graph = [['Final cost'], 
                         ['Average optimisation time (ms)', 'Average time derivs (ms)', 'Average time BP (ms)', 'Average time FP (ms)'], 
                         ['Average dofs']]
    
    generate_plots_confidence(names, dataframes_iLQR_SVR, graphs_to_plot, columns_per_graph)
    

def plot_asynchronus_data(names, dataframes_iLQR, dataframes_iLQR_SVR):
    
    graphs_to_plot = ['Final cost', "Average optimisation time (ms)", 'Average dofs']
    columns_per_graph = [['Final cost'], 
                         ['Average optimisation time (ms)', 'Average time derivs (ms)', 'Average time BP (ms)', 'Average time FP (ms)'], 
                         ['Average dofs']]
    
    generate_plots_confidence(names, dataframes_iLQR, dataframes_iLQR_SVR, graphs_to_plot, columns_per_graph)
    
    
def generate_table_data(names, dataframes_iLQR_SVR, columns):
    
    num_methods = len(names)
    # means = np.zeros((num_methods, len(columns)))
    # std_dev = np.zeros((num_methods, len(columns)))
    means = defaultdict(list)
    std_dev = defaultdict(list)
    confidence_intervals = defaultdict(list)
    
    z = 1.645 #90% confidence interval
         
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
            
            # mean_temp = data_frame[column].mean()
            # std_dev_temp = data_frame[column].std()

            # # Define the lower and upper bounds for outliers
            # # lower_bound = mean_temp - 1 * std_dev_temp
            # upper_bound = mean_temp + 0.1 * std_dev_temp
            # lower_bound = 0

            # Filter the DataFrame to remove outliers
            df_filtered = data_frame[(data_frame[column] >= lower_bound) & (data_frame[column] <= upper_bound)]
            # df_filtered = data_frame
            
        
            print(df_filtered[column].size)
            means[column].append((df_filtered[column].mean()))
            std_dev[column].append((df_filtered[column].std()))
            
            confidence_intervals[column].append(z * (df_filtered[column].std() / np.sqrt(df_filtered[column].size) ))
            
            
            # means[j, i] = (df_filtered[column].mean())
            # std_dev[j, i] = (df_filtered[column].std())
          
    # for i, method_name in enumerate(names):
    #     print("------------------------------------------------")
    #     print(method_name)
    #     print(f'{columns[0]}: {means[i, 0]:.2f}[{std_dev[i, 0]:.2f}]')
    #     print(f'{columns[1]}: {means[i, 1]:.2f}[{std_dev[i, 1]:.2f}]')
    #     print(f'{columns[2]}: {means[i, 2]:.2f}[{std_dev[i, 2]:.2f}]')

    return means, std_dev, confidence_intervals

def generate_plots_confidence(names, dataframes_iLQR_SVR, graphs, columns_per_graph):
    global task_name
    global run_mode
    
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
    num_data_points = len(dataframes_iLQR_SVR)
    x = range(num_data_points)
    
    # confidence interval 95%
    z = 1.95

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
            num_data_rows = dataframes_iLQR_SVR[0].shape[0]
            column_name = columns_per_graph[i][0]
                
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
                # bar_colors = [green_shades[j-1]] * num_iLQR_methods + [blue_shades[j-1]] * (len(names) - num_iLQR_methods)
                
                # Loop through iLQR_SVR
                for data_frame in dataframes_iLQR_SVR:
                    means.append(data_frame[column_name].mean())
                    
                print(f'means: {means}')
                print(f'bottoms: {bottom_of_graphs}')
                axes[i].bar(x, means, bottom = bottom_of_graphs)
                
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
            num_data_rows = dataframes_iLQR_SVR[0].shape[0]
            
            # bar_colors = [green_shades[0]] * num_iLQR_methods + [blue_shades[0]] * (len(names) - num_iLQR_methods)
                
            # Loop through iLQR_SVR
            for data_frame in dataframes_iLQR_SVR:
                means.append(data_frame[graph_name].mean())
                std_dev.append(data_frame[graph_name].std())
                
            margin_of_errors = z * (std_dev / np.sqrt(num_data_rows))
            ci = margin_of_errors
                
            axes[i].bar(x, means, yerr=margin_of_errors, capsize=5)
            axes[i].set_ylabel(graph_name, fontsize = 13)
    
    plt.xticks(x, names, rotation=45, ha='right') 
    figure_title = task_name + "_" + run_mode
    fig.suptitle(figure_title, fontsize = 20)
    
    plt.show()
    
def make_names(iLQR_SVR_yaml_files, desired_param):
    names = []
    values = []
    
    for i in range(len(iLQR_SVR_yaml_files)):
        if(desired_param == 'K_matrix_threshold'):
            name = str(iLQR_SVR_yaml_files[i]['num_dofs_readd'])
            value = iLQR_SVR_yaml_files[i]['num_dofs_readd']
        elif(desired_param == 'num_dofs_readd'):
            name = str(iLQR_SVR_yaml_files[i]['K_matrix_threshold'])
            value = iLQR_SVR_yaml_files[i]['K_matrix_threshold']
        
        names.append(name)
        values.append(value)
    
    return names, values

    
def load_raw_data(run_mode, task_name, parameter, desired_val):
    global base_dir
    
    names = []
    dataframes_iLQR_SVR = []
    yamlfiles_iLQR_SVR = []
        
    # Load all iLQR_SVR data
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
        # print(df.head())
        
        # Load yaml file
        file_name_yaml = folder_path + "/summary.yaml"
        yaml_data = []
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            
        #Temp, dont want to plot the rand val
        if(yaml_data['K_matrix_threshold'] > 1000):
            continue
            
            
        # check if yaml file parameter is what we were searching for
        if(yaml_data[parameter] == desired_val):
            dataframes_iLQR_SVR.append(df)
            names.append(folder)
            yamlfiles_iLQR_SVR.append(yaml_data)  # Add YAML data to the list
        
    return names, dataframes_iLQR_SVR, yamlfiles_iLQR_SVR
        
        
if __name__ == "__main__":
    main()
        
        
        
        
        
    