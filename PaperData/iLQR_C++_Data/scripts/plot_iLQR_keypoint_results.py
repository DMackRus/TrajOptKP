import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv
import os
import sys
import yaml
import math

green_shades = ['#006400', '#2E8B57', '#90EE90']
blue_shades = ['#00008B', '#4169E1', '#ADD8E6']

task_name = "push_mcl"
base_dir = ".."

def main():
    global task_name
    
    names, dataframes_iLQR, yamlfiles_iLQR = load_raw_data(task_name)
    # Pre processing step, make optimisation times in seconds rather than milliseconds
    # Loop through data frames
    for dataframe in dataframes_iLQR:
        dataframe['Optimisation time (ms)'] /= 1000

    plot_openloop_data(names, dataframes_iLQR)

def plot_openloop_data(names, dataframes_iLQR):
    
    # graphs_to_plot = ['Cost reduction', 'Optimisation time (ms)', 'Optimisation time (ms)', 'Number iterations', 'Average percent derivs']
    # columns_per_graph = [['Cost reduction'], 
    #                      ['Optimisation time (ms)', 'Average time derivs (ms)', 'Average time BP (ms)', 'Average time FP (ms)'],
    #                      ['Optimisation time (ms)'],
    #                      ['Number iterations'],
    #                      ['Average percent derivs']]
    
    graphs_to_plot = ['Optimisation time (ms)', 'Cost reduction',  'Number iterations', 'Average percent derivs']
    columns_per_graph = [['Optimisation time (ms)'],
                         ['Cost reduction'], 
                         ['Number iterations'],
                         ['Average percent derivs']]
    
    generate_plots_confidence(names, dataframes_iLQR, graphs_to_plot, columns_per_graph)
    
def generate_plots_confidence(names, dataframes_iLQR, graphs, columns_per_graph):
    global task_name
    
    num_iLQR_methods = len(dataframes_iLQR)
    
    means = np.zeros((num_iLQR_methods, len(graphs)))
    confidence_intervals = np.zeros((num_iLQR_methods, len(graphs)))

    # Number of subplots
    num_subplots = len(graphs)
    num_data_points = len(dataframes_iLQR)
    x = range(num_data_points)
    
    # confidence interval 95%
    z = 1.96 # 90% ci z = 1.645

    # Create a figure and axes for subplots
    fig, axes = plt.subplots(num_subplots, 1, sharex=True, figsize=(8, num_subplots*4))
    
    for i, graph_name in enumerate(graphs):
        
        # if columns_per_graph > 1 (stacking graphs)
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
                    
                # print(f'means: {means}')
                # print(f'bottoms: {bottom_of_graphs}')
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
            # means = []
            # std_dev = []
            # margin_of_errors = []
            # ci = []
            
            # confidence interval of 95 %
            num_data_rows = dataframes_iLQR[0].shape[0]
            
            bar_colors = [green_shades[0]] * num_iLQR_methods + [blue_shades[0]] * (len(names) - num_iLQR_methods)

            
            # Loop through iLQR
            for j, data_frame in enumerate(dataframes_iLQR):
            
                means[j, i] = data_frame[graph_name].mean()
                confidence_intervals[j, i] = z * (data_frame[graph_name].std() / np.sqrt(num_data_rows))
                
            axes[i].bar(x, means[:,i], yerr=confidence_intervals[:,i], capsize=5, color=bar_colors)
            axes[i].set_ylabel(graph_name, fontsize = 13)
            
            
    # Print the data in table format for easy transferance to the paper
    print(f'Methods: {names}')
    for i in range(len(graphs)):
        print(f'{graphs[i]}', end=' ')
        for j in range(len(names)):
            
            mean_val = custom_round(means[j,i])
            ci_value = custom_round(confidence_intervals[j,i])
            print(f'{mean_val}', end=' ')
            print(f' +- {ci_value}', end=' ')
            
        print('')

    # Print average time per iteration for each method (means[0,2] / means[2,2])
    print('Average time per iteration ', end='')
    for j in range(len(names)):
        mean_val = custom_round(means[j,0]/means[j,2])
        ci_val = custom_round(confidence_intervals[j,0]/means[j,2])
        print(f'{mean_val}', end=' ')
        print(f' +- {ci_val}', end=' ')

    print('')
    
    
    plt.xticks(x, names, rotation=45, ha='right') 
    figure_title = task_name
    fig.suptitle(figure_title, fontsize = 20)
    
    plt.show()
    
def make_names(iLQR_yaml_files):
    names = []
    
    for i in range(len(iLQR_yaml_files)):
        names.append(iLQR_yaml_files[i]['keypoint_name'])
    
    return names
    
def load_raw_data(task_name):
    # Load all iLQR algorithms
    global base_dir
    
    dataframes_iLQR = []
    yamlfiles_iLQR = []
    
    current_dir = base_dir + "/iLQR"

    entries = os.listdir(current_dir)

    # Sort the entries alphabetically - ensures that the order is the same for all lists
    entries.sort()

    for folder in entries:
        # Only add the data if the task name is correct
        if task_name not in folder:
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

def custom_round(value):
    # Round to 2 decimal places
    rounded_decimal = round(value, 2)
    
    # Round to 2 significant figures
    if value == 0:
        rounded_significant = 0
    else:
        rounded_significant = round(value, 2 - int(math.floor(math.log10(abs(value)))) - 1)

    # Choose the one with more digits
    if len(str(rounded_significant)) > len(str(rounded_decimal)):
        return rounded_significant
    else:
        return rounded_decimal
    
if __name__ == "__main__":
    main()
