import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv
import os
import sys
import yaml
import tkinter
import glob

green_shades = ['#006400', '#2E8B57', '#90EE90']
blue_shades = ['#00008B', '#4169E1', '#ADD8E6']

# task_name = "push_mcl"
# iterations = "6_6"
# iterations = "3_10"
iterations = "4_10"
# task_name = "push_mcl"
base_dir = ""
run_mode = "openloop"

show_plot = True
paper_data_folder = False

# tasks = ["push_ncl", "push_lcl", "push_mcl", "box_sweep", "impact", "walker", "acrobot"]
# tasks = ["push_lcl", "push_mcl"]
# tasks = ["push_ncl", "push_lcl", "push_mcl", "box_sweep", "impact", "walker"]
tasks = ["box_sweep"]

methods = ["SI_1", "SI_5", "SI_1000", "contact_change", 
                "contact_change_dyn"]

cost_reductions_all = [[] for _ in range(len(methods))]
optimisation_times_all = [[] for _ in range(len(methods))]
number_iterations_all = [[] for _ in range(len(methods))]

def main():
    # global task_name
    
    for task in tasks:
        names, dataframes_iLQR, yamlfiles_iLQR = load_raw_data(task)
    
        methods = ["SI_1", "SI_5", "SI_1000", "contact_change", 
                "contact_change_dyn"]
        # methods = ["SI_1", "SI_5", "SI_1000", "contact_change_dyn"]

        # Keep only entries where the name is in `methods`
        filtered = [(i, name) for i, name in enumerate(names) if name in methods]

        # Sort them based on the method order
        sorted_indices = sorted(filtered, key=lambda x: methods.index(x[1]))
        sorted_indices = [i for i, _ in sorted_indices]

        # Apply ordering
        names = [names[i] for i in sorted_indices]
        dataframes_iLQR = [dataframes_iLQR[i] for i in sorted_indices]
        
        # print(dataframes_iLQR)

        plot_openloop_data(names, dataframes_iLQR, task)
        
        # plot_CR_versus_iteration_timings(task)
        
        # plot_timing_breakdown_data(names, dataframes_iLQR)
        
    print(f" cost reduction shape : {np.array(cost_reductions_all).shape}")
    
    z = 1.96  # 95% CI

    print("Average ", end='')
    for m in range(len(names)):
        cr = np.array(cost_reductions_all[m])
        ot = np.array(optimisation_times_all[m])
        it = np.array(number_iterations_all[m])

        mean_cr = cr.mean()
        ci_cr = z * cr.std(ddof=1) / np.sqrt(len(cr))

        mean_ot = ot.mean()
        ci_ot = z * ot.std(ddof=1) / np.sqrt(len(ot))

        mean_it = it.mean()
        ci_it = z * it.std(ddof=1) / np.sqrt(len(it))

        if(iterations != "1_1"):
            print(
                f"& {mean_ot/1000:.2f}"
                f"& {mean_cr:.2f} $\pm$ {ci_cr:.2f} "
                f"& {mean_it:.2f}",
                end=" "
            )
        else:
            print(
            f"& {mean_ot/1000:.2f}"
            f"& {mean_cr:.2f} $\pm$ {ci_cr:.2f} ",
            end=" "
        )
    print("\\\\")
    
    # print("Average ", end='')
    # for method in range(len(names)):
    #     # Compute mean of cost reduction for each method (column)
    #     mean_cost_reduction = np.mean(cost_reductions_all, axis=1)
    #     # mean_optimisation_time = np.mean([optimisation_times[i][method] for i in range(len(optimisation_times))])
    #     # mean_number_iterations = np.mean([number_iterations[i][method] for i in range(len(number_iterations))])
        
    #     # print(f'& {mean_optimisation_time/1000.0:.2f}', end=' ')
    #     print(f'& {mean_cost_reduction:.2f}', end=' ')
    #     # print(f'& {mean_number_iterations:.2f}', end=' ')
    #     pass
    # print('\\\\')
    
def plot_CR_versus_iteration_timings(task_name):
    global base_dir, run_mode
    current_dir = base_dir + "/iLQR_4_10_results(FINAL)"

    fig, ax = plt.subplots(figsize=(6, 6))
    
    run_mode_search = run_mode + "_" + iterations

    for folder in os.listdir(current_dir):
        if task_name not in folder:
            continue
        
        if run_mode_search not in folder:
            continue
        
        # methods = ["SI_1", "SI_5", "SI_1000", "contact_change", 
        #    "contact_change_dyn", "contact_change_maxN"]
        methods = ["SI_1", "SI_5", "SI_1000", "contact_change", "contact_change_dyn"]

        file_name_yaml = current_dir + "/" + folder + "/summary.yaml"
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)
            method_name = yaml_data["keypoint_name"]

        # ---- FILTER METHODS HERE ----
        if method_name not in methods:
            continue
        
        file_name_yaml = current_dir + "/" + folder + "/summary.yaml"
        method_name = ""
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            method_name = yaml_data["keypoint_name"]

        subfolder_path = os.path.join(current_dir, folder)
        subfolders = [name for name in os.listdir(subfolder_path)
                      if os.path.isdir(os.path.join(subfolder_path, name))]

        all_costs = []
        all_cost_reductions = []
        all_times_per_iteration = []

        for trial in subfolders:
            file = os.path.join(subfolder_path, trial, "summary.csv")
            if not os.path.isfile(file):
                continue
            
            df = pd.read_csv(file)
            # if "Cost" in df.columns:
                
            all_costs.append(df["Cost"])
            all_cost_reductions.append(df["Cost reduction"])
            all_times_per_iteration.append(df["time (ms)"])
            
        # --- PAD TRIALS TO SAME LENGTH ---
        max_len = max(len(s) for s in all_costs)

        def pad_series(s, max_len):
            return (
                s
                .reset_index(drop=True)
                .reindex(range(max_len))
                .ffill()
            )

        all_costs = [pad_series(s, max_len) for s in all_costs]
        all_cost_reductions = [pad_series(s, max_len) for s in all_cost_reductions]
        all_times_per_iteration = [pad_series(s, max_len) for s in all_times_per_iteration]
                
                
        combined_cost = pd.concat(all_costs, axis=1)
        print(f"Combined cost shape: {combined_cost.shape}")
        mean_cost = combined_cost.mean(axis=1)
        print(f"Mean cost shape: {mean_cost.shape}")
        combined_CR = pd.concat(all_cost_reductions, axis=1)
        mean_CR = combined_CR.mean(axis=1)
        combined_times = pd.concat(all_times_per_iteration, axis=1)
        mean_times = combined_times.mean(axis=1)
        
        plt.plot(mean_times, mean_CR, '-o', label=method_name)

    plt.title(f"Trajectory CR Vs Iteration Time for {task_name}")
    plt.xlabel("Cumulative iteration time (ms)")
    plt.ylabel("Cost reduction")
    plt.grid(True)
    plt.legend()

    # plt.tight_layout(rect=[0, 0, 0.85, 0.95])  # Leave space for suptitle and legend
    plt.show()

    
def plot_timing_breakdown_data(names, dataframes_iLQR):
    global task_name
    
    num_iLQR_methods = len(dataframes_iLQR)
    
    # Define the timing categories (excluding the total "optimisation time" for plotting)
    categories = ["Optimisation time (ms)", "Total time keypoints (ms)", 
                  "Total time FD (ms)", "Total time interpolation (ms)", 
                  "Total time cost derivs (ms)", "Total time BP (ms)",
                  "Total time FP (ms)"]
    
    plot_categories = categories[1:]  # Skip the first one for stacking
    # colors = plt.cm.viridis(np.linspace(0.2, 0.9, len(plot_categories)))  # Colour palette
    
    # color_map = {
    #     "Total time keypoints (ms)": "#5DADE2",   # Light blue
    #     "Total time FD (ms)": "#3498DB",         # Medium blue
    #     "Total time interpolation (ms)": "#2E86C1", # Darker blue
    #     "Total time cost derivs (ms)": "#1B4F72", # Very dark blue
    #     "Total time BP (ms)": "#D81B60",         # Bold pink/red
    #     "Total time FP (ms)": "#F39C12"          # Bright orange
    # }
    
    color_map = {
        "Total time keypoints (ms)": "#1f77b4",   # Blue
        "Total time FD (ms)": "#ff7f0e",          # Orange
        "Total time interpolation (ms)": "#2ca02c", # Green
        "Total time cost derivs (ms)": "#9467bd", # Purple
        "Total time BP (ms)": "#d62728",          # Red
        "Total time FP (ms)": "#8c564b"           # Brown
    }

    num_methods = len(dataframes_iLQR)
    bar_data = []

    # Compute mean values for each category per method
    for df in dataframes_iLQR:
        means = df[plot_categories].mean()
        bar_data.append(means.values)
    
    bar_data = np.array(bar_data)  # Shape: (num_methods, num_plot_categories)

    # Plot
    fig, ax = plt.subplots(figsize=(8, 6))
    
    bar_width = 0.5
    indices = np.arange(num_methods)
    
    bottoms = np.zeros(num_methods)
    for i, category in enumerate(plot_categories):
        values = bar_data[:, i]
        ax.bar(indices, values, bar_width, bottom=bottoms,
               label=category, color=color_map[category])
        # ax.bar(indices, values, bar_width, bottom=bottoms, label=category, color=colors[i])
        bottoms += values

    ax.set_ylabel('Average Time per Run (ms)')
    ax.set_title('Average Timing Breakdown per iLQR Method')
    ax.set_xticks(indices)
    ax.set_xticklabels(names)
    # ax.legend(loc='upper right', bbox_to_anchor=(1.35, 1.0))
    ax.legend()

    plt.tight_layout()
    plt.show()

    
def plot_openloop_data(names, dataframes_iLQR, task):
    
    # graphs_to_plot = ['Cost reduction', 'Optimisation time (ms)', 'Optimisation time (ms)', 'Number iterations', 'Average percent derivs']
    # columns_per_graph = [['Cost reduction'], 
    #                      ['Optimisation time (ms)', 'Average time derivs (ms)', 'Average time BP (ms)', 'Average time FP (ms)'],
    #                      ['Optimisation time (ms)'],
    #                      ['Number iterations'],
    #                      ['Average percent derivs']]
    
    graphs_to_plot = ['Optimisation time (ms)', 'Cost reduction',  'Final cost', 'Number iterations', 'Average percent derivs']
    columns_per_graph = [['Optimisation time (ms)'],
                         ['Cost reduction'], 
                         ['Final cost'],
                         ['Number iterations'],
                         ['Average percent derivs']]
    
    generate_plots_confidence(names, dataframes_iLQR, graphs_to_plot, columns_per_graph, task)
    
def generate_plots_confidence(names, dataframes_iLQR, graphs, columns_per_graph, task):
    
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
            
            # Put raw data as points on the graph
            for j in range(num_iLQR_methods):
                axes[i].scatter([x[j]] * num_data_rows, dataframes_iLQR[j][graph_name], color='black', s=10, alpha=0.5)
            
    # Need to sort methods into particular order, the order should be 
    # SI1, SI5, SI1000, contact_change


            
    # Print the data in latex code format to copy and paste into paper
    # Format is final cost, then average optimisation time, then percentage derivatives
    print(f'{task}', end=' ')
    for i in range(len(names)):
        
        #OT no CI and CR with CI
        print(f'& {means[i,0]/1000:.2f}', end=' ')
        
        # With confidence intervals
        # print(f'& {means[i,0]/1000:.2f} $\pm$ {confidence_intervals[i,0]/1000:.2f}', end=' ')
        print(f'& {means[i,1]:.2f} $\pm$ {confidence_intervals[i,1]:.2f}', end=' ')
        # print(f'& {means[i,3]:.2f} $\pm$ {confidence_intervals[i,2]:.2f}', end=' ')
        
        # if run_mode != "1_1":
        #     print(f'& {means[i,2]:.2f}', end=' ')
        # print(f'& {means[i,3]:.2f}', end=' ')
        

    print(f'\\\\')
    
    global cost_reductions_all, optimisation_times_all, number_iterations_all

    for m, df in enumerate(dataframes_iLQR):
        cost_reductions_all[m].extend(df["Cost reduction"].values)
        optimisation_times_all[m].extend(df["Optimisation time (ms)"].values)
        number_iterations_all[m].extend(df["Number iterations"].values)

    
    # Save data per task per trial for all methods to array so we can average over all tasks
    
    # print(dataframes_iLQR[0][:]['Cost reduction'].values)
    
    # cost_reductions = np.append(cost_reductions, )
    
    # cost_reductions.append(dataframes_iLQR[:]['Cost reduction'].values)
    # print(f' cost reduction shape inside plot function : {np.array(cost_reductions).shape}')
    # optimisation_times.append(dataframes_iLQR[:]['Optimisation time (ms)'].values)
    # number_iterations.append(dataframes_iLQR[:]['Number iterations'].values)
    
    # Save data per task for all methods to array so we can average over all tasks 
    # cost_reductions.append(means[:,1])
    # optimisation_times.append(means[:,0])
    # number_iterations.append(means[:,3])
            
            
    # Print the data in table format for easy transferance to the paper
    # print(f'Methods: {names}')
    # for i in range(len(graphs)):
    #     print(f'{graphs[i]}', end=' ')
    #     for j in range(len(names)):
            
    #         print(f'{means[j,i]:.2f}', end=' ')
    #         print(f' +- {confidence_intervals[j,i]:.2f}', end=' ')
            
    #     print('')

    # # Print average time per iteration for each method (means[0,2] / means[2,2])
    # print('Average time per iteration ', end='')
    # for j in range(len(names)):
    #     print(f'{means[j,0]/means[j,2]:.2f}', end=' ')
    #     print(f' +- {confidence_intervals[j,0]/means[j,2]:.2f}', end=' ')

    # print('')
    
    
    plt.xticks(x, names, rotation=45, ha='right') 
    figure_title = task
    fig.suptitle(figure_title, fontsize = 20)
    
    global show_plot
    if(show_plot):
        plt.show()
    
def make_names(iLQR_yaml_files):
    names = []
    
    for i in range(len(iLQR_yaml_files)):
        names.append(iLQR_yaml_files[i]['keypoint_name'])
    
    return names
    
def load_raw_data(task):
    # Load all iLQR algorithms
    global base_dir
    global run_mode
    global iterations
    
    run_iterations = run_mode + "_" + iterations
    
    dataframes_iLQR = []
    yamlfiles_iLQR = []
    
    if paper_data_folder:
        current_dir = base_dir + "/../PaperData/new_paper_data/Openloop_fixed_6_iters"
    else:
        # current_dir = base_dir + "/iLQR"
        # current_dir = base_dir + "/iLQR_1_1_results"
        current_dir = "iLQR_4_10_results"

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
        
        dataframes_iLQR.append(df)
        
        # Load yaml file
        file_name_yaml = folder_path + "/summary.yaml"
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            yamlfiles_iLQR.append(yaml_data)  # Add YAML data to the list
            
    names = make_names(yamlfiles_iLQR)
        
        
    return names, dataframes_iLQR, yamlfiles_iLQR
    
if __name__ == "__main__":
    
    # plot_CR_versus_iteration_timings("acrobot")
    # plot_CR_versus_iteration_timings("box_sweep")
    # plot_CR_versus_iteration_timings("push_ncl")
    # plot_CR_versus_iteration_timings("push_lcl")
    # plot_CR_versus_iteration_timings("push_mcl")
    # plot_CR_versus_iteration_timings("impact_large_box")
    main()
