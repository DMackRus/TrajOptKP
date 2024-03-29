import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv
import os
import sys

task = "walker_plane"
task_num = -1

show_ind_trajecs = True


def main():
    global task
    global task_num
    
    if(len(sys.argv) < 2):
        pass
    else:
        # argument 0 is the program name
        task = sys.argv[1]
        if(len(sys.argv) >= 2):
            task_num = sys.argv[2]
            
    if(task_num != -1):
        plot_task(task, "AA_1_50", task_num)
    plot_summary(task)

    # 6 elements in data, final cost, opt time, % derivs, time derivs, time bp, time fp
    
def plot_task(task_name, method_name, task_number):
    
    
    directory = "../" + task_name + "/" + method_name
    file_name = directory + "/" + str(task_number) + ".csv"
    
    # load data
    data = np.array([genfromtxt(file_name, delimiter = ',')])
    data = data[0]
    data = data[1:]
    
    # Data headers are: 
    # opt time (ms), derivs time, bp time, fp time, & derivs, surprise, expected, new_cost
    
    fig, axs = plt.subplots(3, 3, sharex=False, figsize = (15, 12))

    # Overall optimisation time
    axs[0, 0].plot(data[:, 0])
    axs[0, 0].set_xlabel("Time")
    axs[0, 0].set_ylabel("Optimisation time (ms)")
    
    axs[1, 0].plot(data[:, 1])
    axs[1, 0].set_xlabel("Time")
    axs[1, 0].set_ylabel("Derivs time (ms)")
    
    axs[2, 0].plot(data[:, 4])
    axs[2, 0].set_xlabel("Time")
    axs[2, 0].set_ylabel("% Derivs")
    
    # ---- time fp, time bp -----
    axs[0, 1].plot(data[:, 3])
    axs[0, 1].set_xlabel("Time")
    axs[0, 1].set_ylabel("Time FP (ms)")
    
    axs[1, 1].plot(data[:, 2])
    axs[1, 1].set_xlabel("Time")
    axs[1, 1].set_ylabel("Time BP (ms)")
    
    # ----- Surprise, expected, new cost ------
    axs[0, 2].plot(data[:, 5])
    axs[0, 2].set_xlabel("Time")
    axs[0, 2].set_ylabel("Surprise")
    
    axs[1, 2].plot(data[:, 6])
    axs[1, 2].set_xlabel("Time")
    axs[1, 2].set_ylabel("Expected")
    axs[1, 2].set_ylim(bottom=0, top = 10)
    
    axs[2, 2].plot(data[:, 7])
    axs[2, 2].set_xlabel("Time")
    axs[2, 2].set_ylabel("New cost")
    
    
    
    
    
    

    # # Average optimisation times per iteration
    # ax2.plot(opt_times)
    # ax2.set_xticks(x_ticks)
    # ax2.set_xticklabels(methods)
    # ax2.set_xlabel("method name")
    # ax2.set_ylabel("Mean optimisation time (ms)")

    # # Mean percentage derivatives
    # ax3.plot(percent_derivs)
    # ax3.set_xticks(x_ticks)
    # ax3.set_xticklabels(methods)
    # ax3.set_xlabel("method name")
    # ax3.set_ylabel("Mean percent derivatives")

    # # set figure title
    figure_title = task_name + "_" + method_name + "_" + str(task_number)
    fig.suptitle(figure_title, fontsize = 20)

    plt.show()
    
    
def plot_summary(task_name):
    
    directory = "../" + task_name
    all_method_file_names = list_directories(directory)
    
    # load the data
    file_count = len(all_method_file_names)
    all_data = load_all_data(directory, all_method_file_names)

    method_names = extract_method_name(task_name, all_method_file_names)

    # Loop through all data and compute mean and standard deviation information
    mean_final_cost = []
    mean_opt_time = []
    mean_percent_derivs = []

    std_deviation = []
    for i in range(file_count):
        mean_, std_deviation_ = compute_mean_and_std_deviation(all_data[i])
        mean_final_cost.append(mean_[0])
        mean_opt_time.append(mean_[1])
        mean_percent_derivs.append(mean_[2])
        # mean.append(mean_)
        # std_deviation.append(std_deviation_)

    # Package all the data together and then sort from high to low for mean percent times
    final_data = [[method_names[0], mean_final_cost[0], mean_opt_time[0], mean_percent_derivs[0]]]
    for i in range(1, file_count):
        final_data.append([method_names[i], mean_final_cost[i], mean_opt_time[i], mean_percent_derivs[i]])

    #sort the data
    final_data = sorted(final_data, key=lambda x: x[3])

    # extract the data for plotting
    methods = [item[0] for item in final_data]
    final_costs = [item[1] for item in final_data]
    opt_times = [item[2] for item in final_data]
    percent_derivs = [item[3] for item in final_data]


    # -------- Plotting -----------------
    x_ticks = list(range(0, file_count))

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=False, figsize = (15, 12))

    # Final trajectory cost
    ax1.plot(final_costs)
    ax1.set_xticks(x_ticks)
    ax1.set_xticklabels(methods)
    ax1.set_xlabel("method name")
    ax1.set_ylabel("Final trajectory cost")
    # ax1.set_ylim(6, 15)

    # Average optimisation times per iteration
    ax2.plot(opt_times)
    ax2.set_xticks(x_ticks)
    ax2.set_xticklabels(methods)
    ax2.set_xlabel("method name")
    ax2.set_ylabel("Mean optimisation time (ms)")

    # Mean percentage derivatives
    ax3.plot(percent_derivs)
    ax3.set_xticks(x_ticks)
    ax3.set_xticklabels(methods)
    ax3.set_xlabel("method name")
    ax3.set_ylabel("Mean percent derivatives")

    # set figure title
    fig.suptitle(task_name, fontsize = 20)

    plt.show()
    


def compute_mean_and_std_deviation(data):
    size = len(data[1])
    mean = np.zeros(size)
    std_deviation = np.zeros(size)

    for i in range(size):
        mean[i] = np.mean(data[:,i])
        std_deviation[i] = np.std(data[:,i])

    return mean, std_deviation

def load_data_from_path(file_path):
    data = np.array([genfromtxt(file_path, delimiter = ',')])

    return data

def count_files_in_directory(directory_path):
    try:
        # List all files in the directory
        files = os.listdir(directory_path)

        # Count the number of files
        file_count = len(files)

        print(f'The number of files in {directory_path} is: {file_count}')

    except FileNotFoundError:
        print(f'The directory {directory_path} does not exist.')

    return file_count


def list_files_in_directory(directory):
    files_list = []
    for file_name in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, file_name)):
            files_list.append(file_name)
    return files_list

def list_directories(directory):
    directories = []
    for item in os.listdir(directory):
        if os.path.isdir(os.path.join(directory, item)):
            directories.append(item)
    return directories

def load_all_data(directory, method_names):

    # Load all the data
    all_data = []
    for folder in method_names:
        file_path = directory + "/" + folder + "/summary.csv"
        data = load_data_from_path(file_path)

        # drop top row as its header which are converted to Nan
        data = data[0]
        data = data[1:]
        all_data.append(data)

    return all_data

def extract_method_name(task_name, file_names):
    method_names = []
    keypoint_name = "magvel_change"

    for file in file_names:
        method = file.replace(task_name, "")
        method = method.replace("_" + keypoint_name + "_", "")
        method = method.replace("_testingData.csv", "")
        method_names.append(method)


    return method_names

if __name__ == "__main__":
    main()