import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv
import os

def main():
    task_name = "push_nCl"
    directory = "../" + task_name
    all_method_file_names = list_files_in_directory(directory)


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
    final_data = sorted(final_data, key=lambda x: x[2])

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

    # 6 elements in data, final cost, opt time, % derivs, time derivs, time bp, time fp


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

def load_all_data(directory, file_names):

    # Load all the data
    all_data = []
    for file in file_names:
        file_path = directory + "/" + file
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