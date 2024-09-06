import matplotlib.pyplot as plt
import numpy as np
import os

def plot_open_loop_data(task_name, method_names, opt_times, cost_reductions, num_iterations, avg_percent_derivs):
    fig, axs = plt.subplots(4, figsize = (8, 15))
    fig.suptitle('optimization time and cost reduction for ' + task_name)
    axs[0].set_ylabel('optimization time (s)')
    axs[0].plot(method_names, opt_times, 'o')
    axs[1].set_ylabel('cost reduction')
    axs[1].plot(method_names, cost_reductions, 'o')
    axs[2].set_ylabel('number of iterations')
    axs[2].plot(method_names, num_iterations, 'o')
    axs[3].set_ylabel('percent of derivs')
    axs[3].plot(method_names, avg_percent_derivs, 'o')
    plt.savefig(task_name + '.png')
    plt.show()

def plot_cheetah(task_name, method_names, opt_times, final_costs, num_iterations):
    fig, axs = plt.subplots(3, figsize = (8, 10))
    fig.suptitle('optimization time and final cost for ' + task_name)
    axs[0].set_ylabel('optimization time (s)')
    axs[0].plot(method_names, opt_times, 'o')
    axs[1].set_ylabel('final cost')
    axs[1].plot(method_names, final_costs, 'o')
    axs[2].set_ylabel('number of iterations')
    axs[2].plot(method_names, num_iterations, 'o')
    plt.savefig(task_name + '.png')
    plt.show()

def save_data_open_loop(method_names, opt_times, cost_reductions, avg_percent_derivs, num_iterations, task_name):

    # Specify the path of the directory
    directory_path = 'data/' + task_name

    # Check if the directory exists
    if not os.path.exists(directory_path):
        # If the directory does not exist, create it
        os.makedirs(directory_path)
        print(f"Directory '{directory_path}' created.")
    else:
        print(f"Directory '{directory_path}' already exists.")

    for i, method in enumerate(method_names):
        file_path = directory_path + '/' + method + '_data.npz'

        np.savez(file_path, method_name=method, opt_time=opt_times[i], cost_reduction=cost_reductions[i],
                  avg_percent_derivs=avg_percent_derivs[i], num_iterations=num_iterations[i],
                  time_per_iteration=opt_times[i]/num_iterations[i])

def load_data(task_name, method_names):

    for method in method_names:
        file_path = 'data/' + task_name + '/' + method + '_data.npz'
        data = np.load(file_path)
        print(data['method_name'])
        print(data['opt_time'])
        print(data['cost_reduction'])
        print(data['avg_percent_derivs'])
        print(data['num_iterations'])
        print(data['time_per_iteration'])


if __name__ == "__main__":
    #load mini cheetah data
    data = []

    # methods = ["baseline", "adapJerk_2", "IE_2", "adapJerk_1-5-10", "SI_1-5-10", "IE_1-5-10"]
    # file_names = ["baseline", "adaptiveJerk_2", "iterativeError_2", "adapJerk_1-5-10", "SI_1_5_10", "iterError_1-5-10"]

    # # data.append(np.genfromtxt('cheetah_data/baseline_data.csv', delimiter=','))
    # # data.append(np.genfromtxt('cheetah_data/adaptiveJerk_2_data.csv', delimiter=','))
    # # data.append(np.genfromtxt('cheetah_data/iterativeError_2_data.csv', delimiter=','))

    # for i in range(len(file_names)):
    #     data.append(np.genfromtxt('cheetah_data/' + file_names[i] + '_data.csv', delimiter=','))


    # print(data)
    # final_opt_times = []
    # final_costs = []
    # num_iterations = []

    # for i in range(len(data)):
    #     final_opt_times.append(data[i][0])
    #     final_costs.append(data[i][1])
    #     num_iterations.append(data[i][2])


    # plot_cheetah('mini cheetah', file_names, final_opt_times, final_costs, num_iterations)

    data = np.load('data/kinova_lift/optData.npz')
    print(data['method_names'])
    print(data['opt_times'])
    print(data['cost_reductions'])

