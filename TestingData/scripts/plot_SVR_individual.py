import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv
import os
import sys
import yaml

task_name = "push_mcl"
run_mode = "asynchronus" # openloop or asynchronus
trajec_num = 2


def main():
    global task_name
    global trajec_num
    global run_mode
    
    if(len(sys.argv) < 2):
        pass
    else:
        # argument 0 is the program name
        trajec_num = int(sys.argv[1])
    
    names, dataframes_iLQR_SVR = load_raw_data(run_mode, task_name, 1, 4)
    
    plot_individual_trajec_data(dataframes_iLQR_SVR, trajec_num)
    
    
def plot_individual_trajec_data(dataframes_iLQR_SVR, trajec_number):
    
    print(dataframes_iLQR_SVR[trajec_number].head())
    
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    
    axes[0].plot(dataframes_iLQR_SVR[trajec_number]['num dofs'])
    axes[0].set_ylabel("Number of Dofs in state vector")
    
    axes[1].plot(dataframes_iLQR_SVR[trajec_number]['Optimisation time (ms)'])
    axes[1].set_ylabel("Optimisation time of iteration")
    axes[1].set_xlabel("Iteration number")
    
    fig.suptitle("Pushing in moderate clutter, MPC for a single trajectory")
    # fig.x("Iteration number")
    plt.show()
    

def load_raw_data(run_mode, task_name, desired_threshold, desired_readd):
    # Loop through summary.yaml
    # find specific parametrisation you care about
    
    found_method = False
    directory_name = ""
    
    # Load all 100 .csv's
    # Load all iLQR_SVR data
    base_dir = "../iLQR_SVR"
    for folder in os.listdir(base_dir):
        
        # Only add the data if the task name is correct
        if task_name not in folder:
            continue
        
        # Only add the data if the run_mode is correct
        if run_mode not in folder:
            continue
        
        folder_path = os.path.join(base_dir, folder)
        file_path = folder_path + "/summary.csv"
        
         # Load yaml file
        file_name_yaml = folder_path + "/summary.yaml"
        yaml_data = []
        with open(file_name_yaml, 'r') as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)  # Load YAML data
            
        # If yaml file config matches what we want
        K_threshold = yaml_data["K_matrix_threshold"]
        num_dofs_readd = yaml_data['num_dofs_readd']
        
        if(num_dofs_readd == desired_readd):
            if(abs(desired_threshold - K_threshold) < 0.01):
                found_method = True 
                directory_name = folder_path
                break
            
            
    print(directory_name)
    
    # Load all 100 CSV's
    
    names = []
    dataframes_iLQR = []
    # yamlfiles_iLQR = []
    dataframes_iLQR_SVR = []
    # yamlfiles_iLQR_SVR = []
    
    # Loop through all 0 -> 99 csvs in folder_path
    for filename in os.listdir(directory_name):
        if os.path.isfile(os.path.join(folder_path, filename)):
            print(filename)
            
            # if ".csv" in filename:
            #     continue
            
            file_path = folder_path + "/" + filename
            

            df = pd.read_csv(file_path)

            
            
            dataframes_iLQR_SVR.append(df)
        
        
    return names, dataframes_iLQR_SVR


if __name__ == "__main__":
    main()