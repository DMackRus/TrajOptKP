import os
import pandas as pd
import matplotlib.pyplot as plt


def collate_data(folder_path):
    # Collect results
    results = []

    for filename in os.listdir(folder_path):
        if filename.endswith(".csv"):
            filepath = os.path.join(folder_path, filename)
            df = pd.read_csv(filepath)

            # Compute means for all metrics
            mean_mse = df["MSE"].mean()
            mean_frob = df["Frobenius Error"].mean()
            mean_elem = df["Elementnorm Error"].mean()
            mean_max_abs = df["Max Error (abs)"].mean()
            mean_max_rel = df["Max Error (rel)"].mean()
            mean_deriv = df["% Derivatives"].mean()
            mean_cost_reduction = df["Cost Reduction"].mean()

            results.append({
                "method": os.path.splitext(filename)[0],
                "MSE": mean_mse,
                "Frobenius Error": mean_frob,
                "Elementnorm Error": mean_elem,
                "Max Error (abs)": mean_max_abs,
                "Max Error (rel)": mean_max_rel,
                "Cost Reduction": mean_cost_reduction,
                "% Derivatives": mean_deriv
            })

    # Convert results into DataFrame
    summary_df = pd.DataFrame(results)
    return summary_df

def plot_error_metrics(folder_path):
    
    summary_df = collate_data(folder_path)
    
    print(summary_df)

    # Sort the data into this order via method names
    order = ["SI2", "SI5", "SI20", "SI1000", "contact_change", "contact_change_dyn"]
    summary_df["sort_key"] = summary_df["method"].map(
        lambda x: order.index(x) if x in order else len(order)
    )
    summary_df = summary_df.sort_values("sort_key").drop(columns="sort_key").reset_index(drop=True)

    # Print method names and summary line for LaTeX-style output
    for i in range(len(summary_df)):
        print(f'{summary_df["method"][i]}', end=' ')
    print()
    print(f'{folder_path}', end=' ')
    for i in range(len(summary_df)):
        if summary_df["method"][i] == "SI2":
            continue
        print(
            f'& {summary_df["MSE"][i]:.3f} '
            f'& {summary_df["Frobenius Error"][i]:.3f} '
            f'& {summary_df["Elementnorm Error"][i]:.3f} '
            f'& {summary_df["Max Error (abs)"][i]:.3f} '
            f'& {summary_df["Max Error (rel)"][i]:.3f} '
            f'& {summary_df["% Derivatives"][i]:.1f}',
            end=''
        )
    print(' \\\\')

    # Plotting
    # metrics = ["MSE", "Frobenius Error", "Elementnorm Error", "Max Error (abs)", "Max Error (rel)"]
    # fig, axes = plt.subplots(1, len(metrics), figsize=(5 * len(metrics), 5))

    # for i, metric in enumerate(metrics):
    #     ax = axes[i]
    #     for _, row in summary_df.iterrows():
    #         ax.scatter(row["% Derivatives"], row[metric], label=row["method"])
    #     ax.set_xlabel("% Derivatives")
    #     # ax.set_ylabel(metric)
    #     ax.set_title(f"{metric} vs % Derivatives")
    #     ax.legend()
        
    # # Add title to the figure for task name
    # fig.suptitle(f"Error Metrics for {folder_path}", fontsize=16)

    # plt.tight_layout()
    # plt.show()
    
    # metrics = ["MSE", "Frobenius Error", "Elementnorm Error", "Max Error (abs)", "Max Error (rel)"]
    metrics = ["MSE", "Elementnorm Error", "Max Error (abs)"]
    
    # Plot Error versus cost reduction
    fig, axes = plt.subplots(2, len(metrics), figsize=(5 * len(metrics), 10))
    print(axes.shape)
    for i, metric in enumerate(metrics):
        ax = axes[0, i]
        for _, row in summary_df.iterrows():
            if "contact" in row["method"]:
                ax.scatter(row[metric], row["Cost Reduction"], label=row["method"], marker='x')
            else:
                ax.scatter(row[metric], row["Cost Reduction"], label=row["method"], marker='o')
            
        ax.set_xlabel(f"{metric}")
        # ax.set_ylabel(metric)
        ax.set_title(f"Cost Reduction vs {metric}")
        ax.legend()
        
    # Make similar plots for % Derivatives versus errors
    for i, metric in enumerate(metrics):
        ax = axes[1, i]
        for _, row in summary_df.iterrows():
            ax.scatter(row[metric], row["% Derivatives"], label=row["method"])
        ax.set_xlabel(f"{metric}")
        # ax.set_ylabel(metric)
        ax.set_title(f"% Derivatives vs {metric}")
        ax.legend()
        
    fig.suptitle(f"Error Metrics for {folder_path}", fontsize=16)
        
    plt.tight_layout()
    plt.show()
    
    
    # Compute correlations between the different error metrics and Cost reduction
    metrics_to_correlate = ["MSE", "Elementnorm Error", "Max Error (abs)"]
    corr_results = {}

    for metric in metrics_to_correlate:
        corr_value = summary_df[metric].corr(summary_df["Cost Reduction"], method='pearson')
        corr_results[metric] = corr_value

    print("\nCorrelation between error metrics and Cost Reduction:")
    for metric, corr_value in corr_results.items():
        print(f"  {metric:20s}: {corr_value:.3f}")
    # corr_matrix = summary_df.corr(method='pearson')
    # print(corr_matrix)
    
def print_correlation_CR_versus_errors(tasks_folder, error_names):
    
    # Compute average correlations over all tasks
    correlations = {error_name: [] for error_name in error_names}
    
    for task in tasks_folder:
        
        summary_df = collate_data(tasks_folder[task])
        
        # print(f"\nCorrelation between Cost Reduction and error metrics for {task_name}:")
        for error_name in error_names:
            corr_value = summary_df[error_name].corr(summary_df["Cost Reduction"], method='pearson')
            # print(f"  {error_name:20s}: {corr_value:.3f}")
            
        # Print {task_name} & {corr1} & {corr2} & {corr3} \\
        print(f'{task}', end=' ')
        for error_name in error_names:
            corr_value = summary_df[error_name].corr(summary_df["Cost Reduction"], method='pearson')
            print(f'& {corr_value:.3f} ', end=' ')
        print('\\\\')
        
        correlations = {error_name: correlations[error_name] + [summary_df[error_name].corr(summary_df["Cost Reduction"], method='pearson')] for error_name in error_names}
        
    print("\nAverage Correlations between Cost Reduction and error metrics over all tasks:")
    for error_name in error_names:
        avg_corr = sum(correlations[error_name]) / len(correlations[error_name])
        print(f"  {error_name:20s}: {avg_corr:.3f}")
    

def print_error_percentage_derivs(tasks_folder, error_names):
    
    order = ["SI2", "SI20", "contact_change", "contact_change_dyn"]
    
    for task in tasks_folder:
        
        summary_df = collate_data(tasks_folder[task])
        
        print(f'{task}', end=' ')
        
        
        # Loop through key-point methds , only print those in order
        for method in order:
            row = summary_df[summary_df["method"] == method]
            if not row.empty:
                
                
                #Print MMSE (scale down by ten thousand)
                error_value = row["Max Error (abs)"].values[0]
                error_value = error_value / 10000
                print(f'& {error_value:.3f} ', end=' ')
                
                #Print EMSE
                error_value = row["Elementnorm Error"].values[0]
                error_value = error_value * 100
                print(f'& {error_value:.3f} ', end=' ')
                
                perc_deriv = row["% Derivatives"].values[0]
                print(f'& {perc_deriv:.1f} ', end=' ')
                
        print('\\\\')
        
def plot_error_versus_CR(tasks, error_name):
    summary_dfs = [collate_data(task) for task in tasks]
    
    order = ["SI2", "SI5", "SI10", "SI20", "SI100", "SI200", "SI500", "SI1000", "contact_change", "contact_change_dyn"]
    
    for i in range(len(summary_dfs)):
        summary_dfs[i]["sort_key"] = summary_dfs[i]["method"].map(
            lambda x: order.index(x) if x in order else len(order)
        )
        summary_dfs[i] = summary_dfs[i].sort_values("sort_key").drop(columns="sort_key").reset_index(drop=True)
    
    fig, axes = plt.subplots(1, 1, figsize=(6, 6)) # sharey = True
    
    for i, df in enumerate(summary_dfs):
        for _, row in summary_dfs[i].iterrows():
            if "contact" in row["method"]:
                axes.scatter(row[error_name], row["Cost Reduction"], label=row["method"], marker='x')
            else:
                axes.scatter(row[error_name], row["Cost Reduction"], label=row["method"], marker='o')
                
        axes.set_xlabel(f"{error_name}")
        axes.set_ylabel("Cost Reduction")
        axes.legend()
        axes.set_title(tasks[i])
            
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.suptitle(f"Cost Reduction vs {error_name}")
    # plt.legend()
    # plt.show()
    
    # plt.savefig(f"cost_reduction_vs_{error_name.replace(' ', '_')}.png")
    plt.savefig(f"cost_reduction_vs_{error_name.replace(' ', '_')}.svg", format="svg")
    # ax.set_title(f"Cost Reduction vs {metric}")



if __name__ == "__main__":
    
    # plot_error_versus_CR(["pushing_low_clutter_1000_3", "box_sweep_1500_3"], "Elementnorm Error")
    plot_error_versus_CR(["walker_run_200_3"], "Elementnorm Error")
    # plot_error_versus_CR("box_sweep_1500_3", "Elementnorm Error")
    
    # Dictionary of folder names and task names
    tasks_folder = {"Acrobot": "acrobot_2000_3",
                    "Walker": "walker_run_200_3",
                    "Box Sweep": "box_sweep_1500_3",
                    "Impact Large Box 8": "impact_large_box_2000_8",
                    "Pushing No Clutter 3": "pushing_no_clutter_1000_3",
                    "Pushing Low Clutter 3": "pushing_low_clutter_1000_3",
                    "Pushing Moderate Clutter 3": "pushing_moderate_clutter_1000_3",
    }   
    # tasks_folder = {
    #         "Box Sweep": "box_sweep_1500_3",
    #         "Piston Block": "piston_block_1000_3",
    #         "Pushing No Clutter 3": "pushing_no_clutter_1000_3",
    #         "Impact Large Box 8": "impact_large_box_2000_8",
    # }   

    # error_metrics = ["MSE", "Max Error (abs)", "Elementnorm Error"]
    # print_correlation_CR_versus_errors(tasks_folder, error_metrics)
    
    # error_metrics = ["Max Error (abs)", "Elementnorm Error"]
    # print_error_percentage_derivs(tasks_folder, error_metrics)

    # for task in tasks_folder:
    #     plot_error_metrics(tasks_folder[task])
