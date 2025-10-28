# import os
# import pandas as pd
# import matplotlib.pyplot as plt

# def plot_error_metrics(folder_path):
#     # Collect results
#     results = []

#     for filename in os.listdir(folder_path):
#         if filename.endswith(".csv"):
#             filepath = os.path.join(folder_path, filename)
#             df = pd.read_csv(filepath)

#             # Compute means
#             mean_mse = df["MSE"].mean()
#             mean_rms = df["RMS"].mean()
#             mean_max_error = df["Max Error"].mean()
#             mean_deriv = df["% Derivatives"].mean()

#             results.append({
#                 "method": os.path.splitext(filename)[0],
#                 "MSE": mean_mse,
#                 "RMS": mean_rms,
#                 "Max Error": mean_max_error,
#                 "% Derivatives": mean_deriv
#             })

#     # Convert results into DataFrame for convenience
#     summary_df = pd.DataFrame(results)
    
#     # print(summary_df)

#     # Plotting
#     fig, axes = plt.subplots(1, 3, figsize=(15, 5))

#     # Scatter plots
#     axes[0].scatter(summary_df["% Derivatives"], summary_df["MSE"])
#     axes[1].scatter(summary_df["% Derivatives"], summary_df["RMS"])
#     axes[2].scatter(summary_df["% Derivatives"], summary_df["Max Error"])

#     # Set labels and titles
#     axes[0].set_xlabel("% Derivatives")
#     axes[0].set_ylabel("MSE")
#     axes[0].set_title("MSE vs % Derivatives")

#     axes[1].set_xlabel("% Derivatives")
#     axes[1].set_ylabel("RMS")
#     axes[1].set_title("RMS vs % Derivatives")

#     axes[2].set_xlabel("% Derivatives")
#     axes[2].set_ylabel("Max Error")
#     axes[2].set_title("Max Error vs % Derivatives")
    
#     #Sort the data into this order via method names
#     order = ["SI2", "SI5", "SI20", "SI1000", "contact_change", "contact_change_dyn"]
    
#     summary_df["sort_key"] = summary_df["method"].map(
#         lambda x: order.index(x) if x in order else len(order)
#     )

#     # Sort and drop the helper column
#     summary_df = summary_df.sort_values("sort_key").drop(columns="sort_key").reset_index(drop=True)
    
#     # Print method names
#     for i in range(len(summary_df)):
#         print(f'{summary_df["method"][i]}', end=' ')
    
#     print()
#     print(f'{task}', end=' ')
#     for i in range(len(summary_df)):
#         if summary_df["method"][i] in ["SI2"]:
#             continue
#         # print(f'& {summary_df["MSE"][i]:.6f} & {summary_df["RMS"][i]:.6f} & {summary_df["Max Error"][i]:.6f} & {summary_df["% Derivatives"][i]:.6f}', end = '')
#         print(f'& {summary_df["RMS"][i]:.3f} & {summary_df["Max Error"][i]:.2f} & {summary_df["% Derivatives"][i]:.1f}', end = '')
        
#     print(f'\\\\')

#     # Add legend (shared across plots)
#     for ax in axes:
#         for i, row in summary_df.iterrows():
#             ax.scatter(row["% Derivatives"], row[ax.get_ylabel()], label=row["method"])
#         ax.legend()

#     plt.tight_layout()
#     plt.show()


# tasks = ["acrobot_1000_5", "impact_large_box_2000_8", "pushing_no_clutter_2000_3", "pushing_low_clutter_2000_3"]

# for task in tasks:
#     plot_error_metrics(task)

# # Example usage:
# # plot_error_metrics("acrobot_1000_5")
# # plot_error_metrics("impact_large_box_2000_8")
# # plot_error_metrics("walker_run_200_3")
# # plot_error_metrics("pushing_no_clutter_2000_3")


import os
import pandas as pd
import matplotlib.pyplot as plt

def plot_error_metrics(folder_path):
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

            results.append({
                "method": os.path.splitext(filename)[0],
                "MSE": mean_mse,
                "Frobenius Error": mean_frob,
                "Elementnorm Error": mean_elem,
                "Max Error (abs)": mean_max_abs,
                "Max Error (rel)": mean_max_rel,
                "% Derivatives": mean_deriv
            })

    # Convert results into DataFrame
    summary_df = pd.DataFrame(results)

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
    metrics = ["MSE", "Frobenius Error", "Elementnorm Error", "Max Error (abs)", "Max Error (rel)"]
    fig, axes = plt.subplots(1, len(metrics), figsize=(5 * len(metrics), 5))

    for i, metric in enumerate(metrics):
        ax = axes[i]
        for _, row in summary_df.iterrows():
            ax.scatter(row["% Derivatives"], row[metric], label=row["method"])
        ax.set_xlabel("% Derivatives")
        # ax.set_ylabel(metric)
        ax.set_title(f"{metric} vs % Derivatives")
        ax.legend()
        
    # Add title to the figure for task name
    fig.suptitle(f"Error Metrics for {folder_path}", fontsize=16)

    plt.tight_layout()
    plt.show()


# Example usage
tasks = [
    "acrobot_1000_5",
    "pushing_no_clutter_2000_3",
    "impact_large_box_2000_8",
    "walker_run_200_3",
    "pushing_low_clutter_2000_3",
    "pushing_moderate_clutter_2000_3"
]

for task in tasks:
    plot_error_metrics(task)
