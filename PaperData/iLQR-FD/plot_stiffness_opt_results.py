import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Set path to the outer directory, e.g., path/to/root/solref[0]
# outer_path = "../stiffness_tests/pushing_no_clutter_solref[0]"
# task = "pushing_no_clutter"
# outer_path = "../stiffness_tests/box_sweep_solref[0]"
# outer_path = "../stiffness_tests/impact_large_box_solref[0]"
task = "push ncl"
outer_path = "stiffness_tests/pushing_no_clutter_solref[0]"
# outer_path = "../stiffness_tests/solimp[0]"
# outer_path = "../stiffness_tests_6_iters_100_tasks_box_sweep/solimp[0]"
outer_folder = os.path.basename(outer_path)

# Use this for x-axis label and title
x_axis_label = outer_folder
plot_title = f"Cost Reduction vs {outer_folder} for different methods"

# Gather all x values (i.e., subdirectories inside outer folder)
x_dirs = sorted([d for d in os.listdir(outer_path) if os.path.isdir(os.path.join(outer_path, d))])

# Initialise: method -> {x_val: (mean, lower_ci, upper_ci)}
cost_reduction_data = {}
final_cost_data = {}
optimisation_time_data = {}

z_threshold = 1000

for x_val in x_dirs:
    x_val_path = os.path.join(outer_path, x_val)
    method_dirs = [d for d in os.listdir(x_val_path) if os.path.isdir(os.path.join(x_val_path, d))]

    for method in method_dirs:
        summary_file = os.path.join(x_val_path, method, "summary.csv")
        if os.path.isfile(summary_file):
            try:
                df = pd.read_csv(summary_file)
                if "Cost reduction" in df.columns:
                    values = df["Cost reduction"].dropna()
                    filtered = values[np.abs((values - values.mean()) / values.std(ddof=1)) < z_threshold]
                    n = len(values)
                    if n > 0:
                        mean = filtered.mean()
                        std = filtered.std(ddof=1)
                        margin = 1.96 * (std / np.sqrt(n)) if n > 1 else 0
                        lower = mean - margin
                        upper = mean + margin
                        cost_reduction_data.setdefault(method, {})[x_val] = (mean, lower, upper)
                        
                if "Final cost" in df.columns:
                    values = df["Final cost"].dropna()
                    filtered = values[np.abs((values - values.mean()) / values.std(ddof=1)) < z_threshold]
                    # print(f'Processing {summary_file} for method {method} with x_val {x_val}')
                    # print(f'Values: {values}')
                    n = len(values)
                    if n > 0:
                        mean = filtered.mean()
                        # print(f'Mean: {mean}')
                        std = filtered.std(ddof=1)
                        margin = 1.96 * (std / np.sqrt(n)) if n > 1 else 0
                        lower = mean - margin
                        upper = mean + margin
                        final_cost_data.setdefault(method, {})[x_val] = (mean, lower, upper)
                        
            except Exception as e:
                print(f"Warning: Failed to read {summary_file}: {e}")

# Plotting
fig, axes = plt.subplots(1, 1, sharex=True, figsize=(8, 2*4))
colors = plt.cm.get_cmap('tab10')

# First plot - Cost reduction
for i, (method, results) in enumerate(cost_reduction_data.items()):
    means = []
    lowers = []
    uppers = []
    for x in x_dirs:
        mean, lower, upper = results.get(x, (np.nan, np.nan, np.nan))
        means.append(mean)
        lowers.append(lower)
        uppers.append(upper)

    axes.plot(x_dirs, means, label=method, color=colors(i), marker='o')
    axes.fill_between(x_dirs, lowers, uppers, color=colors(i), alpha=0.2)
    
axes.set_ylabel("Average Cost Reduction")
axes.grid(True)
    
# for i, (method, results) in enumerate(final_cost_data.items()):
#     means = []
#     lowers = []
#     uppers = []
#     for x in x_dirs:
#         mean, lower, upper = results.get(x, (np.nan, np.nan, np.nan))
#         means.append(mean)
#         lowers.append(lower)
#         uppers.append(upper)

#     axes[1].plot(x_dirs, means, label=method, color=colors(i), marker='o')
#     axes[1].fill_between(x_dirs, lowers, uppers, color=colors(i), alpha=0.2)
    
# axes[1].set_ylabel("Average Final Cost")
# axes[1].grid(True)
    

fig.suptitle(task, fontsize = 20)

plt.xlabel(x_axis_label)
plt.legend()
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()