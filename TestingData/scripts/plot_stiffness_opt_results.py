# import os
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # Set path to the outer directory, e.g., path/to/root/solref[0]
# outer_path = "../stiffness_tests/solref[0]"
# outer_folder = os.path.basename(outer_path)

# # Use this for x-axis label and title
# x_axis_label = outer_folder
# plot_title = f"Cost Reduction vs {outer_folder} for different methods"

# # Gather all x values (i.e., subdirectories inside outer folder)
# x_dirs = sorted([d for d in os.listdir(outer_path) if os.path.isdir(os.path.join(outer_path, d))])

# # Initialise: method -> {x_value: (mean, std_error)}
# method_data = {}

# for x_val in x_dirs:
#     x_val_path = os.path.join(outer_path, x_val)
#     method_dirs = [d for d in os.listdir(x_val_path) if os.path.isdir(os.path.join(x_val_path, d))]

#     for method in method_dirs:
#         summary_file = os.path.join(x_val_path, method, "summary.csv")
#         if os.path.isfile(summary_file):
#             try:
#                 df = pd.read_csv(summary_file)
#                 if "Cost reduction" in df.columns:
#                     values = df["Cost reduction"].dropna()
#                     mean = values.mean()
#                     sem = values.std(ddof=1) / np.sqrt(len(values)) if len(values) > 1 else 0.0
#                     method_data.setdefault(method, {})[x_val] = (mean, sem)
#             except Exception as e:
#                 print(f"Warning: Failed to read {summary_file}: {e}")

# # Plotting
# plt.figure(figsize=(10, 6))

# for method, results in method_data.items():
#     means = [results.get(x, (np.nan, 0))[0] for x in x_dirs]
#     errors = [results.get(x, (np.nan, 0))[1] for x in x_dirs]
#     plt.errorbar(x_dirs, means, yerr=errors, label=method, marker='o', capsize=5)

# plt.xlabel(x_axis_label)
# plt.ylabel("Average Cost Reduction")
# plt.title(plot_title)
# plt.legend()
# plt.grid(True)
# plt.xticks(rotation=45)
# plt.tight_layout()
# plt.show()

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Set path to the outer directory, e.g., path/to/root/solref[0]
outer_path = "../stiffness_tests/solref[0]"
# outer_path = "../stiffness_tests/solimp[0]"
# outer_path = "../stiffness_tests_6_iters_100_tasks_box_sweep/solimp[0]"
outer_folder = os.path.basename(outer_path)

# Use this for x-axis label and title
x_axis_label = outer_folder
plot_title = f"Cost Reduction vs {outer_folder} for different methods"

# Gather all x values (i.e., subdirectories inside outer folder)
x_dirs = sorted([d for d in os.listdir(outer_path) if os.path.isdir(os.path.join(outer_path, d))])

# Initialise: method -> {x_val: (mean, lower_ci, upper_ci)}
method_data = {}

for x_val in x_dirs:
    x_val_path = os.path.join(outer_path, x_val)
    method_dirs = [d for d in os.listdir(x_val_path) if os.path.isdir(os.path.join(x_val_path, d))]

    for method in method_dirs:
        summary_file = os.path.join(x_val_path, method, "summary.csv")
        if os.path.isfile(summary_file):
            try:
                df = pd.read_csv(summary_file)
                if "Final cost" in df.columns:
                    values = df["Final cost"].dropna()
                    n = len(values)
                    if n > 0:
                        mean = values.mean()
                        std = values.std(ddof=1)
                        margin = 1.96 * (std / np.sqrt(n)) if n > 1 else 0
                        lower = mean - margin
                        upper = mean + margin
                        method_data.setdefault(method, {})[x_val] = (mean, lower, upper)
            except Exception as e:
                print(f"Warning: Failed to read {summary_file}: {e}")

# Plotting
plt.figure(figsize=(10, 6))
colors = plt.cm.get_cmap('tab10')

for i, (method, results) in enumerate(method_data.items()):
    means = []
    lowers = []
    uppers = []
    for x in x_dirs:
        mean, lower, upper = results.get(x, (np.nan, np.nan, np.nan))
        means.append(mean)
        lowers.append(lower)
        uppers.append(upper)

    plt.plot(x_dirs, means, label=method, color=colors(i), marker='o')
    plt.fill_between(x_dirs, lowers, uppers, color=colors(i), alpha=0.2)

plt.xlabel(x_axis_label)
plt.ylabel("Average Cost Reduction")
plt.title(plot_title)
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()