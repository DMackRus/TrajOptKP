import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Directory containing the CSV files
task_name = "articulated_contact_NO_CONTACT"
data_dir = os.path.join("..", task_name)
csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]



for file in csv_files:
    filepath = os.path.join(data_dir, file)
    df = pd.read_csv(filepath)

    file_size = len(df.columns) - 2
    print(f'file size: {file_size}')
    coeffs = [1, 1, -file_size]
    for root in np.roots(coeffs):
        if root > 0:
            dim_state = int(root)
    print(f'dim_state: {dim_state}')

    # Extract x-axis (first column)
    x = df.iloc[:, 0]
    x_label = df.columns[0]

    # Plot 4x4 matrix (next 16 columns)
    fig1, axes1 = plt.subplots(dim_state, dim_state, figsize=(12, 10))
    fig1.suptitle(f'4x4 Matrix Plot - {file}')
    for i in range(dim_state**2):
        row, col = divmod(i, dim_state)
        ax = axes1[row, col]
        y = df.iloc[:, i + 1]
        label = df.columns[i + 1]
        ax.plot(x, y, label=label)
        ax.ticklabel_format(style='plain', axis='y')
        y_min, y_max = y.min(), y.max()
        if abs(y_max - y_min) < 1e-4:
            mid = 0.5 * (y_max + y_min)
            ax.set_ylim(mid - 5e-5, mid + 5e-5)
        ax.legend()
        ax.set_xlabel(x_label)

    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for suptitle

    # Plot 4x1 matrix (last 4 columns)
    fig2, axes2 = plt.subplots(dim_state, 1, figsize=(8, 10))
    fig2.suptitle(f'6x1 Matrix Plot - {file}')
    for i in range(dim_state):
        ax = axes2[i]
        y = df.iloc[:, i + (dim_state**2) + 1]
        label = df.columns[i + 17]
        ax.plot(x, y, label=label)
        ax.ticklabel_format(style='plain', axis='y')
        y_min, y_max = y.min(), y.max()
        if abs(y_max - y_min) < 1e-4:
            mid = 0.5 * (y_max + y_min)
            ax.set_ylim(mid - 5e-5, mid + 5e-5)
        ax.legend()
        ax.set_xlabel(x_label)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    fig1.savefig(os.path.join(task_name, f"{file}_A.png"))
    fig2.savefig(os.path.join(task_name, f"{file}_B.png"))