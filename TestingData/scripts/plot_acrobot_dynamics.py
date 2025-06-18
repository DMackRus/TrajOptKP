import os
import pandas as pd
import matplotlib.pyplot as plt

# Directory containing the CSV files
task_name = "pendubot"
data_dir = os.path.join("..", task_name)
csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

for file in csv_files:
    filepath = os.path.join(data_dir, file)
    df = pd.read_csv(filepath)

    # Extract x-axis (first column)
    x = df.iloc[:, 0]
    x_label = df.columns[0]

    # Plot 4x4 matrix (next 16 columns)
    fig1, axes1 = plt.subplots(4, 4, figsize=(12, 10))
    fig1.suptitle(f'4x4 Matrix Plot - {file}')
    for i in range(16):
        row, col = divmod(i, 4)
        ax = axes1[row, col]
        y = df.iloc[:, i + 1]
        label = df.columns[i + 1]
        ax.plot(x, y, label=label)
        ax.legend()
        ax.set_xlabel(x_label)

    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for suptitle

    # Plot 4x1 matrix (last 4 columns)
    fig2, axes2 = plt.subplots(4, 1, figsize=(8, 10))
    fig2.suptitle(f'4x1 Matrix Plot - {file}')
    for i in range(4):
        ax = axes2[i]
        y = df.iloc[:, i + 17]
        label = df.columns[i + 17]
        ax.plot(x, y, label=label)
        ax.legend()
        ax.set_xlabel(x_label)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    fig1.savefig(os.path.join(task_name, f"{file}_A.png"))
    fig2.savefig(os.path.join(task_name, f"{file}_B.png"))

    # plt.show()