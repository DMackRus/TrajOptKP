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

            # Compute means
            mean_mse = df["MSE"].mean()
            mean_rms = df["RMS"].mean()
            mean_max_error = df["Max Error"].mean()
            mean_deriv = df["% Derivatives"].mean()

            results.append({
                "method": os.path.splitext(filename)[0],
                "MSE": mean_mse,
                "RMS": mean_rms,
                "Max Error": mean_max_error,
                "% Derivatives": mean_deriv
            })

    # Convert results into DataFrame for convenience
    summary_df = pd.DataFrame(results)
    
    print(summary_df)

    # Plotting
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Scatter plots
    axes[0].scatter(summary_df["% Derivatives"], summary_df["MSE"])
    axes[1].scatter(summary_df["% Derivatives"], summary_df["RMS"])
    axes[2].scatter(summary_df["% Derivatives"], summary_df["Max Error"])

    # Set labels and titles
    axes[0].set_xlabel("% Derivatives")
    axes[0].set_ylabel("MSE")
    axes[0].set_title("MSE vs % Derivatives")

    axes[1].set_xlabel("% Derivatives")
    axes[1].set_ylabel("RMS")
    axes[1].set_title("RMS vs % Derivatives")

    axes[2].set_xlabel("% Derivatives")
    axes[2].set_ylabel("Max Error")
    axes[2].set_title("Max Error vs % Derivatives")

    # Add legend (shared across plots)
    for ax in axes:
        for i, row in summary_df.iterrows():
            ax.scatter(row["% Derivatives"], row[ax.get_ylabel()], label=row["method"])
        ax.legend()

    plt.tight_layout()
    plt.show()


# Example usage:
# plot_error_metrics("acrobot_1000_5")
plot_error_metrics("impact_large_box_2000_8")
