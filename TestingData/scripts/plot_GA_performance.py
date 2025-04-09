import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("../acrobot_fitness_tracking.csv")

# Normalise fitness columns between 0 and 1
fitness_cols = ["Best fitness", "Average fitness", "Worst fitness"]

# Get the global min and max across all selected columns
global_min = df[fitness_cols].min().min()
global_max = df[fitness_cols].max().max()

# Apply normalisation using the global min and max
df[fitness_cols] = (df[fitness_cols] - global_min) / (global_max - global_min)

# x-axis
x = range(len(df))

# Define colours
fitness_colors = {'Best': '#A6CEE3', 'Average': '#1F78B4', 'Worst': '#B2DF8A'}
cost_colors = {'Best': '#FB9A99', 'Average': '#E31A1C', 'Worst': '#FDBF6F'}
deriv_colors = {'Best': '#CAB2D6', 'Average': '#6A3D9A', 'Worst': '#FFFF99'}

# Create figure and left axis
fig, ax1 = plt.subplots(figsize=(14, 8))

# Left axis (fitness and cost reduction) — range 0 to 1
ax1.set_ylim(0, 1)

# Plot fitness
ax1.plot(x, df["Best fitness"], label="Best Fitness", color=fitness_colors['Best'], linestyle='--', alpha=0.6)
ax1.plot(x, df["Average fitness"], label="Average Fitness", color=fitness_colors['Average'], linewidth=3)
ax1.plot(x, df["Worst fitness"], label="Worst Fitness", color=fitness_colors['Worst'], linestyle='--', alpha=0.6)

# Plot cost reduction
ax1.plot(x, df["Best cost reduction"], label="Best Cost Reduction", color=cost_colors['Best'], linestyle='--', alpha=0.6)
ax1.plot(x, df["Average cost reduction"], label="Average Cost Reduction", color=cost_colors['Average'], linewidth=3)
ax1.plot(x, df["Worst cost reduction"], label="Worst Cost Reduction", color=cost_colors['Worst'], linestyle='--', alpha=0.6)

ax1.set_xlabel("Iteration")
ax1.set_ylabel("Fitness & Cost Reduction (Normalised)")

# Right axis (percent derivatives) — range 0 to 100
ax2 = ax1.twinx()
ax2.set_ylim(0, 100)

# Plot percent derivatives
ax2.plot(x, df["Best percent derivatives"], label="Best % Derivatives", color=deriv_colors['Best'], linestyle='--', alpha=0.6)
ax2.plot(x, df["Average percent derivatives"], label="Average % Derivatives", color=deriv_colors['Average'], linewidth=3)
ax2.plot(x, df["Worst percent derivatives"], label="Worst % Derivatives", color=deriv_colors['Worst'], linestyle='--', alpha=0.6)

ax2.set_ylabel("Percent Derivatives (%)")

# Combine legends from both axes
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper right', fontsize='small')

# Grid and layout
ax1.grid(True)
plt.title("GA Performance Over Iterations")
plt.tight_layout()

# Save the plot
plt.savefig("GA_dual_axis_plot.png", dpi=300)