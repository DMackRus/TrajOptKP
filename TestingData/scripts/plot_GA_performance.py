import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("../acrobot_fitness_tracking.csv")

# Set the number of rows as the x-axis (e.g., iterations or generations)
x = range(len(df))

# Define colour themes
fitness_colors = {'Best': '#A6CEE3', 'Average': '#1F78B4', 'Worst': '#B2DF8A'}
cost_colors = {'Best': '#FB9A99', 'Average': '#E31A1C', 'Worst': '#FDBF6F'}
deriv_colors = {'Best': '#CAB2D6', 'Average': '#6A3D9A', 'Worst': '#FFFF99'}

# Create plot
plt.figure(figsize=(14, 8))

# Fitness plots
plt.plot(x, df["Best fitness"], label="Best Fitness", color=fitness_colors['Best'], linestyle='--', alpha=0.6)
plt.plot(x, df["Average fitness"], label="Average Fitness", color=fitness_colors['Average'], linewidth=3)
plt.plot(x, df["Worst fitness"], label="Worst Fitness", color=fitness_colors['Worst'], linestyle='--', alpha=0.6)

# Cost reduction plots
plt.plot(x, df["Best cost reduction"], label="Best Cost Reduction", color=cost_colors['Best'], linestyle='--', alpha=0.6)
plt.plot(x, df["Average cost reduction"], label="Average Cost Reduction", color=cost_colors['Average'], linewidth=3)
plt.plot(x, df["Worst cost reduction"], label="Worst Cost Reduction", color=cost_colors['Worst'], linestyle='--', alpha=0.6)

# Percent derivatives plots
plt.plot(x, df["Best percent derivatives"], label="Best % Derivatives", color=deriv_colors['Best'], linestyle='--', alpha=0.6)
plt.plot(x, df["Average percent derivatives"], label="Average % Derivatives", color=deriv_colors['Average'], linewidth=3)
plt.plot(x, df["Worst percent derivatives"], label="Worst % Derivatives", color=deriv_colors['Worst'], linestyle='--', alpha=0.6)

# Add labels and legend
plt.xlabel("Iteration")
plt.ylabel("Value")
plt.title("Performance Metrics Over Time")
plt.legend(loc='upper right', fontsize='small')
plt.grid(True)
plt.tight_layout()

# Show plot
# plt.show()
plt.savefig("GA_performance_plot.png", dpi=300)