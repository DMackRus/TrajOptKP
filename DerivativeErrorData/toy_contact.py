import os
import yaml
import numpy as np
import matplotlib.pyplot as plt

ROOT_DIR = "piston_block"   # folder containing all method folders
BASELINE = "SI1"            # special baseline folder name

def load_A_matrix(path):
    """
    Loads A_matrices.csv and reshapes automatically to [T, 4, 4],
    regardless of row/column orientation.
    """
    A = np.loadtxt(path, delimiter=",")

    # flatten to 1D
    A_flat = A.reshape(-1)

    total_values = A_flat.size
    if total_values % 16 != 0:
        raise ValueError(
            f"A matrix file {path} does not contain a multiple of 16 values. "
            f"Found {total_values}."
        )

    T = total_values // 16
    return A_flat.reshape(T, 4, 4)


def main():
    # --- identify method folders ---
    methods = [
        f for f in os.listdir(ROOT_DIR)
        if os.path.isdir(os.path.join(ROOT_DIR, f))
    ]

    if BASELINE not in methods:
        raise ValueError(f"Baseline folder '{BASELINE}' not found.")

    # Load baseline A matrices
    base_path = os.path.join(ROOT_DIR, BASELINE, "A_matrices.csv")
    A_base = load_A_matrix(base_path)
    T = A_base.shape[0]

    # Process other methods
    for method in methods:
        if method == BASELINE:
            continue

        print(f"Processing method: {method}")

        A_path = os.path.join(ROOT_DIR, method, "A_matrices.csv")
        if not os.path.exists(A_path):
            print(f"  Missing A_matrices.csv in {method}, skipping.")
            continue

        A_method = load_A_matrix(A_path)

        # Ensure same number of timesteps
        if A_method.shape[0] != T:
            print(f"  Warning: timestep mismatch for {method}. Skipping.")
            continue

        # --- Plotting 4x4 ---
        fig, axes = plt.subplots(4, 4, figsize=(14, 12))
        fig.suptitle(f"A-matrix Comparison: {method} vs {BASELINE}", fontsize=16)

        t = np.arange(T)

        for i in range(4):
            for j in range(4):
                ax = axes[i, j]
                ax.plot(t, A_base[:, i, j], label=BASELINE)
                ax.plot(t, A_method[:, i, j], label=method)
                ax.set_title(f"A[{i},{j}]")
                ax.grid(True)

                # Only place legend in top-left plot (to reduce clutter)
                if i == 0 and j == 0:
                    ax.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.97])
        out_name = f"A_compare_{method}.png"
        plt.savefig(out_name, dpi=200)
        plt.close()
        print(f"  Saved figure: {out_name}")

    print("Done.")


if __name__ == "__main__":
    main()
