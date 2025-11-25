import os
import yaml
import numpy as np
import matplotlib.pyplot as plt

ROOT_DIR = "piston_block"   # folder containing all method folders
BASELINE = "SI1"            # special baseline folder name

def load_keypoints(path):
    """
    Loads keypoints.csv where each row may be:
      - empty (no keypoint),
      - '0', '1', or '0,1' (order doesn't matter),
      - may contain spaces: ' 0 , 1 '.
    Returns two boolean numpy arrays of shape (T,):
      - has0[t] True if value 0 present on row t (mark columns 0 & 2)
      - has1[t] True if value 1 present on row t (mark columns 1 & 3)
    """
    has0 = []
    has1 = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if line == "":
                has0.append(False)
                has1.append(False)
                continue

            # split on comma and parse tokens tolerant to whitespace
            tokens = [tok.strip() for tok in line.split(",") if tok.strip() != ""]
            # allow tokens like '0' or '1', ignore anything else
            set_tokens = set(tokens)
            has0.append("0" in set_tokens)
            has1.append("1" in set_tokens)

    return np.array(has0, dtype=bool), np.array(has1, dtype=bool)


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

    # Load baseline keypoints (if present)
    base_kp_path = os.path.join(ROOT_DIR, BASELINE, "keypoints.csv")
    kp_base_has0 = kp_base_has1 = None
    if os.path.exists(base_kp_path):
        kp_base_has0, kp_base_has1 = load_keypoints(base_kp_path)
        if kp_base_has0.size != T:
            print(f"Warning: baseline keypoints length ({kp_base_has0.size}) != T ({T}). Ignoring baseline keypoints.")
            kp_base_has0 = kp_base_has1 = None
    else:
        # no baseline keypoints file — that's fine
        kp_base_has0 = kp_base_has1 = None

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
            print(f"  Warning: timestep mismatch for {method} (method T={A_method.shape[0]}, baseline T={T}). Skipping.")
            continue

        # Try to load method keypoints
        kp_path = os.path.join(ROOT_DIR, method, "keypoints.csv")
        kp_method_has0 = kp_method_has1 = None
        if os.path.exists(kp_path):
            kp_method_has0, kp_method_has1 = load_keypoints(kp_path)
            if kp_method_has0.size != T:
                print(f"  Warning: keypoint timestep mismatch for {method} (kp T={kp_method_has0.size}, A T={T}). Skipping keypoints for this method.")
                kp_method_has0 = kp_method_has1 = None
        else:
            # missing keypoints file is not fatal
            kp_method_has0 = kp_method_has1 = None

        # --- Plotting 4x4 ---
        fig, axes = plt.subplots(4, 4, figsize=(14, 12))
        fig.suptitle(f"A-matrix Comparison: {method} vs {BASELINE}", fontsize=16)

        t = np.arange(T)

        # We'll add legend entries only once (top-left subplot)
        legend_added = False

        for i in range(4):
            for j in range(4):
                ax = axes[i, j]
                ax.plot(t, A_base[:, i, j], label=BASELINE if not legend_added else None)
                ax.plot(t, A_method[:, i, j], label=method if not legend_added else None)
                ax.set_title(f"A[{i},{j}]")
                ax.grid(True)

                # --- Overlay baseline keypoints (if any) ---
                if kp_base_has0 is not None:
                    # kp=0 maps to columns 0 and 2 -> check j in {0,2}
                    if j in (0, 2):
                        idx = np.where(kp_base_has0)[0]
                        if idx.size:
                            ax.scatter(idx, A_base[idx, i, j], marker="x", s=30, label=f"{BASELINE} kp0" if not legend_added else None)

                    # kp=1 maps to columns 1 and 3 -> check j in {1,3}
                    if j in (1, 3):
                        idx = np.where(kp_base_has1)[0]
                        if idx.size:
                            ax.scatter(idx, A_base[idx, i, j], marker="x", s=30, label=f"{BASELINE} kp1" if not legend_added else None)

                # --- Overlay method keypoints (if any) ---
                if kp_method_has0 is not None:
                    if j in (0, 2):
                        idx = np.where(kp_method_has0)[0]
                        if idx.size:
                            ax.scatter(idx, A_method[idx, i, j], marker="o", s=20, label=f"{method} kp0" if not legend_added else None)

                    if j in (1, 3):
                        idx = np.where(kp_method_has1)[0]
                        if idx.size:
                            ax.scatter(idx, A_method[idx, i, j], marker="o", s=20, label=f"{method} kp1" if not legend_added else None)

                # Only place legend in top-left plot (to reduce clutter)
                if i == 0 and j == 0:
                    ax.legend(loc="upper right", fontsize="small")
                    legend_added = True

        plt.tight_layout(rect=[0, 0, 1, 0.97])
        out_name = f"A_compare_{method}.png"
        plt.savefig(out_name, dpi=200)
        plt.close()
        print(f"  Saved figure: {out_name}")

    print("Done.")

# def main():
#     # --- identify method folders ---
#     methods = [
#         f for f in os.listdir(ROOT_DIR)
#         if os.path.isdir(os.path.join(ROOT_DIR, f))
#     ]

#     if BASELINE not in methods:
#         raise ValueError(f"Baseline folder '{BASELINE}' not found.")

#     # Load baseline A matrices
#     base_path = os.path.join(ROOT_DIR, BASELINE, "A_matrices.csv")
#     A_base = load_A_matrix(base_path)
#     T = A_base.shape[0]
    
#     # Load baseline keypoints
#     base_kp_path = os.path.join(ROOT_DIR, BASELINE, "keypoints.csv")
#     kp_base = load_keypoints(base_kp_path)

#     if kp_base.size != T:
#         raise ValueError("Baseline keypoints length does not match A matrix timesteps.")

#     # Process other methods
#     for method in methods:
#         if method == BASELINE:
#             continue

#         print(f"Processing method: {method}")

#         A_path = os.path.join(ROOT_DIR, method, "A_matrices.csv")
#         if not os.path.exists(A_path):
#             print(f"  Missing A_matrices.csv in {method}, skipping.")
#             continue

#         A_method = load_A_matrix(A_path)

#         # Ensure same number of timesteps
#         if A_method.shape[0] != T:
#             print(f"  Warning: timestep mismatch for {method}. Skipping.")
#             continue

#         # --- Plotting 4x4 ---
#         fig, axes = plt.subplots(4, 4, figsize=(14, 12))
#         fig.suptitle(f"A-matrix Comparison: {method} vs {BASELINE}", fontsize=16)

#         t = np.arange(T)
        
#         # Load method keypoints
#         kp_path = os.path.join(ROOT_DIR, method, "keypoints.csv")
#         if not os.path.exists(kp_path):
#             print(f"  Missing keypoints.csv in {method}, skipping keypoint overlay.")
#             kp_method = None
#         else:
#             kp_method = load_keypoints(kp_path)
#             if kp_method.size != T:
#                 print(f"  Warning: keypoint timestep mismatch for {method}. Skipping keypoints.")
#                 kp_method = None
                
#         print(kp_method)

#         for i in range(4):
#             for j in range(4):
#                 ax = axes[i, j]
#                 ax.plot(t, A_base[:, i, j], label=BASELINE)
#                 ax.plot(t, A_method[:, i, j], label=method)
#                 ax.set_title(f"A[{i},{j}]")
#                 ax.grid(True)
                
#                 # --- Keypoint overlays ---
#                 if kp_method is not None:
#                     # Indices where a keypoint = 0 or 1
#                     idx0 = np.where(kp_method == 0)[0]
#                     idx1 = np.where(kp_method == 1)[0]

#                     # For kp=0 → mark columns 0 and 2
#                     if (i, j) in [(0,0), (0,2), (1,0), (1,2), (2,0), (2,2), (3,0), (3,2)]:
#                         ax.scatter(idx0, A_method[idx0, i, j], marker="o", s=15, color="red")

#                     # For kp=1 → mark columns 1 and 3
#                     if (i, j) in [(0,1), (0,3), (1,1), (1,3), (2,1), (2,3), (3,1), (3,3)]:
#                         ax.scatter(idx1, A_method[idx1, i, j], marker="o", s=15, color="blue")

#                                 # # Only place legend in top-left plot (to reduce clutter)
#                                 # if i == 0 and j == 0:
#                                 #     ax.legend()

#         plt.tight_layout(rect=[0, 0, 1, 0.97])
#         out_name = f"A_compare_{method}.png"
#         plt.savefig(out_name, dpi=200)
#         plt.close()
#         print(f"  Saved figure: {out_name}")

#     print("Done.")


if __name__ == "__main__":
    main()
