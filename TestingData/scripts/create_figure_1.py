import argparse
import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import csv

SUMMARY_FILE = "summary.csv"
A_FILE = "A_contact_change.csv"

# Columns we want in the stacked bar
TIME_COLS = [
    "Total time derivs (ms)",
    "Total time BP (ms)",
    "Total time FP (ms)",
]

# SNAPSHOT_FRAMES = [1, 136, 199]   # <-- change these easily
SNAPSHOT_FRAMES = [40, 160, 187, 199]   # <-- change these easily
SNAPSHOT_CROP = dict(
    left=0.3,
    right=0.4,
    top=0.25,
    bottom=0.35,
)


METHODS = ["SI_1", "contact_change"]

def load_snapshot_image(media_root: Path, task_name: str, method: str, t: int):
    """
    Expects: {media_root}/{method}/{t}.png
    Example: ../media/videos/SI1/20.png
    """
    #method is task_name + method
    folder_name = f"{task_name}_{method}" if task_name else method
    
    img_path = media_root / folder_name / f"frame_{t}.png"
    if not img_path.exists():
        raise FileNotFoundError(f"Snapshot not found: {img_path}")
    img = mpimg.imread(img_path)
    return img, img_path

def crop_image(img, crop):
    """
    Crop an image using fractional margins.

    crop = dict(left=..., right=..., top=..., bottom=...)
    Each value is a fraction of width/height to remove from that side.

    Example:
        crop = dict(left=0.20, right=0.20, top=0.05, bottom=0.05)
    """
    if crop is None:
        return img

    H, W = img.shape[:2]

    left   = int(W * crop.get("left", 0.0))
    right  = int(W * crop.get("right", 0.0))
    top    = int(H * crop.get("top", 0.0))
    bottom = int(H * crop.get("bottom", 0.0))

    x0 = left
    x1 = W - right
    y0 = top
    y1 = H - bottom

    # safety clamp
    x0 = max(0, min(x0, W - 1))
    x1 = max(x0 + 1, min(x1, W))
    y0 = max(0, min(y0, H - 1))
    y1 = max(y0 + 1, min(y1, H))

    return img[y0:y1, x0:x1]


def load_summary_csv(summary_path: Path) -> pd.DataFrame:
    if not summary_path.exists():
        raise FileNotFoundError(f"Missing summary.csv: {summary_path}")
    df = pd.read_csv(summary_path)

    # If there are multiple rows (e.g., multiple runs), we average them.
    # If it's single row, this does nothing.
    return df


def get_timing_breakdown(summary_df: pd.DataFrame) -> dict:
    # Robust: allow multiple rows, take mean
    timing = {}
    for col in TIME_COLS:
        if col not in summary_df.columns:
            raise KeyError(f"Expected column '{col}' in summary.csv. Got: {list(summary_df.columns)}")
        timing[col] = float(summary_df[col].mean())
    return timing

def load_keypoints(kp_path: Path):
    
    kp = []
    with open(kp_path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            # row is a list of strings (possibly empty)
            row = [x.strip() for x in row if x.strip() != ""]
            kp.append([int(x) for x in row])

    # print(kp)
    
    return kp


def load_A_timeseries(A_path: Path):
    """
    A_contact_change.csv is stored as stacked matrices:
        shape = (T * n_x, n_x)
    where each block of n_x rows is A_t.

    We infer n_x by:
        - number of columns
        - checking that number of rows is divisible by n_x
    """
    if not A_path.exists():
        raise FileNotFoundError(f"Missing A matrix file: {A_path}")

    A2d = np.loadtxt(A_path, delimiter=",")
    if A2d.ndim == 1:
        # handle degenerate case: a single row in file
        A2d = A2d.reshape(1, -1)

    n_rows, n_cols = A2d.shape
    n_x = n_cols

    if n_x <= 0:
        raise ValueError(f"Invalid inferred n_x={n_x} from columns in {A_path}")

    if n_rows % n_x != 0:
        raise ValueError(
            f"Cannot infer horizon: rows={n_rows} not divisible by n_x={n_x} "
            f"for file {A_path}. Expected shape (T*n_x, n_x)."
        )

    T = n_rows // n_x

    # Reshape into (T, n_x, n_x)
    A3d = A2d.reshape(T, n_x, n_x)
    return A3d, n_x, T

def contact_event_times(contacts_over_time):
    """
    Returns:
      made_times   : sorted list of t where a contact is made
      broken_times : sorted list of t where a contact is broken
    """
    made_times = []
    broken_times = []

    prev = set()
    for t, contacts_t in enumerate(contacts_over_time):
        curr = set(contacts_t)

        if t > 0:
            if len(curr - prev) > 0:
                made_times.append(t)
            if len(prev - curr) > 0:
                broken_times.append(t)

        prev = curr

    return made_times, broken_times

def contact_times_from_list(contacts_over_time):
    """
    contacts_over_time[t] = list of contacts at time t
    Returns: list of t where at least one contact exists
    """
    return [t for t, c in enumerate(contacts_over_time) if len(c) > 0]

def load_contacts_csv(path):
    df = pd.read_csv(path)

    # reconstruct vector<vector<pair>>
    contacts_over_time = []
    max_t = int(df["t"].max()) if len(df) > 0 else -1

    grouped = df.groupby("t")
    for t in range(max_t + 1):
        if t in grouped.groups:
            rows = grouped.get_group(t).sort_values("contact_id")
            contacts_t = list(zip(rows["body_a"].astype(int), rows["body_b"].astype(int)))
        else:
            contacts_t = []
        contacts_over_time.append(contacts_t)

    return contacts_over_time, df


def plot_figure(
    task_name: str,
    timing_by_method: dict,
    A_si1: np.ndarray,
    A_cc: np.ndarray,
    element_ij=(0, 0),
    save_path=None,
    show=True,
    contacts_made=None,
    contacts_broken=None,
    keypoints=None,
    media_root=None,
    snapshot_method="contact_change",
    snapshot_frames=None,
):
    
    fig = plt.figure(figsize=(15.5, 4.6), constrained_layout=True)
    gs = fig.add_gridspec(
        nrows=2, ncols=3,
        width_ratios=[1.05, 1.45, 1.25]
    )

    ax_bar = fig.add_subplot(gs[:, 2])   # left spans both rows
    ax_top = fig.add_subplot(gs[0, 1])   # middle-top
    ax_bot = fig.add_subplot(gs[1, 1])   # middle-bottom

    # # Right column: 3 snapshot axes spanning both rows
    snap_gs = gs[:, 0].subgridspec(2, 2, wspace=0.01)
    ax_snap1 = fig.add_subplot(snap_gs[0, 0])
    ax_snap2 = fig.add_subplot(snap_gs[0, 1])
    ax_snap3 = fig.add_subplot(snap_gs[1, 0])
    ax_snap4 = fig.add_subplot(snap_gs[1, 1])
    snap_axes = [ax_snap1, ax_snap2, ax_snap3, ax_snap4]

    # # --- Stacked bar chart
    method_labels = METHODS
    x = np.arange(len(method_labels))


    # ------------------------------------------------------------------
    
    bottoms = np.zeros(len(method_labels))

    # --- Derivatives ---
    deriv_vals = [
        timing_by_method[m]["Total time derivs (ms)"]
        for m in method_labels
    ]
    ax_bar.bar(x, deriv_vals, bottom=bottoms, label="Derivatives")
    bottoms += np.array(deriv_vals)

    # --- Other (BP + FP) ---
    other_vals = [
        timing_by_method[m]["Total time BP (ms)"] +
        timing_by_method[m]["Total time FP (ms)"]
        for m in method_labels
    ]
    ax_bar.bar(x, other_vals, bottom=bottoms, label="Other")
    
    
    
    
    
    
    
    # ----------------------------------------------------------------------
    # Build stacks in consistent order
    # stacks = [TIME_COLS[0], TIME_COLS[1], TIME_COLS[2]]
    # bottoms = np.zeros(len(method_labels))

    # for col in stacks:
    #     vals = [timing_by_method[m][col] for m in method_labels]
    #     ax_bar.bar(x, vals, bottom=bottoms, label=col)
    #     bottoms += np.array(vals)

    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(method_labels)
    ax_bar.set_ylabel("Time (ms)")
    ax_bar.set_title("Optimisation timing breakdown")
    ax_bar.legend(frameon=False, fontsize=8, loc="upper right")

    # # Make it readable
    ax_bar.spines["top"].set_visible(False)
    ax_bar.spines["right"].set_visible(False)


    #     return y  # <-- return the plotted data so we can sync y-lims
    def plot_A_element(ax, A3d, keypoints, show_kp_vs_interp, title):
        T, n_x, _ = A3d.shape
        i, j = element_ij
        print(f"Plotting A[{i},{j}] with n_x={n_x}, T={T*5}")
        
        print(f"dof is {j}")

        if not (0 <= i < n_x and 0 <= j < n_x):
            raise IndexError(f"A[{i},{j}] invalid for n_x={n_x}")

        y = A3d[:, i, j]
        ax.plot(np.arange(T), y, linewidth=1.8)

        # ---- Keypoints ----
        t = 0
        kp_times = []
        non_kp_times = []
        
        # ----- Add green vertical lines for snapshots -----
        if show_kp_vs_interp:
            for snap_t in snapshot_frames:
                ax.axvline(
                    snap_t * 5,
                    color="green",
                    linestyle="--",
                    linewidth=1.2,
                    alpha=0.6,
                    label="snapshot" if snap_t == snapshot_frames[0] else None  # only label first line
            )

        for row in keypoints:
            for dof in row:
                if j == dof or j == (dof + (n_x // 2)):  # use integer division!
                    kp_times.append(t)
                else:
                    non_kp_times.append(t)
                    
            t += 1

        # Remove duplicates (optional but cleaner)
        kp_times = list(set(kp_times))
        non_kp_times = list(set(non_kp_times) - set(kp_times))
        print(kp_times)

        # Plot keypoints (RED)
        if show_kp_vs_interp:
            ax.scatter(kp_times, y[kp_times], color="red", s=20, zorder=3, label="keypoints")

        ax.set_xlim(0, T - 1)
        ax.set_ylabel(f"A[{i},{j}]")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

        return y  # <-- return the plotted data so we can sync y-lims


    # ---- Plot both, capture y values ----
    y_top = plot_A_element(ax_top, A_si1, keypoints, False,
                        title="SI1: A element over horizon")
    y_bot = plot_A_element(ax_bot, A_cc, keypoints, True,
                        title="Contact-change: A element over horizon")

    # ---- Force same y-axis range ----
    ymin = min(np.min(y_top), np.min(y_bot))
    ymax = max(np.max(y_top), np.max(y_bot))

    ax_top.set_ylim(ymin, ymax)
    ax_bot.set_ylim(ymin, ymax)

    ax_bot.set_xlabel("Horizon index (t)")

    
    # --- Snapshots ---
    # Top row snapshots (SI₁)
    # for ax, t in zip(ax_snap_top, SNAPSHOT_FRAMES):
    #     img, img_path = load_snapshot_image(Path(media_root), task_name, "SI_1", t)
    #     img = crop_image(img, SNAPSHOT_CROP)
    #     ax.imshow(img)
    #     ax.set_title(f"t={t}")
    #     ax.axis("off")

    # # Bottom row snapshots (contact-change)
    # for ax, t in zip(ax_snap_bot, SNAPSHOT_FRAMES):
    #     img, img_path = load_snapshot_image(Path(media_root), task_name, "contact_change", t)
    #     img = crop_image(img, SNAPSHOT_CROP)
    #     ax.imshow(img)
    #     ax.set_title(f"t={t}")
    #     ax.axis("off")

    
    
    
    if media_root is not None and snapshot_frames is not None:
        for ax, t in zip(snap_axes, snapshot_frames):
            img, img_path = load_snapshot_image(Path(media_root), task_name, snapshot_method, t)
            img = crop_image(img, SNAPSHOT_CROP)
            ax.imshow(img)
            ax.set_title(f"t={t*5}", fontsize=9)
            ax.axis("off")
    else:
        # If no media provided, just hide axes cleanly
        for ax in snap_axes:
            ax.axis("off")

    if save_path is not None:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"[OK] Saved figure to: {save_path}")

    if show:
        plt.show()

    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--task_name", type=str, required=True,
                        help="Task folder name, e.g. pick_and_place")
    parser.add_argument("--i", type=int, default=0, help="Row index of A element to plot")
    parser.add_argument("--j", type=int, default=0, help="Col index of A element to plot")
    parser.add_argument("--save", type=str, default=None,
                        help="Optional path to save figure, e.g. fig1.png")
    parser.add_argument("--no_show", action="store_true", help="Do not show the plot window")
    args = parser.parse_args()
    
    task_dir = Path("../figure_1/" + args.task_name)

    # task_dir = Path(args.root) / args.task_name
    if not task_dir.exists():
        raise FileNotFoundError(f"Task directory does not exist: {task_dir}")

    timing_by_method = {}
    A_by_method = {}
    
    A_SI1_path = task_dir / "contact_change" / "A_SI1.csv"
    A_contact_change_path = task_dir / "contact_change" / "A_contact_change.csv"
    
    print(A_SI1_path)
    
    A_by_method["SI_1"], n_x, T = load_A_timeseries(Path(A_SI1_path))
    A_by_method["contact_change"], n_x, T = load_A_timeseries(Path(A_contact_change_path))
    
    timing_by_method["SI_1"] = get_timing_breakdown(load_summary_csv(task_dir / "SI_1" / SUMMARY_FILE))
    timing_by_method["contact_change"] = get_timing_breakdown(load_summary_csv(task_dir / "contact_change" / SUMMARY_FILE))
    
    A_contact_change_path = task_dir / "contact_change" / "contact_sequence.csv"
    contacts, df= load_contacts_csv(A_contact_change_path)
    
    contact_made_times, contact_broken_times = contact_event_times(contacts)
    # contacts = contact_times_from_list(contacts)
    
    kp_path = task_dir / "contact_change" / "keypoints_contact_change.csv"
    keypoints = load_keypoints(kp_path)
    
    print(df)
    
    media_root = Path("../../media/videos")

    # IMPORTANT: your folders are SI1 and contact_change (SI1 not SI_1)
    # so snapshot_method should be "SI1" or "contact_change"
    snapshot_method = "contact_change"   # or "SI1"

    plot_figure(
        task_name=args.task_name,
        timing_by_method=timing_by_method,
        A_si1=A_by_method["SI_1"],
        A_cc=A_by_method["contact_change"],
        element_ij=(args.i, args.j),
        save_path=args.save,
        show=(not args.no_show),
        contacts_made=contact_made_times,
        contacts_broken=contact_broken_times,
        keypoints=keypoints,
        media_root=media_root,
        snapshot_method=snapshot_method,
        snapshot_frames=SNAPSHOT_FRAMES,
    )


if __name__ == "__main__":
    main()
