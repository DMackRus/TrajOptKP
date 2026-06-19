import os
import yaml
import numpy as np
from collections import defaultdict

def load_summary(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def collect_results(root_dir):
    """
    Traverse directory structure:
    root_dir / method / trial_number / summary.yaml
    """
    results = defaultdict(list)

    for method in os.listdir(root_dir):
        method_path = os.path.join(root_dir, method)
        if not os.path.isdir(method_path):
            continue

        for trial in os.listdir(method_path):
            trial_path = os.path.join(method_path, trial)
            summary_file = os.path.join(trial_path, "summary.yaml")
            if os.path.isfile(summary_file):
                data = load_summary(summary_file)
                results[method].append(data)

    return results

def compute_stats(results):
    """
    For each method, compute success counts, failure counts,
    averages and std for execution time and goal distance error.
    """
    stats = {}

    for method, trials in results.items():
        n_trials = len(trials)
        successes = sum(1 for t in trials if t.get("Task success", False))
        force_failures = sum(1 for t in trials if t.get("Force failure", False))
        detached = sum(1 for t in trials if t.get("Object detached", False))

        exec_times = [t.get("Execution time (s)", np.nan) for t in trials]
        distances = [t.get("Goal distance error (m)", np.nan) for t in trials]
        task_costs = [t.get("Task cost", np.nan) for t in trials]
        optimization_times = [t.get("Average optimisation time (ms)", np.nan) for t in trials]

        stats[method] = {
            "n_trials": n_trials,
            "successes": successes,
            "force_failures": force_failures,
            "object_detached": detached,
            "execution_time_mean": np.nanmean(exec_times),
            "execution_time_std": np.nanstd(exec_times),
            "task_cost_mean": np.nanmean(task_costs),
            "task_cost_std": np.nanstd(task_costs),
            "optimization_time_mean": np.nanmean(optimization_times),
            "optimization_time_std": np.nanstd(optimization_times),
            "distance_mean": np.nanmean(distances),
            "distance_std": np.nanstd(distances),
        }

    return stats

def print_stats(stats):
    print("\n=== Method Statistics ===")
    for method, s in stats.items():
        print(f"\nMethod: {method}")
        print(f"  Trials: {s['n_trials']}")
        print(f"  Successes: {s['successes']} / {s['n_trials']} "
              f"({s['successes']/s['n_trials']*100:.1f}%)")
        # print(f"  Force failures: {s['force_failures']}")
        # print(f"  Object detached: {s['object_detached']}")
        print(f"  Execution time: {s['execution_time_mean']:.3f} ± {s['execution_time_std']:.3f} s")
        print(f"  Task cost: {s['task_cost_mean']:.3f} ± {s['task_cost_std']:.3f}")
        print(f"  Optimization time: {s['optimization_time_mean']:.3f} ± {s['optimization_time_std']:.3f} ms")
        # print(f"  Goal distance error: {s['distance_mean']:.4f} ± {s['distance_std']:.4f} m")

if __name__ == "__main__":
    root_dir = "."  # change if needed
    results = collect_results(root_dir)
    stats = compute_stats(results)
    print_stats(stats)