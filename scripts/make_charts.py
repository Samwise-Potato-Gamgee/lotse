"""Reads the benchmark result files in results/ and writes the four result
charts into docs/images/. Run inside the development image, which has
matplotlib: docker run --rm -v "$PWD:/workspace" -w /workspace lotse-dev:jazzy
python3 scripts/make_charts.py
"""

import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RESULTS = "results"
IMAGES = "docs/images"
FLEET_SIZES = (2, 5, 10, 15, 20)
LIVENESS_RUNS = 1080


def read_rows(name):
    """Reads one result CSV into a list of dictionaries.
    @param name the file name under results/
    """
    with open(os.path.join(RESULTS, name), encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def save(figure, name):
    """Writes a figure to docs/images/ and closes it.
    @param figure the matplotlib figure
    @param name the output file name
    """
    os.makedirs(IMAGES, exist_ok=True)
    path = os.path.join(IMAGES, name)
    figure.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(figure)
    print("wrote %s" % path)


def safety_delays():
    """Plots the share of delayed runs with a collision or swap per delay level."""
    pass_rows = read_rows("safety_runs.csv")
    control_rows = read_rows("safety_control_runs.csv")
    delays = sorted({float(row["delay"]) for row in pass_rows})
    figure, axes = plt.subplots(figsize=(7.2, 4.4))
    for rows, label, colour in ((pass_rows, "pass-order rule on", "#1b6ca8"),
                                (control_rows, "pass-order rule off", "#c0392b")):
        shares = []
        for delay in delays:
            level = [row for row in rows if float(row["delay"]) == delay]
            bad = sum(1 for row in level
                      if int(row["collisions"]) > 0 or int(row["swaps"]) > 0)
            shares.append(100.0 * bad / len(level))
        axes.plot(delays, shares, marker="o", label=label, color=colour)
    axes.set_title("Safety under injected delays (2,000 runs)")
    axes.set_xlabel("delay probability per step")
    axes.set_ylabel("runs with a collision or swap (%)")
    axes.grid(True, alpha=0.3)
    axes.legend()
    save(figure, "safety_delays.png")


def throughput_fleet():
    """Plots orders completed per 1,000 steps against fleet size, one line per map."""
    rows = [row for row in read_rows("throughput_runs.csv") if float(row["delay"]) == 0.0]
    maps = sorted({row["map"] for row in rows})
    figure, axes = plt.subplots(figsize=(7.2, 4.6))
    for index, name in enumerate(maps):
        map_rows = [row for row in rows if row["map"] == name]
        per_fleet = []
        for fleet in FLEET_SIZES:
            level = [row for row in map_rows if int(row["fleet"]) == fleet]
            values = [1000.0 * int(row["orders_completed"]) / int(row["steps_run"])
                      for row in level]
            per_fleet.append(float(np.mean(values)))
        axes.plot(FLEET_SIZES, per_fleet, marker="o", label=os.path.basename(name))
    axes.set_title("Throughput without delays (continuous orders)")
    axes.set_xlabel("robots in the fleet")
    axes.set_ylabel("orders completed per 1,000 steps")
    axes.set_xticks(FLEET_SIZES)
    axes.grid(True, alpha=0.3)
    axes.legend(fontsize=8)
    save(figure, "throughput_fleet.png")


def planner_comparison():
    """Plots solved share and planning time against robot count for both planners."""
    rows = read_rows("planner_comparison_runs.csv")
    robots = sorted({int(row["robots"]) for row in rows})
    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.2))
    for planner, label, colour in (("prioritized", "prioritized planning", "#1b6ca8"),
                                   ("cbs", "conflict-based search", "#c0392b")):
        solved = []
        seconds = []
        for count in robots:
            level = [row for row in rows
                     if row["planner"] == planner and int(row["robots"]) == count]
            solved.append(100.0 * sum(int(row["solved"]) for row in level) / len(level))
            seconds.append(float(np.mean([float(row["seconds"]) for row in level])))
        axes[0].plot(robots, solved, marker="o", label=label, color=colour)
        axes[1].plot(robots, seconds, marker="o", label=label, color=colour)
    axes[0].set_title("Scenarios solved within the time limit")
    axes[0].set_xlabel("robots")
    axes[0].set_ylabel("scenarios solved (%)")
    axes[1].set_title("Mean planning time")
    axes[1].set_xlabel("robots")
    axes[1].set_ylabel("seconds per planning call")
    for axis in axes:
        axis.set_xticks(robots)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)
    figure.suptitle("Planner comparison on 200 MAPF benchmark scenarios per planner")
    save(figure, "planner_comparison.png")


def realtime_planning():
    """Plots slow and frozen runs out of 1,080 for the three planning modes."""
    files = ("liveness_stale_plans_runs.csv", "liveness_current_positions_pre_stable_runs.csv",
             "liveness_current_plans_runs.csv")
    labels = ("background\ntakeover (old)", "current positions\n(no stable order)",
              "current positions\n+ stable order")
    slow = []
    frozen = []
    for name in files:
        rows = read_rows(name)
        slow.append(sum(1 for row in rows if int(row["slow"]) == 1))
        frozen.append(sum(1 for row in rows if int(row["deadlocked"]) == 1))
    positions = np.arange(len(files))
    width = 0.38
    figure, axes = plt.subplots(figsize=(7.6, 4.4))
    axes.bar(positions - width / 2, slow, width, label="slow runs", color="#e67e22")
    axes.bar(positions + width / 2, frozen, width, label="frozen runs (deadlocked)",
             color="#c0392b")
    for index, value in enumerate(slow):
        axes.text(index - width / 2, value + 5, str(value), ha="center", fontsize=9)
    for index, value in enumerate(frozen):
        axes.text(index + width / 2, value + 5, str(value), ha="center", fontsize=9)
    axes.axhline(0, color="black", linewidth=0.8)
    axes.set_title("Real-time planning modes, 1,080 runs each")
    axes.set_xlabel("planning mode")
    axes.set_ylabel("runs out of 1,080")
    axes.set_xticks(positions)
    axes.set_xticklabels(labels)
    axes.legend()
    axes.grid(True, axis="y", alpha=0.3)
    save(figure, "realtime_planning.png")


def main():
    """Writes all four result charts into docs/images/."""
    safety_delays()
    throughput_fleet()
    planner_comparison()
    realtime_planning()


if __name__ == "__main__":
    main()
