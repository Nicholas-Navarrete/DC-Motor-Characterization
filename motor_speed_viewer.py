"""
motor_speed_viewer.py
----------------------
Loads one or more motor characterization CSV files, computes RPM from the
encoder voltage column using the same adaptive Schmitt trigger algorithm as
motor_characterization.py, and displays an interactive plot.

Nothing is saved to disk.

Usage:
    python motor_speed_viewer.py                  # prompts for file(s)
    python motor_speed_viewer.py data.csv         # single file
    python motor_speed_viewer.py a.csv b.csv ...  # overlay multiple files
"""

import sys
import os

try:
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    print("[ERROR] numpy / matplotlib not found.")
    print("        pip install numpy matplotlib")
    sys.exit(1)

try:
    import pandas as pd
except ImportError:
    print("[ERROR] pandas not found.")
    print("        pip install pandas")
    sys.exit(1)

# -----------------------------------------------------------------------------
# Configuration — must match motor_characterization.py
# -----------------------------------------------------------------------------
NUM_SLOTS           = 13      # slots on the rotor disc
ADAPTIVE_WINDOW_SEC = 0.05    # sliding window for local min/max (seconds)
HYSTERESIS          = 0.25    # Schmitt trigger dead-band fraction (0-0.5)

# -----------------------------------------------------------------------------
# RPM algorithm — exact copy from motor_characterization.py / data logger
# -----------------------------------------------------------------------------

def compute_rpm(times, voltages, num_slots,
                window_sec=ADAPTIVE_WINDOW_SEC,
                hysteresis=HYSTERESIS):
    """
    Detect rising edges using a local adaptive Schmitt trigger and return
    per-revolution RPM values.
    """
    times    = np.array(times,    dtype=float)
    voltages = np.array(voltages, dtype=float)
    n        = len(times)

    # -- Adaptive thresholds --------------------------------------------------
    thresh_hi = np.empty(n)
    thresh_lo = np.empty(n)

    for i in range(n):
        t      = times[i]
        lo_idx = np.searchsorted(times, t - window_sec, side='left')
        hi_idx = np.searchsorted(times, t + window_sec, side='right')
        win    = voltages[lo_idx:hi_idx]
        v_min  = win.min()
        v_max  = win.max()
        swing  = v_max - v_min
        mid    = (v_min + v_max) / 2.0
        thresh_hi[i] = mid + hysteresis * swing
        thresh_lo[i] = mid - hysteresis * swing

    # -- Schmitt trigger edge detection ---------------------------------------
    edge_times = []
    state_high = voltages[0] > (thresh_hi[0] + thresh_lo[0]) / 2.0

    for i in range(1, n):
        if not state_high and voltages[i] > thresh_hi[i]:
            edge_times.append(times[i])
            state_high = True
        elif state_high and voltages[i] < thresh_lo[i]:
            state_high = False

    n_revs = len(edge_times) // num_slots
    print(f"    Detected {len(edge_times)} rising edges -> {n_revs} complete revolutions")

    if len(edge_times) < num_slots + 1:
        print("    [WARN] Not enough edges to compute RPM.")
        print("           Try adjusting ADAPTIVE_WINDOW_SEC or HYSTERESIS.")
        return np.array([]), np.array([]), edge_times, thresh_hi, thresh_lo

    # -- RPM per revolution ---------------------------------------------------
    rpm_times, rpm_values = [], []
    for i in range(num_slots, len(edge_times)):
        t_start = edge_times[i - num_slots]
        t_end   = edge_times[i]
        period  = t_end - t_start
        if period > 0:
            rpm_values.append(60.0 / period)
            rpm_times.append((t_start + t_end) / 2.0)

    return (np.array(rpm_times), np.array(rpm_values),
            edge_times, thresh_hi, thresh_lo)

# -----------------------------------------------------------------------------
# File loading
# -----------------------------------------------------------------------------

def load_csv(path):
    """Load a motor characterization CSV and return a dict of columns."""
    try:
        df = pd.read_csv(path)
    except Exception as exc:
        print(f"[ERROR] Could not read '{path}': {exc}")
        return None

    required = {"time_s", "encoder_V", "current_A", "duty_pct"}
    missing  = required - set(df.columns)
    if missing:
        print(f"[ERROR] '{path}' is missing columns: {missing}")
        print(f"        Found columns: {list(df.columns)}")
        return None

    return {
        "time_s":    df["time_s"].to_numpy(),
        "encoder_V": df["encoder_V"].to_numpy(),
        "current_A": df["current_A"].to_numpy(),
        "duty_pct":  df["duty_pct"].to_numpy(),
        "name":      os.path.basename(path),
    }

# -----------------------------------------------------------------------------
# File selection
# -----------------------------------------------------------------------------

def choose_files():
    """Prompt user to enter one or more CSV file paths."""
    print("\nEnter CSV file path(s). Press ENTER on an empty line when done.")
    paths = []
    while True:
        p = input(f"  File {len(paths)+1}: ").strip().strip('"').strip("'")
        if not p:
            if paths:
                break
            print("  Please enter at least one file.")
            continue
        if not os.path.isfile(p):
            print(f"  [WARN] File not found: {p}")
            continue
        paths.append(p)
    return paths

# -----------------------------------------------------------------------------
# Plotting
# -----------------------------------------------------------------------------

def plot_files(datasets):
    """
    For each loaded dataset plot:
      Panel 1 - Encoder voltage + adaptive thresholds + detected edges
      Panel 2 - RPM over time + duty cycle overlay
      Panel 3 - Current over time
    Each dataset gets its own figure.
    """
    for data in datasets:
        times    = data["time_s"]
        enc_v    = data["encoder_V"]
        current  = data["current_A"]
        duty     = data["duty_pct"]
        name     = data["name"]

        print(f"\n[INFO] Processing: {name}")
        print(f"    Duration : {times[-1]:.2f} s")
        print(f"    Samples  : {len(times):,}")
        print(f"    Enc V    : min={enc_v.min():.3f}  max={enc_v.max():.3f}  "
              f"swing={enc_v.max()-enc_v.min():.3f} V")

        rpm_times, rpm_values, edge_times, thresh_hi, thresh_lo = compute_rpm(
            times, enc_v, NUM_SLOTS)

        has_rpm = len(rpm_times) > 0

        if has_rpm:
            mean_rpm = np.mean(rpm_values)
            min_rpm  = np.min(rpm_values)
            max_rpm  = np.max(rpm_values)
            print(f"    RPM      : mean={mean_rpm:.1f}  "
                  f"min={min_rpm:.1f}  max={max_rpm:.1f}")

        # -- Figure layout ----------------------------------------------------
        fig = plt.figure(figsize=(15, 9))
        fig.suptitle(f"Motor Speed Analysis\n{name}", fontsize=12, y=0.98)

        n_panels = 3 if has_rpm else 2
        gs = gridspec.GridSpec(n_panels, 1, figure=fig,
                               hspace=0.40, top=0.91, bottom=0.07)

        # -- Panel 1: Encoder voltage + thresholds + edges --------------------
        ax1 = fig.add_subplot(gs[0])
        ax1.plot(times, enc_v,    lw=0.5, color="#2196F3", label="Encoder V",    zorder=2)
        ax1.plot(times, thresh_hi, lw=0.8, color="#FF9800", linestyle="--",
                 label="Thresh high", zorder=3)
        ax1.plot(times, thresh_lo, lw=0.8, color="#FF9800", linestyle="--",
                 label="Thresh low",  zorder=3)
        if edge_times:
            ax1.vlines(edge_times,
                       ymin=enc_v.min(), ymax=enc_v.max(),
                       colors="#FF5722", lw=0.4, alpha=0.35,
                       label=f"Rising edges ({len(edge_times)})")
        ax1.set_ylabel("Voltage (V)", fontsize=10)
        ax1.set_title("Encoder Signal + Adaptive Thresholds", fontsize=10)
        ax1.legend(fontsize=8, loc="upper right")
        ax1.grid(True, linestyle="--", alpha=0.35)

        # -- Panel 2: RPM + duty cycle ----------------------------------------
        if has_rpm:
            ax2 = fig.add_subplot(gs[1], sharex=ax1)

            # RPM on left axis
            color_rpm = "#4CAF50"
            ax2.plot(rpm_times, rpm_values, lw=1.2,
                     color=color_rpm, label="RPM", zorder=3)
            ax2.axhline(mean_rpm, color="#F44336", linestyle="--", lw=0.9,
                        label=f"Mean: {mean_rpm:.1f} RPM", zorder=2)
            ax2.set_ylabel("Speed (RPM)", fontsize=10, color=color_rpm)
            ax2.tick_params(axis='y', labelcolor=color_rpm)

            # Duty cycle on right axis
            ax2r = ax2.twinx()
            ax2r.fill_between(times, duty, alpha=0.12,
                              color="#9C27B0", step="post")
            ax2r.step(times, duty, lw=0.8, color="#9C27B0",
                      alpha=0.6, label="Duty %", where="post")
            ax2r.set_ylabel("Duty (%)", fontsize=10, color="#9C27B0")
            ax2r.tick_params(axis='y', labelcolor="#9C27B0")
            ax2r.set_ylim(-10, 150)

            # Combined legend
            lines1, labs1 = ax2.get_legend_handles_labels()
            lines2, labs2 = ax2r.get_legend_handles_labels()
            ax2.legend(lines1 + lines2, labs1 + labs2,
                       fontsize=8, loc="upper right")
            ax2.set_title("Motor Speed + Commanded Duty Cycle", fontsize=10)
            ax2.grid(True, linestyle="--", alpha=0.35)

            current_panel = gs[2]
        else:
            current_panel = gs[1]

        # -- Panel 3 (or 2 if no RPM): Current --------------------------------
        ax3 = fig.add_subplot(current_panel, sharex=ax1)
        ax3.plot(times, current, lw=0.6, color="#E91E63", label="Current (A)")
        ax3.set_ylabel("Current (A)", fontsize=10)
        ax3.set_xlabel("Time (s)",    fontsize=10)
        ax3.set_title("Motor Current", fontsize=10)
        ax3.legend(fontsize=8, loc="upper right")
        ax3.grid(True, linestyle="--", alpha=0.35)

        # Tighten x-axis to data range
        ax1.set_xlim(times[0], times[-1])

        plt.show()

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    print("=" * 50)
    print("  Motor Speed Viewer")
    print("=" * 50)

    # Accept file paths as command-line args or prompt interactively
    if len(sys.argv) > 1:
        paths = [p for p in sys.argv[1:] if os.path.isfile(p)]
        missing = [p for p in sys.argv[1:] if not os.path.isfile(p)]
        for p in missing:
            print(f"[WARN] File not found, skipping: {p}")
    else:
        paths = choose_files()

    if not paths:
        print("[ERROR] No valid files provided. Exiting.")
        sys.exit(1)

    # Load all files
    datasets = []
    for path in paths:
        print(f"\n[INFO] Loading: {path}")
        data = load_csv(path)
        if data is not None:
            datasets.append(data)

    if not datasets:
        print("[ERROR] No files loaded successfully.")
        sys.exit(1)

    print(f"\n[INFO] Loaded {len(datasets)} file(s). Generating plots...")
    plot_files(datasets)


if __name__ == "__main__":
    main()
