"""
Arduino Serial Reader — Binary Mode
------------------------------------
Reads 7-byte binary packets from Arduino:
  [0xAA][adc_hi][adc_lo][dt_b3][dt_b2][dt_b1][dt_b0]

Converts raw ADC counts to voltage and delta microseconds to seconds.
Press ENTER to stop. Saves CSV, plots voltage time series, and computes RPM
from rising edge detection using a local adaptive threshold.
"""

import serial
import serial.tools.list_ports
import csv
import struct
import threading
import time
import sys
import os
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    print("[ERROR] matplotlib/numpy not found. Install with: pip install matplotlib numpy")
    sys.exit(1)

# ── Configuration ─────────────────────────────────────────────────────────────
BAUD_RATE   = 500000
TIMEOUT_SEC = 2
CSV_DIR     = "."
VREF        = 5.0
ADC_MAX     = 1023

SYNC_BYTE   = 0xAA

NUM_SLOTS   = 18     # Number of slots on your rotor disc

# Adaptive threshold tuning
# Window over which local min/max are computed. Should be long enough to
# contain several full slot cycles but short enough to track slow hand-movement
# drift. ~50ms is a good starting point for 100–1000 RPM with 18 slots.
ADAPTIVE_WINDOW_SEC = 0.05   # seconds
HYSTERESIS          = 0.25   # fraction of local swing used as dead-band (0–0.5)
# ─────────────────────────────────────────────────────────────────────────────


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def choose_port():
    ports = list_serial_ports()
    if not ports:
        print("[ERROR] No serial ports found. Is your Arduino connected?")
        sys.exit(1)

    print("\nAvailable serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")

    while True:
        choice = input("Select port number (or type the port name directly): ").strip()
        if choice.isdigit() and int(choice) < len(ports):
            return ports[int(choice)]
        elif choice in ports:
            return choice
        else:
            print("  Invalid choice, try again.")


def read_serial(ser, readings, stop_event):
    print("\n[INFO] Reading from Arduino… Press ENTER to stop.\n")
    while not stop_event.is_set():
        try:
            byte = ser.read(1)
            if not byte or byte[0] != SYNC_BYTE:
                continue
            payload = ser.read(6)
            if len(payload) < 6:
                continue
            adc_raw, delta_us = struct.unpack(">HI", payload)
            voltage = adc_raw * (VREF / ADC_MAX)
            delta_s = delta_us * 1e-6
            readings.append((voltage, delta_s))
            if len(readings) % 500 == 0:
                print(f"  Voltage: {voltage:.4f} V  |  Δt: {delta_s:.6f} s  "
                      f"(total: {len(readings)})", end="\r")
        except serial.SerialException as exc:
            print(f"\n[ERROR] Serial error: {exc}")
            stop_event.set()
            break
    print()


def compute_rpm(times, voltages, num_slots,
                window_sec=ADAPTIVE_WINDOW_SEC,
                hysteresis=HYSTERESIS):
    """
    Detect rising edges using a LOCAL adaptive Schmitt trigger.

    For each sample the local min and max are computed over a sliding window
    of ±window_sec. The high/low thresholds track the signal's local baseline,
    making the detector immune to slow voltage drift from sensor movement.

    RPM is then computed per full revolution (every num_slots rising edges).
    """
    times    = np.array(times)
    voltages = np.array(voltages)
    n        = len(times)

    # ── Build adaptive thresholds ─────────────────────────────────────────────
    thresh_hi = np.empty(n)
    thresh_lo = np.empty(n)

    # Use searchsorted for fast window indexing
    for i in range(n):
        t = times[i]
        lo_idx = np.searchsorted(times, t - window_sec, side='left')
        hi_idx = np.searchsorted(times, t + window_sec, side='right')
        window = voltages[lo_idx:hi_idx]
        v_min  = window.min()
        v_max  = window.max()
        swing  = v_max - v_min
        mid    = (v_min + v_max) / 2.0
        thresh_hi[i] = mid + hysteresis * swing
        thresh_lo[i] = mid - hysteresis * swing

    # ── Schmitt trigger edge detection ────────────────────────────────────────
    edge_times = []
    state_high = voltages[0] > (thresh_hi[0] + thresh_lo[0]) / 2.0

    for i in range(1, n):
        if not state_high and voltages[i] > thresh_hi[i]:
            edge_times.append(times[i])
            state_high = True
        elif state_high and voltages[i] < thresh_lo[i]:
            state_high = False

    print(f"\n[INFO] Detected {len(edge_times)} rising edges  →  "
          f"{len(edge_times) // num_slots} complete revolutions")

    if len(edge_times) < num_slots + 1:
        print("[WARN] Not enough edges to compute RPM.\n"
              "       Try adjusting ADAPTIVE_WINDOW_SEC or HYSTERESIS.")
        return [], [], edge_times, thresh_hi, thresh_lo

    # ── RPM per revolution ────────────────────────────────────────────────────
    rpm_times, rpm_values = [], []
    for i in range(num_slots, len(edge_times)):
        t_start = edge_times[i - num_slots]
        t_end   = edge_times[i]
        period  = t_end - t_start
        if period > 0:
            rpm_values.append(60.0 / period)
            rpm_times.append((t_start + t_end) / 2.0)

    return rpm_times, rpm_values, edge_times, thresh_hi, thresh_lo


def save_csv(readings, csv_path):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "time_s", "voltage_V", "delta_time_s"])
        t = 0.0
        for i, (voltage, delta_t) in enumerate(readings):
            writer.writerow([i, f"{t:.6f}", f"{voltage:.6f}", f"{delta_t:.6f}"])
            t += delta_t
    print(f"[INFO] Data saved to: {os.path.abspath(csv_path)}")


def plot_results(times, voltages, rpm_times, rpm_values,
                 edge_times, thresh_hi, thresh_lo, csv_path):
    has_rpm = len(rpm_times) > 0

    fig, axes = plt.subplots(2 if has_rpm else 1, 1,
                             figsize=(14, 8 if has_rpm else 5),
                             sharex=False)
    if not has_rpm:
        axes = [axes]

    # ── Panel 1: Voltage + adaptive thresholds + edges ───────────────────────
    ax1 = axes[0]
    ax1.plot(times, voltages,  lw=0.5,  color="#2196F3", label="Voltage",       zorder=2)
    ax1.plot(times, thresh_hi, lw=0.8,  color="#FF9800", label="Thresh high",   zorder=3, linestyle="--")
    ax1.plot(times, thresh_lo, lw=0.8,  color="#FF9800", label="Thresh low",    zorder=3, linestyle="--")
    if edge_times:
        ax1.vlines(edge_times, ymin=min(voltages), ymax=max(voltages),
                   colors="#FF5722", lw=0.4, alpha=0.4, label="Rising edges")
    ax1.set_ylabel("Voltage (V)", fontsize=11)
    ax1.set_title("Arduino Voltage Time Series", fontsize=13)
    ax1.grid(True, linestyle="--", alpha=0.4)
    ax1.legend(fontsize=9)

    # ── Panel 2: RPM ─────────────────────────────────────────────────────────
    if has_rpm:
        ax2 = axes[1]
        ax2.plot(rpm_times, rpm_values, lw=1.2, color="#4CAF50", label="RPM")
        mean_rpm = np.mean(rpm_values)
        ax2.axhline(mean_rpm, color="#F44336", linestyle="--",
                    lw=0.9, label=f"Mean: {mean_rpm:.1f} RPM")
        ax2.set_xlabel("Time (s)", fontsize=11)
        ax2.set_ylabel("Speed (RPM)", fontsize=11)
        ax2.set_title("Motor Speed", fontsize=13)
        ax2.grid(True, linestyle="--", alpha=0.4)
        ax2.legend(fontsize=9)
        print(f"[INFO] Mean RPM: {mean_rpm:.1f}  |  "
              f"Min: {min(rpm_values):.1f}  |  Max: {max(rpm_values):.1f}")
    else:
        axes[0].set_xlabel("Time (s)", fontsize=11)

    fig.tight_layout()
    plot_path = csv_path.replace(".csv", ".png")
    fig.savefig(plot_path, dpi=150)
    print(f"[INFO] Plot saved to: {os.path.abspath(plot_path)}")
    plt.show()


def main():
    port = choose_port()

    print(f"\n[INFO] Opening {port} at {BAUD_RATE} baud…")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_SEC)
        time.sleep(2)
        ser.reset_input_buffer()
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open port: {exc}")
        sys.exit(1)

    readings   = []
    stop_event = threading.Event()
    reader_thread = threading.Thread(target=read_serial,
                                     args=(ser, readings, stop_event),
                                     daemon=True)
    reader_thread.start()

    input()  # block until ENTER
    stop_event.set()
    reader_thread.join(timeout=3)
    ser.close()

    print(f"\n[INFO] Stopped. Collected {len(readings)} samples.")
    if not readings:
        print("[WARN] No data collected. Exiting.")
        return

    # Reconstruct absolute time axis
    times, voltages = [], []
    t = 0.0
    for voltage, delta_t in readings:
        times.append(t)
        voltages.append(voltage)
        t += delta_t

    print("[INFO] Computing RPM (adaptive threshold)…")
    rpm_times, rpm_values, edge_times, thresh_hi, thresh_lo = compute_rpm(
        times, voltages, NUM_SLOTS)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path  = os.path.join(CSV_DIR, f"arduino_data_{timestamp}.csv")
    save_csv(readings, csv_path)
    plot_results(times, voltages, rpm_times, rpm_values,
                 edge_times, thresh_hi, thresh_lo, csv_path)


if __name__ == "__main__":
    main()