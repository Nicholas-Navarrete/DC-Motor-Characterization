"""
motor_characterization.py
--------------------------
Receives 11-byte binary packets from the Arduino motor characterization sketch,
records measurement groups, and on exit computes the motor's:
  • Coefficient of torque  Kt  (N·m / A)
  • Coefficient of friction  B  (N·m·s / rad)

Packet format (11 bytes, big-endian):
  [0]      0xBB  sync byte
  [1-2]    uint16  A0 raw ADC  (shunt / current sense)
  [3-4]    uint16  A1 raw ADC  (optical encoder)
  [5]      uint8   commanded duty cycle 0-100 (%)
  [6-9]    uint32  delta time in microseconds
  [10]     0xEE  end byte

Stop recording by typing 'esc' and pressing ENTER.

Author: Nicholas Navarrete
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
    import numpy as np
    from scipy.signal import savgol_filter
except ImportError:
    print("[ERROR] numpy / scipy not found. Install with:")
    print("        pip install numpy scipy")
    sys.exit(1)

try:
    from pynput import keyboard as pynput_keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False

# ── Packet constants ──────────────────────────────────────────────────────────
BAUD_RATE  = 500_000
TIMEOUT_SEC = 2
SYNC_BYTE  = 0xBB
END_BYTE   = 0xEE
PACKET_LEN = 11       # total bytes including sync + end

VREF    = 5.0
ADC_MAX = 1023.0

# ── Encoder disc ──────────────────────────────────────────────────────────────
NUM_SLOTS = 18        # slots on the rotor disc

# ── Adaptive RPM threshold tuning (same approach as existing logger) ──────────
ADAPTIVE_WINDOW_SEC = 0.05
HYSTERESIS          = 0.25

# ─────────────────────────────────────────────────────────────────────────────
# Helper: list / choose serial port
# ─────────────────────────────────────────────────────────────────────────────

def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def choose_port():
    ports = list_serial_ports()
    if not ports:
        print("[ERROR] No serial ports found. Is the Arduino connected?")
        sys.exit(1)
    print("\nAvailable serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")
    while True:
        choice = input("Select port number (or type the port name directly): ").strip()
        if choice.isdigit() and int(choice) < len(ports):
            return ports[int(choice)]
        if choice in ports:
            return choice
        print("  Invalid choice, try again.")

# ─────────────────────────────────────────────────────────────────────────────
# Serial reader thread
# ─────────────────────────────────────────────────────────────────────────────

def read_serial(ser, records, stop_event):
    """
    Reads 11-byte packets:
      sync | adc_current(2) | adc_encoder(2) | duty(1) | delta_us(4) | end
    Appends (adc_current, adc_encoder, duty_pct, delta_us) tuples to records.
    """
    print("\n[INFO] Reading from Arduino…  Type 'esc' then ENTER to stop.\n")
    buf = bytearray()

    while not stop_event.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf.extend(chunk)

            # Process as many complete packets as are buffered
            while len(buf) >= PACKET_LEN:
                # Hunt for sync byte
                if buf[0] != SYNC_BYTE:
                    buf.pop(0)
                    continue
                # Check end byte at expected position
                if buf[PACKET_LEN - 1] != END_BYTE:
                    buf.pop(0)
                    continue

                # Unpack: >HHBIx  → but easier with manual slice
                adc_current = (buf[1] << 8) | buf[2]
                adc_encoder = (buf[3] << 8) | buf[4]
                duty_pct    = buf[5]
                delta_us    = struct.unpack(">I", bytes(buf[6:10]))[0]

                records.append((adc_current, adc_encoder, duty_pct, delta_us))
                del buf[:PACKET_LEN]

                if len(records) % 1000 == 0:
                    print(f"  Packets received: {len(records):,}", end="\r")

        except serial.SerialException as exc:
            print(f"\n[ERROR] Serial error: {exc}")
            stop_event.set()
            break
    print()

# ─────────────────────────────────────────────────────────────────────────────
# RPM via adaptive Schmitt trigger (same algorithm as existing logger)
# ─────────────────────────────────────────────────────────────────────────────

def compute_rpm(times, enc_voltages, num_slots,
                window_sec=ADAPTIVE_WINDOW_SEC,
                hysteresis=HYSTERESIS):
    times        = np.array(times)
    enc_voltages = np.array(enc_voltages)
    n            = len(times)

    thresh_hi = np.empty(n)
    thresh_lo = np.empty(n)

    for i in range(n):
        t      = times[i]
        lo_idx = np.searchsorted(times, t - window_sec, side='left')
        hi_idx = np.searchsorted(times, t + window_sec, side='right')
        window = enc_voltages[lo_idx:hi_idx]
        v_min  = window.min()
        v_max  = window.max()
        swing  = v_max - v_min
        mid    = (v_min + v_max) / 2.0
        thresh_hi[i] = mid + hysteresis * swing
        thresh_lo[i] = mid - hysteresis * swing

    edge_times = []
    state_high = enc_voltages[0] > (thresh_hi[0] + thresh_lo[0]) / 2.0

    for i in range(1, n):
        if not state_high and enc_voltages[i] > thresh_hi[i]:
            edge_times.append(times[i])
            state_high = True
        elif state_high and enc_voltages[i] < thresh_lo[i]:
            state_high = False

    print(f"[INFO] Detected {len(edge_times)} rising edges → "
          f"{len(edge_times) // num_slots} complete revolutions")

    if len(edge_times) < num_slots + 1:
        print("[WARN] Not enough edges to compute RPM. "
              "Check ADAPTIVE_WINDOW_SEC / HYSTERESIS or encoder wiring.")
        return np.array([]), np.array([])

    rpm_times, rpm_values = [], []
    for i in range(num_slots, len(edge_times)):
        t_start = edge_times[i - num_slots]
        t_end   = edge_times[i]
        period  = t_end - t_start
        if period > 0:
            rpm_values.append(60.0 / period)
            rpm_times.append((t_start + t_end) / 2.0)

    return np.array(rpm_times), np.array(rpm_values)

# ─────────────────────────────────────────────────────────────────────────────
# Motor parameter estimation
# ─────────────────────────────────────────────────────────────────────────────

def estimate_motor_params(times, currents, rpm_times, rpm_values, J):
    """
    Solves the overdetermined linear system derived from the mechanical equation:
        Kt * I(t) = J * dω/dt + B * ω(t)
    Rearranged:
        [I(t), -ω(t)] * [Kt, B]^T = J * dω/dt
    Solved via least squares.

    Parameters
    ----------
    times      : 1-D array, absolute time axis of current samples (s)
    currents   : 1-D array, motor current at each time step (A)
    rpm_times  : 1-D array, time stamps of RPM estimates
    rpm_values : 1-D array, RPM at each rpm_times point
    J          : float, rotor moment of inertia (kg·m²)

    Returns
    -------
    Kt : float  torque constant  (N·m / A)
    B  : float  viscous friction (N·m·s / rad)
    residuals : array
    """
    if len(rpm_times) < 4:
        raise ValueError("Too few RPM points to estimate parameters. "
                         "Collect more data.")

    # Convert RPM → rad/s
    omega_rpm = rpm_values * (2.0 * np.pi / 60.0)

    # Smooth omega before differentiating to suppress noise
    # Savitzky-Golay: window must be odd and > polyorder
    sg_window = min(51, len(omega_rpm) if len(omega_rpm) % 2 == 1
                    else len(omega_rpm) - 1)
    sg_window = max(sg_window, 5)
    omega_smooth = savgol_filter(omega_rpm, window_length=sg_window, polyorder=3)

    # Numerical derivative dω/dt using central differences on smoothed ω
    domega_dt = np.gradient(omega_smooth, rpm_times)

    # Interpolate current onto the RPM time grid
    current_interp = np.interp(rpm_times, times, currents)

    # Build least-squares system:  A * [Kt, B]^T = b
    #   A[:,0] = I(t)       coefficient of Kt
    #   A[:,1] = -ω(t)      coefficient of B  (note sign: Kt*I - B*ω = J*dω/dt)
    A = np.column_stack([current_interp, -omega_smooth])
    b = J * domega_dt

    result = np.linalg.lstsq(A, b, rcond=None)
    params    = result[0]
    residuals = result[1]

    Kt = params[0]
    B  = params[1]
    return Kt, B, residuals

# ─────────────────────────────────────────────────────────────────────────────
# CSV save
# ─────────────────────────────────────────────────────────────────────────────

def save_csv(records, times, currents, enc_voltages, csv_path):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "time_s", "current_A",
                         "encoder_V", "duty_pct", "delta_us"])
        for i, (adc_c, adc_e, duty, delta_us) in enumerate(records):
            writer.writerow([
                i,
                f"{times[i]:.6f}",
                f"{currents[i]:.6f}",
                f"{enc_voltages[i]:.6f}",
                duty,
                delta_us,
            ])
    print(f"[INFO] Data saved → {os.path.abspath(csv_path)}")

# ─────────────────────────────────────────────────────────────────────────────
# Results text file
# ─────────────────────────────────────────────────────────────────────────────

def save_results(Kt, B, J, n_samples, txt_path):
    lines = [
        "========================================",
        " Arduino Motor Characterization Results",
        "========================================",
        f"  Timestamp  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"  Samples    : {n_samples:,}",
        f"  J (rotor)  : {J:.6e}  kg·m²",
        "",
        f"  Kt (torque constant)   = {Kt:.6f}  N·m / A",
        f"  B  (viscous friction)  = {B:.6e}  N·m·s / rad",
        "",
        "  Note: Kt ≈ Ke (back-EMF constant) in SI units.",
        "========================================",
    ]
    with open(txt_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    for line in lines:
        print(line)
    print(f"\n[INFO] Results saved → {os.path.abspath(txt_path)}")

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 50)
    print("  Arduino Motor Characterization System")
    print("=" * 50)

    # ── User inputs ───────────────────────────────────────────────────────────
    print("\n-- Current sensor calibration --")
    print("  (Values from CurrentSensingCalibration.ino)")
    while True:
        try:
            cal_slope  = float(input("  Slope  [AnalogRead / A]: ").strip())
            cal_offset = float(input("  Offset [AnalogRead]    : ").strip())
            break
        except ValueError:
            print("  Please enter numeric values.")

    print("\n-- Rotor moment of inertia --")
    while True:
        try:
            J = float(input("  J [kg·m²]: ").strip())
            if J > 0:
                break
            print("  J must be positive.")
        except ValueError:
            print("  Please enter a numeric value.")

    port = choose_port()

    # ── Open serial port ──────────────────────────────────────────────────────
    print(f"\n[INFO] Opening {port} at {BAUD_RATE} baud…")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_SEC)
        time.sleep(2)          # let Arduino reset
        ser.reset_input_buffer()
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open port: {exc}")
        sys.exit(1)

    # ── Start reader thread ───────────────────────────────────────────────────
    records    = []
    stop_event = threading.Event()
    reader     = threading.Thread(target=read_serial,
                                  args=(ser, records, stop_event),
                                  daemon=True)
    reader.start()

    # ── Wait for Escape key ───────────────────────────────────────────────────
    if PYNPUT_AVAILABLE:
        print("[INFO] Press the  Esc  key to stop recording.\n")

        def on_press(key):
            if key == pynput_keyboard.Key.esc:
                stop_event.set()
                return False   # stops the pynput listener

        with pynput_keyboard.Listener(on_press=on_press) as listener:
            listener.join()    # blocks until Esc is pressed
    else:
        # Fallback: pynput not installed — poll for Escape via stdin raw mode.
        print("[INFO] pynput not found. Install with: pip install pynput")
        print("[INFO] Falling back to raw-terminal mode. Press Esc to stop.\n")
        import tty, termios
        fd   = sys.stdin.fileno()
        old  = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not stop_event.is_set():
                ch = sys.stdin.read(1)
                if ch == "\x1b":   # ESC character
                    stop_event.set()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    stop_event.set()
    reader.join(timeout=3)
    ser.close()

    n = len(records)
    print(f"\n[INFO] Stopped. Collected {n:,} samples.")
    if n < 100:
        print("[WARN] Very few samples — results may be unreliable.")
    if n == 0:
        print("[ERROR] No data collected. Exiting.")
        return

    # ── Reconstruct time axis and physical quantities ─────────────────────────
    times        = np.empty(n)
    currents     = np.empty(n)
    enc_voltages = np.empty(n)
    duty_arr     = np.empty(n, dtype=np.uint8)

    t = 0.0
    for i, (adc_c, adc_e, duty, delta_us) in enumerate(records):
        times[i]        = t
        # Current from shunt: (ADC_raw - offset) / slope
        currents[i]     = (adc_c - cal_offset) / cal_slope
        enc_voltages[i] = adc_e * (VREF / ADC_MAX)
        duty_arr[i]     = duty
        t              += delta_us * 1e-6

    # ── Build file names ──────────────────────────────────────────────────────
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path  = f"Arduino motor data_{timestamp}_{n}pts.csv"
    txt_path  = f"Arduino motor characteristics_{timestamp}_{n}pts.txt"

    # ── Save raw CSV ──────────────────────────────────────────────────────────
    save_csv(records, times, currents, enc_voltages, csv_path)

    # ── Compute RPM ───────────────────────────────────────────────────────────
    print("[INFO] Computing RPM (adaptive threshold)…")
    rpm_times, rpm_values = compute_rpm(times, enc_voltages, NUM_SLOTS)

    if len(rpm_times) == 0:
        print("[ERROR] Could not compute RPM — cannot estimate motor parameters.")
        return

    print(f"[INFO] RPM  mean={np.mean(rpm_values):.1f}  "
          f"min={np.min(rpm_values):.1f}  max={np.max(rpm_values):.1f}")

    # ── Estimate Kt and B ─────────────────────────────────────────────────────
    print("[INFO] Estimating Kt and B via least squares…")
    try:
        Kt, B, residuals = estimate_motor_params(
            times, currents, rpm_times, rpm_values, J)
    except ValueError as exc:
        print(f"[ERROR] Parameter estimation failed: {exc}")
        return

    if Kt <= 0:
        print("[WARN] Kt is negative or zero — check current sensor polarity "
              "and calibration values.")
    if B < 0:
        print("[WARN] B is negative — check encoder signal or increase data "
              "length to cover more spin-down phases.")

    # ── Save and display results ──────────────────────────────────────────────
    save_results(Kt, B, J, n, txt_path)


if __name__ == "__main__":
    main()
