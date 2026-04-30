"""
motor_characterization.py
--------------------------
Receives 11-byte binary packets from the Arduino motor characterization sketch,
records measurement groups, and on exit computes the motor's:
  * Coefficient of torque  Kt  (N.m / A)
  * Coefficient of friction  B  (N.m.s / rad)

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

# -- Packet constants ----------------------------------------------------------
BAUD_RATE  = 500_000
TIMEOUT_SEC = 2
SYNC_BYTE  = 0xBB
END_BYTE   = 0xEE
PACKET_LEN = 11       # total bytes including sync + end

VREF    = 5.0
ADC_MAX = 1023.0

# -- Encoder disc --------------------------------------------------------------
NUM_SLOTS = 18        # slots on the rotor disc

# -- Adaptive RPM threshold tuning (same approach as existing logger) ----------
ADAPTIVE_WINDOW_SEC = 0.05
HYSTERESIS          = 0.25

# -----------------------------------------------------------------------------
# Helper: list / choose serial port
# -----------------------------------------------------------------------------

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

# -----------------------------------------------------------------------------
# Serial reader thread
# -----------------------------------------------------------------------------

def read_serial(ser, records, stop_event):
    """
    Reads 11-byte packets:
      sync | adc_current(2) | adc_encoder(2) | duty(1) | delta_us(4) | end
    Appends (adc_current, adc_encoder, duty_pct, delta_us) tuples to records.
    """
    print("\n[INFO] Reading from Arduino...  Type 'esc' then ENTER to stop.\n")
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

                # Unpack: >HHBIx  -> but easier with manual slice
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

# -----------------------------------------------------------------------------
# RPM via adaptive Schmitt trigger
# Ported exactly from Arduino_Port_Data_Logger.py :: compute_rpm()
# -----------------------------------------------------------------------------

def compute_rpm(times, voltages, num_slots,
                window_sec=ADAPTIVE_WINDOW_SEC,
                hysteresis=HYSTERESIS):
    """
    Detect rising edges using a LOCAL adaptive Schmitt trigger.

    For each sample the local min and max are computed over a sliding window
    of +-window_sec. The high/low thresholds track the signal's local baseline,
    making the detector immune to slow voltage drift from sensor movement.

    RPM is then computed per full revolution (every num_slots rising edges).
    """
    times    = np.array(times)
    voltages = np.array(voltages)
    n        = len(times)

    # -- Build adaptive thresholds --------------------------------------------
    thresh_hi = np.empty(n)
    thresh_lo = np.empty(n)

    # Use searchsorted for fast window indexing
    for i in range(n):
        t      = times[i]
        lo_idx = np.searchsorted(times, t - window_sec, side='left')
        hi_idx = np.searchsorted(times, t + window_sec, side='right')
        window = voltages[lo_idx:hi_idx]
        v_min  = window.min()
        v_max  = window.max()
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

    print(f"\n[INFO] Detected {len(edge_times)} rising edges  ->  "
          f"{len(edge_times) // num_slots} complete revolutions")

    if len(edge_times) < num_slots + 1:
        print("[WARN] Not enough edges to compute RPM.\n"
              "       Try adjusting ADAPTIVE_WINDOW_SEC or HYSTERESIS.")
        return np.array([]), np.array([])

    # -- RPM per revolution ---------------------------------------------------
    rpm_times, rpm_values = [], []
    for i in range(num_slots, len(edge_times)):
        t_start = edge_times[i - num_slots]
        t_end   = edge_times[i]
        period  = t_end - t_start
        if period > 0:
            rpm_values.append(60.0 / period)
            rpm_times.append((t_start + t_end) / 2.0)

    return np.array(rpm_times), np.array(rpm_values)

# -----------------------------------------------------------------------------
# Motor parameter estimation
# -----------------------------------------------------------------------------

def estimate_motor_params(times, currents, rpm_times, rpm_values, J):
    """
    Solves the overdetermined linear system derived from the mechanical equation:
        Kt * I(t) = J * domega/dt + B * omega(t)
    Rearranged:
        [I(t), -omega(t)] * [Kt, B]^T = J * domega/dt
    Solved via least squares.

    Parameters
    ----------
    times      : 1-D array, absolute time axis of current samples (s)
    currents   : 1-D array, motor current at each time step (A)
    rpm_times  : 1-D array, time stamps of RPM estimates
    rpm_values : 1-D array, RPM at each rpm_times point
    J          : float, rotor moment of inertia (kg.m^2)

    Returns
    -------
    Kt : float  torque constant  (N.m / A)
    B  : float  viscous friction (N.m.s / rad)
    residuals : array
    """
    if len(rpm_times) < 4:
        raise ValueError("Too few RPM points to estimate parameters. "
                         "Collect more data.")

    # Convert RPM -> rad/s
    omega_raw = rpm_values * (2.0 * np.pi / 60.0)

    # -- Savitzky-Golay smoothing ----------------------------------------------
    # Use a large window (~20 % of data length) to suppress noise before
    # differentiation. The wide RPM swings seen during phase transitions make
    # a small window produce a very noisy derivative and a negative B estimate.
    n_pts     = len(omega_raw)
    sg_window = max(5, int(n_pts * 0.20))       # 20 % of points
    if sg_window % 2 == 0:
        sg_window += 1                           # must be odd
    sg_window = min(sg_window, n_pts if n_pts % 2 == 1 else n_pts - 1)
    omega_smooth = savgol_filter(omega_raw, window_length=sg_window, polyorder=3)

    # -- Numerical derivative --------------------------------------------------
    domega_dt = np.gradient(omega_smooth, rpm_times)

    # -- Exclude phase-transition transients -----------------------------------
    # Rows where |dw/dt| is in the top 10 % are at spin-up / spin-down edges.
    # These transient samples have very large acceleration and disproportionately
    # corrupt the least-squares fit, biasing B negative.
    threshold  = np.percentile(np.abs(domega_dt), 90)
    keep       = np.abs(domega_dt) <= threshold

    # Interpolate current onto the RPM time grid then apply mask
    current_interp = np.interp(rpm_times, times, currents)

    omega_fit   = omega_smooth[keep]
    domega_fit  = domega_dt[keep]
    current_fit = current_interp[keep]

    if len(omega_fit) < 4:
        raise ValueError("Too few non-transient points for least squares. "
                         "Collect more data or reduce the transient mask threshold.")

    # -- Least-squares solve:  A * [Kt, B]^T = b ------------------------------
    #   A[:,0] =  I(t)    coefficient of Kt
    #   A[:,1] = -omega   coefficient of B  (Kt*I - B*w = J*dw/dt)
    A = np.column_stack([current_fit, -omega_fit])
    b = J * domega_fit

    result    = np.linalg.lstsq(A, b, rcond=None)
    params    = result[0]
    residuals = result[1]

    Kt = params[0]
    B  = params[1]
    return Kt, B, residuals

# -----------------------------------------------------------------------------
# CSV save
# -----------------------------------------------------------------------------

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
    print(f"[INFO] Data saved -> {os.path.abspath(csv_path)}")

# -----------------------------------------------------------------------------
# Results text file
# -----------------------------------------------------------------------------

def save_results(Kt, B, J, n_samples, txt_path):
    lines = [
        "========================================",
        " Arduino Motor Characterization Results",
        "========================================",
        f"  Timestamp  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"  Samples    : {n_samples:,}",
        f"  J (rotor)  : {J:.6e}  kg m^2",
        "",
        f"  Kt (torque constant)   = {Kt:.6f}  N m / A",
        f"  B  (viscous friction)  = {B:.6e}  N m s / rad",
        "",
        "  Note: Kt ~= Ke (back-EMF constant) in SI units.",
        "========================================",
    ]
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    for line in lines:
        print(line)
    print(f"\n[INFO] Results saved -> {os.path.abspath(txt_path)}")

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    print("=" * 50)
    print("  Arduino Motor Characterization System")
    print("=" * 50)

    # -- User inputs -----------------------------------------------------------
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
    print("  Typical small DC motor rotor: 1e-7 to 1e-5 kg.m^2")
    print("  To convert:  g.cm^2 -> kg.m^2  multiply by 1e-7")
    print("               g.mm^2 -> kg.m^2  multiply by 1e-9")
    while True:
        try:
            J = float(input("  J [kg.m^2]: ").strip())
            if J <= 0:
                print("  J must be positive.")
                continue
            if J > 0.1:
                print(f"  [WARN] J = {J:.4e} kg.m^2 seems very large for a motor rotor.")
                print(f"         Did you mean {J*1e-7:.4e} kg.m^2 (if entered in g.cm^2)?")
                confirm = input("  Continue with this value? [y/n]: ").strip().lower()
                if confirm != 'y':
                    continue
            break
        except ValueError:
            print("  Please enter a numeric value.")

    port = choose_port()

    # -- Open serial port ------------------------------------------------------
    print(f"\n[INFO] Opening {port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_SEC)
        time.sleep(2)          # let Arduino reset
        ser.reset_input_buffer()
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open port: {exc}")
        sys.exit(1)

    # -- Start reader thread ---------------------------------------------------
    records    = []
    stop_event = threading.Event()
    reader     = threading.Thread(target=read_serial,
                                  args=(ser, records, stop_event),
                                  daemon=True)
    reader.start()

    # -- Wait for Escape key ---------------------------------------------------
    if PYNPUT_AVAILABLE:
        print("[INFO] Press the  Esc  key to stop recording.\n")

        def on_press(key):
            if key == pynput_keyboard.Key.esc:
                stop_event.set()
                return False   # stops the pynput listener

        with pynput_keyboard.Listener(on_press=on_press) as listener:
            listener.join()    # blocks until Esc is pressed
    else:
        # Fallback: pynput not installed - poll for Escape via stdin raw mode.
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
        print("[WARN] Very few samples - results may be unreliable.")
    if n == 0:
        print("[ERROR] No data collected. Exiting.")
        return

    # -- Reconstruct time axis and physical quantities -------------------------
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

    # -- Build file names ------------------------------------------------------
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path  = f"Data\Arduino motor data_{timestamp}_{n}pts.csv"
    txt_path  = f"Data\Arduino motor characteristics_{timestamp}_{n}pts.txt"

    # -- Save raw CSV ----------------------------------------------------------
    save_csv(records, times, currents, enc_voltages, csv_path)

    # -- Compute RPM -----------------------------------------------------------
    print("[INFO] Computing RPM (adaptive threshold)...")
    rpm_times, rpm_values = compute_rpm(times, enc_voltages, NUM_SLOTS)

    if len(rpm_times) == 0:
        print("[ERROR] Could not compute RPM - cannot estimate motor parameters.")
        return

    print(f"[INFO] RPM  mean={np.mean(rpm_values):.1f}  "
          f"min={np.min(rpm_values):.1f}  max={np.max(rpm_values):.1f}")

    # -- Estimate Kt and B -----------------------------------------------------
    print("[INFO] Estimating Kt and B via least squares...")
    try:
        Kt, B, residuals = estimate_motor_params(
            times, currents, rpm_times, rpm_values, J)
    except ValueError as exc:
        print(f"[ERROR] Parameter estimation failed: {exc}")
        return

    if Kt <= 0:
        print("[WARN] Kt is negative or zero - check current sensor polarity "
              "and calibration values.")
    if B < 0:
        print("[WARN] B is negative - check encoder signal or increase data "
              "length to cover more spin-down phases.")

    # -- Save and display results ----------------------------------------------
    save_results(Kt, B, J, n, txt_path)


if __name__ == "__main__":
    main()