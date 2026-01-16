import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# -----------------------
# Configuration
# -----------------------
MAX_FREQ = 30000.0
HISTORY_LENGTH = 50
SAMPLE_FREQ = 44138
FFT_BUFF = 128
SAMPLES_PER_SEND = 256 
X_MS_PER_POINT = (1000.0 * SAMPLES_PER_SEND) / SAMPLE_FREQ
print(X_MS_PER_POINT)
# -----------------------
# Serial setup (OPEN ONCE)
# -----------------------
ser = serial.Serial(
    port='COM8',          # change if needed
    baudrate=115200,
    timeout=1
)

# -----------------------
# Data buffers
# -----------------------
freq_history = deque(maxlen=HISTORY_LENGTH)
xy_history_y = deque(maxlen=HISTORY_LENGTH)

# -----------------------
# Figure & axes
# -----------------------
fig = plt.figure(figsize=(11, 5))

ax_xy = fig.add_subplot(1, 2, 1)
ax_polar = fig.add_subplot(1, 2, 2, projection='polar')

# -----------------------
# XY plot setup
# -----------------------
ax_xy.set_title("Basic X-Y Plot")
ax_xy.set_ylim(0, 1)
ax_xy.set_xlim(0, HISTORY_LENGTH)

xy_line, = ax_xy.plot([], [], lw=2)

# -----------------------
# Polar plot setup
# -----------------------
ax_polar.set_title("Frequency History (Hz)")
ax_polar.set_rlim(0, 1)

# Frequency ticks instead of degrees
freq_ticks = np.linspace(0, MAX_FREQ, 6)
angle_ticks = (freq_ticks / MAX_FREQ) * 2 * np.pi
ax_polar.set_thetagrids(angle_ticks * 180 / np.pi,
                        labels=[f"{int(f)} Hz" for f in freq_ticks])

polar_lines = []

# -----------------------
# Read serial data
# -----------------------
def read_signal_data():
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            return None

        parts = line.split(',')

        data = {
            "total_energy": float(parts[0]),
            "average_energy": float(parts[1]),
            "current_rms": float(parts[2]),
            "top_bins": [
                int(parts[3]),
                int(parts[4]),
                int(parts[5])
            ],
            "current_avg_voltage": float(parts[6])
        }

        return data

    except (ValueError, IndexError) as e:
        print("Parse error:", e, "Line:", line)
        return None

def bin_to_freq(bin_val):
    global SAMPLE_FREQ
    global FFT_BUFF
    return (bin_val*SAMPLE_FREQ)/FFT_BUFF

# -----------------------
# Animation update
# -----------------------
def update(frame):
    data = read_signal_data()
    if data is None:
        return []

    # --- Add new data ---
    for f in data["top_bins"]:
        freq_history.append(bin_to_freq(f))
        #print(bin_to_freq(f))

    xy_history_y.append(data["current_avg_voltage"])
    print(data["current_avg_voltage"])

    # -----------------------
    # XY LINE (with fade)
    # -----------------------
    ax_xy.cla()
    ax_xy.set_title("Basic X-Y Plot")
    ax_xy.set_ylim(0, 1)
    ax_xy.set_xlim(0, HISTORY_LENGTH)

    y_vals = list(xy_history_y)
    x_vals = np.arange(len(y_vals)) * X_MS_PER_POINT

    for i in range(len(x_vals) - 1):
        alpha = (i + 1) / len(x_vals)
        ax_xy.plot(
            x_vals[i:i+2],
            y_vals[i:i+2],
            color=(0, 0, 1, alpha),
            lw=2
        )

    # -----------------------
    # POLAR LINE (with fade)
    # -----------------------
    ax_polar.cla()
    ax_polar.set_title("Frequency History (Hz)")
    ax_polar.set_rlim(0, 1)

    ax_polar.set_thetagrids(
        angle_ticks * 180 / np.pi,
        labels=[f"{int(f)} Hz" for f in freq_ticks]
    )

    freqs = list(freq_history)
    angles = [(f / MAX_FREQ) * 2 * np.pi for f in freqs]
    radii = np.ones(len(angles))

    for i in range(len(angles) - 1):
        alpha = (i + 1) / len(angles)
        ax_polar.plot(
            angles[i:i+2],
            radii[i:i+2],
            color=(1, 0, 0, alpha),
            lw=2
        )

    return []

# -----------------------
# Run animation
# -----------------------
ani = FuncAnimation(fig, update, interval=200)

plt.tight_layout()
plt.show()
