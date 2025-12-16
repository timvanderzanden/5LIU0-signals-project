import matplotlib.pyplot as plt
import numpy as np
import time
import serial
from collections import deque
import struct

# ---------- Configuration ----------
PORT = "COM8"
BAUDRATE = 115200
TIMEOUT = 0.1
WINDOW_SIZE = 500  # Number of points visible on x-axis
FIG_SIZE = (10, 5)  # Width, Height in inches

plt.ion()
fig, ax = plt.subplots(figsize=FIG_SIZE)

# Using deque for efficient fixed-size scrolling window
x = deque(maxlen=WINDOW_SIZE)
y = deque(maxlen=WINDOW_SIZE)
line, = ax.plot([], [])

ax.set_xlabel("Time (ms)")
ax.set_ylabel("Voltage")
ax.set_title("Real-Time Voltage Plot")

lifetime_energy = 0


def read_serial_string(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT):
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            buffer = ""
            while True:
                chunk = ser.read(ser.in_waiting or 1).decode('utf-8', errors='ignore')
                if chunk:
                    buffer += chunk
                    if '\n' in buffer:
                        line_str, buffer = buffer.split('\n', 1)
                        return line_str.strip()
    except serial.SerialException as e:
        print("Serial error:", e)
        return None

HEADER = b'\xAA\x55'
PACKET_SIZE = 14  # 2 bytes header + 12 bytes payload

def read_binary_packet(ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)):
    while True:
        # Step 1: find the header
        byte = ser.read(1)
        if byte == HEADER[:1]:
            second = ser.read(1)
            if second == HEADER[1:2]:
                # Step 2: read the payload (12 bytes)
                payload = ser.read(12)
                if len(payload) == 12:
                    # Step 3: unpack payload: time1, voltage1, time2, voltage2
                    time1, voltage1, time2, voltage2 = struct.unpack('<IHIH', payload)
                    return time1, voltage1, time2, voltage2



def get_new_signal_values():
    global lifetime_energy
    '''
    global lifetime_energy
    string_data = read_serial_string()
    if string_data is None:
        return None

    data_array = string_data.split(";")
    if len(data_array) != 5:
        return None
    if data_array[0] != "?":
        return None
    '''
    data_array = read_binary_packet()

    try:
        time_stamp = int(data_array[0])
        current_voltage = int(data_array[1])
        rms_voltage = int(data_array[2])
        short_time_energy = int(data_array[3])
        lifetime_energy += short_time_energy
        return time_stamp, current_voltage, rms_voltage, short_time_energy, lifetime_energy
    except ValueError:
        return None


while True:
    if not plt.fignum_exists(fig.number):
        break

    signal_values = get_new_signal_values()
    if signal_values is None:
        continue

    time_stamp, current_voltage, rms_voltage, short_time_energy, lifetime_energy = signal_values

    x.append(time_stamp)
    y.append(current_voltage)

    line.set_xdata(x)
    line.set_ydata(y)

    # Adjust x-axis to show only the last WINDOW_SIZE points
    if len(x) > 1:
        ax.set_xlim(x[0], x[-1])
        ax.set_ylim(min(y) - 10, max(y) + 10)  # Add small margin

    plt.draw()
    plt.pause(0.01)

print("Plot closed. Exiting cleanly.")
