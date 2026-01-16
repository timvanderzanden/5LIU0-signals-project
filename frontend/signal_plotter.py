import serial
import numpy as np
import struct

# ----------------- CONFIG -----------------
SERIAL_PORT = 'COM8'  # Replace with your actual port
BAUD = 115200
ADC_LEN = 128          # samples per packet
PACKET_SIZE = 4 + 2*ADC_LEN  # 4 bytes timestamp + 128 * 2 bytes samples

# ----------------- OPEN SERIAL -----------------
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

print("Starting debug read...")

try:
    while True:
        raw = ser.read(PACKET_SIZE)
        if len(raw) != PACKET_SIZE:
            print("Incomplete packet, skipping")
            continue

        # Unpack timestamp
        t0_us, = struct.unpack('<I', raw[:4])
        # Unpack ADC samples
        samples = np.frombuffer(raw[4:], dtype=np.uint16)

        # Print timestamp and first 10 samples
        print(f"Timestamp: {t0_us} us | First 10 ADC samples: {samples[:10]}")
except KeyboardInterrupt:
    ser.close()
    print("Stopped.")
