import serial
import struct

PORT = "COM8"
BAUDRATE = 115200
TIMEOUT = 0.1

def read_adc_value(ser=None):
    """Read a single 4-byte ADC value from UART."""
    if ser is None:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

    data = ser.read(4)
    if len(data) < 4:
        return None  # incomplete read

    # Convert 4 bytes to integer (little-endian)
    value = struct.unpack("<I", data)[0]
    return value


# Example usage:
ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
values = []

while True:
    val = read_adc_value(ser)
    if val is not None:
        values.append(val)
        print(val)
