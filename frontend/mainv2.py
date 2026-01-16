import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
import sys
import struct

# -----------------------
# Configuration
# -----------------------
PACKET_FORMAT = "<Iffff128f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
SYNC_WORD = 0xABCDEFFF

MAX_FREQ = 20000.0
SAMPLE_FREQ = 44138
FFT_BUFF = 128
NUM_BINS = FFT_BUFF // 2

RADIUS = 1.0
NUM_FREQ_TICKS = 12

# -----------------------
# Serial Setup
# -----------------------
try:
  ser = serial.Serial(port='COM8', baudrate=921600, timeout=0.1)
  ser.flushInput()
except Exception as e:
  print(f"Serial Error: {e}")
  sys.exit(1)

# -----------------------
# Helpers
# -----------------------
def reconstruct_signal(fft_complex_half):
  full_fft = np.zeros(FFT_BUFF, dtype=complex)
  full_fft[:64] = fft_complex_half
  full_fft[65:] = np.conj(fft_complex_half[1:64][::-1])
 
  # 1. Perform the Inverse FFT
  raw_reconstructed = np.fft.ifft(full_fft).real
 
  # 2. Add back the 2048 we subtracted in firmware
  adc_counts = raw_reconstructed + 2048.0
 
  # 3. Convert ADC counts to Volts (3.3V / 4095)
  voltage = adc_counts * (3.3 / 4095.0)
 
  return voltage

def read_signal_data():
  if ser.in_waiting < PACKET_SIZE:
    return None
  raw_data = ser.read(ser.in_waiting)
  sync_bytes = struct.pack("<I", SYNC_WORD)
  sync_idx = raw_data.rfind(sync_bytes)
 
  if sync_idx == -1 or len(raw_data[sync_idx:]) < PACKET_SIZE:
    return None
   
  packet = raw_data[sync_idx : sync_idx + PACKET_SIZE]
  try:
    unpacked = struct.unpack(PACKET_FORMAT, packet)
    real_parts = np.array(unpacked[5:69])
    imag_parts = np.array(unpacked[69:133])
    return {
      "total_energy" : unpacked[1],
      "average_energy" : unpacked[2],
      "rms": unpacked[3],
      "avg_v": unpacked[4],
      "fft_complex": real_parts + 1j * imag_parts,
      "fft_mags": np.abs(real_parts + 1j * imag_parts)
    }
  except: return None

# -----------------------
# UI Setup
# -----------------------
app = QtWidgets.QApplication(sys.argv)
pg.setConfigOptions(antialias=True)
win = pg.GraphicsLayoutWidget(title="Signal Analyzer")
win.resize(1200, 800)
win.show()

hud = win.addLabel(text="Initializing...", color='w', size='12pt')
hud.setMaximumHeight(60)

win.nextRow()
wave_plot = win.addPlot(title="Waveform (x=samples,y=voltage)")
wave_curve = wave_plot.plot(pen='g')

radial_plot = win.addPlot(title="Radial Spectrum in Hz")
radial_plot.setAspectLocked()
radial_plot.hideAxis('bottom')
radial_plot.hideAxis('left')

circle_item = QtWidgets.QGraphicsEllipseItem(-RADIUS, -RADIUS, RADIUS*2, RADIUS*2)
circle_item.setPen(pg.mkPen(color=(60, 60, 60), width=1))
radial_plot.addItem(circle_item)

# --- UPDATED FILTER LOGIC ---
bin_freqs = np.linspace(0, SAMPLE_FREQ/2, NUM_BINS)
# 1. Skip index 0 (DC/0Hz)
# 2. Only go up to MAX_FREQ, but exclude the very last bin if it's exactly on the boundary
valid_idx = np.where((bin_freqs > 100) & (bin_freqs < (MAX_FREQ - 100)))[0]

active_freqs = bin_freqs[valid_idx]
radial_angles = (active_freqs / MAX_FREQ) * 2 * np.pi

# Labels
for f in np.linspace(0, MAX_FREQ, NUM_FREQ_TICKS):
  ang = (f / MAX_FREQ) * 2 * np.pi
  lbl = pg.TextItem(f"{int(f/1000)}k", color=(150,150,150), anchor=(0.5,0.5))
  lbl.setPos(1.15 * RADIUS * np.cos(ang), 1.15 * RADIUS * np.sin(ang))
  radial_plot.addItem(lbl)

radial_spokes = radial_plot.plot(pen=pg.mkPen('r', width=1.5), connect='pairs')

win.nextRow()
spec_plot = win.addPlot(title="Power Spectrum (x=Hz, y=dB)")
spec_curve = spec_plot.plot(pen='y')
spec_plot.setYRange(-20, 100)

# -----------------------
# Update Logic
# -----------------------
def update():
  data = read_signal_data()
  if data is None: return
  print(f'mags: {data["fft_mags"]}')
  print(f'complex: {data["fft_complex"]}')
  mags = data["fft_mags"][:NUM_BINS]

  spec_curve.setData(bin_freqs, 20 * np.log10(mags + 1e-9))
  wave_curve.setData(reconstruct_signal(data["fft_complex"]))

  # Filtered Radial Update
  mags_filtered = mags[valid_idx]
 
  # Optional: Noise Floor Filter (removes tiny lines that look like a solid red circle)
  mags_filtered[mags_filtered < 0.05] = 0

  max_val = np.max(mags_filtered) if np.max(mags_filtered) > 0.1 else 1.0
  norm_mags = (mags_filtered / max_val) * RADIUS

  num_pts = len(norm_mags)
  pts_x = np.zeros(num_pts * 2)
  pts_y = np.zeros(num_pts * 2)

  # Evens are center (0,0), Odds are tips
  pts_x[1::2] = np.cos(radial_angles) * norm_mags
  pts_y[1::2] = np.sin(radial_angles) * norm_mags

  radial_spokes.setData(pts_x, pts_y)

  top_bin = np.argmax(mags[1:]) + 1
  peak_hz = (top_bin * SAMPLE_FREQ) / FFT_BUFF
  hud.setText(f"Energy (1s): {data['total_energy']:.2f}J | Average Energy (10s): {data['average_energy']:.2f}J | RMS: {data['rms']:.3f}V | Peak: {peak_hz:.1f}Hz")

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(30)

if __name__ == "__main__":
  instance = QtWidgets.QApplication.instance()
  if hasattr(instance, 'exec'):
    instance.exec()
  else:
    instance.exec_()