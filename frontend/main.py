import sys
import struct
import serial
import numpy as np
import pyqtgraph as pg

from serial.tools import list_ports
from pyqtgraph.Qt import QtCore, QtWidgets

# =======================
# Configuration
# =======================
PACKET_FORMAT = "<Iffff128f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
SYNC_WORD = 0xABCDEFFF

BAUDRATE = 921600
SERIAL_TIMEOUT = 0.1

SAMPLE_FREQ = 44138
FFT_SIZE = 128
NUM_BINS = FFT_SIZE // 2

MAX_FREQ = 20_000.0
RADIUS = 1.0
NUM_FREQ_TICKS = 12
NOISE_FLOOR = 0.05


# =======================
# Serial Port Selection
# =======================
def select_serial_port():
    ports = list_ports.comports()
    if not ports:
        QtWidgets.QMessageBox.critical(
            None, "No Serial Ports", "No serial ports were found."
        )
        return None

    dialog = QtWidgets.QDialog()
    dialog.setWindowTitle("Select Serial Port")
    layout = QtWidgets.QVBoxLayout(dialog)

    combo = QtWidgets.QComboBox()
    for p in ports:
        combo.addItem(f"{p.device} — {p.description}", p.device)

    layout.addWidget(QtWidgets.QLabel("Select a serial port:"))
    layout.addWidget(combo)

    buttons = QtWidgets.QDialogButtonBox(
        QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
    )
    layout.addWidget(buttons)

    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)

    return combo.currentData() if dialog.exec() else None


# =======================
# DSP Helpers
# =======================
def reconstruct_signal(fft_half):
    """Rebuild time-domain signal from half FFT"""
    full_fft = np.zeros(FFT_SIZE, dtype=complex)
    full_fft[:NUM_BINS] = fft_half
    full_fft[NUM_BINS + 1:] = np.conj(fft_half[1:][::-1])

    adc = np.fft.ifft(full_fft).real + 2048.0
    return adc * (3.3 / 4095.0)


# =======================
# Main Application
# =======================
class SignalAnalyzerApp(QtWidgets.QWidget):
    def __init__(self, serial_port):
        super().__init__()

        self.ser = serial.Serial(
            port=serial_port,
            baudrate=BAUDRATE,
            timeout=SERIAL_TIMEOUT
        )
        self.ser.flushInput()

        self._init_frequency_mapping()
        self._init_ui()
        self._init_timer()

    # ---------- Init ----------
    def _init_frequency_mapping(self):
        self.bin_freqs = np.linspace(0, SAMPLE_FREQ / 2, NUM_BINS)
        self.valid_idx = np.where(
            (self.bin_freqs > 100) &
            (self.bin_freqs < (MAX_FREQ - 100))
        )[0]

        self.active_freqs = self.bin_freqs[self.valid_idx]
        self.radial_angles = (self.active_freqs / MAX_FREQ) * 2 * np.pi

    def _init_ui(self):
        pg.setConfigOptions(antialias=True)

        self.win = pg.GraphicsLayoutWidget(title="Signal Analyzer")
        self.win.resize(1200, 800)
        self.win.show()

        self.hud = self.win.addLabel(size='12pt')
        self.win.nextRow()

        # Waveform
        self.wave_plot = self.win.addPlot(title="Waveform (Voltage)")
        self.wave_curve = self.wave_plot.plot(pen='g')

        # Radial Spectrum
        self.radial_plot = self.win.addPlot(title="Radial Spectrum (Hz)")
        self.radial_plot.setAspectLocked()
        self.radial_plot.hideAxis('bottom')
        self.radial_plot.hideAxis('left')

        self.radial_plot.addItem(
            QtWidgets.QGraphicsEllipseItem(
                -RADIUS, -RADIUS, RADIUS * 2, RADIUS * 2
            )
        )

        self.radial_spokes = self.radial_plot.plot(
            pen=pg.mkPen('r', width=1.5), connect='pairs'
        )

        self._add_radial_labels()

        self.win.nextRow()

        # Spectrum
        self.spec_plot = self.win.addPlot(title="Power Spectrum (dB)")
        self.spec_curve = self.spec_plot.plot(pen='y')
        self.spec_plot.setYRange(-20, 100)

    def _add_radial_labels(self):
        for f in np.linspace(0, MAX_FREQ, NUM_FREQ_TICKS):
            angle = (f / MAX_FREQ) * 2 * np.pi
            label = pg.TextItem(f"{int(f / 1000)}k", anchor=(0.5, 0.5))
            label.setPos(
                1.15 * RADIUS * np.cos(angle),
                1.15 * RADIUS * np.sin(angle)
            )
            self.radial_plot.addItem(label)

    def _init_timer(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(30)

    # ---------- Serial ----------
    def read_packet(self):
        if self.ser.in_waiting < PACKET_SIZE:
            return None

        raw = self.ser.read(self.ser.in_waiting)
        sync = struct.pack("<I", SYNC_WORD)
        idx = raw.rfind(sync)

        if idx == -1 or len(raw[idx:]) < PACKET_SIZE:
            return None

        packet = raw[idx:idx + PACKET_SIZE]
        try:
            unpacked = struct.unpack(PACKET_FORMAT, packet)
        except struct.error:
            return None

        real = np.array(unpacked[5:69])
        imag = np.array(unpacked[69:133])
        fft = real + 1j * imag

        return {
            "energy": unpacked[1],
            "avg_energy": unpacked[2],
            "rms": unpacked[3],
            "avg_v": unpacked[4],
            "fft": fft,
            "mags": np.abs(fft)
        }

    # ---------- Update ----------
    def update(self):
        data = self.read_packet()
        if not data:
            return

        mags = data["mags"][:NUM_BINS]

        self.spec_curve.setData(
            self.bin_freqs,
            20 * np.log10(mags + 1e-9)
        )

        self.wave_curve.setData(
            reconstruct_signal(data["fft"])
        )

        self._update_radial(mags)
        self._update_hud(data, mags)

    def _update_radial(self, mags):
        mags_f = mags[self.valid_idx]
        mags_f[mags_f < NOISE_FLOOR] = 0

        scale = np.max(mags_f) or 1.0
        lengths = (mags_f / scale) * RADIUS

        pts_x = np.zeros(len(lengths) * 2)
        pts_y = np.zeros(len(lengths) * 2)

        pts_x[1::2] = np.cos(self.radial_angles) * lengths
        pts_y[1::2] = np.sin(self.radial_angles) * lengths

        self.radial_spokes.setData(pts_x, pts_y)

    def _update_hud(self, data, mags):
        peak_bin = np.argmax(mags[1:]) + 1
        peak_hz = peak_bin * SAMPLE_FREQ / FFT_SIZE

        self.hud.setText(
            f"Energy: {data['energy']:.2f}J | "
            f"Avg Energy: {data['avg_energy']:.2f}J | "
            f"RMS: {data['rms']:.3f}V | "
            f"Peak: {peak_hz:.1f}Hz"
        )


# =======================
# Entry Point
# =======================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    port = select_serial_port()
    if not port:
        sys.exit(0)

    analyzer = SignalAnalyzerApp(port)
    sys.exit(app.exec())
