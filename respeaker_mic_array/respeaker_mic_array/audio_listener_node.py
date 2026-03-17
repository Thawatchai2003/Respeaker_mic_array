import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Int16MultiArray
from std_msgs.msg import Bool
import numpy as np
import pyqtgraph as pg
import json
import signal
from PyQt5 import QtWidgets, QtCore


# CONFIG 
RATE = 16000
CHANNELS = 6
USE_CHANNEL = 0
CHUNK = 1024

# NORMALIZE
def normalize(x):
    peak = np.max(np.abs(x)) + 1e-9
    gain = min(12000.0 / peak, 8.0)
    y = x.astype(np.float32) * gain
    return np.clip(y, -32768, 32767).astype(np.int16)

# BIQUAD (Butterworth 2nd order section)
class Biquad:

    def __init__(self, fs, cutoff, filter_type="low"):
        self.fs = fs
        self.cutoff = cutoff
        self.filter_type = filter_type
        self.reset()
        self.update_coeff()

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0

    def update_coeff(self):
        omega = 2 * np.pi * self.cutoff / self.fs
        cosw = np.cos(omega)
        sinw = np.sin(omega)

        # Butterworth 2nd order
        Q = 1 / np.sqrt(2)
        alpha = sinw / (2 * Q)

        if self.filter_type == "low":
            b0 = (1 - cosw) / 2
            b1 = 1 - cosw
            b2 = (1 - cosw) / 2
            a0 = 1 + alpha
            a1 = -2 * cosw
            a2 = 1 - alpha

        elif self.filter_type == "high":
            b0 = (1 + cosw) / 2
            b1 = -(1 + cosw)
            b2 = (1 + cosw) / 2
            a0 = 1 + alpha
            a1 = -2 * cosw
            a2 = 1 - alpha

        self.b0 = b0 / a0
        self.b1 = b1 / a0
        self.b2 = b2 / a0
        self.a1 = a1 / a0
        self.a2 = a2 / a0

    def set_cutoff(self, cutoff):
        self.cutoff = cutoff
        self.update_coeff()
        self.reset()

    def process(self, x):
        y = np.zeros_like(x, dtype=np.float32)
        for i in range(len(x)):
            out = self.b0 * x[i] + self.z1
            self.z1 = self.b1 * x[i] - self.a1 * out + self.z2
            self.z2 = self.b2 * x[i] - self.a2 * out
            y[i] = out
        return y


# CASCADE 4th ORDER (2x Biquad)
class Butterworth4:

    def __init__(self, fs, cutoff, filter_type="low"):
        self.stage1 = Biquad(fs, cutoff, filter_type)
        self.stage2 = Biquad(fs, cutoff, filter_type)

    def set_cutoff(self, cutoff):
        self.stage1.set_cutoff(cutoff)
        self.stage2.set_cutoff(cutoff)

    def process(self, x):
        y = self.stage1.process(x)
        y = self.stage2.process(y)
        return y

class SignalBridge(QtCore.QObject):
    show_gui_signal = QtCore.pyqtSignal()
    hide_gui_signal = QtCore.pyqtSignal()

# ROS2 NODE
class AudioListener(Node):
    def __init__(self):
        super().__init__('audio_listener_node')#, namespace='audio'

        self.gui_enabled = False #GUI Open or Close
        self.window = None

        # ----- Signal bridge -----
        self.signal_bridge = SignalBridge()
        self.signal_bridge.show_gui_signal.connect(self.open_gui)
        self.signal_bridge.hide_gui_signal.connect(self.close_gui)

        # ----- ROS Subscribers -----
        self.sub = self.create_subscription(UInt8MultiArray,'/respeaker/audio_raw', self.audio_callback, 10)
        self.gui_sub = self.create_subscription(Bool,'/gui_control/gui_enable', self.gui_callback, 10)
        self.pub = self.create_publisher(Int16MultiArray,'/audio/mono', 10)

        self.buffer = np.zeros(CHUNK, dtype=np.int16)
        self.raw_buffer = np.zeros(CHUNK, dtype=np.int16)

        # ----- EQ Parameters -----
        self.low_cut = 300
        self.high_cut = 3000
        self.low_gain = 1.0
        self.mid_gain = 1.0
        self.high_gain = 1.0

        self.low_filter = Butterworth4(RATE, self.low_cut, "low")
        self.high_filter = Butterworth4(RATE, self.high_cut, "high")
        self.get_logger().info("Audio Listener Ready")

    def update_filters(self):
        self.low_filter.set_cutoff(self.low_cut)
        self.high_filter.set_cutoff(self.high_cut)

    def butter_eq(self, x):
        x = x.astype(np.float32)
        low = self.low_filter.process(x)
        high = self.high_filter.process(x)
        mid = x - low - high
        y = (low * self.low_gain +
             mid * self.mid_gain +
             high * self.high_gain)

        # Soft limiter
        y = np.clip(y, -32768, 32767)
        return y.astype(np.int16)

    def audio_callback(self, msg):
        data = bytes(msg.data)
        pcm = np.frombuffer(data, dtype=np.int16)
        try:
            pcm = pcm.reshape(-1, CHANNELS)
        except:
            return

        mono = pcm[:, USE_CHANNEL].astype(np.float32)
        if len(mono) >= CHUNK:
            self.raw_buffer = mono[:CHUNK].copy()
            # apply current EQ
            processed = self.butter_eq(self.raw_buffer)
            self.buffer = processed.copy()
            out = Int16MultiArray()
            out.data = processed.tolist()
            self.pub.publish(out)

    def gui_callback(self, msg):
        if msg.data == self.gui_enabled:
            return
        self.gui_enabled = msg.data

        if msg.data:
            self.get_logger().info("GUI ENABLED")
            self.signal_bridge.show_gui_signal.emit()
        else:
            self.get_logger().info("GUI DISABLED")
            self.signal_bridge.hide_gui_signal.emit()

    def open_gui(self):
        if self.window is None:
            self.window = AudioWindow(self)
            self.window.show()
        else:
            self.window.show()  
            self.window.raise_()
            self.window.activateWindow()
            
    def close_gui(self):
        if self.window is not None:
            self.window.hide()
# GUI
class AudioWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.prev_db = -60.0
        self.setWindowTitle("Audio Monitor Pro")
        self.resize(1350, 850)

        # Dark Modern Style
        self.setStyleSheet("""
            QMainWindow { background-color: #101010; }
            QLabel { color: #dddddd; font-size: 13px; }
            QGroupBox {
                border: 1px solid #333;
                margin-top: 10px;
                padding: 10px;
                color: #ffffff;
                font-weight: bold;
            }
            QSlider::groove:horizontal {
                background: #2a2a2a;
                height: 6px;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #00ffaa;
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QProgressBar {
                background-color: #222;
                border: none;
                height: 18px;
            }
            QProgressBar::chunk {
                background-color: #00ffaa;
            }
                           
            QPushButton {
                background-color: #1e1e1e;
                color: #dddddd;
                border: 1px solid #333333;
                border-radius: 8px;
                padding: 8px 18px;
                font-size: 13px;
            }

            QPushButton:hover {
                background-color: #2a2a2a;
                border: 1px solid #00ffaa;
            }

            QPushButton:pressed {
                background-color: #00ffaa;
                color: #000000;
            }

            QPushButton#resetBtn {
                border: 1px solid #ff5555;
            }

            QPushButton#resetBtn:hover {
                background-color: #ff5555;
                color: #000000;
            }

            QPushButton#saveBtn {
                border: 1px solid #00aaff;
            }

            QPushButton#saveBtn:hover {
                background-color: #00aaff;
                color: #000000;
            }

            QPushButton#loadBtn {
                border: 1px solid #ffaa00;
            }

            QPushButton#loadBtn:hover {
                background-color: #ffaa00;
                color: #000000;
            }
        """)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setSpacing(15)
        pg.setConfigOptions(antialias=True, background="#101010", foreground="#dddddd")

        # Waveform
        self.wave_plot = pg.PlotWidget()
        self.wave_plot.setTitle("Waveform (Amplitude)", color="#00ffaa", size="14pt")
        self.wave_plot.setLabel("left", "Amplitude")
        self.wave_plot.setLabel("bottom", "Samples")
        self.wave_plot.setYRange(-32768, 32767)
        self.wave_plot.showGrid(x=True, y=True, alpha=0.2)
        self.wave_curve = self.wave_plot.plot(pen=pg.mkPen("#00ffaa", width=2))
        main_layout.addWidget(self.wave_plot)

        # FFT
        self.fft_plot = pg.PlotWidget()
        self.fft_plot.setTitle("Spectrum (dB)", color="#ffaa00", size="14pt")
        self.fft_plot.setLabel("left", "Level (dB)")
        self.fft_plot.setLabel("bottom", "Frequency (Hz)")
        self.fft_plot.setYRange(-100, 120)
        self.fft_plot.setXRange(0, RATE // 2)
        self.fft_plot.showGrid(x=True, y=True, alpha=0.2)
        self.fft_curve = self.fft_plot.plot(pen=pg.mkPen("#ffaa00", width=2))
        main_layout.addWidget(self.fft_plot)

        # Equalizer Panel
        control_box = QtWidgets.QGroupBox("3-Band Equalizer")
        control_layout = QtWidgets.QGridLayout()
        control_layout.setSpacing(10)
        control_box.setLayout(control_layout)
        main_layout.addWidget(control_box)

        # Gain sliders 
        self.low_slider, self.low_label = self.create_db_slider("Low Gain", control_layout, 0)
        self.mid_slider, self.mid_label = self.create_db_slider("Mid Gain", control_layout, 1)
        self.high_slider, self.high_label = self.create_db_slider("High Gain", control_layout, 2)

        # Cutoff sliders
        self.low_cut_slider, self.low_cut_label = self.create_hz_slider("Low Cut", 50, 1000, 300, control_layout, 3)
        self.high_cut_slider, self.high_cut_label = self.create_hz_slider("High Cut", 1000, 8000, 3000, control_layout, 4)

        # RMS Level Meter
        self.level_label = QtWidgets.QLabel("Output Level (RMS): -∞ dBFS")
        main_layout.addWidget(self.level_label)
        self.level_bar = QtWidgets.QProgressBar()
        self.level_bar.setRange(-60, 0)
        self.level_bar.setTextVisible(False)
        main_layout.addWidget(self.level_bar)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

        # Preset Buttons
        button_layout = QtWidgets.QHBoxLayout()
        self.reset_btn = QtWidgets.QPushButton("Reset EQ")
        self.save_btn = QtWidgets.QPushButton("Save Preset")
        self.load_btn = QtWidgets.QPushButton("Load Preset")
        button_layout.addWidget(self.reset_btn)
        button_layout.addWidget(self.save_btn)
        button_layout.addWidget(self.load_btn)
        main_layout.addLayout(button_layout)

        # connect
        self.reset_btn.clicked.connect(self.reset_eq)
        self.save_btn.clicked.connect(self.save_preset)
        self.load_btn.clicked.connect(self.load_preset)
        self.prev_fft = None
        self.reset_btn.setObjectName("resetBtn")
        self.save_btn.setObjectName("saveBtn")
        self.load_btn.setObjectName("loadBtn")

    # dB Slider
    def create_db_slider(self, title, layout, row):
        label = QtWidgets.QLabel(title)
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(-30, 30)
        slider.setValue(0)
        value_label = QtWidgets.QLabel("0 dB")
        value_label.setFixedWidth(70)
        slider.valueChanged.connect(
            lambda v: value_label.setText(f"{v} dB")
        )
        slider.valueChanged.connect(self.update_eq)
        layout.addWidget(label, row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(value_label, row, 2)
        return slider, value_label

    # Hz Slider
    def create_hz_slider(self, title, minv, maxv, val, layout, row):
        label = QtWidgets.QLabel(title)
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(minv, maxv)
        slider.setValue(val)
        value_label = QtWidgets.QLabel(f"{val} Hz")
        value_label.setFixedWidth(80)
        slider.valueChanged.connect(lambda v: value_label.setText(f"{v} Hz"))
        slider.valueChanged.connect(self.update_eq)
        layout.addWidget(label, row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(value_label, row, 2)
        return slider, value_label

    # Update EQ
    def update_eq(self):
        # dB linear gain
        self.node.low_gain = 10 ** (self.low_slider.value() / 20)
        self.node.mid_gain = 10 ** (self.mid_slider.value() / 20)
        self.node.high_gain = 10 ** (self.high_slider.value() / 20)
        self.node.low_cut = self.low_cut_slider.value()
        self.node.high_cut = self.high_cut_slider.value()
        self.node.update_filters()
        if len(self.node.raw_buffer) > 0:
            buf = self.node.raw_buffer.astype(np.float32)
            self.node.buffer = self.node.butter_eq(buf)

    # Update Plot + RMS
    def update_plot(self):
        buf = self.node.buffer
        self.wave_curve.setData(buf)
        fft = np.abs(np.fft.rfft(buf))
        fft[fft == 0] = 1e-12
        fft_db = 20 * np.log10(fft)

        if self.prev_fft is None:
            self.prev_fft = fft_db
        else:
            beta = 0.3
            fft_db = beta * fft_db + (1 - beta) * self.prev_fft
            self.prev_fft = fft_db
        freq = np.fft.rfftfreq(len(buf), 1.0 / RATE)
        self.fft_curve.setData(freq, fft_db)

        # RMS dBFS
        buf_float = buf.astype(np.float32) / 32768.0
        rms = np.sqrt(np.mean(buf_float ** 2))

        if rms < 1e-9:
            db = -60
        else:
            db = 20 * np.log10(rms)
        db = max(db, -60)

        # smoothing
        alpha = 0.2
        db = alpha * db + (1 - alpha) * self.prev_db
        self.prev_db = db
        self.level_bar.setValue(int(db))
        self.level_label.setText(f"Output Level (RMS): {db:.1f} dBFS")

    def reset_eq(self):
        self.low_slider.setValue(0)
        self.mid_slider.setValue(0)
        self.high_slider.setValue(0)
        self.low_cut_slider.setValue(300)
        self.high_cut_slider.setValue(3000)
        self.update_eq()

    def save_preset(self):
        file_name, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Preset", "", "Preset Files (*.json)"
        )

        if not file_name:
            return

        if not file_name.endswith(".json"):
            file_name += ".json"

        preset = {
            "low_gain": self.low_slider.value(),
            "mid_gain": self.mid_slider.value(),
            "high_gain": self.high_slider.value(),
            "low_cut": self.low_cut_slider.value(),
            "high_cut": self.high_cut_slider.value()
        }
        with open(file_name, "w") as f:
            json.dump(preset, f, indent=4)
        QtWidgets.QMessageBox.information(self, "Saved", "Preset saved successfully!")

    def load_preset(self):
        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Preset", "", "Preset Files (*.json)"
        )

        if not file_name:
            return

        try:
            with open(file_name, "r") as f:
                preset = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Error", "Invalid preset file!")
            return
        self.low_slider.setValue(preset.get("low_gain", 0))
        self.mid_slider.setValue(preset.get("mid_gain", 0))
        self.high_slider.setValue(preset.get("high_gain", 0))
        self.low_cut_slider.setValue(preset.get("low_cut", 300))
        self.high_cut_slider.setValue(preset.get("high_cut", 3000))
        self.update_eq()
        QtWidgets.QMessageBox.information(self, "Loaded", "Preset loaded successfully!")

    def closeEvent(self, event):
        event.ignore()
        self.hide()
# MAIN
def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)
    node = AudioListener()
    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    ros_timer.start(10)

    # Clean shutdown handler
    def shutdown_handler(signum, frame):
        print("\nShutting down cleanly...")
        ros_timer.stop()
        node.destroy_node()
        rclpy.shutdown()
        app.quit()

    signal.signal(signal.SIGINT, shutdown_handler)
    app.exec_()

if __name__ == '__main__':
    main()
