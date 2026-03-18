
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Int16MultiArray, Bool
import numpy as np
import pyqtgraph as pg
import json
import signal
import time
from PyQt5 import QtWidgets, QtCore


# DEFAULT CONFIG
RATE = 16000
DEFAULT_USE_CHANNEL = 0
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
        else:
            # fallback safe
            b0, b1, b2 = 1.0, 0.0, 0.0
            a0, a1, a2 = 1.0, 0.0, 0.0

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
        self.fs = fs
        self.filter_type = filter_type
        self.stage1 = Biquad(fs, cutoff, filter_type)
        self.stage2 = Biquad(fs, cutoff, filter_type)

    def set_cutoff(self, cutoff):
        self.stage1.set_cutoff(cutoff)
        self.stage2.set_cutoff(cutoff)

    def process(self, x):
        y = self.stage1.process(x)
        y = self.stage2.process(y)
        return y



# Qt Signal Bridge

class SignalBridge(QtCore.QObject):
    show_gui_signal = QtCore.pyqtSignal()
    hide_gui_signal = QtCore.pyqtSignal()



# ROS2 NODE

class AudioListener(Node):
    def __init__(self):
        super().__init__('audio_listener_node')

        # Parameters
        self.declare_parameter("sample_rate", RATE)
        self.declare_parameter("chunk_size", CHUNK)
        self.declare_parameter("use_channel", DEFAULT_USE_CHANNEL)
        self.declare_parameter("debug_log", True)

        # NEW: mode
        # mode=1 => manual use_channel
        # mode=2 => auto scan channel by RMS
        self.declare_parameter("mode", 1)
        self.declare_parameter("auto_scan_interval_sec", 0.5)
        self.declare_parameter("auto_switch_threshold_rms", 300.0)
        self.declare_parameter("switch_margin_ratio", 1.15)  # candidate ต้องดีกว่า current กี่เท่า
        self.declare_parameter("normalize_output", False)

        self.rate = int(self.get_parameter("sample_rate").value)
        self.chunk = int(self.get_parameter("chunk_size").value)
        self.use_channel = int(self.get_parameter("use_channel").value)
        self.debug_log = bool(self.get_parameter("debug_log").value)

        self.mode = int(self.get_parameter("mode").value)
        self.auto_scan_interval_sec = float(self.get_parameter("auto_scan_interval_sec").value)
        self.auto_switch_threshold_rms = float(self.get_parameter("auto_switch_threshold_rms").value)
        self.switch_margin_ratio = float(self.get_parameter("switch_margin_ratio").value)
        self.normalize_output = bool(self.get_parameter("normalize_output").value)

        # runtime detected from packet
        self.detected_channels = 0
        self._last_logged_channels = -1

        # active channel runtime
        self.active_channel = self.use_channel
        self._last_scan_ts = 0.0
        self._last_switch_ts = 0.0

        # GUI state
        self.gui_enabled = False
        self.window = None

        # ----- Signal bridge -----
        self.signal_bridge = SignalBridge()
        self.signal_bridge.show_gui_signal.connect(self.open_gui)
        self.signal_bridge.hide_gui_signal.connect(self.close_gui)

        # ----- ROS Subscribers / Publishers -----
        self.sub = self.create_subscription(
            UInt8MultiArray,
            '/respeaker/audio_raw',
            self.audio_callback,
            10
        )
        self.gui_sub = self.create_subscription(
            Bool,
            '/gui_control/gui_enable',
            self.gui_callback,
            10
        )
        self.pub = self.create_publisher(
            Int16MultiArray,
            '/audio/mono',
            10
        )

        # Buffers
        self.buffer = np.zeros(self.chunk, dtype=np.int16)
        self.raw_buffer = np.zeros(self.chunk, dtype=np.int16)

        # ----- EQ Parameters -----
        self.low_cut = 300
        self.high_cut = 3000
        self.low_gain = 1.0
        self.mid_gain = 1.0
        self.high_gain = 1.0

        self.low_filter = Butterworth4(self.rate, self.low_cut, "low")
        self.high_filter = Butterworth4(self.rate, self.high_cut, "high")

        self.get_logger().info(
            f"✅ Audio Listener Ready | rate={self.rate} | chunk={self.chunk} | "
            f"use_channel={self.use_channel} | mode={self.mode}"
        )

    
    # Filter update
    
    def update_filters(self):
        self.low_filter.set_cutoff(self.low_cut)
        self.high_filter.set_cutoff(self.high_cut)

    
    # 3-band EQ
    
    def butter_eq(self, x):
        x = x.astype(np.float32)

        low = self.low_filter.process(x)
        high = self.high_filter.process(x)
        mid = x - low - high

        y = (
            low * self.low_gain +
            mid * self.mid_gain +
            high * self.high_gain
        )

        # Soft limiter
        y = np.clip(y, -32768, 32767)
        return y.astype(np.int16)

    
    # RMS helper
    
    def calc_rms(self, x):
        if len(x) == 0:
            return 0.0
        x = x.astype(np.float32)
        return float(np.sqrt(np.mean(x * x)))

    
    # Auto channel selection (MODE 2)
    
    def auto_select_channel(self, pcm_2d):
        """
        pcm_2d shape = [chunk, channels]
        เลือก channel ที่ RMS สูงสุด
        มี threshold + margin + cooldown กันสลับรัว
        """
        now = time.time()

        # scan เป็นช่วง ๆ ไม่ต้องทุก packet
        if (now - self._last_scan_ts) < self.auto_scan_interval_sec:
            return

        self._last_scan_ts = now

        if pcm_2d.ndim != 2 or pcm_2d.shape[1] <= 0:
            return

        rms_list = []
        for ch in range(pcm_2d.shape[1]):
            ch_data = pcm_2d[:, ch]
            rms = self.calc_rms(ch_data)
            rms_list.append(rms)

        if len(rms_list) == 0:
            return

        best_ch = int(np.argmax(rms_list))
        best_rms = float(rms_list[best_ch])

        current_ch = min(max(0, self.active_channel), pcm_2d.shape[1] - 1)
        current_rms = float(rms_list[current_ch])

        # ถ้าเงียบมาก ไม่ต้องสลับ
        if best_rms < self.auto_switch_threshold_rms:
            return

        # ถ้ายังไม่เคยเลือก หรือ channel ปัจจุบันเกิน range
        if self.active_channel >= pcm_2d.shape[1]:
            self.active_channel = best_ch
            self.get_logger().info(
                f"🎯 Auto channel init -> CH{best_ch} (RMS={best_rms:.1f})"
            )
            return

        # ถ้า best เป็น channel เดิมอยู่แล้ว ไม่ต้องทำอะไร
        if best_ch == current_ch:
            return

        # ต้องดีกว่าช่องปัจจุบันพอสมควร
        if current_rms > 1e-6:
            if best_rms < (current_rms * self.switch_margin_ratio):
                return

        # กันสลับถี่เกินไป
        if (now - self._last_switch_ts) < self.auto_scan_interval_sec:
            return

        self.active_channel = best_ch
        self._last_switch_ts = now

        if self.debug_log:
            rms_text = ", ".join([f"CH{i}:{r:.0f}" for i, r in enumerate(rms_list)])
            self.get_logger().info(
                f"🔄 Auto switched channel -> CH{best_ch} | {rms_text}"
            )

    
    # Audio callback (AUTO-DETECT CHANNELS)
    
    def audio_callback(self, msg):
        data = bytes(msg.data)
        pcm = np.frombuffer(data, dtype=np.int16)

        if len(pcm) == 0:
            return

        # ต้องหาร chunk ลงตัว เพราะ audio_node ส่งทีละ chunk * channels
        if self.chunk <= 0:
            self.get_logger().warning("Invalid chunk_size <= 0")
            return

        if len(pcm) % self.chunk != 0:
            if self.debug_log:
                self.get_logger().warning(
                    f"Invalid audio packet size: total_samples={len(pcm)} "
                    f"not divisible by chunk={self.chunk}"
                )
            return

        detected_channels = len(pcm) // self.chunk
        if detected_channels <= 0:
            return

        self.detected_channels = detected_channels

        if self._last_logged_channels != detected_channels:
            self._last_logged_channels = detected_channels
            self.get_logger().info(
                f"🎧 Detected input channels={detected_channels} "
                f"(payload_samples={len(pcm)}, chunk={self.chunk})"
            )

            # ถ้า active_channel เกิน range ให้ reset
            if self.active_channel >= detected_channels:
                self.active_channel = min(max(0, self.use_channel), detected_channels - 1)

        try:
            pcm = pcm.reshape(-1, detected_channels)
        except Exception as e:
            if self.debug_log:
                self.get_logger().warning(f"Reshape failed: {e}")
            return

        # Select channel
        if self.mode == 2:
            self.auto_select_channel(pcm)
            use_ch = min(max(0, self.active_channel), detected_channels - 1)
        else:
            use_ch = min(max(0, self.use_channel), detected_channels - 1)
            self.active_channel = use_ch

        mono = pcm[:, use_ch].astype(np.float32)

        if len(mono) >= self.chunk:
            self.raw_buffer = mono[:self.chunk].copy()

            # Apply EQ
            processed = self.butter_eq(self.raw_buffer)

            # optional normalize
            if self.normalize_output:
                processed = normalize(processed)

            self.buffer = processed.copy()

            out = Int16MultiArray()
            out.data = processed.tolist()
            self.pub.publish(out)

    # GUI control
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
        self.prev_fft = None

        self.setWindowTitle("Audio Monitor Pro")
        self.resize(1350, 900)

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

            QComboBox {
                background-color: #1e1e1e;
                color: #dddddd;
                border: 1px solid #333333;
                border-radius: 6px;
                padding: 6px;
                min-width: 120px;
            }

            QComboBox:hover {
                border: 1px solid #00ffaa;
            }
        """)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setSpacing(15)

        pg.setConfigOptions(
            antialias=True,
            background="#101010",
            foreground="#dddddd"
        )

        # Info label
        self.info_label = QtWidgets.QLabel("")
        main_layout.addWidget(self.info_label)

        # Top control bar
        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Mode:"))

        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Manual (1)", "Auto Scan (2)"])
        self.mode_combo.setCurrentIndex(0 if self.node.mode == 1 else 1)
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        top_bar.addWidget(self.mode_combo)

        top_bar.addWidget(QtWidgets.QLabel("Manual Channel:"))

        self.channel_combo = QtWidgets.QComboBox()
        for i in range(8):
            self.channel_combo.addItem(f"CH{i}", i)
        self.channel_combo.setCurrentIndex(max(0, min(7, self.node.use_channel)))
        self.channel_combo.currentIndexChanged.connect(self.on_channel_changed)
        top_bar.addWidget(self.channel_combo)

        self.active_ch_label = QtWidgets.QLabel("Active: CH0")
        top_bar.addWidget(self.active_ch_label)

        top_bar.addStretch()
        main_layout.addLayout(top_bar)

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
        self.fft_plot.setXRange(0, self.node.rate // 2)
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
        self.low_cut_slider, self.low_cut_label = self.create_hz_slider(
            "Low Cut", 50, 1000, 300, control_layout, 3
        )
        self.high_cut_slider, self.high_cut_label = self.create_hz_slider(
            "High Cut", 1000, 8000, 3000, control_layout, 4
        )

        # RMS Level Meter
        self.level_label = QtWidgets.QLabel("Output Level (RMS): -∞ dBFS")
        main_layout.addWidget(self.level_label)

        self.level_bar = QtWidgets.QProgressBar()
        self.level_bar.setRange(-60, 0)
        self.level_bar.setTextVisible(False)
        main_layout.addWidget(self.level_bar)

        # Preset Buttons
        button_layout = QtWidgets.QHBoxLayout()
        self.reset_btn = QtWidgets.QPushButton("Reset EQ")
        self.save_btn = QtWidgets.QPushButton("Save Preset")
        self.load_btn = QtWidgets.QPushButton("Load Preset")

        self.reset_btn.setObjectName("resetBtn")
        self.save_btn.setObjectName("saveBtn")
        self.load_btn.setObjectName("loadBtn")

        button_layout.addWidget(self.reset_btn)
        button_layout.addWidget(self.save_btn)
        button_layout.addWidget(self.load_btn)
        main_layout.addLayout(button_layout)

        # connect
        self.reset_btn.clicked.connect(self.reset_eq)
        self.save_btn.clicked.connect(self.save_preset)
        self.load_btn.clicked.connect(self.load_preset)

        # Timer update GUI
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

    # Top controls
    def on_mode_changed(self, idx):
        self.node.mode = 1 if idx == 0 else 2

    def on_channel_changed(self, idx):
        self.node.use_channel = idx
        if self.node.mode == 1:
            self.node.active_channel = idx

    # Slider creators
    def create_db_slider(self, title, layout, row):
        label = QtWidgets.QLabel(title)
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(-30, 30)
        slider.setValue(0)

        value_label = QtWidgets.QLabel("0 dB")
        value_label.setFixedWidth(70)

        slider.valueChanged.connect(lambda v: value_label.setText(f"{v} dB"))
        slider.valueChanged.connect(self.update_eq)

        layout.addWidget(label, row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(value_label, row, 2)

        return slider, value_label

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
        # dB -> linear gain
        self.node.low_gain = 10 ** (self.low_slider.value() / 20)
        self.node.mid_gain = 10 ** (self.mid_slider.value() / 20)
        self.node.high_gain = 10 ** (self.high_slider.value() / 20)

        self.node.low_cut = self.low_cut_slider.value()
        self.node.high_cut = self.high_cut_slider.value()
        self.node.update_filters()

        if len(self.node.raw_buffer) > 0:
            buf = self.node.raw_buffer.astype(np.float32)
            processed = self.node.butter_eq(buf)
            if self.node.normalize_output:
                processed = normalize(processed)
            self.node.buffer = processed

    # Update Plot + RMS
    def update_plot(self):
        buf = self.node.buffer

        # Update info label
        mode_text = "Manual" if self.node.mode == 1 else "Auto Scan"
        self.info_label.setText(
            f"Rate: {self.node.rate} Hz | Chunk: {self.node.chunk} | "
            f"Manual CH: {self.node.use_channel} | Active CH: {self.node.active_channel} | "
            f"Detected CH: {self.node.detected_channels} | Mode: {mode_text}"
        )
        self.active_ch_label.setText(f"Active: CH{self.node.active_channel}")

        # sync combo if changed from code
        if self.node.mode == 1 and self.mode_combo.currentIndex() != 0:
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentIndex(0)
            self.mode_combo.blockSignals(False)
        elif self.node.mode == 2 and self.mode_combo.currentIndex() != 1:
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentIndex(1)
            self.mode_combo.blockSignals(False)

        # Waveform
        self.wave_curve.setData(buf)

        # FFT
        fft = np.abs(np.fft.rfft(buf))
        fft[fft == 0] = 1e-12
        fft_db = 20 * np.log10(fft)

        if self.prev_fft is None:
            self.prev_fft = fft_db
        else:
            beta = 0.3
            fft_db = beta * fft_db + (1 - beta) * self.prev_fft
            self.prev_fft = fft_db

        freq = np.fft.rfftfreq(len(buf), 1.0 / self.node.rate)
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

    # Presets
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
        except Exception:
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

        try:
            if node.window is not None:
                node.window.close()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        app.quit()

    signal.signal(signal.SIGINT, shutdown_handler)
    app.exec_()


if __name__ == '__main__':
    main()
