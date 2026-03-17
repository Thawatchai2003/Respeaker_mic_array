import sys
import time
import json
import os
import signal
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import UInt8MultiArray, Bool
from ament_index_python.packages import get_package_share_directory

import usb.core
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

from .usb_4_mic_array.tuning import Tuning

CHUNK = 1024
RATE = 16000
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_SHARE = get_package_share_directory("respeaker_mic_array")
CONFIG_FILE = os.path.join(
    PKG_SHARE,
    "config",
    "respeaker_dsp_config.json"
)
_app_window = None

class GraphNode(Node):
    def __init__(self):
        super().__init__('graph_node')

        # PARAMETERS
        self.declare_parameter("read_only", False)
        self.read_only = self.get_parameter("read_only").value
        mode_text = "READ-ONLY MODE" if self.read_only else "DSP CONTROL MODE"
        self.get_logger().info(f"GRAPH NODE: {mode_text}")

        # AUDIO BUFFER
        self.buffer = np.zeros(CHUNK, dtype=np.int16)

        # ROS SUBSCRIPTIONS
        self.gui_callback = None
        self.create_subscription(UInt8MultiArray, 'respeaker/audio_raw', self.audio_callback, 10)
        self.create_subscription(Bool,'/gui_control/gui_enable_firmware', self.gui_control_callback, 10)

        # DSP / USB INITIALIZATION
        self.mic = None
        if not self.read_only:
            dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if dev is None:
                self.get_logger().error("ReSpeaker not found")
            else:
                self.mic = Tuning(dev)
                self.get_logger().info("ReSpeaker DSP connected")

    def audio_callback(self, msg):
        audio = np.frombuffer(bytes(msg.data), dtype=np.int16)
        if len(audio) >= CHUNK:
            self.buffer = audio[:CHUNK].copy()

    def set_dsp_param(self, name, value):
        if self.read_only or self.mic is None:
            return
        try:
            self.mic.write(name, value)
            self.get_logger().info(f"DSP SET {name} = {value}")
        except Exception as e:
            self.get_logger().warn(f"DSP FAIL {name}: {e}")

    def gui_control_callback(self, msg):
        if self.gui_callback is not None:
            self.gui_callback(msg.data)

# GUI
class AudioWindow(QtWidgets.QMainWindow):

    CORE_PARAMS = [
        "AGCONOFF","ECHOONOFF","STATNOISEONOFF","NONSTATNOISEONOFF",
        "TRANSIENTONOFF","RT60ONOFF","CNIONOFF","NLATTENONOFF","HPFONOFF",
    ]

    AGC_PARAMS = {
        "AGCMAXGAIN": dict(min=0, max=40, scale=1),
        "AGCDESIREDLEVEL": dict(min=0.001, max=0.05, scale=1000),
        "AGCTIME": dict(min=0.1, max=1.0, scale=100),
    }

    NOISE_PARAMS = {
        "GAMMA_NS": dict(min=0.5, max=3.0, scale=10),
        "GAMMA_NN": dict(min=0.5, max=3.0, scale=10),
        "MIN_NS": dict(min=0.0, max=1.0, scale=100),
        "MIN_NN": dict(min=0.0, max=1.0, scale=100),
    }

    ASR_PARAMS = {
        "GAMMA_NS_SR": dict(min=0.5, max=3.0, scale=10),
        "GAMMA_NN_SR": dict(min=0.5, max=3.0, scale=10),
        "MIN_NS_SR": dict(min=0.0, max=1.0, scale=100),
        "MIN_NN_SR": dict(min=0.0, max=1.0, scale=100),
    }

    BEAMFORMING_PARAMS = {
        "GAMMA_E": dict(min=0.5, max=3.0, scale=10),
        "GAMMA_ETAIL": dict(min=0.5, max=3.0, scale=10),
        "GAMMA_ENL": dict(min=0.5, max=3.0, scale=10),
        "AECNORM": dict(min=0.5, max=2.0, scale=10),
    }

    DEFAULT_VALUES = {
        # CORE
        "AGCONOFF": 1,
        "ECHOONOFF": 1,
        "STATNOISEONOFF": 1,
        "NONSTATNOISEONOFF": 1,
        "FREEZEONOFF": 0,
        "AECFREEZEONOFF": 0,
        "TRANSIENTONOFF": 1,
        "RT60ONOFF": 1,
        "CNIONOFF": 1,
        "NLATTENONOFF": 1,
        "NLAEC_MODE": 2,

        # AGC
        "AGCMAXGAIN": 28.0,
        "AGCDESIREDLEVEL": 0.01,
        "AGCTIME": 0.4,

        # NOISE SUPPRESSION
        "GAMMA_NS": 1.6,
        "GAMMA_NN": 1.4,
        "MIN_NS": 0.12,
        "MIN_NN": 0.2,

        # ASR
        "STATNOISEONOFF_SR": 1,
        "NONSTATNOISEONOFF_SR": 1,
        "GAMMA_NS_SR": 1.2,
        "GAMMA_NN_SR": 1.2,
        "MIN_NS_SR": 0.18,
        "MIN_NN_SR": 0.2,

        # VAD
        "GAMMAVAD_SR": 2.6,

        # BEAMFORMING
        "GAMMA_E": 1.4,
        "GAMMA_ETAIL": 1.2,

        # AEC
        "GAMMA_ENL": 2.2,
        "AECNORM": 1.0,

        # HPF
        "HPFONOFF": 2,
    }
    
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.last_write = {}
        self.loading_config = False
        self.checkboxes = {}
        self.sliders = {}
        self.setWindowTitle("ReSpeaker DSP Control & Monitor")
        self.resize(1500, 820)
        self.setStyleSheet("""
            QWidget { background:#111; color:#DDD; font-size:11px; }
            QLabel { font-weight:bold; color:#AAA; }
            QCheckBox { spacing:6px; }
        """)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # LEFT
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(340)
        ctrl = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(ctrl)
        scroll.setWidget(ctrl)

        def section(title):
            lbl = QtWidgets.QLabel(title)
            lbl.setStyleSheet("color:#5DADE2; margin-top:12px;")
            layout.addWidget(lbl)

        def chk(name):
            c = QtWidgets.QCheckBox(name)
            default = self.DEFAULT_VALUES.get(name, 0)
            c.setChecked(bool(default))
            c.setEnabled(not self.node.read_only)
            c.stateChanged.connect(lambda s, n=name: self.on_toggle(n, s))
            layout.addWidget(c)
            self.checkboxes[name] = c

        def slider(name, cfg):
            scale = cfg["scale"]
            lbl = QtWidgets.QLabel(f"{name}: {cfg['min']}")
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setRange(int(cfg["min"] * scale), int(cfg["max"] * scale))
            s.setValue(int(cfg["min"] * scale))
            s.setEnabled(not self.node.read_only)

            def update(v):
                val = v / scale
                lbl.setText(f"{name}: {val:.3f}")
                self.on_slider(name, val)
            s.valueChanged.connect(update)
            layout.addWidget(lbl)
            layout.addWidget(s)
            self.sliders[name] = (s, cfg)

        # Control Buttons (บนสุด)
        btn_layout = QtWidgets.QHBoxLayout()

        self.btn_toggle = QtWidgets.QPushButton()
        self.btn_toggle.clicked.connect(self.toggle_edit_mode)
        self.edit_enabled = not self.node.read_only

        if self.edit_enabled:
            self.btn_toggle.setText("🔓 EDIT MODE")
        else:
            self.btn_toggle.setText("🔒 LOCKED")
        btn_save = QtWidgets.QPushButton("💾 Save")
        btn_load = QtWidgets.QPushButton("📂 Load")
        btn_reset = QtWidgets.QPushButton("🔄 Reset")

        btn_save.clicked.connect(self.save_config)
        btn_load.clicked.connect(self.load_config)
        btn_reset.clicked.connect(self.reset_config)
        btn_layout.addWidget(btn_save)
        btn_layout.addWidget(btn_load)
        btn_layout.addWidget(btn_reset)
        layout.addLayout(btn_layout)
        layout.addSpacing(10)   

        # Toggle Edit Button 
        self.btn_toggle = QtWidgets.QPushButton()
        self.btn_toggle.clicked.connect(self.toggle_edit_mode)
        self.edit_enabled = not self.node.read_only

        if self.edit_enabled:
            self.btn_toggle.setText("🔓 EDIT MODE")
        else:
            self.btn_toggle.setText("🔒 LOCKED")
        layout.addWidget(self.btn_toggle)
        layout.addSpacing(10)

        # Sections
        section("CORE DSP SWITCHES")
        for p in self.CORE_PARAMS:
            chk(p)

        section("AGC (Automatic Gain Control)")
        for p, cfg in self.AGC_PARAMS.items():
            slider(p, cfg)

        section("NOISE SUPPRESSION")
        for p, cfg in self.NOISE_PARAMS.items():
            slider(p, cfg)

        section("ASR OPTIMIZATION")
        for p, cfg in self.ASR_PARAMS.items():
            slider(p, cfg)

        section("BEAMFORMING / AEC")
        for p, cfg in self.BEAMFORMING_PARAMS.items():
            slider(p, cfg)

        layout.addStretch()  
        ctrl.setLayout(layout)     
        scroll.setWidget(ctrl)      
        ctrl.adjustSize()           
                
        # RIGHT (GRAPHS)
        pg.setConfigOptions(antialias=True, foreground='#AAA', background='#111')
        plots = pg.GraphicsLayoutWidget()

        pw = plots.addPlot(title="Waveform (Time Domain)")
        pw.showGrid(x=True, y=True, alpha=0.15)
        pw.setYRange(-32768, 32767)
        pw.setLabel('left', 'Amplitude (ADC)')
        pw.setLabel('bottom', 'Samples')
        self.curve_wave = pw.plot(pen=pg.mkPen('#F4D03F', width=1))
        plots.nextRow()

        pf = plots.addPlot(title="FFT Spectrum (Frequency Domain)")
        pf.showGrid(x=True, y=True, alpha=0.15)
        pf.setXRange(0, 8000)
        pf.setYRange(0, 120)
        pf.setLabel('left', 'Magnitude (dB)')
        pf.setLabel('bottom', 'Frequency (Hz)')
        self.curve_fft = pf.plot(pen=pg.mkPen('#5DADE2', width=1))

        main_layout.addWidget(scroll)
        main_layout.addWidget(plots, 1)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)
        QtCore.QTimer.singleShot(800, self.load_config)

    # CALLBACKS
    def on_toggle(self, name, state):
        if not self.edit_enabled:
            return
        self.node.set_dsp_param(name, int(state > 0))

    def on_slider(self, name, value):
        if not self.edit_enabled:
            return

        if not self.loading_config:
            now = time.time()
            last = self.last_write.get(name, 0.0)
            if now - last < 0.15:
                return
            self.last_write[name] = now

        self.node.set_dsp_param(name, value)

    # SAVE / LOAD
    def save_config(self):
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        data = {}

        for k, cb in self.checkboxes.items():
            data[k] = int(cb.isChecked())
        for k, (s, cfg) in self.sliders.items():
            data[k] = s.value() / cfg["scale"]
        with open(CONFIG_FILE, "w") as f:
            json.dump(data, f, indent=2)

    def load_config(self):
        if not os.path.exists(CONFIG_FILE):
            return

        self.loading_config = True
        with open(CONFIG_FILE, "r") as f:
            data = json.load(f)

        for k, v in data.items():
            if k in self.checkboxes:
                self.checkboxes[k].setChecked(bool(v))
                self.node.set_dsp_param(k, v)
            elif k in self.sliders:
                s, cfg = self.sliders[k]
                s.setValue(int(v * cfg["scale"]))
                self.node.set_dsp_param(k, v)

        self.loading_config = False

    def closeEvent(self, event):
        self.save_config()
        event.accept()

    # UPDATE
    def update_plot(self):
        buf = self.node.buffer
        self.curve_wave.setData(buf)
        fft = np.abs(np.fft.rfft(buf))
        fft[fft == 0] = 1e-12
        fft_db = 20 * np.log10(fft)
        freq = np.fft.rfftfreq(len(buf), 1.0 / RATE)
        self.curve_fft.setData(freq, fft_db)
    
    def set_visibility(self, enable: bool):
        if enable:
            self.show()
        else:
            self.hide()

    def reset_config(self):
        self.loading_config = True
        for name, value in self.DEFAULT_VALUES.items():
            if name in self.checkboxes:
                self.checkboxes[name].setChecked(bool(value))
                self.node.set_dsp_param(name, value)

            elif name in self.sliders:
                slider, cfg = self.sliders[name]
                slider.setValue(int(value * cfg["scale"]))
                self.node.set_dsp_param(name, value)

        self.loading_config = False

    def toggle_edit_mode(self):
        self.edit_enabled = not self.edit_enabled
        if self.edit_enabled:
            self.btn_toggle.setText("🔓 EDIT MODE")
        else:
            self.btn_toggle.setText("🔒 LOCKED")

        for cb in self.checkboxes.values():
            cb.setEnabled(self.edit_enabled)

        for slider, _ in self.sliders.values():
            slider.setEnabled(self.edit_enabled)


# MAIN
def main():
    global _app_window
    rclpy.init()
    node = GraphNode()
    app = QtWidgets.QApplication(sys.argv)
    win = AudioWindow(node)
    _app_window = win

    # callback
    node.gui_callback = win.set_visibility
    # win.show()   
    win.hide() 
    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    def handle_sigint(sig, frame):
        print("\n💾 Ctrl+C detected, saving DSP config...")
        if _app_window is not None:
            _app_window.save_config()
        QtWidgets.QApplication.quit()
    signal.signal(signal.SIGINT, handle_sigint)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
