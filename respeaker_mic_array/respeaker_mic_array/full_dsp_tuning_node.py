import rclpy
from rclpy.node import Node
import usb.core
import sys
import os

pkg_path = os.path.dirname(__file__)
sys.path.append(os.path.join(pkg_path, "usb_4_mic_array"))

from .usb_4_mic_array.tuning import Tuning


class FullDSPTuningNode(Node):
    def __init__(self):
        super().__init__('respeaker_full_dsp')

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            self.get_logger().error("ReSpeaker not found")
            return

        self.mic = Tuning(dev)

        # ==========================
        # CORE DSP SWITCHES
        # ==========================
        self.declare_parameter("AGCONOFF", 1)
        self.declare_parameter("ECHOONOFF", 1)
        self.declare_parameter("STATNOISEONOFF", 1)
        self.declare_parameter("NONSTATNOISEONOFF", 1)
        self.declare_parameter("FREEZEONOFF", 0)
        self.declare_parameter("AECFREEZEONOFF", 0)
        self.declare_parameter("TRANSIENTONOFF", 1)
        self.declare_parameter("RT60ONOFF", 1)
        self.declare_parameter("CNIONOFF", 1)
        self.declare_parameter("NLATTENONOFF", 1)
        self.declare_parameter("NLAEC_MODE", 2)

        # ==========================
        # GAIN / AGC CONTROL
        # ==========================
        self.declare_parameter("AGCMAXGAIN", 28.0)
        self.declare_parameter("AGCDESIREDLEVEL", 0.01)
        self.declare_parameter("AGCTIME", 0.4)

        # ==========================
        # NOISE SUPPRESSION
        # ==========================
        self.declare_parameter("GAMMA_NS", 1.6)
        self.declare_parameter("GAMMA_NN", 1.4)
        self.declare_parameter("MIN_NS", 0.12)
        self.declare_parameter("MIN_NN", 0.2)

        # ==========================
        # ASR OPTIMIZATION
        # ==========================
        self.declare_parameter("STATNOISEONOFF_SR", 1)
        self.declare_parameter("NONSTATNOISEONOFF_SR", 1)
        self.declare_parameter("GAMMA_NS_SR", 1.2)
        self.declare_parameter("GAMMA_NN_SR", 1.2)
        self.declare_parameter("MIN_NS_SR", 0.18)
        self.declare_parameter("MIN_NN_SR", 0.2)

        # ==========================
        # VAD
        # ==========================
        self.declare_parameter("GAMMAVAD_SR", 2.6)

        # ==========================
        # BEAMFORMING / DOA
        # ==========================
        self.declare_parameter("GAMMA_E", 1.4)
        self.declare_parameter("GAMMA_ETAIL", 1.2)

        # ==========================
        # ECHO / AEC
        # ==========================
        self.declare_parameter("GAMMA_ENL", 2.2)
        self.declare_parameter("AECNORM", 1.0)

        # ==========================
        # HIGH PASS FILTER
        # ==========================
        self.declare_parameter("HPFONOFF", 2)

        # ==========================
        # TIMER APPLY DSP
        # ==========================
        self.timer = self.create_timer(1.0, self.apply_all_params)

    def apply_all_params(self):
        for name, param in self._parameters.items():
            val = param.value
            try:
                self.mic.write(name, val)
            except Exception as e:
                self.get_logger().warn(f"Skip {name}: {e}")

        self.get_logger().info("DSP FULL PROFILE APPLIED")

def main():
    rclpy.init()
    node = FullDSPTuningNode()
    rclpy.spin(node)
    rclpy.shutdown()
