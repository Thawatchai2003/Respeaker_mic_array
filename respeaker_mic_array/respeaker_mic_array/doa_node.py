import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import usb.core

pkg_path = os.path.dirname(__file__)
sys.path.append(os.path.join(pkg_path, "usb_4_mic_array"))

from .usb_4_mic_array.tuning import Tuning



class DOANode(Node):
    def __init__(self):
        super().__init__('respeaker_doa_node')

        self.pub = self.create_publisher(Int32, 'respeaker/doa', 10)

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            self.get_logger().error("ReSpeaker device not found!")
            return

        self.mic = Tuning(dev)
        self.timer = self.create_timer(0.1, self.publish_doa)

    def publish_doa(self):
        msg = Int32()
        msg.data = self.mic.direction
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = DOANode()
    rclpy.spin(node)
    rclpy.shutdown()
