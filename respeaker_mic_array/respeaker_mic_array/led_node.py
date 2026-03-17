import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pixel_ring import pixel_ring

# สีสว่าง (ดวงที่ชี้ทิศเสียง / manual)
COLOR_MAP = {
    'blue':   0x0000FF,
    'green':  0x00FF00,
    'yellow': 0xFFFF00,
    'red':    0xFF0000,
    'white':  0xFFFFFF,
    'purple': 0x800080,
}

# สีมืด (ดวงอื่น ๆ ตอน DOA)
DIM_COLOR_MAP = {
    'blue':   0x000033,
    'green':  0x003300,
    'yellow': 0x333300,
    'red':    0x330000,
    'white':  0x333333,
    'purple': 0x330033,
}


class ReSpeakerLEDController(Node):

    def __init__(self):
        super().__init__('respeaker_led_controller')

        # default state
        self.mode = 'doa'      # doa | manual | off
        self.color = 'blue'    # default DOA color

        # subscribers
        self.create_subscription(
            String,
            '/led/mode',
            self.mode_callback,
            10
        )

        self.create_subscription(
            String,
            '/led/color',
            self.color_callback,
            10
        )

        # initial LED state
        self.apply_led()

        self.get_logger().info('ReSpeaker LED Controller started')

    # ---------- callbacks ----------

    def mode_callback(self, msg: String):
        self.mode = msg.data.lower()
        self.get_logger().info(f'LED mode -> {self.mode}')
        self.apply_led()

    def color_callback(self, msg: String):
        self.color = msg.data.lower()
        self.get_logger().info(f'LED color -> {self.color}')
        self.apply_led()

    # ---------- LED logic ----------

    def apply_led(self):
        try:
            if self.mode == 'doa':
                self.apply_doa()

            elif self.mode == 'manual':
                self.apply_manual()

            elif self.mode == 'off':
                pixel_ring.off()

            else:
                self.get_logger().warn(f'Unknown LED mode: {self.mode}')

        except Exception as e:
            self.get_logger().error(f'LED error: {e}')

    def apply_doa(self):
        if self.color not in COLOR_MAP:
            self.get_logger().warn(f'Unknown DOA color: {self.color}')
            return

        bright = COLOR_MAP[self.color]
        dim = DIM_COLOR_MAP[self.color]

        pixel_ring.set_color_palette(bright, dim)

        pixel_ring.listen()

    def apply_manual(self):
        if self.color not in COLOR_MAP:
            self.get_logger().warn(f'Unknown manual color: {self.color}')
            return

        pixel_ring.mono(COLOR_MAP[self.color])


def main():
    rclpy.init()
    node = ReSpeakerLEDController()
    rclpy.spin(node)

    # cleanup
    pixel_ring.off()
    rclpy.shutdown()
