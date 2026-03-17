# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import UInt8MultiArray
# import pyaudio

# RATE = 16000
# CHANNELS = 6   # ใช้ 6 ถ้า firmware เป็น 6ch
# WIDTH = 2
# CHUNK = 1024

# class AudioNode(Node):
#     def __init__(self):
#         super().__init__('respeaker_audio_node')

#         self.pub = self.create_publisher(UInt8MultiArray, 'respeaker/audio_raw', 10)

#         p = pyaudio.PyAudio()

#         # Find ReSpeaker device index
#         device_index = None
#         for i in range(p.get_device_count()):
#             name = p.get_device_info_by_index(i)['name']
#             if 'ReSpeaker' in name or 'ArrayUAC10' in name:
#                 device_index = i
#                 break

#         if device_index is None:
#             self.get_logger().error("ReSpeaker device not found!")
#             return

#         self.get_logger().info(f"Using ReSpeaker input device index: {device_index}")

#         self.stream = p.open(
#             rate=RATE,
#             channels=CHANNELS,
#             format=p.get_format_from_width(WIDTH),
#             input=True,
#             input_device_index=device_index,
#             frames_per_buffer=CHUNK
#         )

#         self.timer = self.create_timer(0.05, self.publish_audio)

#     def publish_audio(self):
#         data = self.stream.read(CHUNK, exception_on_overflow=False)
#         msg = UInt8MultiArray()
#         msg.data = list(data)
#         self.pub.publish(msg)

# def main():
#     rclpy.init()
#     node = AudioNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import UInt8MultiArray
import pyaudio

RATE = 16000
CHANNELS = 6      
WIDTH = 2
CHUNK = 1024


class AudioNode(Node):
    def __init__(self):
        super().__init__('respeaker_audio_node')

        # Parameters
        self.declare_parameter("mic_mode", "mode1")
        # mode1 = ReSpeaker
        # mode2 = auto scan device 

        self.declare_parameter("preferred_name", "ReSpeaker")
        self.declare_parameter("sample_rate", RATE)
        self.declare_parameter("channels", CHANNELS)
        self.declare_parameter("chunk_size", CHUNK)
        self.declare_parameter("debug_log", True)

        self.mic_mode = self.get_parameter("mic_mode").value
        self.preferred_name = self.get_parameter("preferred_name").value
        self.rate = int(self.get_parameter("sample_rate").value)
        self.channels = int(self.get_parameter("channels").value)
        self.chunk = int(self.get_parameter("chunk_size").value)
        self.debug_log = bool(self.get_parameter("debug_log").value)

        if not self.debug_log:
            self.get_logger().set_level(LoggingSeverity.WARN)

        # ROS publisher
        self.pub = self.create_publisher(UInt8MultiArray, 'respeaker/audio_raw', 10)

        # PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.device_index = None

        # Select device by mode
        if self.mic_mode == "mode1":
            self.device_index = self.find_respeaker_device()
        elif self.mic_mode == "mode2":
            self.device_index = self.find_best_input_device()
        else:
            self.get_logger().error(
                f"Invalid mic_mode='{self.mic_mode}'. Use 'mode1' or 'mode2'."
            )
            return

        if self.device_index is None:
            self.get_logger().error("No valid input microphone device found!")
            return

        # Open stream
        try:
            dev_info = self.p.get_device_info_by_index(self.device_index)
            max_input_channels = int(dev_info.get("maxInputChannels", 0))
            use_channels = min(self.channels, max_input_channels)

            if use_channels <= 0:
                self.get_logger().error("Selected device has no input channels.")
                return

            self.channels = use_channels

            self.stream = self.p.open(
                rate=self.rate,
                channels=self.channels,
                format=self.p.get_format_from_width(WIDTH),
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=self.chunk
            )

            self.get_logger().info(
                f"Audio stream opened | mode={self.mic_mode} | "
                f"device_index={self.device_index} | "
                f"channels={self.channels} | rate={self.rate}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}")
            self.stream = None
            return

        # Timer
        self.timer = self.create_timer(0.05, self.publish_audio)

    # Device selection
    def list_input_devices(self):
        devices = []
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            max_in = int(info.get("maxInputChannels", 0))
            if max_in > 0:
                devices.append({
                    "index": i,
                    "name": info.get("name", ""),
                    "max_input_channels": max_in,
                    "default_rate": info.get("defaultSampleRate", 0),
                })
        return devices

    def find_respeaker_device(self):
        """
        mode1:
        หาเฉพาะ ReSpeaker / ArrayUAC10 / preferred_name
        """
        candidates = self.list_input_devices()

        # priority names
        keywords = [
            self.preferred_name.lower(),
            "respeaker",
            "arrayuac10",
        ]

        for dev in candidates:
            name_lower = dev["name"].lower()
            for kw in keywords:
                if kw and kw in name_lower:
                    self.get_logger().info(
                        f"[mode1] Selected ReSpeaker-like device: "
                        f"index={dev['index']} name='{dev['name']}' "
                        f"max_in={dev['max_input_channels']}"
                    )
                    return dev["index"]

        self.get_logger().warn("[mode1] ReSpeaker device not found.")
        return None

    def find_best_input_device(self):
        """
        mode2:
        สแกนทุก input device แล้วเลือกอัตโนมัติ
        strategy:
        1) ถ้าเจอ ReSpeaker-like -> ใช้อันนั้นก่อน
        2) ถ้าไม่เจอ -> เลือก device ที่มี input channels มากที่สุด
        """
        candidates = self.list_input_devices()

        if not candidates:
            self.get_logger().warn("[mode2] No input devices found.")
            return None

        if self.debug_log:
            self.get_logger().info("Available input devices:")
            for dev in candidates:
                self.get_logger().info(
                    f"  index={dev['index']} | name='{dev['name']}' | "
                    f"max_in={dev['max_input_channels']} | "
                    f"default_rate={dev['default_rate']}"
                )

        for dev in candidates:
            name_lower = dev["name"].lower()
            if (
                self.preferred_name.lower() in name_lower
                or "respeaker" in name_lower
                or "arrayuac10" in name_lower
            ):
                self.get_logger().info(
                    f"[mode2] Auto-selected preferred device: "
                    f"index={dev['index']} name='{dev['name']}'"
                )
                return dev["index"]

        best = max(candidates, key=lambda d: d["max_input_channels"])

        self.get_logger().info(
            f"[mode2] Auto-selected best input device: "
            f"index={best['index']} name='{best['name']}' "
            f"max_in={best['max_input_channels']}"
        )
        return best["index"]

    # Publish audio
    def publish_audio(self):
        if self.stream is None:
            return

        try:
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            msg = UInt8MultiArray()
            msg.data = list(data)
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Audio read failed: {e}")

    # Cleanup
    def cleanup(self):
        if self.debug_log:
            self.get_logger().info("Cleaning up audio resources...")

        try:
            if self.stream is not None:
                self.stream.stop_stream()
                self.stream.close()
        except Exception:
            pass

        try:
            if self.p is not None:
                self.p.terminate()
        except Exception:
            pass


def main():
    rclpy.init()
    node = AudioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl+C pressed, shutting down audio node...")
    finally:
        try:
            node.cleanup()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()