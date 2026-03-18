#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
        self.declare_parameter("mode", 1)  
        self.declare_parameter("device_index", -1)  
        self.declare_parameter("rate", RATE)
        self.declare_parameter("channels", CHANNELS)
        self.declare_parameter("chunk", CHUNK)
        self.declare_parameter("auto_scan_interval_sec", 2.0)

        self.mode = int(self.get_parameter("mode").value)
        self.manual_device_index = int(self.get_parameter("device_index").value)
        self.rate = int(self.get_parameter("rate").value)
        self.channels = int(self.get_parameter("channels").value)
        self.chunk = int(self.get_parameter("chunk").value)
        self.auto_scan_interval_sec = float(self.get_parameter("auto_scan_interval_sec").value)

        # Publisher
        self.pub = self.create_publisher(UInt8MultiArray, 'respeaker/audio_raw', 10)

        # PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.device_index = None

        # Start logic
        if self.mode == 1:
            # mode1 
            ok = self.try_open_stream()
            if ok:
                self.get_logger().info("✅ mode=1: microphone ready")
            else:
                self.get_logger().error("❌ mode=1: ReSpeaker device not found at startup")

        elif self.mode == 2:
            # mode2 = auto scan / auto reconnect
            self.get_logger().info("🔄 mode=2: auto scan enabled")
            self.try_open_stream()

            # timer สำหรับสแกน/เช็ค reconnect
            self.scan_timer = self.create_timer(self.auto_scan_interval_sec, self.scan_and_recover)

        else:
            self.get_logger().warning(f"Unknown mode={self.mode}, fallback to mode=1")
            self.try_open_stream()

        # publish timer
        self.publish_timer = self.create_timer(0.05, self.publish_audio)

    # Find device
    def find_respeaker_device(self):
        # ถ้าระบุ device index มาเอง
        if self.manual_device_index >= 0:
            try:
                info = self.p.get_device_info_by_index(self.manual_device_index)
                if int(info.get("maxInputChannels", 0)) > 0:
                    self.get_logger().info(
                        f"Using manual device index: {self.manual_device_index} ({info['name']})"
                    )
                    return self.manual_device_index
                else:
                    self.get_logger().warning(
                        f"manual device_index={self.manual_device_index} is not input device"
                    )
            except Exception as e:
                self.get_logger().warning(f"Invalid manual device_index={self.manual_device_index}: {e}")

        # auto search
        best_index = None
        input_candidates = []

        for i in range(self.p.get_device_count()):
            try:
                info = self.p.get_device_info_by_index(i)
                name = str(info.get("name", ""))
                max_in = int(info.get("maxInputChannels", 0))
                if max_in <= 0:
                    continue

                input_candidates.append((i, name))

                lname = name.lower()
                if 'respeaker' in lname or 'arrayuac10' in lname:
                    self.get_logger().info(f"🎤 Found ReSpeaker candidate: index={i}, name={name}")
                    return i

                # fallback preference
                if best_index is None:
                    if lname.strip() in ["pulse", "default"]:
                        best_index = i
            except Exception:
                continue

        if best_index is not None:
            self.get_logger().warning(f"⚠️ ReSpeaker not found, fallback input device index={best_index}")
            return best_index

        return None

    # Open / Close stream
    def try_open_stream(self):
        if self.stream is not None:
            return True

        device_index = self.find_respeaker_device()
        if device_index is None:
            self.device_index = None
            return False

        try:
            self.stream = self.p.open(
                rate=self.rate,
                channels=self.channels,
                format=self.p.get_format_from_width(WIDTH),
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk
            )
            self.device_index = device_index
            self.get_logger().info(
                f"✅ Opened mic stream: device_index={device_index}, rate={self.rate}, ch={self.channels}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Failed to open mic stream on device {device_index}: {e}")
            self.stream = None
            self.device_index = None
            return False

    def close_stream(self):
        if self.stream is not None:
            try:
                self.stream.stop_stream()
            except Exception:
                pass
            try:
                self.stream.close()
            except Exception:
                pass

        self.stream = None
        self.device_index = None

    # Mode2 auto scan / recover
    def scan_and_recover(self):
        if self.mode != 2:
            return

        # ถ้ายังไม่มี stream -> พยายามหาใหม่
        if self.stream is None:
            self.get_logger().info("🔍 Auto-scan: trying to find/open microphone...")
            self.try_open_stream()

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
            self.get_logger().warning(f"⚠️ Audio read failed: {e}")

            # ถ้า mode2 -> ปิดแล้วรอ auto reconnect
            if self.mode == 2:
                self.get_logger().warning("🔄 Closing stream and waiting for auto reconnect...")
                self.close_stream()
            else:
                # mode1 = ไม่ reconnect
                self.get_logger().error("❌ mode=1: stream failed, no auto reconnect")

    # Cleanup
    def destroy_node(self):
        self.close_stream()
        try:
            self.p.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
