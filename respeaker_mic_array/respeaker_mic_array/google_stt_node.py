#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import speech_recognition as sr
import threading
import time
import queue
import numpy as np


RATE = 16000


class GoogleSTTNode(Node):
    def __init__(self):
        super().__init__('google_stt_node')

        # ===== รับเสียงที่ผ่าน filter แล้ว =====
        self.sub_audio = self.create_subscription(
            Int16MultiArray,
            '/audio/mono',
            self.audio_callback,
            10
        )

        # ===== รับคำสั่งจาก GUI =====
        self.sub_gui = self.create_subscription(
            String,
            'voice/gui_event',
            self.gui_event_callback,
            10
        )

        # ===== publish ผล STT =====
        self.pub = self.create_publisher(String, 'voice/text', 10)

        self.recognizer = sr.Recognizer()

        self.audio_queue = queue.Queue()

        # ----- สถานะ -----
        self.armed = False
        self.one_shot = True

        # สำหรับ VAD
        self.speech_started = False

        # worker thread
        self.worker = threading.Thread(target=self.process_loop)
        self.worker.daemon = True
        self.worker.start()

        self.get_logger().info("Google STT Node Ready (VAD mode)")


    # =====================================================
    #  รับคำสั่งจาก GUI
    # =====================================================
    def gui_event_callback(self, msg):
        cmd = msg.data.strip().upper()

        if cmd == "MANUAL_ARM":

            # ===== ล้างเสียงเก่า =====
            with self.audio_queue.mutex:
                self.audio_queue.queue.clear()

            self.speech_started = False
            self.armed = True

            self.get_logger().info("STT ARMED by GUI (wait real voice)")


    # =====================================================
    #  รับเสียงจาก audio_listener
    # =====================================================
    def audio_callback(self, msg):
        try:
            mono = np.array(msg.data, dtype=np.int16)
            data_mono = mono.tobytes()

            self.audio_queue.put(data_mono)

        except Exception as e:
            self.get_logger().error(f"convert error: {e}")


    # =====================================================
    #  VAD ตรวจว่าเริ่มพูดจริงหรือยัง
    # =====================================================
    def has_voice(self, pcm: bytes):
        arr = np.frombuffer(pcm, dtype=np.int16)

        if len(arr) < 2000:
            return False

        rms = np.sqrt(np.mean(arr.astype(np.float32)**2))

        # ===== debug =====
        self.get_logger().info(f"RMS = {rms:.1f}")

        # ---- ค่านี้เดี๋ยวจูนตามห้องคุณ ----
        return rms > 1400


    # =====================================================
    #  Thread หลักของ STT
    # =====================================================
    def process_loop(self):
        buffer = b""
        last_time = time.time()

        while rclpy.ok():

            # ===== รอ ARM =====
            if not self.armed:
                time.sleep(0.05)
                continue

            try:
                data = self.audio_queue.get(timeout=0.1)
                buffer += data
            except queue.Empty:
                pass

            # ===== 1) ยังไม่เริ่มพูดจริง =====
            if not self.speech_started:
                if self.has_voice(buffer):
                    self.speech_started = True
                    last_time = time.time()
                    self.get_logger().info("Voice detected → start collect")
                continue

            # ===== 2) เริ่มพูดแล้ว รอให้ยาวพอ =====
            if time.time() - last_time < 3.5:
                continue

            # ===== 3) ส่ง STT =====
            self.do_stt(buffer)

            buffer = b""
            self.speech_started = False
            last_time = time.time()


    # =====================================================
    #  ส่งเข้า Google STT
    # =====================================================
    def do_stt(self, pcm_data: bytes):
        try:
            audio = sr.AudioData(pcm_data, RATE, 2)

            text = self.recognizer.recognize_google(
                audio,
                language="th-TH"
            )

            if text.strip() != "":
                msg = String()
                msg.data = text
                self.pub.publish(msg)

                self.get_logger().info(f"Recognized: {text}")

        except sr.UnknownValueError:
            self.get_logger().info("not understand")

        except sr.RequestError as e:
            self.get_logger().error(f"Google STT error: {e}")

        except Exception as e:
            self.get_logger().error(f"STT exception: {e}")

        # ===== one shot =====
        if self.one_shot:
            self.armed = False
            self.get_logger().info("STT DISARM (one shot)")


# =====================================================
def main():
    rclpy.init()
    node = GoogleSTTNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
