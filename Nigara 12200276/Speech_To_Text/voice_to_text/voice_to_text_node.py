#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from threading import Thread

import speech_recognition as sr

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        self.__listen_thread = None

        self.__rec = sr.Recognizer()
        self.__mic = sr.Microphone()
        self.__rec.dynamic_energy_threshold = True
        self._energy_threshold = 0

        self.started = self.declare_parameter('started', True).value

        if self.started:
            self.calibrate_stt(2)
            self._start_stt()

        self.__pub = self.create_publisher(String, 'stt', 10)

        self.__start_server = self.create_service(Empty, 'start_listening', self.__start_stt_srv)
        self.__stop_server = self.create_service(Empty, 'stop_listening', self.__stop_stt_srv)
        self.__calibrate_server = self.create_service(Empty, 'calibrate_listening', self.__calibrate_stt_srv)

    def __calibrate_stt_srv(self, req, res):
        self.calibrate_stt(2)
        return res

    def calibrate_stt(self, seconds):
        rec = sr.Recognizer()
        mic = sr.Microphone()
        self.get_logger().info("A moment of silence, please...")
        with mic as source:
            rec.adjust_for_ambient_noise(source, duration=seconds)
        self._energy_threshold = rec.energy_threshold
        self.__rec.energy_threshold = self._energy_threshold
        self.get_logger().info(f"Set minimum energy threshold to {self._energy_threshold}")

    def listen_from_mic(self):
        while self.started and rclpy.ok():
            self.get_logger().info(f"Threshold {self.__rec.energy_threshold}")
            self.get_logger().info("Say something!")
            with self.__mic as source:
                audio = self.__rec.listen(source)
                self._energy_threshold = self.__rec.energy_threshold
                self.get_logger().info("Got it! Now to recognize it...")

                stt_result = String()

                try:
                    value = self.__rec.recognize_google(audio)

                    if value is bytes:
                        self.get_logger().info(format(value).encode('utf-8'))
                        stt_result.data = format(value).encode('utf-8')
                    else:
                        self.get_logger().info(format(value))
                        stt_result.data = format(value)

                except sr.UnknownValueError:
                    stt_result.data = 'UnknownValueError'

                except sr.RequestError as error:
                    stt_result.data = f"Couldn't request results from Google STT; {error}"

                finally:
                    self.__pub.publish(stt_result)

    def __listen_stt_thread_cb(self):
        try:
            self.get_logger().info("listen_thread starts listening")
            self.listen_from_mic()
        finally:
            self.get_logger().info("listen_thread ends")

    def __start_stt_srv(self, req, res):
        self.start_stt()
        return res

    def start_stt(self):
        if not self.started:
            self.started = not self.started
            self._start_stt()
        else:
            self.get_logger().info("stt is already running")

    def _start_stt(self):
        while self.__listen_thread is not None and self.__listen_thread.is_alive():
            time.sleep(0.01)

        self.__rec.energy_threshold = self._energy_threshold
        self.__listen_thread = Thread(target=self.__listen_stt_thread_cb)
        self.__listen_thread.start()
        self.get_logger().info(f"start listening, Threshold {self.__rec.energy_threshold}")

    def __stop_stt_srv(self, req, res):
        self.stop_stt()
        return res

    def stop_stt(self):
        if self.started:
            self.started = not self.started
            self._stop_stt()
        else:
            self.get_logger().info("stt is already stopped")

    def _stop_stt(self):
        if self.__listen_thread is not None and self.__listen_thread.is_alive():
            self.__listen_thread.terminate()
        self.get_logger().info(f"stop listening, Threshold {self.__rec.energy_threshold}")

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

