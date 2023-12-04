#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gtts import gTTS
from fuzzywuzzy import fuzz
from pydub import AudioSegment
from pydub.playback import play
import speech_recognition as sr

class VoiceOrderNode(Node):
    def __init__(self):
        super().__init__('voice_order_node')
        self.publisher_ = self.create_publisher(String, 'order_feedback', 10)
        self.menu = ["coke", "juice", "pizza", "cheeseburger", "noodles", "chicken"]
        self.recognizer = sr.Recognizer()
        self.confirmation_timer = None

    def speech_to_text(self):
        try:
            with sr.Microphone() as source:
                self.get_logger().info("I'm ready to take your order:")
                self.recognizer.adjust_for_ambient_noise(source)
                audio = self.recognizer.listen(source)

            try:
                text = self.recognizer.recognize_google(audio).lower()
                return text

            except sr.UnknownValueError:
                self.get_logger().info("Sorry, could you repeat?")
                return None
            except sr.RequestError as e:
                self.get_logger().error(f"No results from Google Speech Recognition service; {e}")
                return None

        except Exception as e:
            self.get_logger().error(f"Error in speech_to_text: {e}")
            return None

    def compare_to_menu(self, order_items):
        try:
            found_items = []
            not_found_items = []

            for order in order_items:
                found = False
                for item in self.menu:
                    similarity = fuzz.ratio(order, item.lower())
                    if similarity >= 80:
                        found_items.append(item)
                        found = True
                        break

                if not found:
                    not_found_items.append(order)

            return found_items, not_found_items

        except Exception as e:
            self.get_logger().error(f"Error in compare_to_menu: {e}")
            return [], []

    def text_to_speech(self, feedback_list):
        try:
            for text in feedback_list:
                tts = gTTS(text=text, lang='en')
                tts.save("/tmp/output.mp3")
                sound = AudioSegment.from_mp3("/tmp/output.mp3")
                play(sound)
        except Exception as e:
            self.get_logger().error(f"Error in text_to_speech: {e}")

    def confirmation_callback(self):
        try:
            confirmation_text = "Do you want to proceed with your order? Say 'confirm' to confirm or 'add' to add more items."
            self.get_logger().info(confirmation_text)
            self.text_to_speech([confirmation_text])

            # Listen for user response
            response = self.speech_to_text()

            if response:
                if "confirm" in response:
                    self.get_logger().info("Order confirmed!")
                    # Add your logic here for confirmed order
                elif "add" in response:
                    self.get_logger().info("Adding more items to the order.")
                    # Add your logic here for adding more items
                else:
                    self.get_logger().info("Invalid response. Please say 'confirm' or 'add'.")
        except Exception as e:
            self.get_logger().error(f"Error in confirmation_callback: {e}")

    def main(self):
        try:
            order_items = []

            while True:
                item = self.speech_to_text()

                if item is None:
                    continue
                elif "done" in item or "finish" in item:
                    break
                else:
                    order_items.append(item)

            self.get_logger().info(f"Order items: {order_items}")

            if not order_items:
                self.get_logger().info("No items recognized. Exiting.")
                return

            found_items, not_found_items = self.compare_to_menu(order_items)

            self.get_logger().info(f"Found items: {found_items}")
            self.get_logger().info(f"Not found items: {not_found_items}")

            if found_items:
                feedback_text = f"We have {', '.join(found_items)} on the menu"
                self.get_logger().info(feedback_text)
                self.text_to_speech([feedback_text])

                if not_found_items:
                    not_found_text = f"But we don't have {', '.join(map(str.lower, not_found_items))}"
                    self.get_logger().info(not_found_text)
                    self.text_to_speech([not_found_text])

                # Set a timer to ask for confirmation after a delay
                self.confirmation_timer = self.create_timer(10.0, self.confirmation_callback)

                # Use rclpy.spin to keep the node running
                rclpy.spin(self)

            else:
                self.get_logger().info("No dishes from your order are on the menu.")
                self.text_to_speech(["Sorry, we don't have any of these on the menu"])

        except Exception as e:
            self.get_logger().error(f"Error in main: {e}")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = VoiceOrderNode()
        node.main()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

