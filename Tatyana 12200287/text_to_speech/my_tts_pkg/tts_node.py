import rclpy
from std_msgs.msg import String
from gtts import gTTS
import os
import platform

class TextToSpeechNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        self.subscription = self.create_subscription(
            String,
            'tts_request',
            self.tts_callback,
            10
        )
        self.get_logger().info('Text-to-Speech Node is ready')

    def tts_callback(self, msg):
        text = msg.data
        self.text_to_speech(text)

    def text_to_speech(self, text, language='en', save_to_file='output.mp3'):
        # Create a gTTS object
        tts = gTTS(text=text, lang=language, slow=False)

        # Save the speech to a file
        tts.save(save_to_file)

        # Play the saved speech using the default audio player
        if platform.system() == 'Darwin':  # Check if the platform is macOS
            os.system("open " + save_to_file)
        else:
            self.get_logger().info("Unsupported platform for playing audio.")

def main(args=None):
    rclpy.init(args=args)
    text_to_speech_node = TextToSpeechNode()
    rclpy.spin(text_to_speech_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

