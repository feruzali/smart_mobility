import rospy
from std_msgs.msg import String
import click

class WaiterBot:
    def __init__(self, speech_topic, button_topic):
        # Initialize ROS node
        rospy.init_node('waiter_bot')

        # Subscribe to the speech-to-text topic
        rospy.Subscriber(speech_topic, String, self.speech_to_text_callback)

        # Subscribe to the "Call the waiter" button topic
        rospy.Subscriber(button_topic, String, self.call_waiter_callback)

        # Spin to keep the script running
        rospy.spin()

    def speech_to_text_callback(self, data):
        # Process the text received from speech-to-text
        customer_order = data.data
        print(f"Customer order: {customer_order}")

    
    
    def call_waiter_callback(self, data):
        # Process the "Call the waiter" button press
        if data.data == 'pressed':
            self.move_to_customer_location()


@click.command()
@click.option('--speech-topic', default='speech_to_text_topic', help='ROS topic for speech-to-text messages')
@click.option('--button-topic', default='call_waiter_button_topic', help='ROS topic for "Call the waiter" button')
def run_waiter_bot(speech_topic, button_topic):
    waiter_bot = WaiterBot(speech_topic, button_topic)

if __name__ == "__main__":
    run_waiter_bot()
