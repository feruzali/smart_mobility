import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        received_data = msg.data

        # Key words
        strings_to_extract = ["Burger", "burger", "Pizza", "pizza", "Burrito", "burrito",
                              "Kebab", "kebab", "Mandi", "mandi", "Spaghetti", "spaghetti", "No onion", "no onion"]

        # String mapping
        synonym_mapping = {"Without Onion": "No onion", "without Onion": "No onion","without onion": "No onion","Without onion": "No onion",}

        extracted_data = []

        # String mapping to key words
        for string_to_extract in strings_to_extract:
            if string_to_extract in received_data:
                extracted_data.append(synonym_mapping.get(string_to_extract, string_to_extract))

        if extracted_data:
            self.get_logger().info('Extracted: %s' % ', '.join(extracted_data))

            # Append data to a text file
            with open('/home/jvox/Desktop/WS/src/order/order/extracted_data.txt', 'a') as file:
                file.write(', '.join(extracted_data) + '\n')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

