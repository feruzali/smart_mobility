import rclpy
from std_msgs.msg import String

def delivery_status_callback(msg):
    print(f"Received delivery status: {msg.data}")

def delivery_status_subscriber():
    rclpy.init()
    node = rclpy.create_node('delivery_status_subscriber_node')

    # Create a subscriber for delivery status
    delivery_status_subscriber = node.create_subscription(
        String,
        'delivery_status',
        delivery_status_callback,
        10
    )

    # Spin to keep the node alive
    rclpy.spin(node)

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    delivery_status_subscriber()

