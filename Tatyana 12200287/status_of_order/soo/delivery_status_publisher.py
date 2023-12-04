import rclpy
from std_msgs.msg import String

def delivery_status_publisher():
    rclpy.init()
    node = rclpy.create_node('delivery_status_publisher_node')

    # Create a publisher for delivery status
    delivery_status_publisher = node.create_publisher(String, 'delivery_status', 10)

    # Publish delivery status periodically
    delivery_statuses = ["In progress", "Out for delivery", "Delivered"]
    rate = node.create_rate(1)  # 1 Hz

    for status in delivery_statuses:
        delivery_status_msg = String()
        delivery_status_msg.data = status
        delivery_status_publisher.publish(delivery_status_msg)
        rate.sleep()

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    delivery_status_publisher()

