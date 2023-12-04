import pytest
import rclpy
from std_msgs.msg import String
from your_package.order_status_handler import OrderStatusHandler  # Import your actual implementation

@pytest.fixture
def order_status_handler():
    rclpy.init()
    order_status_handler = OrderStatusHandler()  # Instantiate your order status handler
    yield order_status_handler
    order_status_handler.destroy_node()
    rclpy.shutdown()

def test_order_status(order_status_handler):
    # Assuming your order status handler publishes the status to a topic
    order_status_handler.publish_order_status("delivered")

    # Sleep for a short time to allow the message to be processed
    rclpy.spin_once(order_status_handler, timeout_sec=1.0)

    # Assuming your order status handler has a method to get the current status
    current_status = order_status_handler.get_current_status()
    assert current_status == "delivered"
