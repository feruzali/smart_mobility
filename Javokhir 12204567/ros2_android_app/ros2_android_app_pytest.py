# Example test_android_app.py

import pytest
from app import MainActivity  
import Publisher, Message, BaseEntity

@pytest.fixture(scope="class")
def app(request, android_driver):
    app = MainActivity(driver=android_driver)
    request.cls.app = app
    yield app
    app.close()

@pytest.mark.usefixtures("app")
class TestJoystickFunctionality:
    def test_joystick_to_ros_message(self):
        joystick_data = {"axis_x": 0.5, "axis_y": -0.3}
        ros_publisher = Publisher(Message)  

        # Call the toRosMessage function
        self.app.toRosMessage(ros_publisher, joystick_data)

