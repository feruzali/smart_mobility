### Menu App

#### Overview
The menu app is a simple graphical user interface (GUI) created using Python's Tkinter library. It displays a menu with images, names, and prices in a grid layout.

#### Files
- `menu_app.py`: This file contains the code for the menu app.
- `/home/jvox/Desktop/WS/img/`: This folder contains the images used by the menu app.

#### How to Run
1. Ensure you have Python installed on your system.
2. Install the required libraries: Tkinter, Pillow.
    ```bash
    pip install tk pillow
    ```
3. Run the menu app:
    ```bash
    python menu_app.py
    ```

#### Functionality
- The app displays a grid of images, each with its name and price.
- Images are loaded from the specified folder.
- Clicking on an image can trigger further actions, such as placing an order.

#### Modification
- You can customize the image folder path in `menu_app.py`.
- Adjust the layout or add functionality as needed.

---

### ROS2 Publisher and Subscriber

#### Overview
The ROS2 publisher and subscriber are part of a ROS2 (Robot Operating System 2) system written in Python using the `rclpy` library. It demonstrates a minimal publisher-subscriber architecture, where the publisher sends user-input messages to a topic, and the subscriber processes and logs the received messages.

#### Files
- `minimal_publisher.py`: Publisher node code.
- `minimal_subscriber.py`: Subscriber node code.

#### How to Run
1. Install ROS2 on your system following the [official ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html).
2. Create a ROS2 workspace and place the files in the `src` folder.
    ```bash
    mkdir -p ros2_ws/src
    cp minimal_publisher.py minimal_subscriber.py ros2_ws/src/
    cd ros2_ws
    colcon build
    source install/setup.bash
    ```
3. Open two terminal windows.
4. In the first window, run the ROS2 publisher:
    ```bash
    ros2 run <your_package_name> minimal_publisher.py
    ```
5. In the second window, run the ROS2 subscriber:
    ```bash
    ros2 run <your_package_name> minimal_subscriber.py
    ```

#### Functionality
- The publisher collects user input and publishes messages to the 'order' topic.
- The subscriber listens to the 'order' topic, extracts keywords, maps synonyms, and logs the extracted data to a text file.
