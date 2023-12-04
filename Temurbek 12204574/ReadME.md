Temurbek Akhrorov's Order Processing Project
Description
This project contains two main functions for processing orders and parsing data received through a ROS2 topic.

order_processing Function
The order_processing function manages inventory and processes food orders. It takes a list of items and their quantities, checks the inventory, prepares the available items, and updates the inventory accordingly.

Usage
To use the order_processing function, you can provide a list of tuples where each tuple contains the item and its quantity. For example:

python
Copy code
items_to_order = [('pizza', 2)]
result = process_order(items_to_order)
print(result)
parsing_function Function
The parsing_function operates within a ROS2 environment as a subscriber. It listens to a specific topic and extracts keywords from the received data. It then maps synonyms to a predefined set of keywords and logs the extracted data.

ROS2 Setup
To use the parsing_function, follow these steps:

Python Environment Setup

Ensure you have Python installed.
Install required libraries:
bash
Copy code
pip install rclpy std_msgs  # For the ROS2 environment
Running the Function

Set up a ROS2 environment.
Run the ROS2 node using the provided Python script (parsing_function.py).
Ensure the ROS2 topic named 'topic' is publishing data.
Usage
The function operates within the ROS2 ecosystem and doesn't have a direct standalone usage like the order_processing function. It's meant to be subscribed to a ROS2 topic for data processing.

Setup and Running the Functions
To run these functions, ensure you have the necessary Python dependencies installed and set up a ROS2 environment for the parsing_function.

Running the Functions
For the order_processing function:

Run the script containing the order_processing function.
Adjust the items_to_order variable as needed to test different orders.
For the parsing_function function:

Set up a ROS2 environment.
Run the ROS2 node using the provided Python script (parsing_function.py).
Ensure the ROS2 topic named 'topic' is publishing data.
Author
Temurbek Akhrorov (ID: 12204574)
