# Order Processing Project

This project contains two main functions for processing orders and parsing data received through a ROS2 topic.

## order_processing Function

The `order_processing` function manages inventory and processes food orders. It takes a list of items and their quantities, checks the inventory, prepares the available items, and updates the inventory accordingly.

### Usage

To use the `order_processing` function, provide a list of tuples where each tuple contains the item and its quantity. For example:

```python
items_to_order = [('pizza', 2)]
result = process_order(items_to_order)
print(result)
```

## parsing_function Function

The `parsing_function` operates within a ROS2 environment as a subscriber. It listens to a specific topic, extracts keywords from the received data, maps synonyms to predefined keywords, and logs the extracted data.

### ROS2 Setup

#### Python Environment Setup

- Ensure Python is installed.
- Install required libraries:

```bash
pip install rclpy std_msgs  # For the ROS2 environment
```

#### Running the Function

- Set up a ROS2 environment.
- Run the ROS2 node using the provided Python script (`parsing_function.py`).
- Ensure the ROS2 topic named 'topic' is publishing data.

### Usage

The function operates within the ROS2 ecosystem and doesn't have a direct standalone usage like the `order_processing` function. It's meant to be subscribed to a ROS2 topic for data processing.

## Setup and Running the Functions

### Running the order_processing Function

- Run the script containing the `order_processing` function.
- Adjust the `items_to_order` variable as needed to test different orders.

### Running the parsing_function Function

- Set up a ROS2 environment.
- Run the ROS2 node using the provided Python script (`parsing_function.py`).
- Ensure the ROS2 topic named 'topic' is publishing data.

## Author

Temurbek Akhrorov (ID: 12204574)
