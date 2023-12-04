""" 
Author: Feruz 12204578

ROS2 Node to Parse Speech """

from typing import List
import click
import rclpy
import ament_index_python
from std_msgs.msg import String
from speech_to_text_msgs.msg import StringArray
from jsgf import parse_grammar_file
from simple_node import Node


class ParserNode(Node):
    def __init__(self, grammar_path: str):
        # Initialize the ParserNode, inheriting from the Node class
        super().__init__("parser_node")

        # Parse the specified grammar file
        self.jsgf_grammar = parse_grammar_file(grammar_path)

        # Create ROS2 publisher for parsed strings
        self.__pub = self.create_publisher(StringArray, "stt_parse", 10)
        
        # Create ROS2 subscription for receiving raw speech-to-text messages
        self.__subscription = self.create_subscription(
            String,
            "stt_nlp",
            self.__parse,
            10)

    def __parse(self, msg: String):
        # Callback function to parse incoming speech-to-text message
        data = msg.data
        new_msg = StringArray()
        new_msg.strings = self.parse(data)
        
        # Publish the parsed tags
        self.__pub.publish(new_msg)

    def parse(self, data: str) -> List[str]:
        # Parse the input text into a list of tags
        rule = self.jsgf_grammar.find_matching_rules(data)
        tag_list = []
        if rule:
            for tag in rule[0].matched_tags:
                tag_list.append(tag)

        # Log the parsed tags
        self.get_logger().info("Parser: " + str(tag_list))
        return tag_list


@click.command()
@click.option('--grammar', default=ament_index_python.get_package_share_directory(
    "speech_to_text") + "/grammars/example.gram", help='Path to the grammar file')
def cli(grammar):
    # Command-line interface using click

    # Initialize ROS2
    rclpy.init()

    # Create an instance of the ParserNode with the specified grammar file path
    node = ParserNode(grammar)

    try:
        # Run the ROS2 node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup resources on node shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # Run the command-line interface when the script is executed
    cli()
