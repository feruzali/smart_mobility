"""
Author: Feruz 12204578

Speech to Text ROS Launch Configuration

This ROS launch file configures and launches the nodes for speech recognition, natural language processing,
and dialog management within the 'speech_to_text' ROS package.

Overview:
- Defines launch configuration parameters for speech-to-text and parser nodes.
- Configures nodes with specified parameters, including grammar files and service types.
- Sets environment variable for line-buffered console output.
- Launches 'stt_node', 'nlp_node', 'parser_node', and 'dialog_manager_node' nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import ament_index_python

def generate_launch_description():
    # Package name and namespace
    pkg_name = "speech_to_text"
    namespace = "speech_to_text"

    # Set environment variable for line-buffered console output
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    #
    # Launch Arguments
    #

    # Speech-to-text node launch arguments
    stt_grammar = LaunchConfiguration("stt_grammar")
    declare_stt_grammar_cmd = DeclareLaunchArgument(
        "stt_grammar",
        default_value=ament_index_python.get_package_share_directory(pkg_name) + "/grammars/example.gram",
        description="Stt node gram file"
    )

    stt_service = LaunchConfiguration("stt_service")
    declare_stt_service_cmd = DeclareLaunchArgument(
        "stt_service",
        default_value="sphinx",
        description="Speech recognition service"
    )

    stt_started = LaunchConfiguration("stt_started")
    declare_stt_started_cmd = DeclareLaunchArgument(
        "stt_started",
        default_value="false",
        description="Is stt started?"
    )

    # Parser node launch arguments
    parser_grammar = LaunchConfiguration("parser_grammar")
    declare_parser_grammar_cmd = DeclareLaunchArgument(
        "parser_grammar",
        default_value=ament_index_python.get_package_share_directory(pkg_name) + "/grammars/example.gram",
        description="Parser node gram file"
    )

    #
    # Nodes
    #

    # Speech-to-text node
    stt_node_cmd = Node(
        package=pkg_name,
        executable="stt_node",
        name="stt_node",
        namespace=namespace,
        parameters=[{"grammar": stt_grammar},
                    {"service": stt_service},
                    {"started": stt_started}]
    )

    # Natural language processing node
    nlp_node_cmd = Node(
        package=pkg_name,
        executable="nlp_node",
        name="nlp_node",
        namespace=namespace,
    )

    # Parser node
    parser_node_cmd = Node(
        package=pkg_name,
        executable="parser_node",
        name="parser_node",
        namespace=namespace,
        parameters=[{"grammar": parser_grammar}]
    )

    # Dialog manager node
    dialog_manager_node_cmd = Node(
        package=pkg_name,
        executable="dialog_manager_node",
        name="dialog_manager_node",
        namespace=namespace,
    )

    # Launch Description
    ld = LaunchDescription()

    # Add actions to Launch Description
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_stt_grammar_cmd)
    ld.add_action(declare_stt_service_cmd)
    ld.add_action(declare_stt_started_cmd)
    ld.add_action(declare_parser_grammar_cmd)
    ld.add_action(stt_node_cmd)
    ld.add_action(nlp_node_cmd)
    ld.add_action(parser_node_cmd)
    ld.add_action(dialog_manager_node_cmd)

    return ld
