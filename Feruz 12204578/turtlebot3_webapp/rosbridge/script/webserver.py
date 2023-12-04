# Author: Feruz 12204578

"""
ROS Web Server Script

This script implements a web server for the Robot Operating System (ROS) using the ROSWWW package. It provides a
flexible configuration through command-line arguments and serves web pages from a specified package relative path.

Overview of Functionality:
- Parses command-line arguments to configure the ROS web server.
- Initializes a ROS node named "webserver" with signal handling disabled.
- Creates an instance of the ROSWWWServer class to handle web server functionality.
- Starts the web server to serve static files and handle incoming requests.
"""

# Import necessary modules
import click
import roswww
import rospy
import argparse  # Module for parsing command-line arguments

def parse_argument(argv):
    """
    Argument parser for ROSWWW server configuration
    """
    # Create an argument parser object
    parser = argparse.ArgumentParser(description="ROSWWW Server")

    # Define command-line arguments
    parser.add_argument('-n', '--name', default=rospy.get_name(), help='Webserver name')
    parser.add_argument('-p', '--port', default=80, type=int, help='Webserver Port number')
    parser.add_argument('-w', '--webpath', default='www', help='Package relative path to web pages')
    parser.add_argument('--cached', default='true', help='Static file is cached')
    parser.add_argument('--start_port', default=8000, type=int, help='Setting up port scan range start')
    parser.add_argument('--end_port', default=9000, type=int, help='Setting up port scan range end')

    # Parse command-line arguments
    parsed_args = parser.parse_args(argv)

    # Convert 'cached' argument to a boolean
    cached = False if parsed_args.cached in [0, False, 'false', 'False'] else True

    # Return parsed arguments
    return parsed_args.name, parsed_args.webpath, (parsed_args.port, parsed_args.start_port, parsed_args.end_port), cached

@click.command()
@click.option('--name', default=rospy.get_name(), help='Webserver name')
@click.option('--port', default=80, type=int, help='Webserver Port number')
@click.option('--webpath', default='www', help='Package relative path to web pages')
@click.option('--cached', default='true', help='Static file is cached')
@click.option('--start_port', default=8000, type=int, help='Setting up port scan range start')
@click.option('--end_port', default=9000, type=int, help='Setting up port scan range end')
def run_webserver(name, port, webpath, cached, start_port, end_port):
    """
    Run ROSWWW Server
    """
    # Initialize ROS node named "webserver" with signal handling disabled
    rospy.init_node("webserver", disable_signals=True)

    # Create ROSWWWServer instance
    webserver = roswww.ROSWWWServer(name, webpath, (port, start_port, end_port), cached)

    # Log initialization information
    webserver.loginfo("Initialized")

    # Start the ROSWWWServer
    webserver.spin()

if __name__ == '__main__':
    run_webserver()
