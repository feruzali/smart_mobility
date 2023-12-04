           Turtlebot Functionality Overview

 Overview:

The key functionalities include:

     1. Low Battery Function:
Monitors the battery level and navigates the Turtlebot to the charging station when the battery falls below 20%.
Utilizes 3D mapping for spatial awareness and alerts restaurant staff for manual charging intervention.

     2. Error Detection Mechanism:
Identifies obstacles or issues encountered during navigation.
Triggers alerts, halts the Turtlebot, and prompts manual intervention by restaurant staff for obstacle resolution.

        Importance:

Operational Continuity:

The low battery function ensures seamless operations by autonomously managing the Turtlebot's charging needs, reducing downtimes.
Safety and Efficiency: Error detection minimizes risks by promptly addressing obstacles, fostering a safer and more efficient restaurant environment.
Human-Robot Collaboration: Both functionalities emphasize human-robot collaboration, leveraging automation while requiring human assistance for critical tasks, ensuring optimal functionality and safety.

                 Usage:

Installation: Clone this repository and configure ROS environment for usage.
Launching: Start the ROS Master, initialize nodes for low battery and error detection, ensuring the Turtlebot operates effectively within the restaurant environment.
Contributions: Contributions and enhancements are welcomed to refine functionalities and improve overall system performance.
