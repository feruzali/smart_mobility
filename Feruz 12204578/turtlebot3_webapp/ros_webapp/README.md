# Robot Control Console | ROS2

## Feruz 12204578

## Application Introduction

![Robot Control Console Dashboard](resources/robot-control-console-dashboard.png?raw=true)

Welcome to the Robot Control Console, a powerful web application built with React.js and Bootstrap. This console serves as a centralized command hub for controlling robots remotely. The intuitive user interface provides real-time feedback and control over various aspects of the robot's behavior.

### Features

- **Map:** Displays the real-time position of the robot on a map.
- **Velocity:** Control Angular and Linear velocity for precise robot movement.
- **Joy Stick:** Intuitive control mechanism for robot navigation.
- **Navigation Menu:** Save and navigate to predefined locations with ease.

### Communication with ROSbridge using ROSlib.js

This application communicates seamlessly with the Robot Operating System (ROS) through ROSbridge. ROSbridge facilitates communication between the web application and the robot using websockets. The integration of ROSlib.js allows for efficient and real-time data exchange, enabling precise control and monitoring of the robot's state.

### Libraries Used

- **React.js:** A powerful JavaScript library for building user interfaces, providing a dynamic and responsive experience.
- **Bootstrap:** A front-end framework for developing responsive and mobile-first applications, enhancing the application's visual appeal.
- **ROSlib.js:** A JavaScript library for interacting with ROS, enabling communication between the web application and the robot.
- **ROS2D:** A library for visualizing 2D maps and robot positions within the Robot Operating System (ROS), enhancing the map visualization capabilities.
- **NAV2D:** Further expanding navigation capabilities, NAV2D provides tools for visualizing robot paths and efficiently navigating through environments.

## Getting Started

### Prerequisites

Before getting started, ensure you have Node.js and npm installed on your machine. This project is tested on Node.js and ROS2 Humble.

### Installation

1. Navigate to the project directory:

   ```bash
   cd ros_webapp
   ```

2. Install npm packages:

   ```bash
   npm install
   ```

### Start application

```bash
npm start
```

## Application configuration

- Adjust settings in src/data/config.js:

  ```JavaScript

  const Config = {
    ROSBRIDGE_SERVER_IP: '192.168.0.106', // turtlebot IP address
    ROSBRIDGE_SERVER_PORT: '9090', // default port
    CMD_VEL_TOPIC: '/cmd_vel',
    GOAL_TOPIC: '/move_base_simple/goal',
  }

  ```
