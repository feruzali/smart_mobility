# Robot Control Console | ROS2

## Feruz 12204578

# Installation

On Ubuntu:

$ apt-get install ros-humble-rosbridge-server

# Usage

## Simply run web server

To test if `rosbridge` can run on your machine and provide http server feature, run a web server from this package by simply running a launch file:

$ ros2 run rosbridge rosbridge

Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

NODES
/
rosbridge (rosbridge/webserver.py)

auto-starting new master
process[master]: started with pid [26851]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 8482c492-96b5-11e5-bc2d-f816542d218e
process[rosout-1]: started with pid [26864]
started core service [/rosout]
process[rosbridge-2]: started with pid [26867]
2015-11-29 08:23:27,191 - rosbridge - INFO - rosbridge : # of packages : 689
2015-11-29 08:23:27,191 - rosbridge - INFO - rosbridge : Weg Page root : www
2015-11-29 08:23:27,285 - rosbridge - INFO - rosbridge : Initialised
2015-11-29 08:23:27,285 - rosbridge - INFO - rosbridge : Attempting to start webserver on port 8085
2015-11-29 08:23:27,289 - rosbridge - INFO - rosbridge : Webserver successfully started on port 8085
WARNING:tornado.access:404 GET /favicon.ico (127.0.0.1) 1.60ms
WARNING:tornado.access:404 GET /favicon.ico (127.0.0.1) 1.33ms

You can see a web page being published at http://localhost:%PORT_OF_YOURCHOICE%/ like below, which should show the list of ROS packages in your `ROS_PACKAGE_PATH`:

::

ROS web server successfully started.

Package List

abb_driver
abb_irb2400_moveit_plugins
abb_irb2400_support

## Integrate into the ROS package

You can integrate web server capability from `rosbridge` by either launch file or python module.

Integrate by launch
++++++++++++++++++++++++++++++++++++

1. In your own launch, include `rosbridge.launch` file. Customize arguments if necessary.

::

  <arg name="name" default="www server for ros"/>
  <arg name="port" default="8085"/> <!-- avoid to use apache default port -->
  <arg name="webpath" default="www"/> <!-- relative path to the webroot. E.g. place this foloder in the ROS package root dir -->
  <arg name="use_rosbridge" default="true" />
  <include if="$(arg use_rosbridge)" file="$(find rosbridge)/launch/rosbridge.launch">
    <arg name="name" value="$(arg name)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="webpath" value="$(arg webpath)"/>
  </include>

2. Add `<run_depend>rosbridge</run_depend>` in your `package.xml`, to avoid "404 package not found" kind of error when you run.

## Static page

When you launch the rosbridge, you can access static pages(html) which are installed in share/%PACKAGE_NAME%/www folder through http://localhost:%PORT_OF_YOURCHOICE%/%PACKAGE_NAME%/%STATIC_PAGE%.html.

## Simple talker and listener

To play with the rostopic, you can launch a simple talker and listener:

$ ros2 run rosbridge start_bridge.launch

You can send a message through ROS topic from http://localhost:%PORT_OF_YOURCHOICE%/rosbridge/talker.html. And also, you can subscribe the message on http://localhost:%PORT_OF_YOURCHOICE%/rosbridge/listener.html.
