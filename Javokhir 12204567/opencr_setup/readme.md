# OpenCR Setup

Connect the OpenCR to the Rasbperry Pi using the micro USB cable.

# Install required packages on the Raspberry Pi to upload the OpenCR firmware.
```
$ sudo dpkg --add-architecture armhf
$ sudo apt update
$ sudo apt install libc6:armhf
```
# Depending on the platform, use either burger or waffle for the OPENCR_MODEL name.
```
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2
```
# Download the firmware and loader, then extract the file.
```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
$ tar -xvf ./opencr_update.tar.bz2
```
# Upload firmware to the OpenCR.
```
$ cd ~/opencr_update
$ ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```
A successful firmware upload for TurtleBot3 Burger will look like below.
![image](https://github.com/feruzali/smart_mobility/assets/91411930/942e7172-b220-4114-a983-83702245cafd)

If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the STATUS led of OpenCR will blink periodically.
Hold down the PUSH SW2 button.
```
Press the Reset button.
Release the Reset button.
Release the PUSH SW2 button
```

![image](https://github.com/feruzali/smart_mobility/assets/91411930/67273857-d29a-49b7-bf1f-364f8aba6018)

# OpenCR Test
NOTE: If the wheels do not move while performing OpenCR Test instruction, make sure to see “Setup DYNAMIXELs for TurtleBot3” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use PUSH SW 1 and PUSH SW 2 buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the OpenCR board.

![image](https://github.com/feruzali/smart_mobility/assets/91411930/bb4ad3ed-eee6-4f15-8119-e1e0148b91e1)

After assembling TurtleBot3, connect the power to OpenCR and turn on the power switch of OpenCR. The red Power LED will be turned on.
```
Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
Press and hold PUSH SW 1 for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
Press and hold PUSH SW 2 for a few seconds to command the robot to rotate 180 degrees in place.
```


