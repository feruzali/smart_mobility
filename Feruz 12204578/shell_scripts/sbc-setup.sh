Shell Script  
#Open the network configuration file with the command below 
 sudo nano /writable/etc/netplan/50-cloud-init.yaml  


#write the command to edit automatic setiing file 
 sudo nano /etc/apt/apt.conf.d/20auto-upgrades  

#To set mask for the "systemd" process, execute the following command.
 systemctl mask systemd-networkd-wait-online.service 

#Turn off hibernation and suspension
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target 

#Reboot the Raspberry Pi
reboot

#Use the following command from the remote PC terminal if you want to work from the remote PC over SSH. Use the password that you established in Step 1.
ssh ubuntu@{IP Address of Raspberry PI}    

#install python3 dependencies and build-essential for additional system support 
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential 

#install ROS packages
                                                                                                                                                                                                                                                                                                                                                                   
 sudo apt install ros-humble-hls-lfcd-lds-driver 
 sudo apt install ros-humble-turtlebot3-msgs 
 sudo apt install ros-humble-dynamixel-sdk 
 sudo apt install libudev-dev 

#create a workspace directory for TurtleBot3
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src 

#clone TurtleBot3 repository and 	LD-08 repository
 git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git 
 git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git 

#Navigate to the TurtleBot3 source folder 
 cd ~/turtlebot3_ws/src/turtlebot3 

#Remove unnecessary packages from TurtleBot3 source
rm -r turtlebot3_cartographer turtlebot3_navigation2 

#Navigate back to the TurtleBot3 workspace
cd ~/turtlebot3_ws/ 

#Add ROS setup.bash to the bashrc file 
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc 
source ~/.bashrc 

#Build the TurtleBot3 workspace using symlink installation
colcon build --symlink-install --parallel-workers 1 

#Add TurtleBot3 workspace setup.bash
 echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc 
source ~/.bashrc                                                                                                                      

#USB Port settings for Open-source control Module for ROS
 sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/ 
 sudo udevadm control --reload-rules 
 sudo udevadm trigger                                                                                                           

#Commands below show how to assign a ROS_DOMAIN_ID to SBS.
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc 
 source ~/.bashrc 
 
#Depending on your model,use LDS-01 or LDS-02.                                                                                                 
 echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc  
                                                                          
#Implement modifications with the comment below
 source ~/.bashrc
