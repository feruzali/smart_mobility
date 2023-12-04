#in the same terminal we need to write this code and install required packages
$ sudo dpkg --add-architecture armhf 
$ sudo apt-get update 
$ sudo apt-get install libc6:armhf 

#First line tells the system to add ARM architecture (specific rules that dictates to the hardware work when the particular instruction executes). 
#Second line upgrades packages and versions which stored in the local repositories. 
#Third line installs specifically C libraries of ARM architecture

$ export OPENCR_PORT=/dev/ttyACM0 
#Forth line is specifies the port

$ export OPENCR_MODEL=burger
#to specify the model of turtlebot and our bot is butger typed

$ rm -rf ./opencr_update.tar.bz2 
#remove the file in current folder

#to download the file from github repo instead of deleted one and extract it
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2  
$ tar -xvf opencr_update.tar.bz2 

$ cd ./opencr_update 
$ ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr 
#entering to the downloaded folder and open with followed command 
