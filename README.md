# Full guide to install the package on the robot

- ## Hardware Requirements

	- D.W. Platform from Next Generation Robotics srl (add website)
	
	- 1 YD Lidar
	
	- 1 Camera connected through CSI (Camera Serial Interface)
	
	- 2 Realsense D435i (optional)

- ## Configuring PC

	It is needed to:
	
	1. Install Ubuntu 18.04.5 LTS
	
	2. Install CUDA and NVIDIA drivers
	
	3. Install ROS
	
	4. Build Opencv with CUDA enabled (optional) 
	
	5. Install Realsense SDK (optional)
	
	6. Build RTABMAP (optional)
	
	7. Install Realsense-ROS wrapper (optional)
		
	To do so, you can follow these guides, based on the kind of pc you are using:
	
	- **Nvidia Jetson Xavier NX**
		
		https://github.com/abcamiletto/XavierNX-setup-realsense/wiki/Xavier-NX-Setup-Guide
		
	- **Ubuntu Desktop 18.04.5 LTS with NVIDIA GTX or RTX**
		
		https://github.com/abcamiletto/XavierNX-setup-realsense/wiki/Ubuntu-Desktop-Setup-Guide


- ## Create a brand new catkin workspace 

		$ mkdir -p ~/catkin_ws/src
		$ cd ~/catkin_ws/
		$ catkin_make
		$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
		$ source ~/.bashrc


- ## Install Needed Packages

    1. **gscam**:

			$ cd catkin_ws/src
			$ git clone https://github.com/ros-drivers/gscam

	modify the file ./gscam/Makefile with this string:
	
		EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On

    
    2. **image_common**:
    
			$ cd catkin_ws/src
			$ git clone https://github.com/ros-perception/image_common
    
    
    3. **ydlidar_ros-master**:
	
			$ cd catkin_ws/src
			$ git clone -b master --single-branch https://github.com/YDLIDAR/ydlidar_ros
			$ cd
			$ sudo mkdir /dev/ydlidar
			$ cd catkin_ws/src/ydlidar_ros/startup
			$ sudo chmod 777 ./*
			$ sudo sh initenv.sh
    
	in case of errors ignore them.
	
	
    4. **joy**:
    
			$ sudo apt-get update
    		$ sudo apt-get install ros-melodic-joy
    		$ sudo apt-get install ros-melodic-joystick-drivers



- ## Bash and IP Configuration:

    1. Add the following lines at the end of .bashrc file (hidden file in home directory):

			export ROS_MASTER_URI=http://ipaddress:port
			export ROS_IP=ipaddress
    
	where ipaddress is the wifi board ip address (example: 192.168.1.100) and the port is the one used by ROS for communication (standard port: 11311)
    
    
    2. set the manual IP Address of your wifi board accordingly (edit connections --> IPv4)
    
    
    3. set the manual IP Address of your ethernet board:

		- IP: 		192.168.0.100
    
		- Sub: 		255.255.192.0
    
		- Gateway:	192.168.0.1



- ## Build the workspace:

        $ rosdep install --from-paths ~/catkin_ws --ignore-src --rosdistro=ROSmelodic
        $ catkin_make
        $ source ~/.bashrc
        $ source devel/setup.bash



- ## Usage:

	1. Adjust Robot Parameters in the file: ./udp_controller/src/kinematics_matrix.py

	2. Execute the launch file:

			$ roslaunch state_machine sm_startup.launch

	3. Finally launch starteleop.launch on the remote pc (see DiffWheel_Robot_Teleop package).
