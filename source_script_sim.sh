	export PX4_HOME_LAT=0
	export PX4_HOME_LON=0
	export PX4_HOME_ALT=0
	
	make posix_sitl_default
	source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
    	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
    	roslaunch mav_gazebo simulation.launch
