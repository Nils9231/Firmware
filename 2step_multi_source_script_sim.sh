make posix_sitl_default 
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/hippoc/src/sitl_gazebo

#export PX4_HOME_LAT=28.452386
#export PX4_HOME_LON=-13.867138
#export PX4_HOME_ALT=28.5

roslaunch px4 second_hippocampus.launch

