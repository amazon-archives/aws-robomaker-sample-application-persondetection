cmake_minimum_required(VERSION 2.8.3)
project(aws_robomaker_simulation_common)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  turtlebot3_description
  turtlebot3_navigation  # required for copy of .rviz file
)

catkin_package()
catkin_python_setup()

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
  nodes/route_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Copy the rviz models for easier access in AWS RoboMaker RViz 
install(FILES ${turtlebot3_navigation_DIR}/../rviz/turtlebot3_navigation.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  RENAME turtlebot3_navigation.rviz
)
