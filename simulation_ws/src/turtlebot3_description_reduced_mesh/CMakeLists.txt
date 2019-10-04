cmake_minimum_required(VERSION 3.5)
project(turtlebot3_description_reduced_mesh)

find_package(rclcpp REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(turtlebot3 REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
