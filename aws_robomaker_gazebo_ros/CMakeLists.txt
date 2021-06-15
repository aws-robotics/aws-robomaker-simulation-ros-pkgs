cmake_minimum_required(VERSION 3.7.2)
project(aws_robomaker_gazebo_ros)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/worldforge.overlay.sh")

# Install Gazebo launch files
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
