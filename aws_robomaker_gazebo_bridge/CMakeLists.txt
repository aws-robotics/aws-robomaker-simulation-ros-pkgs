cmake_minimum_required(VERSION 3.7.2)
project(aws_robomaker_gazebo_bridge)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_message_files(DIRECTORY msg FILES AwsPerfMetrics.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(
    perf_metrics_bridge
        src/perf_metrics_bridge
)
add_dependencies(perf_metrics_bridge aws_robomaker_gazebo_bridge_generate_messages_cpp)
target_link_libraries(perf_metrics_bridge ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
