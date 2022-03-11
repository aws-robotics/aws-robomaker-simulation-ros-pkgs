cmake_minimum_required(VERSION 3.7.2)
project(aws_robomaker_cloudwatch_publisher)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

# Install CloudWatch publisher module
catkin_install_python(
    PROGRAMS
        nodes/rtf_cloudwatch_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_add_nosetests(test)
