cmake_minimum_required(VERSION 2.8.3)
project(motor_node)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_compile_options(
    -std=c++11
)

add_executable(${PROJECT_NAME}
    src/motor_node.cpp
    src/gpio.cpp
    src/motor.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
