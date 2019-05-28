cmake_minimum_required(VERSION 2.8.3)
project(motor_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_compile_options(
    -std=c++11
)

add_executable(motor_test
    src/motor_test.cpp
    src/gpio.cpp
    src/motor.cpp
)
target_link_libraries(motor_test ${catkin_LIBRARIES})
