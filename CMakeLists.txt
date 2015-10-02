cmake_minimum_required(VERSION 2.8.3)
project(mitsubishi_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED roscpp)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(tcptest
    src/tcptest.cpp
)
target_link_libraries(tcptest
    ${catkin_LIBRARIES}
)

add_executable(mitsubishi_joint_state
    src/mitsubishi_joint_state.cpp
    src/MitsubishiJointState.cpp
)
target_link_libraries(mitsubishi_joint_state
    ${catkin_LIBRARIES}
)
