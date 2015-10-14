cmake_minimum_required(VERSION 2.8.3)
project(mitsubishi_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED roscpp actionlib)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(mitsubishi_action_server
    src/mitsubishi_action_server.cpp
    src/MitsubishiCommander.cpp
    src/MitsubishiActionServer.cpp
)
target_link_libraries(mitsubishi_action_server
    ${catkin_LIBRARIES}
)

add_executable(mitsubishi_joint_state
    src/mitsubishi_joint_state.cpp
    src/MitsubishiJointState.cpp
)
target_link_libraries(mitsubishi_joint_state
    ${catkin_LIBRARIES}
)
