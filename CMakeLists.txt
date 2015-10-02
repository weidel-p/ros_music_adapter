cmake_minimum_required(VERSION 2.8)

project("MUSIC-ROS Adapter")


add_executable(ros_command_adapter ros_command_adapter.cpp)
add_executable(ros_sensor_adapter ros_sensor_adapter.cpp)

include_directories(/home/weidel/opt/music/include/)
find_library(MUSIC music /home/weidel/opt/music/lib)

target_link_libraries(ros_command_adapter ${MUSIC})
target_link_libraries(ros_sensor_adapter ${MUSIC})

# Require MPI for this project:
find_package(MPI REQUIRED)
include_directories(${MPI_INCLUDE_PATH})
set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})

target_link_libraries(ros_command_adapter ${MPI_LIBRARIES})
target_link_libraries(ros_sensor_adapter ${MPI_LIBRARIES})

find_library(ROS_CPP roscpp /opt/ros/jade/lib)
find_library(ROS_CPP_SER roscpp_serialization /opt/ros/jade/lib)
find_library(ROSCONSOLE rosconsole /opt/ros/jade/lib)
find_library(ROSTIME rostime /opt/ros/jade/lib)

include_directories(/opt/ros/jade/include/)


target_link_libraries(ros_command_adapter ${ROS_CPP})
target_link_libraries(ros_command_adapter ${ROS_CPP_SER})
target_link_libraries(ros_command_adapter ${ROSCONSOLE})
target_link_libraries(ros_command_adapter ${ROSTIME})

target_link_libraries(ros_sensor_adapter ${ROS_CPP})
target_link_libraries(ros_sensor_adapter ${ROS_CPP_SER})
target_link_libraries(ros_sensor_adapter ${ROSCONSOLE})
target_link_libraries(ros_sensor_adapter ${ROSTIME})


find_package(Boost COMPONENTS thread system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})


target_link_libraries(ros_command_adapter ${Boost_LIBRARIES})
target_link_libraries(ros_sensor_adapter ${Boost_LIBRARIES})


