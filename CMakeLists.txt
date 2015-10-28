cmake_minimum_required(VERSION 2.8)

project("MUSIC-ROS Adapter")


add_executable(ros_command_adapter adapters/ros_command_adapter.cpp)
add_executable(ros_sensor_adapter adapters/ros_sensor_adapter.cpp)
add_executable(linear_readout_decoder decoder/linear_readout.cpp)
add_executable(nef_encoder encoder/nef_encoder.cpp)
add_executable(connect_adapter adapters/connect.cpp)
add_executable(rate_encoder encoder/rate_encoder.cpp)

find_library(MUSIC music)

# Require MPI for this project:
find_package(MPI REQUIRED)
include_directories(${MPI_INCLUDE_PATH})
set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})


find_library(ROS_CPP roscpp /opt/ros/jade/lib)
find_library(ROS_CPP_SER roscpp_serialization /opt/ros/jade/lib)
find_library(ROSCONSOLE rosconsole /opt/ros/jade/lib)
find_library(ROSTIME rostime /opt/ros/jade/lib)

include_directories(/opt/ros/jade/include/)

find_package(Boost COMPONENTS thread system date_time REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_library(JSON jsoncpp)

target_link_libraries(ros_command_adapter ${MUSIC} ${ROS_CPP} ${ROS_CPP_SER} ${ROSCONSOLE} ${ROSTIME} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ros_sensor_adapter ${MUSIC} ${ROS_CPP} ${ROS_CPP_SER} ${ROSCONSOLE} ${ROSTIME} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(linear_readout_decoder ${MUSIC} ${MPI_LIBRARIES} ${Boost_LIBRARIES} ${JSON})
target_link_libraries(nef_encoder ${MUSIC} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(rate_encoder ${MUSIC} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(connect_adapter ${MUSIC} ${MPI_LIBRARIES} ${Boost_LIBRARIES} ${JSON})


