cmake_minimum_required(VERSION 2.8)

project("ros_music_adapter")

# add modules for finding JsonCpp and MUSIC
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

find_package(catkin REQUIRED COMPONENTS dvs_msgs roscpp roscpp_serialization rosconsole rostime)

find_library(GSL gsl REQUIRED)
find_library(BLAS gslcblas REQUIRED)
find_library(PTHREAD pthread)

add_library(rtclock STATIC lib/rtclock.cpp)
include_directories(lib/)

find_package(MUSIC REQUIRED)
find_package(JsonCpp REQUIRED)

find_package(MPI REQUIRED)
include_directories(${MPI_INCLUDE_PATH})
set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})
set(CMAKE_CXX_FLAGS "-O3")

catkin_package(
    CATKIN_DEPENDS
        dvs_msgs
        roscpp
        roscpp_serialization
        rosconsole
        rostime
    DEPENDS
        MUSIC
        MPI
)

include_directories(${catkin_INCLUDE_DIRS})
# TODO depend on "JsonCpp", but module exports "DIR" instead of "DIRS"
include_directories(${JsonCpp_INCLUDE_DIR})

add_executable(ros_command_adapter adapters/ros_command_adapter.cpp)
add_executable(ros_sensor_adapter adapters/ros_sensor_adapter.cpp)
add_executable(ros_event_sensor_adapter adapters/ros_event_sensor_adapter.cpp)
add_executable(linear_readout_decoder decoder/linear_readout.cpp)
add_executable(nef_encoder encoder/nef_encoder.cpp)
add_executable(connect_adapter adapters/connect.cpp)
add_executable(rate_encoder encoder/rate_encoder.cpp)
add_executable(poisson_encoder encoder/poisson_encoder.cpp)

target_link_libraries(ros_command_adapter ${MUSIC_LIBRARIES} ${ROS_CPP} ${ROS_CPP_SER} ${ROSCONSOLE} ${ROSTIME} ${PTHREAD} ${MPI_LIBRARIES} ${JsonCpp_LIBRARY} rtclock)
target_link_libraries(ros_sensor_adapter ${MUSIC_LIBRARIES} ${ROS_CPP} ${ROS_CPP_SER} ${ROSCONSOLE} ${ROSTIME} ${PTHREAD} ${MPI_LIBRARIES} rtclock)
target_link_libraries(ros_event_sensor_adapter ${MUSIC_LIBRARIES} ${ROS_CPP} ${ROS_CPP_SER} ${ROSCONSOLE} ${ROSTIME} ${PTHREAD} ${MPI_LIBRARIES} rtclock)
target_link_libraries(linear_readout_decoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES} ${JsonCpp_LIBRARY} ${GSL} ${BLAS})
target_link_libraries(nef_encoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES})
target_link_libraries(rate_encoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES}) 
target_link_libraries(poisson_encoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES})
target_link_libraries(connect_adapter ${MUSIC_LIBRARIES} ${MPI_LIBRARIES} ${JsonCpp_LIBRARY} ${GSL} ${BLAS})

# COPY PYTHON FILES TO ROOT DIRECTORY
# TODO what for?
# file(COPY "adapters/pca.py" DESTINATION "./")

install(TARGETS
    ros_command_adapter
    ros_sensor_adapter
    ros_event_sensor_adapter
    linear_readout_decoder
    nef_encoder
    rate_encoder
    poisson_encoder
    connect_adapter
    RUNTIME DESTINATION bin
)
