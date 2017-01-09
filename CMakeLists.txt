cmake_minimum_required(VERSION 2.8)

project("MUSIC-ROS_Adapter")

add_library(rtclock STATIC lib/rtclock.cpp)

add_executable(ros_command_adapter adapters/ros_command_adapter.cpp)
add_executable(ros_sensor_adapter adapters/ros_sensor_adapter.cpp)
add_executable(linear_readout_decoder decoder/linear_readout.cpp)
add_executable(nef_encoder encoder/nef_encoder.cpp)
add_executable(connect_adapter adapters/connect.cpp)
add_executable(rate_encoder encoder/rate_encoder.cpp)
add_executable(poisson_encoder encoder/poisson_encoder.cpp)


find_library(MUSIC music REQUIRED)
include_directories(lib/)

# Require MPI for this project:
find_package(MPI REQUIRED)
include_directories(${MPI_INCLUDE_PATH})
set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})
set(CMAKE_CXX_FLAGS "-O3")        ## Optimize

find_library(ROS_CPP roscpp /opt/ros/kinetic/lib)
find_library(ROS_CPP_SER roscpp_serialization /opt/ros/kinetic/lib)
find_library(ROSCONSOLE rosconsole /opt/ros/kinetic/lib)
find_library(ROSTIME rostime /opt/ros/kinetic/lib)

include_directories(/opt/ros/kinetic/include/)

find_library(GSL gsl REQUIRED)
find_library(BLAS gslcblas REQUIRED)

find_library(JSON jsoncpp)

find_library(PTHREAD pthread)

target_link_libraries(ros_command_adapter ${MUSIC} ${ROS_CPP} ${ROS_CPP_SER} ${ROSCONSOLE} ${ROSTIME} ${PTHREAD} ${MPI_LIBRARIES} ${JSON} rtclock)
target_link_libraries(ros_sensor_adapter ${MUSIC} ${ROS_CPP} ${ROS_CPP_SER} ${ROSCONSOLE} ${ROSTIME} ${PTHREAD} ${MPI_LIBRARIES} rtclock)
target_link_libraries(linear_readout_decoder ${MUSIC} ${MPI_LIBRARIES} ${JSON} ${GSL} ${BLAS})
target_link_libraries(nef_encoder ${MUSIC} ${MPI_LIBRARIES})
target_link_libraries(rate_encoder ${MUSIC} ${MPI_LIBRARIES}) 
target_link_libraries(poisson_encoder ${MUSIC} ${MPI_LIBRARIES})
target_link_libraries(connect_adapter ${MUSIC} ${MPI_LIBRARIES} ${JSON} ${GSL} ${BLAS})


# COPY PYTHON FILES TO ROOT DIRECTORY
file(COPY "adapters/pca.py" DESTINATION "./")

install(TARGETS
    ros_command_adapter
    ros_sensor_adapter
    linear_readout_decoder
    nef_encoder
    rate_encoder
    poisson_encoder
    connect_adapter
    RUNTIME DESTINATION bin
)
