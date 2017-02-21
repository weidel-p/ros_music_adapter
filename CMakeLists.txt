cmake_minimum_required(VERSION 2.8)

project("ros_music_adapter")

# add modules for finding JSONCPP and MUSIC
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# generates catkin_INCLUDE_DIRS and catkin_LIBRARIES for all components
find_package(catkin REQUIRED
    COMPONENTS
    dvs_msgs
    roscpp
    roscpp_serialization
    rosconsole
    rostime
)

find_package(MUSIC REQUIRED)
find_package(JSONCPP REQUIRED)
find_package(MPI REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${MUSIC_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${MPI_INCLUDE_PATH}
)

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
        JSONCPP
)

find_library(GSL gsl REQUIRED)
find_library(BLAS gslcblas REQUIRED)
find_library(PTHREAD pthread REQUIRED)

add_library(rtclock STATIC lib/rtclock.cpp)
include_directories(lib/)

add_executable(ros_command_adapter adapters/ros_command_adapter.cpp)
add_executable(ros_sensor_adapter adapters/ros_sensor_adapter.cpp)
add_executable(ros_event_sensor_adapter adapters/ros_event_sensor_adapter.cpp)
add_executable(linear_readout_decoder decoder/linear_readout.cpp)
add_executable(nef_encoder encoder/nef_encoder.cpp)
add_executable(connect_adapter adapters/connect.cpp)
add_executable(rate_encoder encoder/rate_encoder.cpp)
add_executable(poisson_encoder encoder/poisson_encoder.cpp)

target_link_libraries(ros_command_adapter ${catkin_LIBRARIES} ${MUSIC_LIBRARIES} ${PTHREAD} ${MPI_LIBRARIES} ${JSONCPP_LIBRARY} rtclock)
target_link_libraries(ros_sensor_adapter ${catkin_LIBRARIES} ${MUSIC_LIBRARIES} ${PTHREAD} ${MPI_LIBRARIES} rtclock)
target_link_libraries(ros_event_sensor_adapter ${catkin_LIBRARIES} ${MUSIC_LIBRARIES} ${PTHREAD} ${MPI_LIBRARIES} rtclock)
target_link_libraries(linear_readout_decoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES} ${JSONCPP_LIBRARY} ${GSL} ${BLAS})
target_link_libraries(nef_encoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES})
target_link_libraries(rate_encoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES}) 
target_link_libraries(poisson_encoder ${MUSIC_LIBRARIES} ${MPI_LIBRARIES})
target_link_libraries(connect_adapter ${MUSIC_LIBRARIES} ${MPI_LIBRARIES} ${JSONCPP_LIBRARY} ${GSL} ${BLAS})

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
