cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(coverage RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-std=c++0x")
rosbuild_init()


#set paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CORE_LIB_PATH ${PROJECT_SOURCE_DIR}/src/core)
set(NETUS_LIB_PATH ${PROJECT_SOURCE_DIR}/src/netus2pic)
set(AUSIL_LIB_PATH ${PROJECT_SOURCE_DIR}/src/ausiliarie)
set(SENSOR_LIB_PATH ${PROJECT_SOURCE_DIR}/src/sensor)
set(INCLUDE_PATHS ${NETUS_LIB_PATH} ${CORE_LIB_PATH} ${AUSIL_LIB_PATH} ${SENSOR_LIB_PATH})

link_directories(${LIBRARY_OUTPUT_PATH})

# Set the directories where include files can be found.
include_directories (include)
include_directories (${INCLUDE_PATHS})
include_directories (${saetta_msgs_PACKAGE_PATH}/msg_gen/cpp/include)

<<<<<<< HEAD
#rosbuild_find_ros_package (tf)
=======
rosbuild_find_ros_package (tf)
>>>>>>> fdf1fb0d09c03c40bc715d7bf1694521c49c606f

Find_Package(SDL REQUIRED)
if(NOT SDL_FOUND)
    message( FATAL ERROR "SDL not found!")
endif(NOT SDL_FOUND)

#common commands for building c++ executables and libraries
rosbuild_add_library(base src/mainbase.cpp)
rosbuild_add_library(core src/Saetta_Base_core.cpp)
rosbuild_add_library(robotcore src/core/robot_core.c)
rosbuild_add_library(serial_comm src/netus2pic/serial_comm.c)
rosbuild_add_library(robot_comm src/netus2pic/robot_comm.c)
rosbuild_add_library(ausiliarie src/ausiliarie/funzioni_ausiliarie.c)
rosbuild_add_library(robot_sensors src/sensor/robot_sensors.c)
rosbuild_add_library(pic2netus src/core/pic2netus.c)
rosbuild_add_boost_directories()
rosbuild_add_executable(base src/mainbase.cpp)
rosbuild_link_boost(base thread)
<<<<<<< HEAD
#target_link_libraries(base core pic2netus serial_comm robot_comm ausiliarie robot_sensors rt tf)
target_link_libraries(base core pic2netus serial_comm robot_comm ausiliarie robot_sensors rt)
=======
target_link_libraries(base core pic2netus serial_comm robot_comm ausiliarie robot_sensors rt tf)
>>>>>>> fdf1fb0d09c03c40bc715d7bf1694521c49c606f
target_link_libraries(robot_sensors ausiliarie robotcore)
target_link_libraries(robotcore ausiliarie m)
target_link_libraries(robot_comm serial_comm)
target_link_libraries(pic2netus robot_sensors ausiliarie m robotcore)
