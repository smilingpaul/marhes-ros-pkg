cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(${PROJECT_SOURCE_DIR}/lib32 ${PROJECT_SOURCE_DIR}/lib64)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(vicon_vrpn src/vicon_vrpn_node.cpp) #src/vrpn_subject.cpp)
#target_link_libraries(vicon_vrpn vrpn)

rosbuild_add_executable(vicon_vrpn src/vicon_vrpn_node.cpp)
target_link_libraries(vicon_vrpn vrpn)

rosbuild_add_executable(vicon_tf_to_odom src/vicon_tf_to_odom.cpp)
rosbuild_add_executable(vicon_diff_odom src/vicon_diff_odom.cpp)
rosbuild_add_executable(vicon_pose_to_odom src/vicon_pose_to_odom.cpp)

rosbuild_add_executable(vicon_to_pose src/vicon_to_pose.cpp)
target_link_libraries(vicon_to_pose vrpn)
