cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
find_package( OpenCV REQUIRED )


rosbuild_add_executable(tyros_camera src/tyros_cam.cpp src/libcam.cpp src/find_circles.cpp)
target_link_libraries(${PROJECT_NAME} cvblob)
rosbuild_add_executable(red_object_tracking src/red_object_tracking.cpp)
target_link_libraries(red_object_tracking cvblob)
target_link_libraries(red_object_tracking ${OpenCV_LIBS} )
target_link_libraries(tyros_camera ${OpenCV_LIBS} )

