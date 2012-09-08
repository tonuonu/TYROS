cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(BISON)
find_package(FLEX)

FLEX_TARGET(scanner src/yylex.l  ${CMAKE_CURRENT_BINARY_DIR}/yylex.cc)
BISON_TARGET(parser src/parser.y ${CMAKE_CURRENT_BINARY_DIR}/parser.cc)
ADD_FLEX_BISON_DEPENDENCY(scanner parser)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(publisher src/publisher.cpp ${FLEX_scanner_OUTPUTS} ${BISON_parser_OUTPUTS} )
target_link_libraries(publisher ${FLEX_LIBRARIES} ${BISON_LIBRARIES})

