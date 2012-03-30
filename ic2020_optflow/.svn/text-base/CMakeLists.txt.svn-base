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

find_package(OpenCV REQUIRED)

set(VODOM_PATH "~/ic2020/ic2020_vodom")

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}
${VODOM_PATH}/src
${OpenCV_INCLUDE_DIR}
${COMMONDIR}/inc
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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

rosbuild_add_executable(ic2020_optflow src/ic2020_optflow.cpp)
TARGET_LINK_LIBRARIES(ic2020_optflow ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(ic2020_optflow ${VODOM_PATH}/lib/libKeyframe.so)
TARGET_LINK_LIBRARIES(ic2020_optflow ${VODOM_PATH}/lib/libVisualOdometry.so)
TARGET_LINK_LIBRARIES(ic2020_optflow ${VODOM_PATH}/lib/libCornerHelper.so)


