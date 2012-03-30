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

#INCLUDE(${CMAKE_SOURCE_DIR}/CMake/cuda/FindCUDA.cmake)
find_package(OpenCV REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(CornerHelper src/CornerHelper.cpp)
target_link_libraries(CornerHelper ${OpenCV_LIBS})

rosbuild_add_library(Keyframe src/Keyframe.cpp)
target_link_libraries(Keyframe ${OpenCV_LIBS})

rosbuild_add_library(SURFHelper src/SURFHelper.cpp)
target_link_libraries(SURFHelper ${OpenCV_LIBS})

rosbuild_add_library(Vector3 src/Vector3.cpp)

rosbuild_add_library(VisualOdometry src/VisualOdometry.cpp)
target_link_libraries(VisualOdometry Vector3)

rosbuild_add_executable(ic2020_vodom src/ic2020_vodom.cpp)
target_link_libraries(ic2020_vodom SURFHelper)
target_link_libraries(ic2020_vodom VisualOdometry)
target_link_libraries(ic2020_vodom Keyframe)

TARGET_LINK_LIBRARIES(ic2020_vodom ${OpenCV_LIBS})

