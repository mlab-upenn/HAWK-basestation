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

set(GLUT_INC_PATH "/usr/include")
set(GLUT_LIB_PATH "/usr/lib")
set(GLEW_INC_PATH "/usr/include")
set(GLEW_LIB_PATH "/usr/lib")
set(CG_INC_PATH "/usr/include")
set(CG_LIB_PATH "/usr/lib")
set(IMAGEMAGICK_INC_PATH "/usr/include/ImageMagick")
set(IMAGEMAGICK_LIB_PATH "/usr/lib")

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}
${VODOM_PATH}/src
${GLUT_INC_PATH}
${GLEW_INC_PATH}
${CG_INC_PATH}
${IMAGEMAGICK_INC_PATH}
)

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

# Build Basic Libraries
rosbuild_add_library(Vector3 src/Vector3.cpp)
rosbuild_add_library(KeyframeUpdater src/KeyframeUpdater.cpp)

# Build GL Dependant Libraries
rosbuild_add_library(GLCamera src/GLCamera.cpp)
target_link_libraries(GLCamera ${GLUT_LIB_PATH}/libGLU.so)

# Build Executable
rosbuild_add_executable(renderer src/renderer.cpp)

# Link Dependencies
target_link_libraries(renderer KeyframeUpdater)
target_link_libraries(renderer Vector3)
target_link_libraries(renderer GLCamera)
TARGET_LINK_LIBRARIES(renderer ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(renderer ${VODOM_PATH}/lib/libKeyframe.so)

# For OpenGL
TARGET_LINK_LIBRARIES(renderer ${GLUT_LIB_PATH}/libglut.a)
TARGET_LINK_LIBRARIES(renderer ${GLEW_LIB_PATH}/libGLEW.a)
TARGET_LINK_LIBRARIES(renderer ${CG_LIB_PATH}/libCg.so)
TARGET_LINK_LIBRARIES(renderer ${CG_LIB_PATH}/libCgGL.so)
TARGET_LINK_LIBRARIES(renderer ${IMAGEMAGICK_LIB_PATH}/libGL.so)
TARGET_LINK_LIBRARIES(renderer ${IMAGEMAGICK_LIB_PATH}/libpthread.so)
TARGET_LINK_LIBRARIES(renderer ${IMAGEMAGICK_LIB_PATH}/libMagick++.so)



