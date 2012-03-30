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

set(CUDA_INSTALL_PATH "/usr/local/cuda")
#set(CUDA_SDK_INSTALL_PATH "~/NVIDIA_GPU_Computing_SDK/C")
set(CUDA_SDK_INSTALL_PATH "/home/sanderson/NVIDIA_GPU_Computing_SDK/C")
set(LIBDIR_CUDA "${CUDA_SDK_INSTALL_PATH}/lib")
set(COMMONDIR "${CUDA_SDK_INSTALL_PATH}/common")

set(SURF_INSTALL_PATH "/home/sanderson/ros/libsurf")
set(NCGL_INSTALL_PATH "/home/sanderson/ros/libncgl")
set(GLUT_INC_PATH "/usr/include")
set(GLUT_LIB_PATH "/usr/lib")
set(GLEW_INC_PATH "/usr/include")
set(GLEW_LIB_PATH "/usr/lib")
set(CG_INC_PATH "/usr/include")
set(CG_LIB_PATH "/usr/lib")
set(IMAGEMAGICK_INC_PATH "/usr/include/ImageMagick")
set(IMAGEMAGICK_LIB_PATH "/usr/lib")


#INCLUDE(${CMAKE_SOURCE_DIR}/CMake/cuda/FindCUDA.cmake)
find_package(OpenCV REQUIRED)

set(VODOM_PATH "~/ic2020/ic2020_vodom")

INCLUDE_DIRECTORIES(
${CMAKE_SOURCE_DIR}
${VODOM_PATH}/src
${SURF_INSTALL_PATH}/inc
${NCGL_INSTALL_PATH}/inc
${GLUT_INC_PATH}
${GLEW_INC_PATH}
${CG_INC_PATH}
${IMAGEMAGICK_INC_PATH}
${LIB_SURF_INC}
${OpenCV_INCLUDE_DIR}
${CUDA_INSTALL_PATH}/include
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

rosbuild_add_executable(ic2020_surf src/ic2020_surf.cpp)
TARGET_LINK_LIBRARIES(ic2020_surf ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(ic2020_surf ${VODOM_PATH}/lib/libKeyframe.so)

# For GPU SURF
TARGET_LINK_LIBRARIES(ic2020_surf ${SURF_INSTALL_PATH}/lib/Release/libsurf.a)
TARGET_LINK_LIBRARIES(ic2020_surf ${CUDA_INSTALL_PATH}/lib64/libcudart.so)
TARGET_LINK_LIBRARIES(ic2020_surf ${LIBDIR_CUDA}/libcutil_x86_64.a)
TARGET_LINK_LIBRARIES(ic2020_surf ${CUDA_INSTALL_PATH}/lib64/libcublas.so)
TARGET_LINK_LIBRARIES(ic2020_surf ${NCGL_INSTALL_PATH}/lib/Release/libncgl.a)

# For OpenGL
TARGET_LINK_LIBRARIES(ic2020_surf ${GLUT_LIB_PATH}/libglut.a)
TARGET_LINK_LIBRARIES(ic2020_surf ${GLEW_LIB_PATH}/libGLEW.a)
TARGET_LINK_LIBRARIES(ic2020_surf ${CG_LIB_PATH}/libCg.so)
TARGET_LINK_LIBRARIES(ic2020_surf ${CG_LIB_PATH}/libCgGL.so)
TARGET_LINK_LIBRARIES(ic2020_surf ${IMAGEMAGICK_LIB_PATH}/libGL.so)
TARGET_LINK_LIBRARIES(ic2020_surf ${IMAGEMAGICK_LIB_PATH}/libpthread.so)
TARGET_LINK_LIBRARIES(ic2020_surf ${IMAGEMAGICK_LIB_PATH}/libMagick++.so)

