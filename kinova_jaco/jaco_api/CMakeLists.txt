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

add_subdirectory(src/JacoAPIWrapper)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

#Older mono package
#pkg_check_modules(MONO REQUIRED mono)

#For newer Ubuntu versions
pkg_check_modules(MONO2 REQUIRED mono-2)
pkg_check_modules(GLIB REQUIRED glib-2.0)

include_directories(${MONO_INCLUDE_DIRS} ${MONO2_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS})



rosbuild_add_library(jaco_api src/jaco_api.cpp)
set_target_properties(jaco_api PROPERTIES DEFINE_SYMBOL
    API_PATH=${LIBRARY_OUTPUT_PATH}/)
target_link_libraries(jaco_api ${MONO_LIBRARIES} ${MONO2_LIBRARIES})
add_dependencies(jaco_api JacoAPIWrapper)

