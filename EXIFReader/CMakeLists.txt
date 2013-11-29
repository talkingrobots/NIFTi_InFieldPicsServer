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

# I use InFieldPicsServer instead of ${PROJECT_NAME}

rosbuild_add_executable(InFieldPicsServer src/App.cpp src/AnnotatedPicsManager.cpp src/CVPicUtil.cpp src/InFieldPicsServer.cpp src/EXIFParser.cpp src/FileReadingUtil.cpp src/PicturesLoader.cpp src/InFieldPicturesLoader.cpp src/RobotPicturesLoader.cpp)

target_link_libraries(InFieldPicsServer nifti_pics_server_util)

rosbuild_link_boost(InFieldPicsServer filesystem system)

rosbuild_add_boost_directories()
