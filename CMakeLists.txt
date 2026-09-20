cmake_minimum_required(VERSION 3.16)
# C is not used by our sources: PCL 1.14 pulls in VTK 9.1, whose VTK::mpi target
# names MPI::MPI_C, and that target only exists once the C language is enabled.
project(rtab_slam C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # for clangd / IDE
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)  # keep the binary relocatable with the tree

# ROS distribution that ships RTAB-Map: taken from the sourced environment
# (source /opt/ros/<distro>/setup.bash), jazzy when nothing is sourced.
set(ROS_DISTRO "$ENV{ROS_DISTRO}")
if(NOT ROS_DISTRO)
  set(ROS_DISTRO jazzy)
endif()

# Dependencies fetched by scripts/setup_deps.sh into ./deps (no root needed).
# When ./deps exists it wins over a system-wide install; delete the directory
# (or pass -DDEPS_DIR=/nonexistent) to build against /opt/ros/${ROS_DISTRO} instead.
set(DEPS_DIR "${CMAKE_SOURCE_DIR}/deps" CACHE PATH "Local dependency prefix")
if(EXISTS "${DEPS_DIR}")
  list(APPEND CMAKE_PREFIX_PATH
       "${DEPS_DIR}/opt/ros/${ROS_DISTRO}"
       "${DEPS_DIR}/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu"
       "${DEPS_DIR}/realsense2")
  # Transitive system libs that came with the packages (CXSparse for g2o, ...)
  link_directories("${DEPS_DIR}/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu"
                   "${DEPS_DIR}/usr/lib/x86_64-linux-gnu")
  # Relative to the executable ($ORIGIN = <root>/build), so moving the project
  # tree does not break an already built binary.
  set(DEPS_RPATH "$ORIGIN/../deps/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu;$ORIGIN/../deps/usr/lib/x86_64-linux-gnu;$ORIGIN/../deps/realsense2/lib/x86_64-linux-gnu")
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/${ROS_DISTRO}")

find_package(realsense2 REQUIRED)
find_package(RTABMap REQUIRED COMPONENTS core gui utilite)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(rtab_slam
  src/main.cpp
  src/realsense_capture.cpp
  src/slam_backend.cpp
  src/viewer.cpp
  src/map_export.cpp)
target_compile_options(rtab_slam PRIVATE -Wall -Wextra)
target_link_libraries(rtab_slam
  realsense2::realsense2
  rtabmap::core
  rtabmap::gui
  rtabmap::utilite
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})
# Public headers live in include/rtab_slam, so sources include them as
# "rtab_slam/<name>.hpp" and the prefix stays visible at every use site.
target_include_directories(rtab_slam PRIVATE include ${PCL_INCLUDE_DIRS})

# Run straight from build/ without setting LD_LIBRARY_PATH.
# $ORIGIN entries (when ./deps is used) come first, so the binary keeps working
# if the tree moves; the ROS prefix is the fallback for a system-wide install.
set(RTAB_RPATH ${DEPS_RPATH})
list(APPEND RTAB_RPATH "/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu")
set_target_properties(rtab_slam PROPERTIES BUILD_RPATH "${RTAB_RPATH}")
# DT_RPATH (old dtags) is inherited by the dependencies of our dependencies,
# which is what makes g2o/GTSAM resolve for librtabmap_core.so; with the default
# RUNPATH they would only be found through LD_LIBRARY_PATH.
target_link_options(rtab_slam PRIVATE "-Wl,--disable-new-dtags")

message(STATUS "RTAB-Map ${RTABMap_VERSION}, OpenCV ${OpenCV_VERSION}, PCL ${PCL_VERSION}, librealsense2 ${realsense2_VERSION}")
