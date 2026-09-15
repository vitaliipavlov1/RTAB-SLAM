cmake_minimum_required(VERSION 3.16)
project(rtabmap_minimal CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Dependencies fetched by scripts/setup_deps.sh into ./deps (no root needed).
# If RTAB-Map is installed system-wide (sudo apt install ros-humble-rtabmap),
# this block simply finds nothing extra and the system copy is used instead.
set(DEPS_DIR "${CMAKE_SOURCE_DIR}/deps" CACHE PATH "Local dependency prefix")
if(EXISTS "${DEPS_DIR}")
  list(APPEND CMAKE_PREFIX_PATH
       "${DEPS_DIR}/opt/ros/humble"
       "${DEPS_DIR}/opt/ros/humble/lib/x86_64-linux-gnu"
       "${DEPS_DIR}/realsense2")
  # Transitive system libs that came with the packages (CXSparse for g2o, ...)
  link_directories("${DEPS_DIR}/opt/ros/humble/lib/x86_64-linux-gnu"
                   "${DEPS_DIR}/usr/lib/x86_64-linux-gnu")
  set(DEPS_RPATH "${DEPS_DIR}/opt/ros/humble/lib/x86_64-linux-gnu;${DEPS_DIR}/usr/lib/x86_64-linux-gnu;${DEPS_DIR}/realsense2/lib/x86_64-linux-gnu")
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")

find_package(realsense2 REQUIRED)
find_package(RTABMap REQUIRED COMPONENTS core gui utilite)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(rtabmap_minimal src/main.cpp)
target_link_libraries(rtabmap_minimal
  realsense2::realsense2
  rtabmap::core
  rtabmap::gui
  rtabmap::utilite
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})
target_include_directories(rtabmap_minimal PRIVATE ${PCL_INCLUDE_DIRS})

# Run straight from build/ without setting LD_LIBRARY_PATH.
if(DEPS_RPATH)
  set_target_properties(rtabmap_minimal PROPERTIES BUILD_RPATH "${DEPS_RPATH}")
  # DT_RPATH (old dtags) is inherited by the dependencies of our dependencies,
  # which is what makes g2o/GTSAM resolve for librtabmap_core.so.
  target_link_options(rtabmap_minimal PRIVATE "-Wl,--disable-new-dtags")
endif()

message(STATUS "RTAB-Map ${RTABMap_VERSION}, OpenCV ${OpenCV_VERSION}, PCL ${PCL_VERSION}, librealsense2 ${realsense2_VERSION}")
